"""方案规划：工件点云 + 打磨需求 + 带参数 → 打磨方案（带序 + 工艺参数 + robtarget）。

这是「给定打磨方案」的落地。分工仍守规矩：
  - 数值内核（每点摆位 placement、Preston 反解 solve_depth/passes）由本层算；
  - 带序与去除量分配是**可被智能体覆盖的启发式默认**（离散工艺决策本应归 LLM，
    这里只给一个合理起点，agent 可通过参数改）。

默认分配启发式：按带的粒度粗→精排序，去除量按几何级数递减分配（粗带担大头、精带做
精修），归一化到目标总量。最后落到每条带：选区域点 → Preston 反解压深 → placement
生成 robtarget。

风险提示（warnings）会如实带出：Preston 系数是占位值、接触几何暂定、坐标系未核实——
这些直接决定方案能不能下真控制器，绝不能吞掉。
"""

from __future__ import annotations

import numpy as np

from ..config import BeltParams, StationConfig
from ..ledger import Ledger, new_id
from ..solver import placement
from ..solver import removal as removal_mod
from ..types import (
    BeltPlan,
    GrindPlan,
    GrindSpec,
    GrindStep,
    RemovalField,
    TargetSet,
    Workpiece,
)


def _order_path(points: list[tuple[float, float, float]]) -> list[int]:
    """把一堆点贪心最近邻串成一条路径，返回访问顺序（下标）。

    区域选点是无序的；直接按原序连成路径会有来回大跳，使驻留时间估计失真。
    从最边角点起、每步走最近未访问点，得到相邻间距大致均匀的路径。这是最简的轨迹
    排序占位——真实轨迹规划（行距、抬刀、避让）以后单独做，这里先消除排序伪影。
    """
    n = len(points)
    if n <= 2:
        return list(range(n))
    P = np.array(points, dtype=float)
    start = int(np.argmin(P.sum(axis=1)))     # 一个角点作起点，稳定可复现
    visited = [start]
    seen = {start}
    for _ in range(n - 1):
        cur = P[visited[-1]]
        d = np.linalg.norm(P - cur, axis=1)
        d[list(seen)] = np.inf
        nxt = int(np.argmin(d))
        visited.append(nxt)
        seen.add(nxt)
    return visited


def _mean_segment_len(points: list[tuple[float, float, float]]) -> float:
    """相邻点平均间距（mm），Preston 反解要用。点少时给个安全默认。"""
    if len(points) < 2:
        return 5.0
    P = np.array(points, dtype=float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    m = float(seg.mean())
    return m if m > 1e-6 else 5.0


def _select_region(wp: Workpiece, region: dict | None) -> list[int]:
    """从工件点云选出要磨的点的下标。

    region 支持：
      None / {}                 全部点
      {"normal_axis":[0,0,1], "min_dot":0.7}   只留法向与某轴夹角小的（如上表面）
      {"indices":[...]}         显式下标
    真实的曲面分割（按棱、按区块）以后按需扩展，这里给最小可用集。
    """
    n = len(wp.points)
    if not region:
        return list(range(n))
    if "indices" in region:
        return [int(i) for i in region["indices"] if 0 <= int(i) < n]
    if "normal_axis" in region and wp.normals:
        axis = np.array(region["normal_axis"], dtype=float)
        axis /= max(np.linalg.norm(axis), 1e-9)
        min_dot = float(region.get("min_dot", 0.7))
        out = []
        for i, nm in enumerate(wp.normals):
            if np.dot(np.array(nm, dtype=float), axis) >= min_dot:
                out.append(i)
        return out
    return list(range(n))


def _apportion(total_removal: float, n_belts: int) -> list[float]:
    """把总去除量按几何级数（粗带担大头）分给 n 条带，和为 total。

    权重 [2^(n-1), ..., 2, 1] 归一化。单带则全给它。可被智能体覆盖。
    """
    if n_belts <= 1:
        return [total_removal]
    weights = [2.0 ** (n_belts - 1 - i) for i in range(n_belts)]
    s = sum(weights)
    return [total_removal * w / s for w in weights]


def _order_belts_by_grit(cfg: StationConfig, belt_ids: list[str]) -> list[str]:
    """按粒度粗→精（grit 数字小→大）排序。"""
    return sorted(belt_ids, key=lambda b: cfg.belt(b).grit)


def plan_workflow(
    ledger: Ledger,
    cfg: StationConfig,
    *,
    spec: GrindSpec,
    workpiece: Workpiece,
    region: dict | None = None,
    feed_mm_s: float = 20.0,
    max_passes: int = 5,
    phi_deg: float = 0.0,
    apportion: list[float] | None = None,
) -> GrindPlan:
    """生成完整打磨方案。robtarget 存 ledger，方案对象只带摘要。"""
    ledger.put_workpiece(workpiece)
    warnings: list[str] = []

    idx = _select_region(workpiece, region)
    if not idx:
        warnings.append("区域筛选后无点——检查 region 条件或点云法向。")
    sel_pts = [workpiece.points[i] for i in idx]
    order = _order_path(sel_pts)               # 贪心最近邻排成一条路径
    pts = [sel_pts[i] for i in order]
    nms = ([workpiece.normals[idx[i]] for i in order] if workpiece.normals else [])
    if not nms:
        warnings.append("点云无法向，无法摆位——需先做法向估计。")

    belt_ids = _order_belts_by_grit(cfg, spec.belts)
    shares = apportion or _apportion(spec.target_removal_mm, len(belt_ids))

    belt_plans: list[BeltPlan] = []
    for order, (bid, share) in enumerate(zip(belt_ids, shares)):
        belt = cfg.belt(bid)
        _accumulate_warnings(belt, warnings)

        # 先在名义压深下摆位（段长与压深无关，压深只沿法向平移常量），
        # 用 robtarget 实际段长（wobj 系）反解——与 predict 同源，预测才精确对齐。
        targets = _place_all(pts, nms, belt, _pick_depth(belt), phi_deg)
        seg_len = _effective_seg_len(targets, pts)
        depth, passes = _solve_process(share, belt, feed_mm_s, seg_len, max_passes, warnings)
        predicted = _predict(targets, belt, depth, passes, feed_mm_s)

        # 建真实 GrindStep 存 ledger：方案就是一条工作流，这样 simulate/evaluate
        # 能顺 targets→step→spec 回溯，与手工 add_step 的链路完全一致。
        step = GrindStep(
            step_id=new_id("step"), spec_id=spec.spec_id, belt_id=bid, order=order,
            region=region or {}, passes=passes, feed_mm_s=feed_mm_s, contact_depth_mm=depth,
        )
        ledger.put_step(step)
        ts = TargetSet(
            targets_id=new_id("targets"), step_id=step.step_id,
            belt_id=bid, targets=targets, removal=predicted, posture_cost=0.0,
        )
        ledger.put_targets(ts)

        belt_plans.append(BeltPlan(
            order=order, belt_id=bid, grit=belt.grit,
            apportioned_removal_mm=round(share, 4),
            contact_depth_mm=round(depth, 4), passes=passes, feed_mm_s=feed_mm_s,
            predicted_removal_mm=round(predicted.mean_mm, 4),
            targets_id=ts.targets_id, point_count=len(targets),
            reachable_count=sum(1 for t in targets if t.reachable),
        ))

    plan = GrindPlan(
        plan_id=new_id("plan"), spec_id=spec.spec_id,
        workpiece_id=workpiece.workpiece_id, belt_plans=belt_plans,
        warnings=sorted(set(warnings)),
    )
    ledger.put_plan(plan)
    return plan


# 压入深度的现实上限（mm）。超过它砂带会堵死/工件烧伤，宁可加遍数也不加深压。
_MAX_DEPTH_MM = 0.5


def _pick_depth(belt: BeltParams) -> float:
    """给个初始压深猜测（mm），按接触轮硬度粗调：硬轮压得浅。真优化应结合力学。"""
    shore = belt.contact_wheel_hardness_shore
    return 0.20 if shore < 65 else (0.15 if shore < 80 else 0.10)


def _solve_process(
    share: float, belt: BeltParams, feed_mm_s: float, seg_len: float,
    max_passes: int, warnings: list[str],
) -> tuple[float, int]:
    """Preston 反解：给定分给这条带的去除量，推荐 (压深, 遍数)。

    策略：先用硬度猜的名义压深算需要几遍；遍数受 max_passes 限。若名义压深下遍数够用，
    在该遍数下回解精确压深（并夹在现实上限内）；若封顶遍数仍达不到，如实警告——绝不
    靠推荐一个 1mm+ 的荒谬压深去凑目标。
    """
    nominal = _pick_depth(belt)
    passes = removal_mod.solve_passes_for_removal(share, nominal, feed_mm_s, seg_len, belt)
    passes = max(1, min(passes, max_passes))

    depth = removal_mod.solve_depth_for_removal(share, passes, feed_mm_s, seg_len, belt)
    if depth > _MAX_DEPTH_MM:
        depth = _MAX_DEPTH_MM
        # 封顶压深 + 封顶遍数下的最大可去除量
        max_ach = removal_mod.removal_per_pass(depth, feed_mm_s, seg_len, belt) * passes
        if max_ach < share * 0.95:
            warnings.append(
                f"{belt.belt_id}: 目标去除 {share:.3f}mm 在现实参数内（压深≤{_MAX_DEPTH_MM}"
                f"mm、≤{max_passes}遍）达不到，最多约 {max_ach:.3f}mm——增加遍数或换粗带。")
    return depth, passes


def _accumulate_warnings(belt: BeltParams, warnings: list[str]) -> None:
    if belt.preston.kp and belt.preston.p_exp == 1.0 and belt.preston.v_exp == 1.0:
        warnings.append(f"{belt.belt_id}: Preston 系数为占位值（指数=1未拟合），去除量预测仅定性。")
    if belt.contact is None:
        warnings.append(f"{belt.belt_id}: 无接触几何，无法摆位——先跑 calibrate。")
    elif not belt.contact.trusted:
        warnings.append(f"{belt.belt_id}: 接触几何暂定（残差{belt.contact.resid_rms_mm}mm/坐标系待核实）。")


def _place_all(
    pts: list[tuple[float, float, float]],
    nms: list[tuple[float, float, float]],
    belt: BeltParams,
    depth: float,
    phi_deg: float,
) -> list:
    """对区域每点做摆位，返回 robtarget 列表（无接触几何或无法向则空）。"""
    if belt.contact is None or not nms:
        return []
    return [
        placement.place_point(np.array(q), np.array(m), belt, depth, phi_deg, index=i)
        for i, (q, m) in enumerate(zip(pts, nms))
    ]


def _effective_seg_len(targets: list, pts: list[tuple[float, float, float]]) -> float:
    """反解要用的等效段长：robtarget（wobj 系）逐点段长的均值，与 predict 同源。

    predict 的平均去除 ∝ 平均逐点段长；反解用同一均值，预测才会精确回到分配值。
    无 robtarget（缺接触几何）时退回工件系点间距。
    """
    if targets:
        pp = removal_mod._segment_lengths(targets)
        m = float(pp.mean()) if len(pp) else 0.0
        return m if m > 1e-6 else 5.0
    return _mean_segment_len(pts)


def _predict(targets: list, belt: BeltParams, depth: float, passes: int, feed_mm_s: float) -> RemovalField:
    """用推荐的 (压深, 遍数, 进给) 正向预测去除量分布。参数须与反解一致。"""
    if not targets:
        return RemovalField()
    step = GrindStep(
        step_id=f"plan:{belt.belt_id}", spec_id="", belt_id=belt.belt_id,
        order=0, region={}, passes=passes, feed_mm_s=feed_mm_s, contact_depth_mm=depth,
    )
    return removal_mod.predict(targets, step, belt)
