"""第一步：任务编排的**算法兜底自动建议**。工件 + 规格 → 一批子任务（去哪磨+哪条带+磨多少）。

拆解本身由**大模型主导**：它用 inspect_workpiece 看懂几何、决定哪块区域用哪条带磨多少，
通过 workflow.add_step 逐个下子任务。本模块是**兜底/起点**——当大模型想要一个默认方案时，
按确定性规则给建议：带序粒度粗→精、去除量几何级数分配（粗带担大头）。大模型可通过
regions/apportion 覆盖，或完全不用它、自己用 add_step 编排。

不含打磨点——只定意图，供审核后再进第二步细化。风险提示（占位系数/暂定几何/无接触
几何）在此带出，因为审子任务、选带就在这一步。
"""

from __future__ import annotations

from ..config import BeltParams, StationConfig
from ..ledger import Ledger, new_id
from ..types import GrindSpec, GrindStep, Workpiece


def order_belts_by_grit(cfg: StationConfig, belt_ids: list[str]) -> list[str]:
    """按粒度粗→精（grit 数字小→大）排序。"""
    return sorted(belt_ids, key=lambda b: cfg.belt(b).grit)


def check_grit_order(cfg: StationConfig, steps: list[GrindStep]) -> list[str]:
    """兜底：大模型自排的子任务顺序若不是粒度粗→精，给个提醒（不阻断，可能有意为之）。

    粗→精是打磨的物理常识（先粗磨去量、再细磨修面）。大模型主导拆解时，这条确定性规则
    由代码把关：发现 grit 随 order 非单调递增就提醒，让它确认是不是有意跳序。
    """
    ordered = sorted(steps, key=lambda s: s.order)
    grits = [cfg.belt(s.belt_id).grit for s in ordered]
    if any(a > b for a, b in zip(grits, grits[1:])):
        return [f"子任务带序非粗→精（grit 序列 {grits}）：确认是否有意，否则先粗后精。"]
    return []


def apportion_removal(total_removal: float, n: int) -> list[float]:
    """把总去除量按几何级数（粗带担大头）分给 n 个子任务，和为 total。

    权重 [2^(n-1), ..., 2, 1] 归一化；单带则全给它。可被智能体覆盖。
    """
    if n <= 1:
        return [total_removal]
    weights = [2.0 ** (n - 1 - i) for i in range(n)]
    s = sum(weights)
    return [total_removal * w / s for w in weights]


def belt_risk_warnings(belt: BeltParams) -> list[str]:
    """审子任务时该知道的风险：占位系数、暂定/缺失接触几何。"""
    out: list[str] = []
    if belt.preston.kp and belt.preston.p_exp == 1.0 and belt.preston.v_exp == 1.0:
        out.append(f"{belt.belt_id}: Preston 系数为占位值（指数=1未拟合），去除量预测仅定性。")
    if belt.contact is None:
        out.append(f"{belt.belt_id}: 无接触几何，无法摆位——先跑 calibrate。")
    elif not belt.contact.trusted:
        out.append(f"{belt.belt_id}: 接触几何暂定（残差{belt.contact.resid_rms_mm}mm/坐标系待核实）。")
    return out


def decompose(
    ledger: Ledger,
    cfg: StationConfig,
    *,
    spec: GrindSpec,
    workpiece: Workpiece,
    regions: list[dict] | None = None,
    feed_mm_s: float = 20.0,
    apportion: list[float] | None = None,
) -> tuple[list[GrindStep], list[str]]:
    """产出子任务列表（存 ledger）+ 风险提示。默认：每条带一个子任务、同一区域、
    粒度粗→精、几何级数分配去除量。

    regions：可给每个子任务不同区域（如上表面、某条棱各一个子任务）；省略则全用同一
    区域（默认全部点）。apportion：可覆盖默认去除量分配。
    """
    ledger.put_workpiece(workpiece)

    belt_ids = order_belts_by_grit(cfg, spec.belts)
    shares = apportion or apportion_removal(spec.target_removal_mm, len(belt_ids))

    warnings: list[str] = []
    steps: list[GrindStep] = []
    for order, (bid, share) in enumerate(zip(belt_ids, shares)):
        warnings.extend(belt_risk_warnings(cfg.belt(bid)))
        if regions:
            region = regions[order] if order < len(regions) else regions[-1]
        else:
            region = {}
        step = GrindStep(
            step_id=new_id("step"), spec_id=spec.spec_id,
            workpiece_id=workpiece.workpiece_id, belt_id=bid, order=order,
            region=region, target_removal_mm=round(share, 4), feed_mm_s=feed_mm_s,
        )
        ledger.put_step(step)
        steps.append(step)

    return steps, sorted(set(warnings))
