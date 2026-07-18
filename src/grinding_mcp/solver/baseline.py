"""baseline 第二步求解：能跑通闭环的占位实现，非最优。

作用有二：
  1. 在你的真算法就绪前，让「子任务 → 轨迹 + 打磨点」这一步立刻可运行；
  2. 作为回归基准——真算法的刚度/节拍/去除均匀度应当优于它。

三处 baseline 简化（都是你后面要替换的点）：
  排轨迹     贪心最近邻，仅消除访问顺序伪影，不做行距/抬刀/避让的真轨迹规划。
  摆位       冗余角 φ 固定（默认 0），不搜索刚度最优，仅用 placement 把点摆到砂带接触。
  反解       Preston 单点线性反解压深/遍数，压深封顶在现实上限。
"""

from __future__ import annotations

import numpy as np

from ..config import BeltParams
from . import placement
from . import removal as removal_mod
from .base import TargetSolution, TargetSolver

# 压入深度的现实上限（mm）。超过它砂带会堵死/工件烧伤，宁可加遍数也不加深压。
_MAX_DEPTH_MM = 0.5


def _order_path(points: list[tuple[float, float, float]]) -> list[int]:
    """贪心最近邻把点串成一条路径，返回访问顺序（下标）。

    区域选点是无序的；按原序连成路径会有来回大跳，使驻留时间估计失真。从一个角点起、
    每步走最近未访问点，得相邻间距大致均匀的路径。真轨迹规划（行距/抬刀/避让）以后再做。
    """
    n = len(points)
    if n <= 2:
        return list(range(n))
    P = np.array(points, dtype=float)
    start = int(np.argmin(P.sum(axis=1)))       # 一个角点作起点，稳定可复现
    visited = [start]
    seen = {start}
    for _ in range(n - 1):
        d = np.linalg.norm(P - P[visited[-1]], axis=1)
        d[list(seen)] = np.inf
        nxt = int(np.argmin(d))
        visited.append(nxt)
        seen.add(nxt)
    return visited


def _pick_depth(belt: BeltParams) -> float:
    """初始压深猜测（mm），按接触轮硬度粗调：硬轮压得浅。真优化应结合力学。"""
    shore = belt.contact_wheel_hardness_shore
    return 0.20 if shore < 65 else (0.15 if shore < 80 else 0.10)


class BaselineTargetSolver(TargetSolver):
    def solve(
        self,
        *,
        points: list[tuple[float, float, float]],
        normals: list[tuple[float, float, float]],
        belt: BeltParams,
        target_removal_mm: float,
        feed_mm_s: float,
        max_passes: int,
        phi_deg: float,
        warnings: list[str],
    ) -> TargetSolution:
        if belt.contact is None:
            warnings.append(f"{belt.belt_id}: 无接触几何，无法摆位——先跑 calibrate。")
            return TargetSolution()
        if not points or not normals:
            warnings.append("区域无点或点云无法向，无法摆位。")
            return TargetSolution()

        # 1) 排轨迹
        order = _order_path(points)
        pts = [points[i] for i in order]
        nms = [normals[i] for i in order]

        # 2) 名义压深下先摆位（段长与压深无关，压深只沿法向平移常量）
        nominal = _pick_depth(belt)
        targets = [
            placement.place_point(np.array(q), np.array(m), belt, nominal, phi_deg, index=i)
            for i, (q, m) in enumerate(zip(pts, nms))
        ]

        # 3) 用 robtarget 实际段长（wobj 系）反解——与正向 predict 同源，预测才精确对齐
        seg = removal_mod._segment_lengths(targets)
        seg_len = float(seg.mean()) if len(seg) else 5.0
        seg_len = seg_len if seg_len > 1e-6 else 5.0
        depth, passes = self._solve_process(
            target_removal_mm, belt, feed_mm_s, seg_len, max_passes, warnings)

        # 4) 正向预测（与反解同参数）
        removal = removal_mod.predict_with(targets, belt, depth, passes, feed_mm_s)
        return TargetSolution(targets=targets, contact_depth_mm=depth, passes=passes, removal=removal)

    def _solve_process(
        self, target: float, belt: BeltParams, feed_mm_s: float, seg_len: float,
        max_passes: int, warnings: list[str],
    ) -> tuple[float, int]:
        """Preston 反解：给定目标去除量，推荐 (压深, 遍数)。

        先用硬度猜的名义压深算需要几遍（受 max_passes 限）；再在该遍数下回解精确压深并
        夹在现实上限内。封顶后仍达不到就如实警告——绝不推荐 1mm+ 的荒谬压深去凑目标。
        """
        nominal = _pick_depth(belt)
        passes = removal_mod.solve_passes_for_removal(target, nominal, feed_mm_s, seg_len, belt)
        passes = max(1, min(passes, max_passes))
        depth = removal_mod.solve_depth_for_removal(target, passes, feed_mm_s, seg_len, belt)
        if depth > _MAX_DEPTH_MM:
            depth = _MAX_DEPTH_MM
            max_ach = removal_mod.removal_per_pass(depth, feed_mm_s, seg_len, belt) * passes
            if max_ach < target * 0.95:
                warnings.append(
                    f"{belt.belt_id}: 目标去除 {target:.3f}mm 在现实参数内（压深≤{_MAX_DEPTH_MM}"
                    f"mm、≤{max_passes}遍）达不到，最多约 {max_ach:.3f}mm——增加遍数或换粗带。")
        return depth, passes
