"""第二步：细化。一个子任务（GrindStep）→ 移动轨迹 + 每个打磨点（robtarget）。

流程：取子任务的工件与区域 → 选点 → 交给 TargetSolver 排轨迹+摆位+反解工艺参数 →
把反解出的压深/遍数写回子任务、robtarget 存 ledger。数值全由求解器算（不让 LLM 生成
位姿）。求解器可换：baseline 是占位，你的冗余角优化算法实现同一接口即可。
"""

from __future__ import annotations

from ..config import StationConfig
from ..ledger import Ledger, new_id
from ..solver import TargetSolver
from ..types import GrindStep, TargetSet
from ..workpiece import select_region


def generate_targets(
    ledger: Ledger,
    cfg: StationConfig,
    solver: TargetSolver,
    step: GrindStep,
    *,
    max_passes: int = 5,
    phi_deg: float = 0.0,
) -> tuple[TargetSet, list[str]]:
    """细化一个子任务，返回 (TargetSet, 风险提示)。同时把压深/遍数/targets_id 写回子任务。"""
    belt = cfg.belt(step.belt_id)
    wp = ledger.get_workpiece(step.workpiece_id) if step.workpiece_id else None
    warnings: list[str] = []

    if wp is None:
        warnings.append(f"子任务 {step.step_id} 未关联工件，无法取点。")
        idx: list[int] = []
    else:
        idx = select_region(wp, step.region)
        if not idx:
            warnings.append("区域筛选后无点——检查 region 条件或点云法向。")

    pts = [wp.points[i] for i in idx] if wp else []
    nms = [wp.normals[i] for i in idx] if (wp and wp.normals) else []

    sol = solver.solve(
        points=pts, normals=nms, belt=belt,
        target_removal_mm=step.target_removal_mm, feed_mm_s=step.feed_mm_s,
        max_passes=max_passes, phi_deg=phi_deg, warnings=warnings,
    )

    ts = TargetSet(
        targets_id=new_id("targets"), step_id=step.step_id, belt_id=step.belt_id,
        targets=sol.targets, removal=sol.removal, posture_cost=0.0,
    )
    ledger.put_targets(ts)

    # 把第二步的产出写回子任务：压深/遍数（工艺参数）+ 当前 robtarget 集合
    step.contact_depth_mm = sol.contact_depth_mm
    step.passes = sol.passes
    step.targets_id = ts.targets_id
    ledger.put_step(step)

    return ts, warnings
