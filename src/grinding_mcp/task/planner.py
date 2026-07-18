"""便捷编排器：一键跑完第一步（任务编排）+ 第二步（细化），产出完整打磨方案。

这是 grinding_plan 的后端——把 decompose（分子任务）和 generate（逐子任务细化）串起来，
给「工件+规格→带序+工艺参数+robtarget」一个一步式入口。三步本身仍可单独调用；本模块
只是常用组合的糖。robtarget 存 ledger，方案对象只带摘要。
"""

from __future__ import annotations

from ..config import StationConfig
from ..ledger import Ledger, new_id
from ..solver import TargetSolver
from ..types import BeltPlan, GrindPlan, GrindSpec, Workpiece
from . import decompose as decompose_mod
from . import generate as generate_mod


def plan_workflow(
    ledger: Ledger,
    cfg: StationConfig,
    solver: TargetSolver,
    *,
    spec: GrindSpec,
    workpiece: Workpiece,
    region: dict | None = None,
    feed_mm_s: float = 20.0,
    max_passes: int = 5,
    phi_deg: float = 0.0,
    apportion: list[float] | None = None,
) -> GrindPlan:
    """第一步 decompose → 第二步逐子任务 generate，汇总成 GrindPlan。

    region 是所有子任务共用的区域（便捷入口的简化；要不同区域请分别调 decompose/generate）。
    """
    regions = [region] if region else None
    steps, warnings = decompose_mod.decompose(
        ledger, cfg, spec=spec, workpiece=workpiece, regions=regions,
        feed_mm_s=feed_mm_s, apportion=apportion,
    )

    belt_plans: list[BeltPlan] = []
    for step in steps:
        belt = cfg.belt(step.belt_id)
        ts, warns = generate_mod.generate_targets(
            ledger, cfg, solver, step, max_passes=max_passes, phi_deg=phi_deg)
        warnings.extend(warns)
        belt_plans.append(BeltPlan(
            order=step.order, belt_id=step.belt_id, grit=belt.grit,
            apportioned_removal_mm=step.target_removal_mm,
            contact_depth_mm=round(step.contact_depth_mm, 4), passes=step.passes,
            feed_mm_s=step.feed_mm_s,
            predicted_removal_mm=round(ts.removal.mean_mm, 4),
            targets_id=ts.targets_id, point_count=len(ts.targets),
            reachable_count=sum(1 for t in ts.targets if t.reachable),
        ))

    plan = GrindPlan(
        plan_id=new_id("plan"), spec_id=spec.spec_id,
        workpiece_id=workpiece.workpiece_id, belt_plans=belt_plans,
        warnings=sorted(set(warnings)),
    )
    ledger.put_plan(plan)
    return plan
