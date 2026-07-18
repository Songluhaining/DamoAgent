"""双评价器：把仿真结果对着规格判「达标没有」，产出结构化诊断供智能体 ReAct。

两个正交维度，缺一不可：
  运动学（kinematic）—— 来自 RWS + 碰撞桥：可达？碰撞？奇异？超限？
  工艺（process）—— 来自 Preston 去除模型：去除量够不够？在允差内吗？哪块欠/过磨？

关键点：RobotStudio 只能回答运动学，回答不了工艺。若只看运动学，闭环会「仿真通过但
工件没磨对」地空转成功——这是最坏的失败模式。所以两维都过才算 passed。

本模块只做**确定性判定**，不替智能体做决策：它给出逐项结论 + 欠磨区域 + 改进方向提示，
由智能体（LLM）据此推理下一轮改什么（改工件摆放？改姿态？加遍数？换带？）。
"""

from __future__ import annotations

from .ledger import Ledger
from .types import Evaluation, GrindSpec, SimResult, TargetSet


def evaluate(sim: SimResult, ts: TargetSet, spec: GrindSpec) -> Evaluation:
    # --- 运动学维度 ---
    reach_ok = sim.reachable_count == sim.total_count
    collide_ok = len(sim.collisions) == 0
    singular_ok = len(sim.singularities) == 0
    limit_ok = len(sim.joint_limit_violations) == 0
    kinematic = {
        "reachable": reach_ok,
        "reachable_count": sim.reachable_count,
        "total_count": sim.total_count,
        "collision_free": collide_ok,
        "collision_count": len(sim.collisions),
        "singularity_free": singular_ok,
        "joint_limits_ok": limit_ok,
        "cycle_time_s": sim.cycle_time_s,
        "alarm_count": len(sim.alarms),
    }
    kinematic_ok = reach_ok and collide_ok and singular_ok and limit_ok and not sim.alarms

    # --- 工艺维度：去除量 vs 目标 ± 允差 ---
    target = spec.target_removal_mm
    tol = spec.tolerance_mm
    removal = ts.removal
    lo, hi = target - tol, target + tol

    residual_regions: list[dict] = []
    under = over = 0
    for i, dh in enumerate(removal.per_point_mm):
        if dh < lo:
            under += 1
            residual_regions.append({"index": i, "kind": "under", "removal_mm": dh, "target_mm": target})
        elif dh > hi:
            over += 1
            residual_regions.append({"index": i, "kind": "over", "removal_mm": dh, "target_mm": target})

    process_ok = under == 0 and over == 0
    process = {
        "target_removal_mm": target,
        "tolerance_mm": tol,
        "mean_removal_mm": removal.mean_mm,
        "min_removal_mm": removal.min_mm,
        "max_removal_mm": removal.max_mm,
        "under_count": under,
        "over_count": over,
        "in_tolerance": process_ok,
    }

    # --- 改进方向提示（确定性，非 LLM 决策） ---
    suggestions: list[str] = []
    if not reach_ok:
        suggestions.append("有点不可达：考虑调整工件摆放/夹持姿态，或优化冗余角以改善可达性。")
    if not collide_ok:
        suggestions.append(f"检出 {len(sim.collisions)} 处碰撞：需改姿态或退让接触段。")
    if not singular_ok:
        suggestions.append("路径经过奇异位形：调整冗余角避开。")
    if under:
        suggestions.append(f"{under} 点欠磨：增大压入深度/驻留/遍数，或换更粗的带。")
    if over:
        suggestions.append(f"{over} 点过磨：减小压入深度/进给加快，或换更细的带。")

    passed = kinematic_ok and process_ok
    return Evaluation(
        passed=passed,
        kinematic=kinematic,
        process=process,
        residual_regions=residual_regions[:50],   # 摘要，避免灌爆上下文
        suggestions=suggestions,
    )


def evaluate_plan(ledger: Ledger, spec: GrindSpec) -> dict:
    """方案级聚合评价：整条带序累加的去除量是否达标（单步 evaluate 只看一条带）。

    多带方案里去除量是各带累加的——粗带担大头、精带精修。单步 evaluate 拿单带去除对
    总目标判，必然「不达标」；这里把全部子任务的预测去除**累加**再对总目标 ± 允差判，
    才是「整套方案磨够没有」的正确问法。仅看工艺维度；运动学仍由各步 evaluate 分别把关。
    """
    steps = ledger.steps_for_spec(spec.spec_id)
    target, tol = spec.target_removal_mm, spec.tolerance_mm

    per_belt = []
    total_mean = 0.0
    refined = 0
    for s in steps:
        entry = {
            "step_id": s.step_id, "belt_id": s.belt_id, "order": s.order,
            "target_removal_mm": s.target_removal_mm,
            "refined": bool(s.targets_id),
        }
        if s.targets_id:
            refined += 1
            ts = ledger.get_targets(s.targets_id)
            entry["predicted_removal_mm"] = round(ts.removal.mean_mm, 4)
            total_mean += ts.removal.mean_mm
        per_belt.append(entry)

    all_refined = refined == len(steps) and steps
    in_tol = all_refined and (target - tol) <= total_mean <= (target + tol)
    suggestions: list[str] = []
    if not steps:
        suggestions.append("方案无子任务：先跑第一步 decompose。")
    elif not all_refined:
        suggestions.append(f"{len(steps) - refined} 个子任务尚未细化：对其跑第二步 generate_targets。")
    elif total_mean < target - tol:
        suggestions.append("累计去除不足：增大某带压深/遍数，或提高某子任务的分配去除量。")
    elif total_mean > target + tol:
        suggestions.append("累计去除超标：减小某带压深/遍数，或下调分配去除量。")

    return {
        "spec_id": spec.spec_id,
        "target_removal_mm": target,
        "tolerance_mm": tol,
        "total_predicted_removal_mm": round(total_mean, 4),
        "in_tolerance": in_tol,
        "all_refined": bool(all_refined),
        "belt_breakdown": per_belt,
        "suggestions": suggestions,
    }
