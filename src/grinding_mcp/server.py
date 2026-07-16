"""grinding-mcp 服务端入口（FastMCP, stdio）。

暴露给智能体的工具面。设计准则：
  - 工具在正确的**高度**上——一次调用走完一个完整子问题，不做 RWS 薄封装。
  - 只返回**摘要 + ID**；robtarget/关节数组留在 ledger，用 ID 引用（守上下文缓存不变量）。
  - LLM 决策（选带/排序/诊断）在智能体侧；本服务只提供工具与确定性计算。

典型 ReAct 闭环：
  register_spec → add_step×N → generate_targets → simulate → evaluate
    → 未达标则智能体据 suggestions 改参数 → 重新 generate_targets → …
"""

from __future__ import annotations

import os

from mcp.server.fastmcp import FastMCP

from . import evaluate as evaluate_mod
from .config import StationConfig, load_config
from .ledger import Ledger, new_id
from .sim import collision as collision_mod
from .sim import rapid as rapid_mod
from .sim import rws as rws_mod
from .solver import BaselineSolver, Solver
from .task import workflow as workflow_mod
from .types import SimResult, TargetSet

mcp = FastMCP("grinding")

# 全局单例（进程级）。求解层可换：默认 baseline，你的算法就绪后在此替换。
_cfg: StationConfig = load_config()
_ledger = Ledger()
_solver: Solver = BaselineSolver()


# --- 任务层 -------------------------------------------------------------

@mcp.tool()
def grinding_station_info() -> dict:
    """返回工作站配置摘要：机器人、4 条带的参数、RWS/碰撞桥连接状态。"""
    return {
        "robot": {
            "model": _cfg.robot.model,
            "robotware": _cfg.robot.robotware,
            "target_proc": _cfg.robot.target_proc,
        },
        "belts": {
            bid: {
                "label": b.label, "tool": b.tool, "wobj": b.wobj,
                "grit": b.grit, "belt_speed_m_s": b.belt_speed_m_s,
                "preston_kp": b.preston.kp,
            }
            for bid, b in _cfg.belts.items()
        },
        "rws_connected": _cfg.rws.connected,
        "collision_connected": _cfg.collision.connected,
        "note": "rws/collision 未连接时走桩模式，返回假数据供闭环空跑。",
    }


@mcp.tool()
def grinding_register_spec(
    workpiece: str,
    belts: list[str],
    grit_sequence: list[int],
    target_removal_mm: float,
    tolerance_mm: float,
    surface_ra_um: float | None = None,
    raw_text: str = "",
) -> dict:
    """登记一份打磨规格（智能体解析自然语言要求后提交的结构化决策）。返回 spec_id。"""
    spec = workflow_mod.register_spec(
        _ledger, _cfg,
        workpiece=workpiece, belts=belts, grit_sequence=grit_sequence,
        target_removal_mm=target_removal_mm, tolerance_mm=tolerance_mm,
        surface_ra_um=surface_ra_um, raw_text=raw_text,
    )
    return {"spec_id": spec.spec_id, "workpiece": spec.workpiece, "belts": spec.belts}


@mcp.tool()
def grinding_add_step(
    spec_id: str,
    belt_id: str,
    region: dict,
    order: int,
    passes: int = 1,
    feed_mm_s: float = 20.0,
    contact_depth_mm: float = 0.1,
    dwell_s: float = 0.0,
) -> dict:
    """向工作流添加一个子步骤（某条带、某区域、某组工艺参数的一遍打磨）。

    region 支持：{"points": [{"pos":[x,y,z],"normal":[nx,ny,nz]}, ...]}
             或 {"bbox": {"min":[...],"max":[...]}, "samples": N}
    坐标在该带的 wobj 坐标系里。
    """
    step = workflow_mod.add_step(
        _ledger, _cfg,
        spec_id=spec_id, belt_id=belt_id, region=region, order=order,
        passes=passes, feed_mm_s=feed_mm_s,
        contact_depth_mm=contact_depth_mm, dwell_s=dwell_s,
    )
    return {"step_id": step.step_id, "belt_id": step.belt_id, "order": step.order}


@mcp.tool()
def grinding_list_workflow(spec_id: str) -> dict:
    """按顺序列出某规格下的所有子步骤摘要。"""
    steps = _ledger.steps_for_spec(spec_id)
    return {
        "spec_id": spec_id,
        "step_count": len(steps),
        "steps": [
            {"step_id": s.step_id, "order": s.order, "belt_id": s.belt_id,
             "passes": s.passes, "contact_depth_mm": s.contact_depth_mm}
            for s in steps
        ],
    }


# --- 求解层 -------------------------------------------------------------

@mcp.tool()
def grinding_generate_targets(step_id: str) -> dict:
    """为一个子步骤生成 robtarget：接触点 → 姿态优化（冗余角）→ 去除预测。

    完整数组存 ledger，只回摘要（点数、可达率、冗余角范围、去除量统计、姿态代价）。
    用 targets_id 引用；细看某段用 grinding_inspect_targets。
    """
    step = _ledger.get_step(step_id)
    belt = _cfg.belt(step.belt_id)

    points = _solver.contact_points(step, belt)
    targets, posture_cost = _solver.optimize_posture(points, belt)
    removal = _solver.predict_removal(targets, step, belt)

    ts = TargetSet(
        targets_id=new_id("targets"), step_id=step_id, belt_id=step.belt_id,
        targets=targets, removal=removal, posture_cost=posture_cost,
    )
    _ledger.put_targets(ts)

    angles = [t.redundancy_angle_deg for t in targets]
    reachable = sum(1 for t in targets if t.reachable)
    return {
        "targets_id": ts.targets_id,
        "point_count": len(targets),
        "reachable_count": reachable,
        "reachable_pct": round(100 * reachable / max(len(targets), 1), 1),
        "redundancy_angle_deg": {"min": min(angles, default=0), "max": max(angles, default=0)},
        "predicted_removal_mm": {
            "mean": round(removal.mean_mm, 4),
            "min": round(removal.min_mm, 4),
            "max": round(removal.max_mm, 4),
        },
        "posture_cost": posture_cost,
    }


@mcp.tool()
def grinding_inspect_targets(targets_id: str, start: int = 0, count: int = 10) -> dict:
    """按需查看某个 target 集合的一小段（调试用；不会一次性返回全部）。"""
    ts = _ledger.get_targets(targets_id)
    sl = ts.targets[start:start + count]
    return {
        "targets_id": targets_id,
        "total": len(ts.targets),
        "slice": [
            {"index": t.index, "trans": list(t.trans), "rot": list(t.rot),
             "redundancy_angle_deg": t.redundancy_angle_deg, "reachable": t.reachable}
            for t in sl
        ],
    }


# --- 仿真层 -------------------------------------------------------------

@mcp.tool()
async def grinding_simulate(targets_id: str) -> dict:
    """把 target 集合送进虚拟控制器执行一遍，取回运动学结果 + 几何碰撞。

    未连 RobotStudio 时走桩数据（假装可达、无碰撞、按点数估节拍），闭环照样能跑。
    返回 sim_id + 运动学摘要。
    """
    ts = _ledger.get_targets(targets_id)
    belt = _cfg.belt(ts.belt_id)

    module_text = rapid_mod.build_module(
        ts.targets, belt, _cfg.robot,
        header_comment=f"generated for step {ts.step_id} on {ts.belt_id}",
    )
    module_path = os.path.join(
        os.environ.get("TEMP", "/tmp"), f"{_cfg.robot.target_module}_{targets_id}.modx"
    )
    rapid_mod.write_module(module_text, module_path)

    rws_client = rws_mod.RwsClient(_cfg.rws, _cfg.robot)
    exec_report = await rws_client.load_and_run(module_text, module_path, len(ts.targets))

    coll_client = collision_mod.CollisionClient(_cfg.collision)
    coll_report = await coll_client.check_path(ts.targets)

    reachable = sum(1 for t in ts.targets if t.reachable)
    sim = SimResult(
        sim_id=new_id("sim"),
        targets_id=targets_id,
        total_count=len(ts.targets),
        reachable_count=reachable,
        collisions=coll_report.collisions,
        singularities=exec_report.singularities,
        joint_limit_violations=exec_report.joint_limit_violations,
        cycle_time_s=exec_report.cycle_time_s,
        alarms=exec_report.alarms,
        joint_path=exec_report.joint_path,
        stub=exec_report.stub or coll_report.stub,
    )
    _ledger.put_sim(sim)

    return {
        "sim_id": sim.sim_id,
        "stub": sim.stub,
        "rapid_module": module_path,
        "reachable": f"{reachable}/{sim.total_count}",
        "collision_count": len(sim.collisions),
        "singularity_count": len(sim.singularities),
        "joint_limit_violations": len(sim.joint_limit_violations),
        "cycle_time_s": sim.cycle_time_s,
        "alarm_count": len(sim.alarms),
    }


@mcp.tool()
def grinding_write_rapid(targets_id: str, path: str = "") -> dict:
    """把某 target 集合导出成 RAPID 模块文件（手动导入 RobotStudio 的兜底路径）。"""
    ts = _ledger.get_targets(targets_id)
    belt = _cfg.belt(ts.belt_id)
    text = rapid_mod.build_module(ts.targets, belt, _cfg.robot)
    out = path or os.path.join(
        os.environ.get("TEMP", "/tmp"), f"{_cfg.robot.target_module}_{targets_id}.modx"
    )
    rapid_mod.write_module(text, out)
    return {"path": out, "line_count": text.count(chr(10)), "proc": _cfg.robot.target_proc}


# --- 评价层 -------------------------------------------------------------

@mcp.tool()
def grinding_evaluate(sim_id: str) -> dict:
    """双评价器：对着规格判达标没有。返回运动学 + 工艺逐项结论、欠/过磨区域、改进提示。

    passed=True 需两维都过：可达/无碰撞/无奇异/无超限（运动学） 且 去除量在允差内（工艺）。
    只看运动学会「仿真通过但没磨对」——这是本工具刻意防的失败模式。
    """
    sim = _ledger.get_sim(sim_id)
    ts = _ledger.get_targets(sim.targets_id)
    step = _ledger.get_step(ts.step_id)
    spec = _ledger.get_spec(step.spec_id)

    ev = evaluate_mod.evaluate(sim, ts, spec)
    return {
        "sim_id": sim_id,
        "passed": ev.passed,
        "kinematic": ev.kinematic,
        "process": ev.process,
        "residual_region_count": len(ev.residual_regions),
        "residual_sample": ev.residual_regions[:10],
        "suggestions": ev.suggestions,
        "stub_warning": "结果基于桩数据，未连真控制器" if sim.stub else None,
    }


def main() -> None:
    """控制台入口：以 stdio 运行 MCP 服务端。"""
    mcp.run()


if __name__ == "__main__":
    main()
