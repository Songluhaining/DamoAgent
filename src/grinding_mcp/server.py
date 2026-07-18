"""grinding-mcp 服务端入口（FastMCP, stdio）。

暴露给智能体的工具面。设计准则：
  - 工具在正确的**高度**上——一次调用走完一个完整子问题，不做 RWS 薄封装。
  - 只返回**摘要 + ID**；robtarget/关节数组留在 ledger，用 ID 引用（守上下文缓存不变量）。
  - LLM 决策（选带/排序/诊断）在智能体侧；本服务只提供工具与确定性计算。

三步迭代闭环（与用户设想一一对应）：
  第一步 任务编排：register_spec → decompose（或手工 add_step×N）→ 得到子任务（去哪磨/磨什么），不含点
  第二步 细化：    generate_targets（逐子任务）→ 轨迹 + 每个打磨点（robtarget）
  第三步 仿真评价： simulate → evaluate（单步）/ evaluate_plan（方案级）
    → 未达标则智能体据 suggestions 决定：回第二步调点，还是回第一步重分解
  grinding_plan 是「一键跑完第一、二步」的便捷入口。
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
from .solver import BaselineTargetSolver, TargetSolver
from .task import decompose as decompose_mod
from .task import generate as generate_mod
from .task import planner as planner_mod
from .task import workflow as workflow_mod
from .types import SimResult
from .workpiece import SyntheticSource, XyzFileSource

mcp = FastMCP("grinding")

# 全局单例（进程级）。第二步求解器可换：默认 baseline（φ=0），你的冗余角优化算法就绪后
# 在此替换成实现同一 TargetSolver 接口的对象即可。
_cfg: StationConfig = load_config()
_ledger = Ledger()
_solver: TargetSolver = BaselineTargetSolver()


# --- 站台信息 -----------------------------------------------------------

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
                "has_contact_geom": b.contact is not None,
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
def grinding_load_workpiece(
    source: str = "synthetic",
    path: str = "",
    name: str = "",
) -> dict:
    """加载工件表面点云。source="synthetic" 用内置圆角块（无需真实数据即可跑通），
    source="xyz" 从文本文件（每行 x y z[ nx ny nz]）读。点云存 server 侧，只回摘要+ID。

    真实点云（扫描仪 PLY/PCD、CAD STL）到位后按同一接口加读取实现，本工具签名不变。
    """
    if source == "xyz":
        if not path:
            raise ValueError("source=xyz 需要 path")
        wp = XyzFileSource(path, name=name).load()
    else:
        wp = SyntheticSource(name=name or "synthetic_block").load()
    _ledger.put_workpiece(wp)
    return {
        "workpiece_id": wp.workpiece_id,
        "name": wp.name,
        "point_count": len(wp.points),
        "has_normals": bool(wp.normals),
        "frame": wp.frame,
        "source": wp.source,
    }


# --- 第一步：任务编排（得到子任务，不含点） ------------------------------

@mcp.tool()
def grinding_decompose(
    spec_id: str,
    workpiece_id: str,
    regions: list[dict] | None = None,
    feed_mm_s: float = 20.0,
    apportion: list[float] | None = None,
) -> dict:
    """第一步：把打磨规格分解成一批子任务（去哪磨 + 用哪条带 + 磨掉多少），**不生成点**。

    默认：每条带一个子任务、粒度粗→精、几何级数分配去除量（粗带担大头）。可用 regions
    给每个子任务不同区域（如上表面、某条棱各一个），apportion 覆盖去除量分配。
    产物是可复核的中间态——智能体审过再进第二步 generate_targets 细化。

    regions 每项支持 {"normal_axis":[0,0,1],"min_dot":0.9}（选朝某向的面）或
    {"indices":[...]}（显式点）；省略则子任务共用全部点。
    """
    spec = _ledger.get_spec(spec_id)
    wp = _ledger.get_workpiece(workpiece_id)
    steps, warnings = decompose_mod.decompose(
        _ledger, _cfg, spec=spec, workpiece=wp,
        regions=regions, feed_mm_s=feed_mm_s, apportion=apportion,
    )
    return {
        "spec_id": spec_id,
        "workpiece_id": workpiece_id,
        "subtasks": [
            {"step_id": s.step_id, "order": s.order, "belt_id": s.belt_id,
             "grit": _cfg.belt(s.belt_id).grit,
             "target_removal_mm": s.target_removal_mm,
             "region": s.region or "全部点"}
            for s in steps
        ],
        "warnings": warnings,
        "note": "子任务未含打磨点；逐个调 grinding_generate_targets 进入第二步细化。",
    }


@mcp.tool()
def grinding_add_step(
    spec_id: str,
    belt_id: str,
    region: dict,
    order: int,
    workpiece_id: str = "",
    target_removal_mm: float = 0.0,
    feed_mm_s: float = 20.0,
) -> dict:
    """手工添加一个子任务（第一步的手动入口，与 decompose 的自动分解并列）。

    只定意图（区域 + 带 + 目标去除量）；压深/遍数/打磨点由第二步 generate_targets 填。
    region 支持 {"normal_axis":[0,0,1],"min_dot":0.9} 或 {"indices":[...]}。
    """
    step = workflow_mod.add_step(
        _ledger, _cfg,
        spec_id=spec_id, belt_id=belt_id, region=region, order=order,
        workpiece_id=workpiece_id, target_removal_mm=target_removal_mm, feed_mm_s=feed_mm_s,
    )
    return {"step_id": step.step_id, "belt_id": step.belt_id, "order": step.order,
            "target_removal_mm": step.target_removal_mm}


@mcp.tool()
def grinding_list_workflow(spec_id: str) -> dict:
    """按顺序列出某规格下的所有子任务摘要，标出是否已细化（第二步）。"""
    steps = _ledger.steps_for_spec(spec_id)
    return {
        "spec_id": spec_id,
        "subtask_count": len(steps),
        "subtasks": [
            {"step_id": s.step_id, "order": s.order, "belt_id": s.belt_id,
             "target_removal_mm": s.target_removal_mm,
             "refined": bool(s.targets_id),
             "contact_depth_mm": s.contact_depth_mm, "passes": s.passes}
            for s in steps
        ],
    }


# --- 第二步：细化（子任务 → 轨迹 + 打磨点） ------------------------------

@mcp.tool()
def grinding_generate_targets(
    step_id: str,
    max_passes: int = 5,
    redundancy_angle_deg: float = 0.0,
) -> dict:
    """第二步：把一个子任务细化成移动轨迹 + 每个打磨点（robtarget）。

    内部：取子任务的工件与区域 → 选点 → 求解器排轨迹、逐点摆位（用掉冗余角）、Preston
    反解压深/遍数 → 正向预测去除量。数值全由求解器算（不让 LLM 生成位姿）。压深/遍数
    写回子任务，robtarget 存 ledger。用 targets_id 送第三步 simulate。

    redundancy_angle_deg 是绕接触法向的冗余角（baseline 固定用它；真优化会逐点搜索）。
    """
    step = _ledger.get_step(step_id)
    ts, warnings = generate_mod.generate_targets(
        _ledger, _cfg, _solver, step,
        max_passes=max_passes, phi_deg=redundancy_angle_deg,
    )
    reachable = sum(1 for t in ts.targets if t.reachable)
    return {
        "targets_id": ts.targets_id,
        "step_id": step_id,
        "belt_id": step.belt_id,
        "point_count": len(ts.targets),
        "reachable": f"{reachable}/{len(ts.targets)}",
        "contact_depth_mm": round(step.contact_depth_mm, 4),
        "passes": step.passes,
        "predicted_removal_mm": {
            "mean": round(ts.removal.mean_mm, 4),
            "min": round(ts.removal.min_mm, 4),
            "max": round(ts.removal.max_mm, 4),
        },
        "target_removal_mm": step.target_removal_mm,
        "warnings": warnings,
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


@mcp.tool()
def grinding_plan(
    spec_id: str,
    workpiece_id: str,
    region: dict | None = None,
    feed_mm_s: float = 20.0,
    max_passes: int = 5,
    redundancy_angle_deg: float = 0.0,
) -> dict:
    """便捷入口：一键跑完第一步（编排）+ 第二步（细化），直接产出完整打磨方案。

    等价于 decompose 后对每个子任务 generate_targets。需要子任务用不同区域或想在两步之间
    审核时，请分开调 grinding_decompose / grinding_generate_targets。每带 robtarget 存
    ledger，用其 targets_id 送第三步 simulate。warnings 如实带出占位系数/暂定几何等风险。
    """
    spec = _ledger.get_spec(spec_id)
    wp = _ledger.get_workpiece(workpiece_id)
    plan = planner_mod.plan_workflow(
        _ledger, _cfg, _solver, spec=spec, workpiece=wp, region=region,
        feed_mm_s=feed_mm_s, max_passes=max_passes, phi_deg=redundancy_angle_deg,
    )
    return {
        "plan_id": plan.plan_id,
        "spec_id": plan.spec_id,
        "workpiece_id": plan.workpiece_id,
        "belt_plans": [
            {
                "order": bp.order, "belt_id": bp.belt_id, "grit": bp.grit,
                "apportioned_removal_mm": bp.apportioned_removal_mm,
                "contact_depth_mm": bp.contact_depth_mm, "passes": bp.passes,
                "feed_mm_s": bp.feed_mm_s,
                "predicted_removal_mm": bp.predicted_removal_mm,
                "targets_id": bp.targets_id,
                "reachable": f"{bp.reachable_count}/{bp.point_count}",
            }
            for bp in plan.belt_plans
        ],
        "warnings": plan.warnings,
    }


# --- 第三步：仿真 + 评价 ------------------------------------------------

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


@mcp.tool()
def grinding_evaluate(sim_id: str) -> dict:
    """双评价器（单条带/单子任务）：对着规格判达标没有。返回运动学 + 工艺逐项结论、
    欠/过磨区域、改进提示。

    passed=True 需两维都过：可达/无碰撞/无奇异/无超限（运动学） 且 去除量在允差内（工艺）。
    只看运动学会「仿真通过但没磨对」——这是本工具刻意防的失败模式。
    注意：多带方案里单带去除本就小于总目标，方案整体是否磨够请用 grinding_evaluate_plan。
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


@mcp.tool()
def grinding_evaluate_plan(spec_id: str) -> dict:
    """方案级聚合评价：整条带序累加的去除量是否达标（单步 evaluate 只看一条带）。

    多带方案的去除量是各带累加的，单带对总目标必然「不达标」。本工具把全部子任务的预测
    去除累加再对总目标 ± 允差判——这才是「整套方案磨够没有」的正确问法。运动学仍由各步
    evaluate 分别把关。
    """
    spec = _ledger.get_spec(spec_id)
    return evaluate_mod.evaluate_plan(_ledger, spec)


def main() -> None:
    """控制台入口：以 stdio 运行 MCP 服务端。"""
    mcp.run()


if __name__ == "__main__":
    main()
