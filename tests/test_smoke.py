"""冒烟测试：不装 RobotStudio、桩模式下把整个 ReAct 闭环跑通一遍。

register_spec → add_step → generate_targets → simulate → evaluate
断言的是行为契约（闭环连得通、ID 能串起来、双评价器两维都判），不是快照。
"""

import asyncio

from grinding_mcp import evaluate as evaluate_mod
from grinding_mcp.config import load_config
from grinding_mcp.ledger import Ledger, new_id
from grinding_mcp.sim import collision as collision_mod
from grinding_mcp.sim import rapid as rapid_mod
from grinding_mcp.sim import rws as rws_mod
from grinding_mcp.solver import BaselineSolver
from grinding_mcp.task import workflow as workflow_mod
from grinding_mcp.types import SimResult, TargetSet


def test_config_loads_four_belts():
    cfg = load_config()
    assert set(cfg.belts) == {"belt1", "belt2", "belt3", "belt4"}
    # 每条带参数不同——至少 Preston kp 应有差异
    kps = {b.preston.kp for b in cfg.belts.values()}
    assert len(kps) > 1


def test_full_loop_stub_mode():
    cfg = load_config()
    ledger = Ledger()
    solver = BaselineSolver()

    # 1. 规格
    spec = workflow_mod.register_spec(
        ledger, cfg,
        workpiece="test_part", belts=["belt1", "belt3"],
        grit_sequence=[60, 240], target_removal_mm=0.2, tolerance_mm=0.05,
    )

    # 2. 子步骤：显式给一段接触点
    region = {"points": [
        {"pos": [0, 0, 0], "normal": [0, 0, 1]},
        {"pos": [10, 0, 0], "normal": [0, 0, 1]},
        {"pos": [20, 0, 0], "normal": [0, 0, 1]},
    ]}
    step = workflow_mod.add_step(
        ledger, cfg, spec_id=spec.spec_id, belt_id="belt1",
        region=region, order=0, passes=2, contact_depth_mm=0.15,
    )

    # 3. 求解：接触点 → 姿态 → 去除
    points = solver.contact_points(step, cfg.belt("belt1"))
    assert len(points) == 3
    targets, cost = solver.optimize_posture(points, cfg.belt("belt1"))
    assert len(targets) == 3
    removal = solver.predict_removal(targets, step, cfg.belt("belt1"))
    assert removal.mean_mm >= 0
    ts = TargetSet(targets_id=new_id("targets"), step_id=step.step_id,
                   belt_id="belt1", targets=targets, removal=removal, posture_cost=cost)
    ledger.put_targets(ts)

    # 4. 仿真（桩模式，异步）
    async def _sim():
        module = rapid_mod.build_module(targets, cfg.belt("belt1"), cfg.robot)
        assert "damo_routine" in module and "Sltyuan" in module and "lun1" in module
        rws = rws_mod.RwsClient(cfg.rws, cfg.robot)
        exec_report = await rws.load_and_run(module, "x.modx", len(targets))
        assert exec_report.stub is True
        coll = collision_mod.CollisionClient(cfg.collision)
        cr = await coll.check_path(targets)
        assert cr.stub is True
        return exec_report, cr

    exec_report, cr = asyncio.run(_sim())
    sim = SimResult(
        sim_id=new_id("sim"), targets_id=ts.targets_id,
        total_count=len(targets), reachable_count=sum(t.reachable for t in targets),
        collisions=cr.collisions, singularities=exec_report.singularities,
        joint_limit_violations=exec_report.joint_limit_violations,
        cycle_time_s=exec_report.cycle_time_s, alarms=exec_report.alarms,
        joint_path=exec_report.joint_path, stub=True,
    )
    ledger.put_sim(sim)

    # 5. 双评价器：桩模式下运动学应全过；工艺看 Preston 占位系数
    ev = evaluate_mod.evaluate(sim, ts, spec)
    assert ev.kinematic["collision_free"] is True
    assert ev.kinematic["reachable"] is True
    assert "in_tolerance" in ev.process
    assert isinstance(ev.passed, bool)


def test_ledger_rejects_unknown_id():
    ledger = Ledger()
    try:
        ledger.get_spec("spec_nope")
        assert False, "应当抛 KeyError"
    except KeyError:
        pass
