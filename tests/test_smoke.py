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


def test_calibration_recovers_contact_geometry():
    """接触几何应已从示教点标定进配置：至少一条带的法向接近实测（-Y 主导）。"""
    cfg = load_config()
    with_contact = [b for b in cfg.belts.values() if b.contact is not None]
    assert with_contact, "标定后应至少一条带有接触几何"
    # belt4（Cfx/lun1）最干净，法向应以 -Y 为主
    b4 = cfg.belt("belt4")
    assert b4.contact is not None and b4.contact.trusted is True
    assert abs(b4.contact.normal[1]) > 0.9   # 法向主要沿 Y


def test_synthetic_workpiece_has_unit_normals():
    from grinding_mcp.workpiece import SyntheticSource
    wp = SyntheticSource().load()
    assert len(wp.points) == len(wp.normals) > 0
    import math
    for nx, ny, nz in wp.normals:
        assert math.isclose(math.sqrt(nx * nx + ny * ny + nz * nz), 1.0, abs_tol=1e-6)


def test_placement_reproduces_taught_signature():
    """摆位方向必须对：磨平面时姿态不变、位置移动；磨圆角时姿态跟随法向扫。"""
    import math

    import numpy as np

    from grinding_mcp.solver.placement import build_rotation, place_point

    cfg = load_config()
    belt = cfg.belt("belt4")
    n = np.array(belt.contact.normal)

    # R·m 应对齐到 -n（工件外法向压向砂带）
    R = build_rotation(np.array([0, 0, 1.0]), n, 0.0)
    assert np.allclose(R @ np.array([0, 0, 1.0]), -n / np.linalg.norm(n), atol=1e-3)

    def qang(a, b):
        return math.degrees(2 * math.acos(min(abs(float(np.dot(a, b))), 1.0)))

    # 平面：法向恒定 → 姿态不变、位置随点移动
    flat = [place_point(np.array([x, 0, 0.0]), np.array([0, 0, 1.0]), belt, 0.15, 0.0, i)
            for i, x in enumerate((0, 10, 20, 30))]
    assert qang(np.array(flat[0].rot), np.array(flat[-1].rot)) < 1.0
    assert np.linalg.norm(np.array(flat[0].trans) - np.array(flat[-1].trans)) > 25

    # 圆角：法向从 +Z 扫到 +Y → 姿态大幅变化
    arc = []
    for i, a in enumerate(np.linspace(0, math.pi / 2, 5)):
        m = np.array([0.0, math.sin(a), math.cos(a)])
        arc.append(place_point(np.array([0, 0, 0.0]), m, belt, 0.15, 0.0, i))
    assert qang(np.array(arc[0].rot), np.array(arc[-1].rot)) > 80


def test_plan_inverse_matches_forward():
    """Preston 反解出的 (压深,遍数) 正向预测应精确回到分配的去除量（未封顶时）。"""
    from grinding_mcp.task import planner
    from grinding_mcp.workpiece import SyntheticSource

    cfg = load_config()
    ledger = Ledger()
    wp = SyntheticSource().load()
    spec = workflow_mod.register_spec(
        ledger, cfg, workpiece="block", belts=["belt1"],
        grit_sequence=[60], target_removal_mm=0.05, tolerance_mm=0.01,
    )
    plan = planner.plan_workflow(
        ledger, cfg, spec=spec, workpiece=wp,
        region={"normal_axis": [0, 0, 1], "min_dot": 0.9},
    )
    bp = plan.belt_plans[0]
    assert bp.contact_depth_mm < 0.5   # 未封顶
    rel = abs(bp.predicted_removal_mm - bp.apportioned_removal_mm) / bp.apportioned_removal_mm
    assert rel < 0.02, f"反解与正向应自洽，实际相对误差 {rel:.3f}"
    # robtarget 应确实生成并存进 ledger
    ts = ledger.get_targets(bp.targets_id)
    assert len(ts.targets) == bp.point_count > 0


def test_plan_flags_placeholder_risks():
    """方案必须如实带出风险：占位系数、暂定几何——绝不吞掉。"""
    from grinding_mcp.task import planner
    from grinding_mcp.workpiece import SyntheticSource

    cfg = load_config()
    ledger = Ledger()
    wp = SyntheticSource().load()
    spec = workflow_mod.register_spec(
        ledger, cfg, workpiece="block", belts=["belt1", "belt4"],
        grit_sequence=[60, 400], target_removal_mm=0.2, tolerance_mm=0.05,
    )
    plan = planner.plan_workflow(ledger, cfg, spec=spec, workpiece=wp)
    assert any("占位" in w for w in plan.warnings)
