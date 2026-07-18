"""冒烟测试：桩模式下把三步迭代闭环跑通，断言行为契约而非快照。

三步（与设计一一对应）：
  第一步 编排：register_spec → decompose → 子任务（去哪磨/磨什么，不含点）
  第二步 细化：generate_targets（逐子任务）→ 轨迹 + 打磨点
  第三步 仿真评价：simulate → evaluate（单步）/ evaluate_plan（方案级）
"""

import asyncio
import math

import numpy as np

from grinding_mcp import evaluate as evaluate_mod
from grinding_mcp.config import load_config
from grinding_mcp.ledger import Ledger, new_id
from grinding_mcp.sim import collision as collision_mod
from grinding_mcp.sim import rws as rws_mod
from grinding_mcp.solver import BaselineTargetSolver
from grinding_mcp.solver.placement import build_rotation, place_point
from grinding_mcp.task import decompose as decompose_mod
from grinding_mcp.task import generate as generate_mod
from grinding_mcp.task import workflow as workflow_mod
from grinding_mcp.types import SimResult
from grinding_mcp.workpiece import SyntheticSource, select_region, summarize


def _spec(ledger, cfg, belts, target, tol=0.03):
    return workflow_mod.register_spec(
        ledger, cfg, workpiece="block", belts=belts,
        grit_sequence=[cfg.belt(b).grit for b in belts],
        target_removal_mm=target, tolerance_mm=tol,
    )


# --- 配置 / 标定 / 工件 --------------------------------------------------

def test_config_loads_four_belts():
    cfg = load_config()
    assert set(cfg.belts) == {"belt1", "belt2", "belt3", "belt4"}
    kps = {b.preston.kp for b in cfg.belts.values()}
    assert len(kps) > 1


def test_calibration_recovers_contact_geometry():
    """接触几何应已从示教点标定进配置：最干净的 belt4 法向以 -Y 为主、trusted。"""
    cfg = load_config()
    assert [b for b in cfg.belts.values() if b.contact is not None]
    b4 = cfg.belt("belt4")
    assert b4.contact is not None and b4.contact.trusted is True
    assert abs(b4.contact.normal[1]) > 0.9


def test_synthetic_workpiece_has_unit_normals():
    wp = SyntheticSource().load()
    assert len(wp.points) == len(wp.normals) > 0
    for nx, ny, nz in wp.normals:
        assert math.isclose(math.sqrt(nx * nx + ny * ny + nz * nz), 1.0, abs_tol=1e-6)


# --- 第二步几何：摆位方向必须对 ------------------------------------------

def test_placement_reproduces_taught_signature():
    """磨平面时姿态不变、位置移动；磨圆角时姿态跟随法向扫。"""
    cfg = load_config()
    belt = cfg.belt("belt4")
    n = np.array(belt.contact.normal)

    R = build_rotation(np.array([0, 0, 1.0]), n, 0.0)
    assert np.allclose(R @ np.array([0, 0, 1.0]), -n / np.linalg.norm(n), atol=1e-3)

    def qang(a, b):
        return math.degrees(2 * math.acos(min(abs(float(np.dot(a, b))), 1.0)))

    flat = [place_point(np.array([x, 0, 0.0]), np.array([0, 0, 1.0]), belt, 0.15, 0.0, i)
            for i, x in enumerate((0, 10, 20, 30))]
    assert qang(np.array(flat[0].rot), np.array(flat[-1].rot)) < 1.0
    assert np.linalg.norm(np.array(flat[0].trans) - np.array(flat[-1].trans)) > 25

    arc = []
    for i, a in enumerate(np.linspace(0, math.pi / 2, 5)):
        m = np.array([0.0, math.sin(a), math.cos(a)])
        arc.append(place_point(np.array([0, 0, 0.0]), m, belt, 0.15, 0.0, i))
    assert qang(np.array(arc[0].rot), np.array(arc[-1].rot)) > 80


# --- 第一步：编排产出子任务，不含点 -------------------------------------

def test_decompose_produces_subtasks_without_points():
    cfg = load_config()
    ledger = Ledger()
    wp = SyntheticSource().load()
    spec = _spec(ledger, cfg, ["belt1", "belt3", "belt4"], 0.12)
    steps, warnings = decompose_mod.decompose(ledger, cfg, spec=spec, workpiece=wp)

    assert len(steps) == 3
    # 粒度粗→精排序
    assert [cfg.belt(s.belt_id).grit for s in steps] == [60, 240, 400]
    # 分配去除量之和 ≈ 目标
    assert math.isclose(sum(s.target_removal_mm for s in steps), 0.12, abs_tol=1e-3)
    # 第一步不含点：尚未细化
    for s in steps:
        assert s.targets_id == "" and s.passes == 0
    # 占位系数风险如实带出
    assert any("占位" in w for w in warnings)


# --- 第二步：反解与正向自洽 ----------------------------------------------

def test_generate_targets_inverse_matches_forward():
    """未封顶时，反解出的 (压深,遍数) 正向预测应精确回到子任务的目标去除量。"""
    cfg = load_config()
    ledger = Ledger()
    solver = BaselineTargetSolver()
    wp = SyntheticSource().load()
    spec = _spec(ledger, cfg, ["belt1"], 0.05, tol=0.01)
    steps, _ = decompose_mod.decompose(
        ledger, cfg, spec=spec, workpiece=wp,
        regions=[{"normal_axis": [0, 0, 1], "min_dot": 0.9}],
    )
    ts, _ = generate_mod.generate_targets(ledger, cfg, solver, steps[0])

    step = ledger.get_step(steps[0].step_id)
    assert step.contact_depth_mm < 0.5          # 未封顶
    assert step.targets_id == ts.targets_id     # 回写子任务
    rel = abs(ts.removal.mean_mm - step.target_removal_mm) / step.target_removal_mm
    assert rel < 0.02, f"反解与正向应自洽，实际相对误差 {rel:.3f}"
    assert len(ts.targets) > 0


# --- 全链路：三步闭环（桩模式） ------------------------------------------

def test_three_stage_loop_stub_mode():
    cfg = load_config()
    ledger = Ledger()
    solver = BaselineTargetSolver()
    wp = SyntheticSource().load()
    spec = _spec(ledger, cfg, ["belt1", "belt3"], 0.10)

    # 第一步
    steps, _ = decompose_mod.decompose(
        ledger, cfg, spec=spec, workpiece=wp,
        regions=[{"normal_axis": [0, 0, 1], "min_dot": 0.9}],
    )
    assert len(steps) == 2

    # 第二步：逐子任务细化
    target_sets = [generate_mod.generate_targets(ledger, cfg, solver, s)[0] for s in steps]
    assert all(len(ts.targets) > 0 for ts in target_sets)

    # 第三步：仿真（桩，异步）+ 单步评价 + 方案级聚合评价
    async def _sim(ts):
        rws = rws_mod.RwsClient(cfg.rws, cfg.robot)
        rep = await rws.load_and_run("", "x.modx", len(ts.targets))
        coll = collision_mod.CollisionClient(cfg.collision)
        cr = await coll.check_path(ts.targets)
        return rep, cr

    for ts in target_sets:
        rep, cr = asyncio.run(_sim(ts))
        assert rep.stub is True and cr.stub is True
        sim = SimResult(
            sim_id=new_id("sim"), targets_id=ts.targets_id,
            total_count=len(ts.targets), reachable_count=sum(t.reachable for t in ts.targets),
            collisions=cr.collisions, singularities=rep.singularities,
            joint_limit_violations=rep.joint_limit_violations,
            cycle_time_s=rep.cycle_time_s, alarms=rep.alarms,
            joint_path=rep.joint_path, stub=True,
        )
        ledger.put_sim(sim)
        ev = evaluate_mod.evaluate(sim, ts, spec)
        assert ev.kinematic["reachable"] is True
        assert "in_tolerance" in ev.process

    # 方案级聚合：累计去除对总目标判
    agg = evaluate_mod.evaluate_plan(ledger, spec)
    assert agg["all_refined"] is True
    assert isinstance(agg["in_tolerance"], bool)
    assert math.isclose(
        agg["total_predicted_removal_mm"],
        sum(ts.removal.mean_mm for ts in target_sets), abs_tol=1e-3,
    )


# --- 几何摘要 + 大模型主导拆解 ------------------------------------------

def test_geometry_summary_segments_face_and_edge():
    """圆角块应切出 1 个上表面（face）+ 1 条圆角棱（edge），点数不丢。"""
    wp = SyntheticSource().load()
    overall, cands = summarize(wp)
    assert overall["point_count"] == len(wp.points)
    kinds = [c.kind for c in cands]
    assert "face" in kinds and "edge" in kinds
    assert sum(c.point_count for c in cands) == len(wp.points)
    # 上表面法向应近 +Z
    face = next(c for c in cands if c.kind == "face")
    assert face.normal[2] > 0.9


def test_region_id_selection_roundtrips():
    """inspect 把候选区域存进工件；用 region_id 应能选回同一批点。"""
    wp = SyntheticSource().load()
    _, cands = summarize(wp)
    rid = "region_test"
    wp.regions[rid] = cands[0].indices
    assert select_region(wp, {"region_id": rid}) == cands[0].indices
    # 未知 region_id → 空
    assert select_region(wp, {"region_id": "nope"}) == []


def test_add_step_flags_empty_region():
    """兜底：区域选不到点时 add_step 给 warning，但不阻断。"""
    cfg = load_config()
    ledger = Ledger()
    wp = SyntheticSource().load()
    ledger.put_workpiece(wp)
    spec = _spec(ledger, cfg, ["belt1"], 0.1)
    step, warnings = workflow_mod.add_step(
        ledger, cfg, spec_id=spec.spec_id, belt_id="belt1",
        region={"region_id": "nonexistent"}, order=0,
        workpiece_id=wp.workpiece_id, target_removal_mm=0.1,
    )
    assert step.step_id
    assert any("选不到" in w for w in warnings)


def test_grit_order_fallback_warns_on_reverse():
    """兜底：子任务带序非粗→精时给提醒。"""
    cfg = load_config()
    ledger = Ledger()
    wp = SyntheticSource().load()
    ledger.put_workpiece(wp)
    spec = _spec(ledger, cfg, ["belt1", "belt4"], 0.1)
    # 故意精带在前（belt4 grit400）、粗带在后（belt1 grit60）
    s0, _ = workflow_mod.add_step(ledger, cfg, spec_id=spec.spec_id, belt_id="belt4",
                                  region={}, order=0, workpiece_id=wp.workpiece_id, target_removal_mm=0.03)
    s1, _ = workflow_mod.add_step(ledger, cfg, spec_id=spec.spec_id, belt_id="belt1",
                                  region={}, order=1, workpiece_id=wp.workpiece_id, target_removal_mm=0.07)
    warns = decompose_mod.check_grit_order(cfg, [s0, s1])
    assert warns and "粗→精" in warns[0]


def test_llm_led_decomposition_different_regions():
    """大模型主导：不同区域派不同带（上表面粗磨、圆角精修），全链路跑通。"""
    cfg = load_config()
    ledger = Ledger()
    solver = BaselineTargetSolver()
    wp = SyntheticSource().load()
    ledger.put_workpiece(wp)
    _, cands = summarize(wp)
    for i, c in enumerate(cands):
        wp.regions[f"r{i}"] = c.indices
    face_i = next(i for i, c in enumerate(cands) if c.kind == "face")
    edge_i = next(i for i, c in enumerate(cands) if c.kind == "edge")

    spec = _spec(ledger, cfg, ["belt1", "belt4"], 0.15, tol=0.05)
    s_face, _ = workflow_mod.add_step(ledger, cfg, spec_id=spec.spec_id, belt_id="belt1",
                                      region={"region_id": f"r{face_i}"}, order=0,
                                      workpiece_id=wp.workpiece_id, target_removal_mm=0.12)
    s_edge, _ = workflow_mod.add_step(ledger, cfg, spec_id=spec.spec_id, belt_id="belt4",
                                      region={"region_id": f"r{edge_i}"}, order=1,
                                      workpiece_id=wp.workpiece_id, target_removal_mm=0.03)
    from grinding_mcp.task import generate as generate_mod
    ts_face, _ = generate_mod.generate_targets(ledger, cfg, solver, s_face)
    ts_edge, _ = generate_mod.generate_targets(ledger, cfg, solver, s_edge)
    # 两个子任务的点数应对应各自区域（不同）
    assert len(ts_face.targets) == cands[face_i].point_count
    assert len(ts_edge.targets) == cands[edge_i].point_count
    assert len(ts_face.targets) != len(ts_edge.targets)


def test_ledger_rejects_unknown_id():
    ledger = Ledger()
    try:
        ledger.get_spec("spec_nope")
        assert False, "应当抛 KeyError"
    except KeyError:
        pass
