"""校验并登记智能体提交的打磨规格与子步骤。"""

from __future__ import annotations

from ..config import StationConfig
from ..ledger import Ledger, new_id
from ..types import GrindSpec, GrindStep
from ..workpiece import select_region


def register_spec(
    ledger: Ledger,
    cfg: StationConfig,
    *,
    workpiece: str,
    belts: list[str],
    grit_sequence: list[int],
    target_removal_mm: float,
    tolerance_mm: float,
    surface_ra_um: float | None = None,
    raw_text: str = "",
) -> GrindSpec:
    # 校验带 id 都存在
    for b in belts:
        cfg.belt(b)   # 不存在会抛 KeyError
    if target_removal_mm <= 0:
        raise ValueError("target_removal_mm 必须为正")
    if tolerance_mm < 0:
        raise ValueError("tolerance_mm 不能为负")

    spec = GrindSpec(
        spec_id=new_id("spec"),
        workpiece=workpiece,
        belts=belts,
        grit_sequence=grit_sequence,
        target_removal_mm=target_removal_mm,
        tolerance_mm=tolerance_mm,
        surface_ra_um=surface_ra_um,
        raw_text=raw_text,
    )
    ledger.put_spec(spec)
    return spec


def add_step(
    ledger: Ledger,
    cfg: StationConfig,
    *,
    spec_id: str,
    belt_id: str,
    region: dict,
    order: int,
    workpiece_id: str = "",
    target_removal_mm: float = 0.0,
    feed_mm_s: float = 20.0,
    dwell_s: float = 0.0,
) -> tuple[GrindStep, list[str]]:
    """手工添加一个子任务（大模型主导拆解的入口，与 decompose 的自动建议并列）。

    只定意图（区域 + 带 + 目标去除量）；压深/遍数/打磨点由第二步 generate_targets 填。
    返回 (子任务, 兜底校验警告)。硬错误（spec/带/工件不存在、目标去除≤0）直接抛；
    软问题（区域选不到点、无接触几何）进 warnings 提醒大模型，不阻断。
    """
    ledger.get_spec(spec_id)   # 校验 spec 存在
    belt = cfg.belt(belt_id)   # 校验带存在
    if target_removal_mm < 0:
        raise ValueError("target_removal_mm 不能为负")

    warnings: list[str] = []
    point_count = None
    if workpiece_id:
        wp = ledger.get_workpiece(workpiece_id)   # 校验工件存在
        point_count = len(select_region(wp, region))
        if point_count == 0:
            warnings.append("该区域选不到任何点——检查 region_id 是否有效或筛选条件。")
    if belt.contact is None:
        warnings.append(f"{belt_id}: 无接触几何，第二步无法摆位——先跑 calibrate。")

    step = GrindStep(
        step_id=new_id("step"),
        spec_id=spec_id,
        belt_id=belt_id,
        order=order,
        region=region,
        workpiece_id=workpiece_id,
        target_removal_mm=target_removal_mm,
        feed_mm_s=feed_mm_s,
        dwell_s=dwell_s,
    )
    ledger.put_step(step)
    return step, warnings
