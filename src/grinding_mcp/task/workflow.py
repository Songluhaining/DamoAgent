"""校验并登记智能体提交的打磨规格与子步骤。"""

from __future__ import annotations

from ..config import StationConfig
from ..ledger import Ledger, new_id
from ..types import GrindSpec, GrindStep


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
    passes: int = 1,
    feed_mm_s: float = 20.0,
    contact_depth_mm: float = 0.1,
    dwell_s: float = 0.0,
) -> GrindStep:
    ledger.get_spec(spec_id)   # 校验 spec 存在
    cfg.belt(belt_id)          # 校验带存在

    step = GrindStep(
        step_id=new_id("step"),
        spec_id=spec_id,
        belt_id=belt_id,
        order=order,
        region=region,
        passes=passes,
        feed_mm_s=feed_mm_s,
        contact_depth_mm=contact_depth_mm,
        dwell_s=dwell_s,
    )
    ledger.put_step(step)
    return step
