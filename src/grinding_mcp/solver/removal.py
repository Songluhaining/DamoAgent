"""Preston 材料去除模型。

    dh = kp · p^a · vs^b · dt

  dh  单点去除深度
  kp  Preston 系数（每条带不同，随磨损衰减；当前为占位值，待用历史数据拟合）
  p   接触压强（纯位置控制下由压入深度 contact_depth 决定；严格应走 Hertz 接触分布）
  vs  相对滑动速度（≈ 带速；砂带方向使其与冗余角耦合）
  dt  驻留时间（≈ 段弧长 / 进给速度，再乘遍数）

这是评价器「工艺」半边的真值来源——RobotStudio 只管运动学，不管磨没磨够。
纯位置控制下没有力反馈兜底，所以这个模型的标定精度直接决定闭环是否可靠。

fit_from_history() 是留给你的拟合入口：把 4 条带的历史磨削数据（去除量 vs 压深/带速/
驻留）回归出各自的 (kp, p_exp, v_exp)。当前是占位，返回配置里的默认值。
"""

from __future__ import annotations

import numpy as np

from ..config import BeltParams
from ..types import GrindStep, RemovalField, RobTarget


def _segment_lengths(targets: list[RobTarget]) -> np.ndarray:
    """相邻点间距（mm），用于估驻留时间。"""
    if len(targets) < 2:
        return np.zeros(len(targets))
    pts = np.array([t.trans for t in targets], dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    # 每个点分摊前后半段
    per_point = np.zeros(len(targets))
    per_point[:-1] += seg / 2
    per_point[1:] += seg / 2
    return per_point


def predict(
    targets: list[RobTarget], step: GrindStep, belt: BeltParams
) -> RemovalField:
    if not targets:
        return RemovalField()

    kp = belt.preston.kp
    a = belt.preston.p_exp
    b = belt.preston.v_exp

    # 压强 p：纯位置控制下用压入深度做一阶代理（占位；严格版走 Hertz 弹性接触，
    # 压强沿接触宽度非均匀分布，且依赖接触轮 shore 硬度）。
    p = max(step.contact_depth_mm, 1e-6)

    # 滑动速度 vs ≈ 带速（m/s → mm/s）
    vs = belt.belt_speed_m_s * 1000.0

    # 驻留时间 dt：段弧长 / 进给，乘遍数
    feed = max(step.feed_mm_s, 1e-6)
    dwell_per_point = _segment_lengths(targets) / feed * max(step.passes, 1)
    dwell_per_point += step.dwell_s

    dh = kp * (p ** a) * (vs ** b) * dwell_per_point   # 逐点去除深度 mm
    dh = np.clip(dh, 0.0, None)

    return RemovalField(
        per_point_mm=[float(x) for x in dh],
        mean_mm=float(dh.mean()),
        min_mm=float(dh.min()),
        max_mm=float(dh.max()),
    )


def removal_per_pass(
    contact_depth_mm: float, feed_mm_s: float, seg_len_mm: float, belt: BeltParams
) -> float:
    """单遍、单点的去除深度（mm）。planner 反解时的正向内核。

    dh_pass = kp · p^a · vs^b · (seg_len/feed)
    """
    p = max(contact_depth_mm, 1e-6)
    vs = belt.belt_speed_m_s * 1000.0
    dt = max(seg_len_mm, 1e-6) / max(feed_mm_s, 1e-6)
    return belt.preston.kp * (p ** belt.preston.p_exp) * (vs ** belt.preston.v_exp) * dt


def solve_depth_for_removal(
    target_removal_mm: float,
    passes: int,
    feed_mm_s: float,
    seg_len_mm: float,
    belt: BeltParams,
) -> float:
    """反解：给定目标去除量与遍数，求所需压入深度（mm）。

    对 dh = kp·p^a·vs^b·dt·passes 解 p：
        p = (target / (kp·vs^b·dt·passes))^(1/a)
    a=1（线性）时就是简单除法。这是「给定打磨方案」的数值内核之一。
    """
    vs = belt.belt_speed_m_s * 1000.0
    dt = max(seg_len_mm, 1e-6) / max(feed_mm_s, 1e-6)
    denom = belt.preston.kp * (vs ** belt.preston.v_exp) * dt * max(passes, 1)
    if denom <= 0:
        return 0.0
    ratio = target_removal_mm / denom
    if ratio <= 0:
        return 0.0
    return float(ratio ** (1.0 / belt.preston.p_exp))


def solve_passes_for_removal(
    target_removal_mm: float,
    contact_depth_mm: float,
    feed_mm_s: float,
    seg_len_mm: float,
    belt: BeltParams,
) -> int:
    """反解：给定目标去除量与压深，求所需遍数（向上取整，至少 1）。"""
    per = removal_per_pass(contact_depth_mm, feed_mm_s, seg_len_mm, belt)
    if per <= 0:
        return 1
    return max(1, int(np.ceil(target_removal_mm / per)))


def fit_from_history(records: list[dict]) -> dict[str, tuple[float, float, float]]:
    """占位：从历史磨削数据拟合各带 (kp, p_exp, v_exp)。

    records 期望形如 {belt_id, contact_depth_mm, belt_speed_m_s, dwell_s, measured_removal_mm}。
    实现时对 log(dh) = log(kp) + a·log(p) + b·log(vs·dt) 做线性回归。
    你有历史数据后在此落实；当前返回空表示沿用配置默认值。
    """
    # TODO(user): 用真实历史数据实现最小二乘拟合
    return {}
