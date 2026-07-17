"""摆位求解核心：把工件表面点摆到砂带接触点上，算出 robtarget。

站台约定：机器人夹着工件去蹭固定砂带（tool.robhold=TRUE / wobj.robhold=FALSE）。
要磨工件表面上一点 q（外法向 m），机器人必须把工件摆成——q 贴到砂带接触点 C、
工件外法向 m 压向砂带（对齐到 -n）。于是这一点的 robtarget（TCP 在 wobj 系的位姿）：

    R   把工件法向 m 转到 -n（对齐接触面），再绕接触法向附加冗余角 φ
    trans = C - δ·n - R·q         （δ = 压入深度；假设 TCP 在工件原点，见下）
    rot   = quat(R)

冗余角 φ 是那第 6 个自由度：绕接触法向转工件，不改变去除位置，但改变关节构型刚度，
且因砂带有走向 t，还改变工件相对带速的夹角（Preston 的滑动速度与磨纹方向）。
baseline 取 φ=0；真优化在此搜索 φ 使刚度最大——这是留给你算法的接缝。

关于 TCP 偏置：本模块假设 TCP 落在工件原点（p_tcp=0）。站台真实 TCP（Sltyuan 等）
是工件上某固定点，偏置由装夹决定。要精确复现，把 R·q 换成 R·(q - p_tcp)，p_tcp
从装夹标定得到——换到 RobotStudio 机器后补。当前 p_tcp=0 足够跑通闭环与验证方向。

这个模型能复现示教签名：磨平面（m 恒定）→ 位置沿直线扫、姿态不变；磨圆角（m 旋转）
→ 位置在 C 附近划小弧、姿态大幅扫。正是 Sltcs.modx 里那 490 个点的样子。
"""

from __future__ import annotations

import math

import numpy as np

from ..config import BeltParams
from ..types import RobTarget


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _mat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵 → 四元数 [w,x,y,z]（ABB robtarget 的 q1..q4）。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


def _rodrigues(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """绕单位轴转 angle 的旋转矩阵。"""
    k = _unit(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle_rad) * K + (1 - math.cos(angle_rad)) * (K @ K)


def _align_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """把单位向量 a 转到单位向量 b 的最小旋转矩阵（绕 a×b）。"""
    a, b = _unit(a), _unit(b)
    c = np.dot(a, b)
    if c > 1 - 1e-9:
        return np.eye(3)
    if c < -1 + 1e-9:                     # 反向：绕任意正交轴转 180°
        axis = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
        axis = _unit(axis - np.dot(axis, a) * a)
        return _rodrigues(axis, math.pi)
    return _rodrigues(np.cross(a, b), math.acos(max(-1.0, min(1.0, c))))


def build_rotation(
    m_wp: np.ndarray, normal_wobj: np.ndarray, phi_deg: float
) -> np.ndarray:
    """工件→wobj 的旋转 R：把工件外法向 m 压向砂带（R·m = -n），再绕接触法向转 φ。

    m 转到 -n 固定了 2 个自由度；绕接触法向的自旋是冗余角 φ（第 6 自由度）。
    φ=0 取最小旋转作参考；真优化搜 φ 使刚度最大、并调节工件相对砂带走向的夹角。
    """
    b = _unit(-normal_wobj)               # 工件外法向应指向的 wobj 方向（压向砂带）
    R0 = _align_rotation(_unit(m_wp), b)  # 最小旋转把 m 转到 -n
    spin = _rodrigues(b, math.radians(phi_deg))
    return spin @ R0


def place_point(
    q_wp: np.ndarray,
    m_wp: np.ndarray,
    belt: BeltParams,
    penetration_mm: float,
    phi_deg: float,
    index: int = 0,
) -> RobTarget:
    """把工件点 q（工件系，外法向 m）摆到砂带接触点，返回 robtarget。

    q_wp / m_wp 在工件系里。接触几何 (C, n) 在该带 wobj 系里。假设 TCP 在工件原点，
    故 trans = C - δ·n - R·q（R 为工件→wobj 旋转）。
    """
    if belt.contact is None:
        raise ValueError(f"带 {belt.belt_id} 无接触几何，先跑 calibrate 标定")
    C = np.array(belt.contact.point, dtype=float)
    n = _unit(np.array(belt.contact.normal, dtype=float))

    R = build_rotation(np.array(m_wp, dtype=float), n, phi_deg)
    trans = C - penetration_mm * n - R @ np.array(q_wp, dtype=float)
    return RobTarget(
        index=index,
        trans=tuple(float(v) for v in trans),
        rot=_mat_to_quat(R),
        redundancy_angle_deg=float(phi_deg),
        reachable=True,        # 摆位不做真实 IK，仿真层复核
    )
