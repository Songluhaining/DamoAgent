"""baseline 求解实现——能把闭环跑通的占位版，不是最优。

刻意做得简单、可预测，作用有二：
  1. 在你的真算法就绪前，让整个 parse→plan→solve→simulate→evaluate 闭环立刻可运行；
  2. 作为回归基准——真算法的刚度/节拍/去除均匀度应当优于它。

三处 baseline 简化（都是你后面要替换的点）：
  contact_points   直接取区域里显式给的点集，或在包围盒上等距采样；法向暂用 +Z。
  optimize_posture 冗余角固定为 0，不做刚度优化，仅让工具轴对齐接触法向。
  predict_removal  直接调 removal.predict（Preston 占位系数）。
"""

from __future__ import annotations

import math

import numpy as np

from ..config import BeltParams
from ..types import ContactPoint, GrindStep, RemovalField, RobTarget
from . import removal
from .base import Solver


def _normal_to_quat(normal: tuple[float, float, float], angle_deg: float) -> tuple[float, float, float, float]:
    """让工具 Z 轴对准接触法向的反向，绕该轴附加 angle_deg 冗余转，返回四元数。

    baseline 只做「对准法向」这一最低要求；冗余角当前恒为 0。
    真实姿态优化会在这里搜索 angle 以最大化刚度。
    """
    n = np.array(normal, dtype=float)
    norm = np.linalg.norm(n)
    z = n / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
    z = -z  # 工具指向工件表面

    # 任取与 z 正交的 x
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    # 绕 z 转冗余角
    a = math.radians(angle_deg)
    x2 = math.cos(a) * x + math.sin(a) * y

    R = np.column_stack([x2, np.cross(z, x2), z])
    return _mat_to_quat(R)


def _mat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
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


class BaselineSolver(Solver):
    def contact_points(self, step: GrindStep, belt: BeltParams) -> list[ContactPoint]:
        region = step.region or {}

        # 情况 A：区域直接给了点集
        explicit = region.get("points")
        if explicit:
            pts = []
            for i, p in enumerate(explicit):
                pos = tuple(float(v) for v in p.get("pos", (0, 0, 0)))
                normal = tuple(float(v) for v in p.get("normal", (0, 0, 1)))
                pts.append(ContactPoint(index=i, pos=pos, normal=normal))
            return pts

        # 情况 B：给了包围盒 + 采样数，在一条直线上等距采样（占位）
        bbox = region.get("bbox")
        n = int(region.get("samples", 10))
        if bbox:
            p0 = np.array(bbox.get("min", (0, 0, 0)), dtype=float)
            p1 = np.array(bbox.get("max", (100, 0, 0)), dtype=float)
            pts = []
            for i in range(n):
                t = i / max(n - 1, 1)
                pos = tuple(float(v) for v in (p0 + t * (p1 - p0)))
                pts.append(ContactPoint(index=i, pos=pos, normal=(0.0, 0.0, 1.0)))
            return pts

        # 兜底：单点原点
        return [ContactPoint(index=0, pos=(0.0, 0.0, 0.0), normal=(0.0, 0.0, 1.0))]

    def optimize_posture(
        self, points: list[ContactPoint], belt: BeltParams
    ) -> tuple[list[RobTarget], float]:
        targets: list[RobTarget] = []
        for cp in points:
            quat = _normal_to_quat(cp.normal, angle_deg=0.0)  # baseline 冗余角=0
            targets.append(
                RobTarget(
                    index=cp.index,
                    trans=cp.pos,
                    rot=quat,
                    redundancy_angle_deg=0.0,
                    reachable=True,     # baseline 不做真实 IK 可达判定，仿真层复核
                )
            )
        # baseline 不算刚度，姿态代价记 0（真算法在此返回 Σ 刚度倒数等）
        return targets, 0.0

    def predict_removal(
        self, targets: list[RobTarget], step: GrindStep, belt: BeltParams
    ) -> RemovalField:
        return removal.predict(targets, step, belt)
