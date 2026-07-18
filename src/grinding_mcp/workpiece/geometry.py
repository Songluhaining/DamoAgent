"""几何摘要：把工件点云自动切成候选区域（面 / 棱），供大模型决定「去哪磨」。

大模型主导任务拆解，但它得先看懂工件几何。本模块按**法向聚类**把点云切成：
  - 面（face）：法向一致的一片点（平面/近平面），如上表面、侧面。
  - 棱/过渡（edge）：法向快速变化、不归属任何大面的点，如圆角、倒角、锐边。

产出每个候选区域的代表法向、点数、形心、包围盒 + 点索引。索引留 server 侧（存进
ledger 的命名区域，用 region_id 引用），只把摘要回给大模型——守「不灌数组进上下文」。

这是几何理解的最小可用实现。真实扫描点云的曲率分割/特征识别以后可增强，接口不变。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..types import Workpiece

# 法向同簇阈值：dot > 此值视为同一面（0.95 ≈ 夹角 18°内）
_SAME_NORMAL_DOT = 0.95


@dataclass
class RegionCandidate:
    """一个候选打磨区域。indices 是点下标（留 server 侧），其余是给大模型看的摘要。"""
    kind: str                                  # "face" | "edge"
    normal: tuple[float, float, float]         # 代表法向（面内均值；棱区意义不大）
    point_count: int
    centroid: tuple[float, float, float]
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    indices: list[int] = field(default_factory=list)


def _cluster_by_normal(N: np.ndarray) -> list[list[int]]:
    """贪心法向聚类：每点归入第一个夹角够近的簇，否则开新簇。返回每簇的点下标。"""
    reps: list[np.ndarray] = []
    members: list[list[int]] = []
    for i, n in enumerate(N):
        placed = False
        for k, rep in enumerate(reps):
            if float(np.dot(n, rep)) > _SAME_NORMAL_DOT:
                members[k].append(i)
                # 更新代表法向为running mean并归一化
                m = np.array([N[j] for j in members[k]]).mean(axis=0)
                reps[k] = m / max(np.linalg.norm(m), 1e-9)
                placed = True
                break
        if not placed:
            reps.append(n / max(np.linalg.norm(n), 1e-9))
            members.append([i])
    return members


def _make_candidate(kind: str, idx: list[int], P: np.ndarray, N: np.ndarray) -> RegionCandidate:
    sub_p = P[idx]
    sub_n = N[idx]
    nrm = sub_n.mean(axis=0)
    nrm = nrm / max(np.linalg.norm(nrm), 1e-9)
    return RegionCandidate(
        kind=kind,
        normal=tuple(round(float(v), 4) for v in nrm),
        point_count=len(idx),
        centroid=tuple(round(float(v), 2) for v in sub_p.mean(axis=0)),
        bbox_min=tuple(round(float(v), 2) for v in sub_p.min(axis=0)),
        bbox_max=tuple(round(float(v), 2) for v in sub_p.max(axis=0)),
        indices=idx,
    )


def summarize(wp: Workpiece) -> tuple[dict, list[RegionCandidate]]:
    """→ (整体摘要 dict, 候选区域列表)。无法向时只给包围盒，不切区域。"""
    P = np.array(wp.points, dtype=float)
    overall = {
        "point_count": len(wp.points),
        "frame": wp.frame,
        "bbox_min": [round(float(v), 2) for v in P.min(axis=0)] if len(P) else [],
        "bbox_max": [round(float(v), 2) for v in P.max(axis=0)] if len(P) else [],
    }
    if not wp.normals:
        return overall, []

    N = np.array(wp.normals, dtype=float)
    clusters = _cluster_by_normal(N)

    # 大簇算面，小簇归并成棱/过渡。阈值取 max(20, 10% 点数)：值得单列的面至少占表面 10%，
    # 否则法向连续变化的圆角会被碎成一串小角度带（每带都超小阈值），而非识别成一条棱。
    face_min = max(20, int(0.10 * len(wp.points)))
    faces = [c for c in clusters if len(c) >= face_min]
    edge_idx = [i for c in clusters if len(c) < face_min for i in c]

    candidates = [_make_candidate("face", sorted(c), P, N)
                  for c in sorted(faces, key=len, reverse=True)]
    if edge_idx:
        candidates.append(_make_candidate("edge", sorted(edge_idx), P, N))
    return overall, candidates
