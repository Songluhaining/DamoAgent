"""区域选点：把「去哪磨」的区域描述（region）应用到工件点云，选出点下标。

区域由第一步（任务编排）定义为一个选择器 dict，第二步（细化）在这里应用它取点。
支持的 region：
    None / {}                                全部点
    {"region_id": "region_xxx"}               引用 inspect_workpiece 切好的命名区域（推荐）
    {"indices": [...]}                        显式下标
    {"normal_axis": [0,0,1], "min_dot": 0.7}  只留法向与某轴夹角小的（如上表面）

真实的曲面分割（按棱、按区块、按曲率）以后按需扩展，这里给最小可用集。
"""

from __future__ import annotations

import numpy as np

from ..types import Workpiece


def select_region(wp: Workpiece, region: dict | None) -> list[int]:
    """返回工件点云里落在该区域的点下标。"""
    n = len(wp.points)
    if not region:
        return list(range(n))
    if "region_id" in region:
        return list(wp.regions.get(region["region_id"], []))
    if "indices" in region:
        return [int(i) for i in region["indices"] if 0 <= int(i) < n]
    if "normal_axis" in region and wp.normals:
        axis = np.array(region["normal_axis"], dtype=float)
        axis /= max(np.linalg.norm(axis), 1e-9)
        min_dot = float(region.get("min_dot", 0.7))
        return [
            i for i, nm in enumerate(wp.normals)
            if np.dot(np.array(nm, dtype=float), axis) >= min_dot
        ]
    return list(range(n))
