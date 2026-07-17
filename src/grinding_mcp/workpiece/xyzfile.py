"""纯文本点云读取占位（xyz 或 xyz+法向，每行一点）。

真实点云格式还没定，这里先支持最通用的文本格式：每行
    x y z              —— 只有坐标，法向留空由调用方补估
    x y z nx ny nz     —— 带法向
分隔符空格或逗号皆可，'#' 开头视为注释。

真数据到位后：
  - 若是扫描仪 PLY/PCD，加 PlySource（可 lazy-import open3d），别硬塞进这里。
  - 若不带法向，法向估计（PCA + 一致定向）单独做，不混进读取。
本模块只管把文本变成 Workpiece，不做下采样/去噪/法向估计——各是各的关注点。
"""

from __future__ import annotations

from ..ledger import new_id
from ..types import Workpiece
from .base import WorkpieceSource


class XyzFileSource(WorkpieceSource):
    def __init__(self, path: str, name: str = "", frame: str = "workpiece") -> None:
        self.path = path
        self.name = name or path
        self.frame = frame

    def load(self) -> Workpiece:
        pts: list[tuple[float, float, float]] = []
        nrm: list[tuple[float, float, float]] = []
        has_normal = True
        with open(self.path, encoding="utf-8") as f:   # ruff PLW1514
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [p for p in line.replace(",", " ").split() if p]
                vals = [float(p) for p in parts]
                if len(vals) >= 6:
                    pts.append((vals[0], vals[1], vals[2]))
                    nrm.append((vals[3], vals[4], vals[5]))
                elif len(vals) >= 3:
                    pts.append((vals[0], vals[1], vals[2]))
                    has_normal = False
                # 少于 3 个数的行跳过

        if not has_normal:
            # 没带法向：留空数组，交由上层的法向估计步骤补（本模块不越权估计）
            nrm = []

        return Workpiece(
            workpiece_id=new_id("wp"),
            name=self.name,
            points=pts,
            normals=nrm,
            frame=self.frame,
            source=f"xyz file {self.path}",
        )
