"""合成工件：不依赖真实数据，跑通闭环 + 当回归基准用。

生成一个「圆角块」（rounded box）的表面点云，法向是解析求出的精确值——这点比真实
扫描点云强，扫描点云的法向要靠 PCA 估计、有噪声。等你有真数据了，换成 XyzFileSource
或将来的 PlySource，上层不用改。

圆角块的好处：既有平面（考验均匀去除），又有圆角过渡（考验法向连续变化、姿态跟随），
是打磨规划的一个有代表性的最小测例。
"""

from __future__ import annotations

import numpy as np

from ..ledger import new_id
from ..types import Workpiece
from .base import WorkpieceSource


class SyntheticSource(WorkpieceSource):
    """圆角块表面采样。默认只采「上表面 + 四条圆角棱」——打磨最常见的目标面。

    参数
      size      长宽高 (Lx, Ly, Lz)，mm
      radius    圆角半径，mm
      n_along   沿长边采样数
      n_across  沿宽边采样数
      faces     采哪些面，默认 ("top",)；可加 "fillet" 采圆角棱
    """

    def __init__(
        self,
        name: str = "synthetic_block",
        size: tuple[float, float, float] = (120.0, 60.0, 40.0),
        radius: float = 8.0,
        n_along: int = 24,
        n_across: int = 12,
        faces: tuple[str, ...] = ("top", "fillet"),
    ) -> None:
        self.name = name
        self.size = size
        self.radius = radius
        self.n_along = n_along
        self.n_across = n_across
        self.faces = faces

    def load(self) -> Workpiece:
        lx, ly, lz = self.size
        r = self.radius
        pts: list[tuple[float, float, float]] = []
        nrm: list[tuple[float, float, float]] = []

        # 上平面（z = lz/2），法向 +Z；避开圆角区（|x|<lx/2-r, |y|<ly/2-r）
        if "top" in self.faces:
            xs = np.linspace(-(lx / 2 - r), lx / 2 - r, self.n_along)
            ys = np.linspace(-(ly / 2 - r), ly / 2 - r, self.n_across)
            for x in xs:
                for y in ys:
                    pts.append((float(x), float(y), lz / 2))
                    nrm.append((0.0, 0.0, 1.0))

        # 沿长边（x 方向）的两条顶部圆角棱：绕棱轴扫 0~90°，法向随角度精确变化
        if "fillet" in self.faces:
            xs = np.linspace(-(lx / 2 - r), lx / 2 - r, self.n_along)
            angs = np.linspace(0.0, np.pi / 2, 6)  # 0=朝+Y顶, 90=朝+Z顶
            for sign in (+1, -1):                  # +Y 边与 -Y 边
                cy = sign * (ly / 2 - r)
                cz = lz / 2 - r
                for x in xs:
                    for a in angs:
                        ny, nz = sign * np.cos(a), np.sin(a)
                        pts.append((float(x), float(cy + r * ny), float(cz + r * nz)))
                        nrm.append((0.0, float(ny), float(nz)))

        return Workpiece(
            workpiece_id=new_id("wp"),
            name=self.name,
            points=pts,
            normals=nrm,
            frame="workpiece",
            source=f"synthetic rounded_box size={self.size} r={self.radius}",
        )
