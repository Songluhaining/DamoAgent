"""工件表面读取的可插拔接口。

真实点云的形态还没定（三维扫描仪 PLY/PCD、CAD 采样 STL、或纯 xyz 文本），
坐标系也没定。所以这里只定义一个统一接口 WorkpieceSource，具体读取各自实现：

  - SyntheticSource  合成工件（圆角块，自带精确法向），跑通闭环 + 回归基准用。
  - XyzFileSource    纯文本 xyz[+法向] 读取占位，等真数据来补齐格式。
  - （将来）PlySource / StlSource —— 需要时按同一接口加，上层一行不改。

上层（求解层、MCP 工具）只依赖 WorkpieceSource.load()，不关心数据从哪来。
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..types import Workpiece


class WorkpieceSource(ABC):
    """工件表面来源。实现 load() 返回一个 Workpiece（点 + 法向 + 坐标系标记）。"""

    @abstractmethod
    def load(self) -> Workpiece:
        """读出工件表面点云与逐点外法向。法向必须是单位向量、朝工件外侧。"""
        raise NotImplementedError
