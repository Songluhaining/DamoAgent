"""工件表面读取层：可插拔接口 + 合成/文本实现。

上层只依赖 WorkpieceSource.load() → Workpiece。真实点云格式（PLY/PCD/STL）到位后
按同一接口加实现，上层一行不改。
"""

from .base import WorkpieceSource
from .geometry import RegionCandidate, summarize
from .region import select_region
from .synthetic import SyntheticSource
from .xyzfile import XyzFileSource

__all__ = [
    "WorkpieceSource", "SyntheticSource", "XyzFileSource",
    "select_region", "summarize", "RegionCandidate",
]
