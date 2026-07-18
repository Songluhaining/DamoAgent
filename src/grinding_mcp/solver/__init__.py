"""求解层：第二步（细化）的可插拔接缝——子任务 → 轨迹 + 每个打磨点。

上层只依赖 base.TargetSolver 抽象接口；baseline.BaselineTargetSolver 是能跑通闭环的
占位实现（最近邻排序 + placement 摆位 + Preston 反解，冗余角 φ=0）；你后面设计的冗余角
优化算法（FRIK / 分层动态规划 / PyRoki）实现同一个 solve 接口，替换即可，上层一行不改。

placement（摆位几何）与 removal（Preston 去除模型）是可复用内核，任何 TargetSolver 都能调。
"""

from .base import TargetSolution, TargetSolver
from .baseline import BaselineTargetSolver

__all__ = ["TargetSolver", "TargetSolution", "BaselineTargetSolver"]
