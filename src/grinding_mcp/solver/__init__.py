"""求解层：接触点 → 姿态优化 → 去除预测。

这是整个系统的**可插拔接缝**。上层（server/仿真/评价）只依赖 base.Solver 抽象接口；
baseline.BaselineSolver 是能跑通闭环的占位实现；你后面设计的冗余角优化算法
（FRIK / 分层动态规划 / PyRoki）实现同一个接口，替换即可，上层一行不改。
"""

from .base import Solver
from .baseline import BaselineSolver

__all__ = ["Solver", "BaselineSolver"]
