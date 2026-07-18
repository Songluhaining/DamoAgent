"""第二步（细化）的可插拔接口：子任务的区域点 → 轨迹 + 每个打磨点（robtarget）。

这是整个系统留给「利用现有方法或你后面设计的算法」的接缝。第一步（任务编排）已经定好
「去哪磨、用哪条带、磨掉多少」；第二步在此把它细化成具体的移动轨迹和逐点最优姿态。

TargetSolver.solve 的职责：
  排轨迹（点的访问顺序）→ 逐点摆位（用掉冗余角那第 6 自由度）→ Preston 反解压深/遍数
  → 正向预测去除量。返回 TargetSolution。

功能冗余原理：打磨只约束接触点位置(3) + 接触法向(2) = 5 DOF；机器人有 6 DOF，绕接触
法向那 1 转是自由的（冗余角 φ）。转它不改变材料去除，但改变关节构型刚度；且砂带有走向，
φ 还改变工件相对带速的夹角。baseline 取 φ=0；真优化在此搜索 φ 使刚度最大——你的算法
（FRIK / 分层动态规划 / PyRoki）实现同一个 solve 接口，替换即可，上层一行不改。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from ..config import BeltParams
from ..types import RemovalField, RobTarget


@dataclass
class TargetSolution:
    """第二步的产出：一个子任务细化后的轨迹点 + 反解出的工艺参数。"""
    targets: list[RobTarget] = field(default_factory=list)   # 按轨迹顺序
    contact_depth_mm: float = 0.0
    passes: int = 0
    removal: RemovalField = field(default_factory=RemovalField)


class TargetSolver(ABC):
    @abstractmethod
    def solve(
        self,
        *,
        points: list[tuple[float, float, float]],
        normals: list[tuple[float, float, float]],
        belt: BeltParams,
        target_removal_mm: float,
        feed_mm_s: float,
        max_passes: int,
        phi_deg: float,
        warnings: list[str],
    ) -> TargetSolution:
        """把一个子任务的区域点（工件系，含法向）细化成轨迹 + robtarget + 工艺参数。

        points/normals 一一对应，在工件系里。belt.contact 给出砂带接触几何（wobj 系）。
        达不到目标去除量等风险追加进 warnings（绝不吞掉）。
        """
