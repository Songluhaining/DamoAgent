"""求解层抽象接口。

三个方法对应打磨规划的三个数值子问题：
  contact_points   几何 → 接触点+法向（5-DOF 约束）
  optimize_posture 每点冗余角优化 → robtarget（用掉那第 6 个自由度）
  predict_removal  路径 → 去除量分布（Preston）

功能冗余原理：打磨只约束接触点位置(3) + 接触法向(2) = 5 DOF；机器人有 6 DOF，
绕接触法向那 1 转是自由的。转它不改变材料去除，但改变关节构型与笛卡尔刚度。
optimize_posture 的任务就是为每个点选这个角，主目标是**刚度最大**（抗磨削力变形），
次目标是关节平滑、避奇异、避限位、避碰撞。

注意：砂带不是旋转对称的磨盘——它有明确运行方向。所以冗余角在这里同时是姿态变量
和工艺变量（改变工件与带速夹角 → 改变 Preston 的相对滑动速度与磨纹方向）。
你的算法需要把这一耦合纳入代价函数。
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..config import BeltParams
from ..types import ContactPoint, GrindStep, RemovalField, RobTarget


class Solver(ABC):
    @abstractmethod
    def contact_points(self, step: GrindStep, belt: BeltParams) -> list[ContactPoint]:
        """由子步骤的区域描述生成接触点序列（在带的 wobj 坐标系里）。"""

    @abstractmethod
    def optimize_posture(
        self, points: list[ContactPoint], belt: BeltParams
    ) -> tuple[list[RobTarget], float]:
        """为每个接触点求最优姿态（选冗余角）。返回 (robtargets, 总姿态代价)。"""

    @abstractmethod
    def predict_removal(
        self, targets: list[RobTarget], step: GrindStep, belt: BeltParams
    ) -> RemovalField:
        """预测这条路径的去除量分布。"""
