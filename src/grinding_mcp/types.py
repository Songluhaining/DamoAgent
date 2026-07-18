"""贯穿各层的数据类型。

分工原则（决定字段归属）：
  智能体（LLM）产出 GrindSpec / GrindStep —— 离散工艺决策
  求解层产出 ContactPoint / RobTarget / RemovalField —— 数值，不由 LLM 生成
  仿真层产出 SimResult；评价层产出 Evaluation
"""

from __future__ import annotations

from dataclasses import dataclass, field


# --- 智能体产出（工艺决策） ---------------------------------------------

@dataclass
class GrindSpec:
    """打磨要求解析后的结构化规格。由智能体从自然语言产出。"""
    spec_id: str
    workpiece: str
    belts: list[str]                 # 要用的带，如 ["belt1", "belt3"]
    grit_sequence: list[int]         # 粒度序列，粗→精
    target_removal_mm: float         # 目标去除量
    tolerance_mm: float              # 允差
    surface_ra_um: float | None      # 目标表面粗糙度，可空
    raw_text: str = ""               # 原始要求，留痕


@dataclass
class GrindStep:
    """一个打磨子任务（去哪磨=region + 用哪条带 + 磨掉多少=target_removal）。

    这是**第一步任务编排**的产物：只定「去哪、磨什么」，不含打磨点。
    contact_depth_mm / passes / targets_id 是**第二步细化**才填的输出（初始为空/0）。
    分工：region/belt/target_removal 是离散决策（智能体产出）；压深/遍数/点是数值（求解器算）。
    """
    step_id: str
    spec_id: str
    belt_id: str
    order: int
    region: dict                     # 区域描述：法向筛选/包围盒/显式下标
    workpiece_id: str = ""           # 关联的工件点云（第二步据此取点）
    target_removal_mm: float = 0.0   # 分给这个子任务的目标去除量（第一步意图）
    feed_mm_s: float = 20.0
    dwell_s: float = 0.0
    # --- 以下由第二步（细化）填 ---
    contact_depth_mm: float = 0.0    # 反解出的压入深度（纯位置控制下是去除主控量）
    passes: int = 0                  # 反解出的遍数
    targets_id: str = ""             # 生成的 robtarget 集合 ID（回溯用）


# --- 输入：工件表面 ------------------------------------------------------

@dataclass
class Workpiece:
    """工件表面点云 + 逐点法向，定义在工件自身坐标系里。

    点云读取层（workpiece/）产出。points/normals 是等长数组，一一对应。
    frame 记录点云所在坐标系（工件系/机器人系/扫描仪系），后续摆位要据此换算。
    大数组留 server 侧，用 workpiece_id 引用，不进对话上下文。
    """
    workpiece_id: str
    name: str
    points: list[tuple[float, float, float]]      # mm，工件系
    normals: list[tuple[float, float, float]]     # 单位向量，外法向
    frame: str = "workpiece"                       # workpiece | robot | scanner
    source: str = ""                               # 来源说明（文件/合成）
    # 几何摘要（inspect_workpiece）切出的命名区域：region_id → 点下标。
    # 索引留这里（server 侧），子任务用 {"region_id": ...} 引用，不进对话上下文。
    regions: dict[str, list[int]] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.points)


# --- 求解层产出（数值，非 LLM） ------------------------------------------

@dataclass
class ContactPoint:
    """接触点：定义在带的 wobj 坐标系里。pos + 表面法向。"""
    index: int
    pos: tuple[float, float, float]
    normal: tuple[float, float, float]


@dataclass
class RobTarget:
    """一个 ABB robtarget，附带优化信息。"""
    index: int
    trans: tuple[float, float, float]
    rot: tuple[float, float, float, float]          # 四元数 [q1,q2,q3,q4]
    robconf: tuple[int, int, int, int] = (0, 0, 0, 0)
    extax: tuple[float, ...] = (9e9, 9e9, 9e9, 9e9, 9e9, 9e9)
    redundancy_angle_deg: float = 0.0               # 优化出的绕接触法向的冗余角
    reachable: bool = True                          # 求解层的可达性初判（仿真层再复核）


@dataclass
class RemovalField:
    """一条路径的去除量分布（Preston 预测）。"""
    per_point_mm: list[float] = field(default_factory=list)
    mean_mm: float = 0.0
    min_mm: float = 0.0
    max_mm: float = 0.0


@dataclass
class TargetSet:
    """一个子步骤求解后的完整结果。存 server 侧，用 targets_id 引用。"""
    targets_id: str
    step_id: str
    belt_id: str
    targets: list[RobTarget]         # 完整数组留这里，不进上下文
    removal: RemovalField
    posture_cost: float = 0.0        # 姿态代价（越小越好，如刚度倒数）


# --- 规划层产出（方案） --------------------------------------------------

@dataclass
class BeltPlan:
    """方案里针对一条带的一段：分到多少去除量、推荐什么工艺参数、生成了哪些点。"""
    order: int
    belt_id: str
    grit: int
    apportioned_removal_mm: float    # 分给这条带的去除量
    contact_depth_mm: float          # 反解出的推荐压深
    passes: int                      # 反解出的推荐遍数
    feed_mm_s: float
    predicted_removal_mm: float      # 用推荐参数正向预测的实际去除（应≈分配值）
    targets_id: str                  # 该段生成的 robtarget 集合（存 ledger）
    point_count: int
    reachable_count: int


@dataclass
class GrindPlan:
    """一份完整打磨方案：工件 + 需求 → 带序 + 逐带工艺参数 + robtarget。

    这是「给定打磨方案」的产物。数值内核（摆位、Preston 反解）算，带序与分配是可被
    智能体覆盖的启发式默认。大数组（robtarget）留 ledger，本对象只带摘要。
    """
    plan_id: str
    spec_id: str
    workpiece_id: str
    belt_plans: list[BeltPlan] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)   # 占位系数/暂定几何等风险提示


# --- 仿真层产出 ----------------------------------------------------------

@dataclass
class SimResult:
    """一次仿真的原始结果。存 server 侧，用 sim_id 引用。"""
    sim_id: str
    targets_id: str
    total_count: int
    reachable_count: int
    collisions: list[dict] = field(default_factory=list)       # {seg, pair}
    singularities: list[int] = field(default_factory=list)     # 出问题的点 index
    joint_limit_violations: list[int] = field(default_factory=list)
    cycle_time_s: float = 0.0
    alarms: list[dict] = field(default_factory=list)           # 控制器事件日志
    joint_path: list[list[float]] = field(default_factory=list)  # 关节序列，留 server 侧
    stub: bool = False               # True 表示桩数据（未连真控制器）


# --- 评价层产出 ----------------------------------------------------------

@dataclass
class Evaluation:
    """双评价器结果。供智能体做 ReAct 下一步决策。"""
    passed: bool
    kinematic: dict                  # 运动学：可达/碰撞/奇异/限位 逐项判定
    process: dict                    # 工艺：去除量 vs 目标、粗糙度
    residual_regions: list[dict] = field(default_factory=list)   # 欠磨/过磨区域
    suggestions: list[str] = field(default_factory=list)         # 结构化改进方向提示
