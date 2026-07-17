"""加载 config/belts.yaml，提供带参数、机器人、RWS、碰撞桥的结构化配置。"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml


def _default_config_path() -> Path:
    # 允许用环境变量覆盖（换机器/多站台时方便）
    env = os.environ.get("GRINDING_MCP_CONFIG")
    if env:
        return Path(env)
    # 仓库内默认：<repo>/config/belts.yaml
    return Path(__file__).resolve().parents[2] / "config" / "belts.yaml"


@dataclass
class PrestonParams:
    kp: float = 1e-5
    p_exp: float = 1.0
    v_exp: float = 1.0


@dataclass
class ContactGeometry:
    """砂带接触面几何（由 calibrate.py 从示教点反推）。坐标在标定所用 wobj 系里。

    point/normal/direction 定义了「把工件表面点摆到哪、朝哪压、砂带往哪走」，
    是 placement 求解反解 robtarget 的目标。trusted=False 表示残差偏大或坐标系
    待核实，只够跑通闭环，别拿去下真控制器。
    """
    point: tuple[float, float, float]      # 代表接触点 C（mm）
    normal: tuple[float, float, float]     # 接触面法向，朝工件侧（单位向量）
    direction: tuple[float, float, float]  # 砂带走向候选（面内长轴）
    wheel_radius_mm: float = 100.0
    resid_rms_mm: float = 0.0
    calib_source: str = ""
    trusted: bool = False


@dataclass
class BeltParams:
    belt_id: str
    label: str
    tool: str
    wobj: str
    slot_wobj: str
    speeddata: str
    zonedata: str
    belt_speed_m_s: float
    contact_wheel_hardness_shore: float
    grit: int
    preston: PrestonParams
    contact: ContactGeometry | None = None


@dataclass
class RobotConfig:
    model: str = "IRB2600-20/1.65"
    type: str = "C"
    robotware: str = "7.20.0"
    task: str = "T_ROB1"
    mechunit: str = "ROB_1"
    target_proc: str = "damo_routine"
    target_module: str = "damo_generated"


@dataclass
class RwsConfig:
    base_url: str = "https://127.0.0.1:443"
    username: str = "Default User"
    password: str = "robotics"
    verify_tls: bool = False
    connected: bool = False          # False = 桩模式


@dataclass
class CollisionConfig:
    base_url: str = "http://127.0.0.1:5888"
    connected: bool = False


@dataclass
class StationConfig:
    robot: RobotConfig
    rws: RwsConfig
    collision: CollisionConfig
    belts: dict[str, BeltParams] = field(default_factory=dict)

    def belt(self, belt_id: str) -> BeltParams:
        if belt_id not in self.belts:
            raise KeyError(f"未知打磨带 '{belt_id}'，已配置：{list(self.belts)}")
        return self.belts[belt_id]


def load_config(path: str | Path | None = None) -> StationConfig:
    p = Path(path) if path else _default_config_path()
    with open(p, encoding="utf-8") as f:      # ruff PLW1514：显式 utf-8
        raw = yaml.safe_load(f)

    robot = RobotConfig(**(raw.get("robot") or {}))
    rws = RwsConfig(**(raw.get("rws") or {}))
    collision = CollisionConfig(**(raw.get("collision_addin") or {}))

    belts: dict[str, BeltParams] = {}
    for bid, b in (raw.get("belts") or {}).items():
        preston = PrestonParams(**(b.get("preston") or {}))
        contact = None
        c = b.get("contact")
        if c:
            contact = ContactGeometry(
                point=tuple(float(v) for v in c["point"]),
                normal=tuple(float(v) for v in c["normal"]),
                direction=tuple(float(v) for v in c["direction"]),
                wheel_radius_mm=float(c.get("wheel_radius_mm", 100.0)),
                resid_rms_mm=float(c.get("resid_rms_mm", 0.0)),
                calib_source=str(c.get("calib_source", "")),
                trusted=bool(c.get("trusted", False)),
            )
        belts[bid] = BeltParams(
            belt_id=bid,
            label=b.get("label", bid),
            tool=b["tool"],
            wobj=b["wobj"],
            slot_wobj=b.get("slot_wobj", b["wobj"]),
            speeddata=b.get("speeddata", "v100"),
            zonedata=b.get("zonedata", "z1"),
            belt_speed_m_s=float(b.get("belt_speed_m_s", 20.0)),
            contact_wheel_hardness_shore=float(b.get("contact_wheel_hardness_shore", 60)),
            grit=int(b.get("grit", 60)),
            preston=preston,
            contact=contact,
        )
    return StationConfig(robot=robot, rws=rws, collision=collision, belts=belts)
