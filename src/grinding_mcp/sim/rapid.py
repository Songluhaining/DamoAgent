"""robtarget 数组 → RAPID 模块文本。

生成的模块定义 damo_routine，填进站台主循环里那个空壳 <SMT>。
工具/工件/速度/转弯区取自 belts.yaml 里每条带的实际站台定义。
"""

from __future__ import annotations

from ..config import BeltParams, RobotConfig
from ..types import RobTarget


def _fmt_num(x: float) -> str:
    return f"{x:.4f}".rstrip("0").rstrip(".") if x != 0 else "0"


def _robtarget_literal(t: RobTarget) -> str:
    trans = ",".join(_fmt_num(v) for v in t.trans)
    rot = ",".join(_fmt_num(v) for v in t.rot)
    conf = ",".join(str(int(v)) for v in t.robconf)
    extax = ",".join("9E9" if v >= 9e8 else _fmt_num(v) for v in t.extax)
    return f"[[{trans}],[{rot}],[{conf}],[{extax}]]"


def build_module(
    targets: list[RobTarget],
    belt: BeltParams,
    robot: RobotConfig,
    header_comment: str = "",
) -> str:
    """生成 RAPID 模块文本。第一个点用 MoveJ 逼近，其余 MoveL 走磨削路径。"""
    lines: list[str] = []
    lines.append(f"MODULE {robot.target_module}")
    if header_comment:
        for c in header_comment.splitlines():
            lines.append(f"  ! {c}")
    lines.append(f"  PROC {robot.target_proc}()")

    for i, t in enumerate(targets):
        move = "MoveJ" if i == 0 else "MoveL"
        speed = belt.speeddata if i > 0 else "v200"
        zone = belt.zonedata if i > 0 else "z10"
        lines.append(
            f"    {move} {_robtarget_literal(t)}, {speed}, {zone}, "
            f"{belt.tool}\\WObj:={belt.wobj};"
        )

    lines.append("  ENDPROC")
    lines.append("ENDMODULE")
    return "\n".join(lines) + "\n"


def write_module(text: str, path: str) -> str:
    with open(path, "w", encoding="utf-8") as f:   # ruff PLW1514：显式 utf-8
        f.write(text)
    return path
