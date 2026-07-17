"""从站台示教点标定每条打磨带的接触面几何。

物理模型：机器人夹着工件去蹭固定砂带（tooldata.robhold=TRUE / wobjdata.robhold=FALSE）。
磨削时工件表面点被压在砂带接触面上，示教出的 TCP 位置 p_i（在 wobj 系里）就落在砂带
接触面附近。于是：

  - 对磨削段（speeddata 名以 "Cslun" 打头，排除 v100~v500 的趋近/退回空行程）的 TCP
    位置做平面拟合 → 平面法向 n（砂带接触面朝向）、形心 C（代表接触点）。
  - 平面内主轴 u → 砂带走向候选（符号与确切轴向需上站台核实）。
  - 点到平面残差小，才说明「接触落在一个面上」这个假设成立；残差大说明该段混了
    弧形包裹接触或空行程，标定不可信。

用法（在装好 RobotStudio 或有 modx 备份的机器上重跑）：
    python -m grinding_mcp.calibrate --progmod "E:/.../RAPID/TASK1/PROGMOD"

输出每个 (tool, wobj) 组合的接触几何 + 残差。挑残差小的填进 belts.yaml 的 contact 段。

注意：示教点常在槽位 wobj（如 Lun1_slt）里，而 belts.yaml 的 wobj 可能是 lun1——
两者差一个偏置帧，跨系使用前必须换算。本模块只在原 wobj 系里给结果，不做跨系换算。
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass

import numpy as np

_MOVE_RE = re.compile(r"Move[JLC]\s*\[\[([-\d.eE+, ]+)\],\s*\[([-\d.eE+, ]+)\]", re.I)
_TAIL_RE = re.compile(
    r",\s*([A-Za-z_]\w*)\s*,\s*\w+\s*,\s*([A-Za-z_]\w*)\s*\\WObj\s*:=\s*([A-Za-z_]\w*)",
    re.I,
)


@dataclass
class ContactFit:
    tool: str
    wobj: str
    n_points: int
    normal: tuple[float, float, float]   # 接触面法向（wobj 系，朝工件侧）
    point: tuple[float, float, float]    # 代表接触点（形心）
    direction: tuple[float, float, float]  # 面内长轴，砂带走向候选
    span_long_mm: float
    span_wide_mm: float
    resid_rms_mm: float
    resid_max_mm: float


def _quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def parse_moves(path: str) -> list[tuple[str, str, str, np.ndarray, np.ndarray]]:
    """→ [(speed, tool, wobj, pos(3,), quat(4,)), ...]"""
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = _MOVE_RE.search(line)
            t = _TAIL_RE.search(line)
            if not m or not t:
                continue
            pos = np.array([float(v) for v in m.group(1).split(",")])
            quat = np.array([float(v) for v in m.group(2).split(",")])
            if len(pos) == 3 and len(quat) == 4:
                out.append((t.group(1), t.group(2), t.group(3), pos, quat))
    return out


def fit_contact_plane(
    recs: list[tuple[str, str, str, np.ndarray, np.ndarray]]
) -> ContactFit | None:
    """对一组同 (tool,wobj) 的磨削点拟合接触面。点数不足返回 None。"""
    if len(recs) < 6:
        return None
    P = np.array([r[3] for r in recs])
    C = P.mean(axis=0)
    _, S, Vt = np.linalg.svd(P - C, full_matrices=False)
    n = Vt[2]                      # 最小奇异方向 = 平面法向
    u = Vt[0]                      # 面内长轴
    # 让法向朝工具侧（磨削段平均工具 Z 的反向≈接触面朝工件方向）
    tz = np.mean([_quat_to_mat(r[4])[:, 2] for r in recs], axis=0)
    if np.dot(n, -tz) < 0:
        n = -n
    resid = (P - C) @ n
    k = np.sqrt(len(recs))
    return ContactFit(
        tool=recs[0][1], wobj=recs[0][2], n_points=len(recs),
        normal=tuple(round(float(v), 4) for v in n),
        point=tuple(round(float(v), 2) for v in C),
        direction=tuple(round(float(v), 4) for v in u),
        span_long_mm=round(float(S[0] / k), 1),
        span_wide_mm=round(float(S[1] / k), 1),
        resid_rms_mm=round(float(np.sqrt((resid ** 2).mean())), 2),
        resid_max_mm=round(float(np.abs(resid).max()), 2),
    )


def calibrate(progmod_dir: str, files: list[str] | None = None) -> list[ContactFit]:
    """扫 PROGMOD 目录下的示教模块，按 (tool,wobj) 分组拟合接触面，只用磨削段。"""
    files = files or ["Sltcs.modx", "Cfx_cs.modx"]
    groups: dict[tuple[str, str], list] = {}
    for fn in files:
        for r in parse_moves(f"{progmod_dir}/{fn}"):
            if not r[0].lower().startswith("cslun"):   # 只保留磨削速度段
                continue
            groups.setdefault((r[1], r[2]), []).append(r)
    fits = [fit_contact_plane(g) for g in groups.values()]
    return [f for f in fits if f is not None]


def main() -> None:
    ap = argparse.ArgumentParser(description="从示教点标定砂带接触几何")
    ap.add_argument("--progmod", required=True, help="RAPID/TASK1/PROGMOD 目录")
    ap.add_argument("--files", nargs="*", help="示教模块文件名（默认 Sltcs.modx Cfx_cs.modx）")
    args = ap.parse_args()

    for fit in calibrate(args.progmod, args.files):
        conf = "可信" if fit.resid_rms_mm < 8 else "存疑(残差偏大)"
        print(f"\n# tool={fit.tool} wobj={fit.wobj} 点数={fit.n_points} 残差RMS={fit.resid_rms_mm}mm [{conf}]")
        print("contact:")
        print(f"  point: [{fit.point[0]}, {fit.point[1]}, {fit.point[2]}]")
        print(f"  normal: [{fit.normal[0]}, {fit.normal[1]}, {fit.normal[2]}]")
        print(f"  direction: [{fit.direction[0]}, {fit.direction[1]}, {fit.direction[2]}]")
        print(f"  span_mm: [{fit.span_long_mm}, {fit.span_wide_mm}]")
        print(f"  resid_rms_mm: {fit.resid_rms_mm}")


if __name__ == "__main__":
    main()
