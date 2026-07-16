"""RobotStudio Add-in 碰撞桥客户端。

几何碰撞（机器人/工件 vs 砂带机/机床外壳）是**站台级**能力，RWS 拿不到——只有
进程内的 .NET Add-in 能访问 RobotStudio 的 CollisionSet。方案是：Add-in 在
RobotStudio 里开一个本地 HTTP 端口，这里的客户端去问它。

⚠ Add-in 侧关键坑：RobotStudio SDK 非线程安全。后台 HTTP 线程收到请求后，所有 SDK
调用必须调度回界面线程，并包在 Project.UndoContext 里，否则随机崩溃或静默出错。

未连接时（config.collision.connected = False）走桩模式：返回「无碰撞」，让闭环跑通。
注意站台勘察发现 cs1111 里当前**没有定义 CollisionSet**——真启用碰撞前需先在 Add-in
里为「机器人+工件」对「砂带机+外壳」建碰撞集。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import httpx

from ..config import CollisionConfig
from ..types import RobTarget


@dataclass
class CollisionReport:
    collisions: list[dict] = field(default_factory=list)   # {seg, pair, depth}
    stub: bool = False


class CollisionClient:
    def __init__(self, cfg: CollisionConfig) -> None:
        self.cfg = cfg

    async def check_path(self, targets: list[RobTarget]) -> CollisionReport:
        if not self.cfg.connected:
            return CollisionReport(collisions=[], stub=True)

        # --- 真实：请求 Add-in 沿路径逐点判碰 ---
        payload = {
            "targets": [
                {"trans": list(t.trans), "rot": list(t.rot), "index": t.index}
                for t in targets
            ]
        }
        async with httpx.AsyncClient(base_url=self.cfg.base_url, timeout=120.0) as client:
            resp = await client.post("/collision/check_path", json=payload)
            resp.raise_for_status()
            data = resp.json()
        return CollisionReport(collisions=data.get("collisions", []), stub=False)
