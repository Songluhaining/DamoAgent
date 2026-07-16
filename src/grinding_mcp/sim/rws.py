"""RWS 2.0 客户端（RobotWare 7.x）。

未连真控制器时（config.rws.connected = False）走**桩模式**：返回结构正确的假数据，
让整个仿真闭环在没装 RobotStudio 的机器上也能跑通。换机器后置 connected=True。

⚠ 下列端点是 RWS 2.0 的常见形态，但**必须**对着你的实际控制器核实——RW7 虚拟控制器
的端口不一定是 443，登录/会话/mastership 流程各版本有差异。别照搬网上 RWS 1.0（:80）的教程。
"""

from __future__ import annotations

from dataclasses import dataclass

import httpx

from ..config import RwsConfig, RobotConfig

# RWS 2.0 端点（待实测核实）
EP_EXEC = "/rw/rapid/execution"                       # GET 执行状态 / POST start|stop
EP_LOADMOD = "/rw/rapid/tasks/{task}/loadmod"         # 加载模块文件
EP_MODULES = "/rw/rapid/tasks/{task}/modules"         # 列模块
EP_JOINT = "/rw/motionsystem/mechunits/{unit}/jointtarget"
EP_ELOG = "/rw/elog"                                  # 事件日志（报警）
EP_MASTERSHIP = "/rw/mastership"                      # 独占控制权


@dataclass
class ExecReport:
    """一次执行的运动学回执。"""
    ran: bool
    cycle_time_s: float
    joint_path: list[list[float]]
    alarms: list[dict]
    singularities: list[int]
    joint_limit_violations: list[int]
    stub: bool


class RwsClient:
    def __init__(self, cfg: RwsConfig, robot: RobotConfig) -> None:
        self.cfg = cfg
        self.robot = robot

    # --- 桩数据：未连接时用 ---
    def _stub_report(self, n_targets: int) -> ExecReport:
        # 假装全部可达、无报警，节拍按点数粗估。让上层逻辑能走完。
        return ExecReport(
            ran=True,
            cycle_time_s=round(n_targets * 0.35, 2),
            joint_path=[[0.0] * 6 for _ in range(n_targets)],
            alarms=[],
            singularities=[],
            joint_limit_violations=[],
            stub=True,
        )

    async def load_and_run(self, module_text: str, module_path: str, n_targets: int) -> ExecReport:
        """加载生成的模块并执行一遍，返回运动学回执。

        桩模式直接返回假回执；真实模式按下方 TODO 走 RWS 时序。
        """
        if not self.cfg.connected:
            return self._stub_report(n_targets)

        # --- 真实 RWS 2.0 时序（换机器后实现并核实端点） ---
        # 1. 建立会话 + 拿 mastership
        # 2. PUT/上传 module_path 到控制器，POST EP_LOADMOD 加载
        # 3. PP-to-routine 指向 damo_routine，POST EP_EXEC start（cycle=once）
        # 4. 轮询 EP_EXEC 直到 stopped；期间/结束读 EP_JOINT 采关节
        # 5. GET EP_ELOG 收报警；解析奇异/限位类事件码
        # 6. 释放 mastership
        auth = httpx.DigestAuth(self.cfg.username, self.cfg.password)
        async with httpx.AsyncClient(
            base_url=self.cfg.base_url, auth=auth, verify=self.cfg.verify_tls, timeout=60.0
        ) as client:
            raise NotImplementedError(
                "RWS 2.0 真实执行时序待实现——换到装好 RobotStudio 的机器后，"
                "对照实际控制器端点填充此处，并把 config.rws.connected 置为 True。"
            )
