"""grinding-mcp —— IRB2600 打磨规划与 RobotStudio 仿真的 MCP 服务端。

分层：
  task/     解析规格、编排工作流（承接智能体的结构化决策）
  solver/   接触点 → 姿态优化（冗余角）→ 去除预测（Preston）。可插拔，先给 baseline。
  sim/      RWS 2.0（可达/报警/关节/节拍）+ Add-in（碰撞）+ RAPID 生成
  evaluate  双评价器：运动学评价 + 工艺评价
  ledger    server 侧状态库；robtarget/关节数组用 ID 引用，绝不进对话上下文
"""

__version__ = "0.1.0"
