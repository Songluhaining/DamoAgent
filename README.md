# DamoAgent · grinding-mcp

以 [hermes-agent](../赛宝实习/hermes-agent) 为基座，让大模型（LLM）智能体自动完成
**ABB IRB2600 工件打磨**的规划与仿真闭环：给定工件与打磨要求，智能体解析任务、编排
工作流、为每个子步骤生成最优移动点（robtarget）、经 MCP 驱动 RobotStudio 仿真、
拿回结果分析，未达标则迭代优化再仿真——ReAct 循环直至满足需求。

本仓库是那条链路里的 **MCP 服务端**（一个独立的工具服务），hermes 通过它原生的 MCP
客户端连接。hermes 核心一行不用改。

---

## 为什么这样分工

一句话：**离散工艺决策交给 LLM，数值计算交给求解器。**

| LLM（hermes 智能体侧） | 求解器（本服务侧） |
|---|---|
| 解析打磨要求 → 结构化规格 | 工件几何 → 接触点 + 法向 |
| 选哪几条带、粗→精排序、走几遍 | 冗余角优化 → 每点最优姿态 → robtarget |
| 诊断失败、决定下轮改什么 | Preston 去除量预测 |
| 编排整个 ReAct 流程 | IK / 可达性 / 碰撞判定 |

robtarget 是 6 个浮点数，差 0.1mm 工件就报废；而 LLM 最擅长生成「看似合理、实则错误」
的数值。所以**位姿数值绝不让 LLM 生成**——这条守住，系统才可靠。调研过的最新方法
（FRIK / 分层动态规划 / DecompGrind）没有一个用 LLM 算位姿，印证了这一点。

## 两个必须并存的评价维度

- **运动学**（RobotStudio 能答）：可达？碰撞？奇异？超限？节拍？
- **工艺**（RobotStudio 答不了）：磨够了吗？在允差内吗？哪块欠/过磨？

RobotStudio 是运动学仿真器，不算材料去除。若只看运动学，闭环会「仿真通过但工件没磨对」
地空转成功——最坏的失败模式。所以本服务内置 **Preston 去除模型**作为工艺评价器，
`grinding_evaluate` 两维都过才算 `passed`。

## 三步迭代闭环

系统按三个边界清晰、各自可迭代的阶段组织——与打磨规划的自然分解一一对应：

```
第一步 任务编排 (decompose)      工件+规格 → 子任务（去哪磨 + 用哪条带 + 磨掉多少），不含点
   │   ← 智能体审子任务，觉得合理再往下（离散决策归 LLM）
第二步 细化 (generate_targets)   每个子任务 → 移动轨迹 + 每个打磨点(robtarget) + 工艺参数
   │   ← 数值全由求解器算（不让 LLM 生成位姿）★可插拔接缝
第三步 仿真评价 (simulate/evaluate)  robtarget → RobotStudio 仿真 → 运动学+工艺双评价
   │   ← 未达标：回第二步调点，或回第一步重分解
   └── ReAct 迭代直至达标
```

`grinding_plan` 是「一键跑完第一、二步」的便捷入口；要在两步间审核或让子任务用不同区域，
就分开调 `grinding_decompose` / `grinding_generate_targets`。

## 架构分层

```
Hermes Agent（ReAct 循环，已有，零改动）
  │  MCP over stdio
  ▼
grinding-mcp（本仓库）
  ├─ task/
  │    ├─ decompose.py  第一步：分子任务（带序/去除量分配，可被智能体覆盖）
  │    ├─ generate.py   第二步：调求解器把子任务细化成轨迹+点
  │    ├─ planner.py    便捷编排器：一键串起第一、二步
  │    └─ workflow.py   register_spec + 手工 add_step（第一步的手动入口）
  ├─ solver/     ★第二步可插拔接缝：子任务 → 轨迹 + 每个打磨点
  │    ├─ base.py       TargetSolver 抽象接口 + TargetSolution
  │    ├─ baseline.py   BaselineTargetSolver 占位（最近邻排序+摆位+反解，φ=0）
  │    ├─ placement.py  摆位几何（工件点→砂带接触点，方向按站台真实约定）
  │    └─ removal.py    Preston 去除模型（正向 + 反解，系数待拟合）
  ├─ workpiece/  可插拔点云读取（合成圆角块 / xyz）+ 区域选点
  ├─ calibrate.py 从示教点标定砂带接触几何
  ├─ sim/        RWS 2.0（可达/报警/关节/节拍）+ Add-in（碰撞）+ RAPID 生成
  ├─ evaluate.py 双评价器（单步）+ 方案级聚合评价
  └─ ledger.py   server 侧状态库；大数组用 ID 引用，不进对话上下文
```

## MCP 工具面（按三步分组）

| 工具 | 步骤 | 作用 |
|---|---|---|
| `grinding_station_info` | — | 机器人/4 条带参数/连接状态 |
| `grinding_register_spec` | — | 登记打磨规格（智能体产出） |
| `grinding_load_workpiece` | — | 加载工件点云（合成圆角块 / xyz 文本），回摘要+ID |
| `grinding_decompose` | ① | **规格→子任务**：带序+区域+去除量分配，不含点 |
| `grinding_add_step` | ① | 手工加一个子任务（与 decompose 并列的手动入口） |
| `grinding_list_workflow` | ① | 列出子任务，标出是否已细化 |
| `grinding_generate_targets` | ② | **子任务→轨迹+每个打磨点**，写回工艺参数 |
| `grinding_inspect_targets` | ② | 查看某段 robtarget（调试，分片） |
| `grinding_plan` | ①+② | 便捷：一键跑完第一、二步出完整方案 |
| `grinding_simulate` | ③ | 送虚拟控制器执行，回运动学+碰撞 |
| `grinding_evaluate` | ③ | 双评价器（单子任务），逐项结论+改进提示 |
| `grinding_evaluate_plan` | ③ | 方案级聚合：整条带序累计去除是否达标 |
| `grinding_write_rapid` | — | 导出 RAPID 模块（手动导入兜底） |

所有工具只回**摘要 + ID**，robtarget/关节数组留在 server 侧——既防上下文爆炸，也守
hermes「对话前缀缓存不可变」的硬性不变量。

## 现状（v0.3 骨架）

- ✅ **三步显式分离**：第一步任务编排（`decompose`，出子任务不含点）→ 第二步细化
  （`generate_targets`，出轨迹+点）→ 第三步仿真评价（`simulate`/`evaluate`/`evaluate_plan`）。
  每步边界清晰、各自可迭代；`grinding_plan` 是一键跑完前两步的便捷入口
- ✅ **工件+需求+带参数 → 打磨方案**：排带序、分配去除量、Preston 反解每带压深/遍数、
  逐点摆位出 robtarget。反解与正向预测**精确自洽**（误差 0%）
- ✅ **接触几何已从 490 个示教点标定**（`calibrate.py` 磨削段平面拟合，残差 2~12mm），
  写进 `belts.yaml` 的 `contact` 段；lun1 侧法向一致指向 -Y，可复算
- ✅ **摆位方向修正**（`solver/placement.py`）：按站台真实的「夹工件蹭砂带」约定——
  把工件点摆到砂带接触点、法向对齐、冗余角显式暴露。能复现示教签名（平面姿态不变、
  圆角姿态跟随法向扫）。旧 baseline 的反向约定已弃用
- ✅ **可插拔点云读取层**（`workpiece/`）：合成圆角块（自带精确法向）跑通闭环，
  xyz 文本读取占位；真实 PLY/PCD/STL 按同一接口加，上层不改
- ✅ 全链路可**空跑**：不装 RobotStudio，桩模式返回结构正确的假数据，ReAct 闭环连得通
- ✅ 风险如实带出：占位系数、暂定几何、坐标系未核实都进 `warnings`，绝不吞掉
- ⬜ RWS 2.0 真实执行时序 —— 待换到装好 RobotStudio 的机器后实现并核实端点
- ⬜ RobotStudio Add-in 碰撞桥（C#）—— 站台层碰撞的必需件
- ⬜ 冗余角姿态优化算法 —— 现固定 φ=0，替换为搜索最优 φ（参考 FRIK / 分层 DP / PyRoki）
- ⬜ Preston 系数拟合 —— `solver/removal.py:fit_from_history`，用你的 4 条带历史数据
- ⬜ 坐标系换算 —— 示教点在 `Lun*_slt` 槽位系，与带 wobj 差偏置帧，待站台核实
- ⬜ 装夹位姿标定 —— 摆位现假设 TCP 在工件原点（p_tcp=0），真实偏置待标定

## 站台事实（来自 E:\Myself\cs1111 勘察）

- 机器人 **IRB2600-20/1.65 Type C**，RobotWare **7.20.0** → 走 **RWS 2.0**（https，非 :80）
- **纯位置控制**，未装 Force Control；砂带磨损靠 `make_up_for_lun1` 几何补偿
- 机器人**夹持工件**去磨固定砂带：`tooldata.robhold=TRUE` + `wobjdata.robhold=FALSE`
- 主循环里 **`damo_routine` 是空壳 `<SMT>`** —— 正是规划系统要填的插入点
- `Sltcs.modx`/`Cfx_cs.modx` 有 ~490 个手工示教点，未接主循环 —— 可作对标基准
- 站台**未建 CollisionSet** —— 启用碰撞前需在 Add-in 里补建

## 快速开始

```bash
# 安装（骨架不需要 solver 额外依赖也能空跑）
pip install -e .
pip install -e ".[dev]"      # 跑测试

# 跑冒烟测试（桩模式，无需 RobotStudio）
pytest tests/ -v

# 以 stdio 方式启动 MCP 服务端
grinding-mcp
```

## 接入 hermes

在 `~/.hermes/config.yaml` 加：

```yaml
mcp_servers:
  grinding:
    command: "grinding-mcp"
    # 或： command: "python", args: ["-m", "grinding_mcp.server"]
    env:
      GRINDING_MCP_CONFIG: "E:/Myself/DamoAgent/config/belts.yaml"
```

重启 hermes 会话即生效（MCP 工具动态发现，无需改 hermes 核心）。工具名会带前缀
`mcp_grinding_*`。
