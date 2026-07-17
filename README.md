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

## 架构分层

```
Hermes Agent（ReAct 循环，已有，零改动）
  │  MCP over stdio
  ▼
grinding-mcp（本仓库）
  ├─ task/       解析/编排 —— 承接智能体的结构化决策
  ├─ solver/     接触点 → 姿态优化(冗余角) → 去除预测   ★可插拔接缝
  │    ├─ base.py       Solver 抽象接口
  │    ├─ baseline.py   占位实现（跑通闭环用，非最优）
  │    └─ removal.py    Preston 去除模型（系数待拟合）
  ├─ sim/        RWS 2.0（可达/报警/关节/节拍）+ Add-in（碰撞）+ RAPID 生成
  ├─ evaluate.py 双评价器
  └─ ledger.py   server 侧状态库；大数组用 ID 引用，不进对话上下文
```

## MCP 工具面

| 工具 | 作用 |
|---|---|
| `grinding_station_info` | 机器人/4 条带参数/连接状态 |
| `grinding_register_spec` | 登记打磨规格（智能体产出） |
| `grinding_load_workpiece` | 加载工件点云（合成圆角块 / xyz 文本），存 server 侧回摘要+ID |
| `grinding_plan` | **工件+规格→打磨方案**：带序+逐带工艺参数+robtarget（一步式） |
| `grinding_add_step` | 添加子步骤（带/区域/工艺参数） |
| `grinding_list_workflow` | 列出工作流步骤 |
| `grinding_generate_targets` | 求解：接触点→姿态→去除，返回摘要+ID |
| `grinding_inspect_targets` | 查看某段 robtarget（调试，分片） |
| `grinding_simulate` | 送虚拟控制器执行，回运动学+碰撞 |
| `grinding_evaluate` | 双评价器，返回逐项结论+改进提示 |
| `grinding_write_rapid` | 导出 RAPID 模块（手动导入兜底） |

所有工具只回**摘要 + ID**，robtarget/关节数组留在 server 侧——既防上下文爆炸，也守
hermes「对话前缀缓存不可变」的硬性不变量。

## 现状（v0.2 骨架）

- ✅ **工件+需求+带参数 → 打磨方案** 一步式（`grinding_plan`）：排带序、分配去除量、
  Preston 反解每带压深/遍数、逐点摆位出 robtarget。反解与正向预测**精确自洽**（误差 0%）
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
