# 人机交互与强化学习架构：ShapeIntent 控制层

> 本文把"导丝仿真系统技术设计 v1.0"（人机交互 + RL）落成**与现有代码对齐的工程订正版**。
>
> **定位**：doc/08 已把导丝做成"真物理 60Hz 跟手 + 力驱动 + 扭转/J-tip 转向"（D4/D5 上线）。本文在其**之上**加一层统一控制抽象（ShapeIntent），让"医生人机操作"和"RL 策略"共用同一接口，并把已有的 RL/Gym 栈接到 Newton 力驱动引擎。
>
> **一句话**：物理已经真了（doc/08），本文让"怎么指挥它"统一起来——人和策略都下达 **ShapeIntent（形状/目标意图）**，由一个 **Controller** 解算成真实的 `push/rotate` 喂给纯物理引擎。

---

## 〇、与现有文档的关系

| 文档 | 负责 | 本文的边界 |
|---|---|---|
| doc/08 Warp-XPBD-Newton | 物理引擎选型 + D0–D5（力驱动/软头硬身/扭转/J-tip/autopilot） | 物理层**不动**，本文只在其上加控制抽象 |
| doc/07 Newton 开发记录与规划 | 引擎抽象（`PhysicsEngine` 协议）、实时链路、阶段 A–E | 本文的 Controller/RL 复用其接缝 |
| doc/05 §27.5 平台轨道 | 训练闭环 / RL 推理 / 模式显式化 | 本文是平台轨道 P1(训练闭环)+P2(RL) 的架构细化 |

---

## 一、核心修正：ShapeIntent 是"控制抽象"，不是"物理力场"

v1.0 文档 §4.2 方法2 `F_i = k·(target_position_i − segment_position_i)`（把每一节软拉向目标位置）与"结论1：控制对象必须是 ShapeIntent 而不是 physics force"，**若照字面实现，等于回退到 D3 的 graded soft-anchor**——即 doc/08 §八本次 D4 刚刚替换掉的运动学覆盖驱动。把段落位置软拉向目标会摧毁：真实力传导、tip 滞后、屈曲、真实接触力/风险——也就是 D4 的全部保真度。

**正确的解耦是分层，而不是把意图注入物理**：

```text
ShapeIntent(目标：方向 / waypoint / 强度)      ← Human UI = RL policy 共用的命令抽象
        │  ShapeIntentController(把目标解算成真实动作)   ← PhysicsAutopilot 的泛化
        ▼
   push / rotate   (真实 2-DOF 控制量)
        │
        ▼
   PhysicsEngine.step(push, rotate)             ← 纯物理，不含策略（D4 力驱动不变）
```

- 这样同时满足 v1.0 的三条原则（**控制解耦 / 物理隔离 / 统一接口**），**且不牺牲 D4 保真度**。
- v1.0 §9"ShapeIntent → Controller → PhysicsEngine"本身是对的；跑偏的只是 §4.2 方法2 的**实现方式**。
- 例外：v1.0 §4.2 方法3"中心线引导"作为 **guided 演示/可达性模式**（纯运动学）是合理的，且已由 `KinematicEngine` 实现——但那是**低保真 demo 通道**，不能与 physics/训练模式混用（对齐 doc/05 §27.5 "模式显式化"）。

> **结论1（订正）**：控制层的**对象**是 ShapeIntent；但 ShapeIntent 必须由 Controller 解算成真实 `push/rotate` 施加于纯物理引擎，**不得**实现为对导丝分段的软约束位置场。

---

## 二、分层架构（用真实模块名）

```text
        ┌─────────────────────────────────────────────┐
   人 →  │ Human Interface (Godot: 键盘 / 点击 / 手柄)  │
        └───────────────┬─────────────────────────────┘
   RL → │ RL Policy (SB3 PPO/SAC …)                     │
        └───────────────┬─────────────────────────────┘
                        ▼   下达 ShapeIntent（统一命令）
        ┌─────────────────────────────────────────────┐
        │ ShapeIntentController  (= PhysicsAutopilot 泛化)│  ← 新增/泛化
        │  look-ahead 朝向误差 + J-tip 爬山符号搜索       │
        │  + 力门控推速 + stall 扫掠/回拉 → (push, rotate)│
        └───────────────┬─────────────────────────────┘
                        ▼   真实 2-DOF
        ┌─────────────────────────────────────────────┐
        │ NavigationEngine (编排：progress/deviation/    │  ← 已有
        │  curvature/risk/contact，即 reward/obs 成分)   │
        └───────────────┬─────────────────────────────┘
                        ▼
        ┌─────────────────────────────────────────────┐
        │ PhysicsEngine  (NewtonEngine 力驱动，纯物理)   │  ← D4 已上线
        └───────────────┬─────────────────────────────┘
             PhysicsWorker → WebSocket → Godot 可视化
```

RL 训练时旁路 WebSocket/Godot，直接在进程内 `Gym(NavigationEngine + Controller)` 上跑。

---

## 三、现有资产映射（避免重造）

| v1.0 提出 | 现状 | 需要做的 |
|---|---|---|
| Mode 1 Direct（push/rotate） | ✅ `PhysicsEngine.step(push, rotate)` | 无 |
| Mode 3 Autopilot | ✅ `services/physics_autopilot.py`（D5 已接回 Newton 力驱动，5/6 route 自动到达） | 泛化为接受任意目标（见 §五） |
| 统一低层接口 | ✅ `PhysicsEngine` 协议（`base.py`），Human=RL 共用 | 无 |
| reward/obs 成分（progress/deviation/curvature/risk/contact/wall） | ✅ `NavigationEngine` 已全部派生 | 组装成 reward/obs 向量 |
| guided 中心线引导（低保真） | ✅ `KinematicEngine` | 仅作演示模式，勿混训练 |
| Gym 栈（env + wrappers + PPO） | ⚠️ 已有 `src/cathsim/gym/`，但**绑 dm_control/MuJoCo**（`CathSim(gym.Env)` 包 `make_dm_env`） | 新增 Newton NavigationEngine 版 env，复用 wrappers |
| 实时形变可调 | ✅ `engine_params` WS 消息 + Godot 滑块（本会话上线） | 无（可作 RL domain-randomization 入口） |
| ShapeIntent 数据模型 / Controller | ❌ 未有显式类型 | 新增（§四/§五） |
| 点击导航 / X-ray / ghost 轨迹 | ❌ | Godot 新增（对齐 doc/05 §27.5 P1） |
| 千环境并行 | ❌（单模型/session + 每步 numpy 同步） | Isaac Lab 提升（doc/08 §9.2，大工程） |

---

## 四、ShapeIntent 数据模型

```python
@dataclass
class ShapeIntent:
    # 二选一或并存的目标表达（世界系，米）：
    target_direction: np.ndarray | None = None   # 期望 tip 前进方向（单位向量）
    target_waypoint: np.ndarray | None = None     # 期望 tip 抵达的点（点击导航/RL 规划）
    intensity: float = 1.0                          # 推进强度 [0,1]（多快、多用力）
    # 语义：Controller 用它 + 当前物理状态解算 (push, rotate)。
    # 不携带任何"直接设定段落位置"的字段——那属于运动学覆盖，禁止。
```

- **Human 点击** → 投影到 centerline → `target_waypoint`。
- **Human 手柄** → 摇杆方向 → `target_direction`。
- **RL policy** → 网络输出 `target_direction(3) + intensity(1)`（或 `waypoint_offset(3)`）。
- **循线 autopilot** → `target_waypoint = 前视点`（Controller 内部默认，无需外部下达）。

---

## 五、ShapeIntentController 契约（P0）

由 `PhysicsAutopilot` **泛化**而来：现有 autopilot 已把"前视中心线目标"解算成 `(push, rotate)`；只需把"目标从哪来"抽象成 ShapeIntent 输入。

```python
class ShapeIntentController:
    def __init__(self, path_points, config): ...
    def reset(self): ...
    def compute(self, intent: ShapeIntent | None,
                tip_pos, tip_dir, contact_force) -> tuple[float, float]:
        """intent=None → 退化为沿 centerline 前视（现 autopilot 行为）。
        否则用 intent.target_* 作为朝向目标，复用同一套：
          look-ahead 朝向误差 → J-tip 方位不可观测 → 爬山符号搜索 rotate
          + 力门控推速(intensity 调制 base_push) + stall 扫掠/回拉。
        返回真实 (push, rotate) ∈ [-1,1]²。"""
```

- **不新增物理机制**：完全复用 D4/D5 已验证的力驱动 + 扭转 + 力门控。
- **三模式统一**：Human/RL/Auto 都只是"谁产生 ShapeIntent"，Controller 与引擎完全相同。
- 验收：`intent=None` 时与当前 autopilot 逐帧一致（回归）；给定 `target_waypoint` 时能把 tip 导向该点（复用 §D5 的到达能力）。

---

## 六、RL 环境设计（Newton Gym env，P1）

新增一个包 `NavigationEngine(Newton 力驱动) + ShapeIntentController` 的 `gymnasium.Env`，与现有 dm_control 版并列，复用 `src/cathsim/gym/wrappers`。

### 6.1 Observation（全部来自 `NavigationEngine`/`RawPose`，无需新物理）
```text
tip_position(3), tip_direction(3)
progress_along_centerline(1), deviation(1), local_tangent(3)   # NavigationEngine 已算
contact_force(1), wall_distance(1)                              # 已暴露
curvature(1)                                                    # 已算
[可选] SDF 距离采样(k)  ← tip 周围环境，需从 Mesh.build_sdf 采样（额外工作）
历史堆叠：obs(t), obs(t-1), obs(t-2)                            # wrapper 已有(recursive_wrapper)
```

### 6.2 Action（= ShapeIntent，由 Controller 落地为 push/rotate）
```text
推荐：  action = [desired_direction(3), intensity(1)]   # ShapeIntent → Controller → push/rotate
baseline：action = [push, rotate]                        # 直接 2-DOF，对照用
高级：  action = waypoint_offset(3)                      # 规划型
```

### 6.3 Reward（对齐 v1.0 §七，基于 centerline progress）
```text
R = w_p · Δprogress                 # 主项：沿中心线弧长增量（不是绝对位置）
  + w_a · dot(tip_dir, tangent)     # 对齐
  − w_c · ||contact_force||²        # 接触/穿壁惩罚（现真实接触力可用）
  − w_r · risk_score                # NavigationEngine 已算
  + R_reach (到达 +1) / R_fail (穿壁或超时 −)
```
> **结论3（保留）**：reward 主项用 centerline progress 增量，不用绝对坐标。已由 `NavigationEngine.progress_deviation` 直接支持。

### 6.4 reset 随机化 + curriculum
```text
reset 扰动：起点 s0、摩擦 μ、SDF/半径噪声、初始松弛步数、J-tip 角度（经 engine_params 通道）
curriculum（route 天然就是分级）：
  Stage1 直短   endpoint_0
  Stage2 单弯   endpoint_9
  Stage3 锐弯   endpoint_3
  Stage4 全树   endpoint_26 / 多目标
```

---

## 七、并行与 Isaac Lab（诚实边界）

- **近期**：单环境 = 单 Newton 模型/session + 每步 CPU numpy 同步。并行只能"多进程几个 env"，不是千环境。
- **千环境并行**属 doc/08 §9.2 的**提升通道**：Newton 场景 → `make_instanceable` + 批量 Warp kernel（单 kernel 跑 N 环境）→ Isaac Lab。触发点：RL 训练成为现行目标时启动，不在本文范围。
- 本文只保证**接口就绪**：Gym env 契约与 Isaac Lab 一致（obs/action/reward），将来换底座不改训练代码。

---

## 八、落地路线图（并入 D/P 轨）

| 优先级 | 内容 | 验收 | 关联 |
|---|---|---|---|
| **P0** | `ShapeIntentController`（泛化 autopilot，接受 ShapeIntent） | intent=None 与现 autopilot 逐帧一致；给 waypoint 能到达 | doc/05 P1 |
| **P0** | Godot 点击导航（点击→centerline 投影→waypoint→ShapeIntent） | 点击血管，tip 朝该点推进 | doc/05 §27.5 P1 |
| **P1** | Newton Gym env（obs/action=ShapeIntent/reward/reset/curriculum） | PPO 能在 Stage1–2 收敛到达 | doc/05 P2、doc/08 §9.3 |
| **P1** | 训练闭环（录制/回放/评分） | 会话可录可回放可评分 | doc/05 §27.5 P1 |
| **P2** | 模式显式化（guided/physics/RL HUD 标注） | API/HUD 明确区分保真度 | doc/05 §27.5 P1 |
| **P2** | 千环境并行 / Isaac Lab 提升 | make_instanceable 批量 | doc/08 §9.2 |

---

## 九、关键结论（订正版）

1. **ShapeIntent 是控制抽象，由 Controller 解算成真实 `push/rotate`；物理引擎保持纯物理。** 禁止把意图实现为分段软约束位置场（那是 D3 回退）。
2. **Human 与 RL 共享同一接口**——不是共享"注入物理的力场"，而是共享 **ShapeIntent → Controller → `PhysicsEngine.step`** 这条链；`PhysicsAutopilot` 就是 Controller 雏形。
3. **reward 基于 centerline progress 增量**，不基于绝对位置（`NavigationEngine` 已支持）。
4. **RL 栈已存在但绑 MuJoCo**；接 Newton 只需一个薄 env + Controller 落地 action，不重写。
5. **千环境并行 = Isaac Lab 提升**，是触发式大工程，非现在；现在只保证接口对齐。

---

> 关联：[05-开发进度记录.md](05-开发进度记录.md)（§27.5 平台轨道、§28 autopilot）、[07-Newton导丝物理仿真开发记录与规划.md](07-Newton导丝物理仿真开发记录与规划.md)（引擎抽象、实时链路）、[08-导丝物理迁移技术方案-Warp-XPBD-Newton.md](08-导丝物理迁移技术方案-Warp-XPBD-Newton.md)（§9 D4/D5、§9.2 Isaac Lab 提升）、`spikes/D4_RESULTS.md`（力驱动/扭转/J-tip/autopilot 实测）。
