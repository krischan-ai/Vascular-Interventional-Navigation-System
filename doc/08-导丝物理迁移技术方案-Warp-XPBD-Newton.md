# 导丝物理引擎迁移技术方案：Warp/XPBD 与 Newton 选型

> 本文是 [07-物理引擎抽象与实时性能架构.md](07-物理引擎抽象与实时性能架构.md) **阶段 D（Warp/XPBD 可行性 demo）** 的详细落地方案。
>
> **一句话**：单核 MuJoCo 的 ~3Hz 单步延迟地板（doc/07 §1）无法靠调参或堆硬件解决，要让导丝既"真物理"又"60Hz 跟手"，唯一正路是把导丝换成 **GPU XPBD 连续体求解器**。本文验收双方资产、定选型（结论：**Newton 独立版优先，手写 Warp 兜底**）、给分阶段可回退实施方案。

---

## 〇、背景与目标

- **问题**：aorta_trunk 真物理模式控制延迟 ~1 秒（doc/05 §28、doc/07 §1）。根因是 130 节导丝 + 520 壁砖碰撞的单核串行 `mj_step` ~350ms → 3Hz，再叠客户端插值的一帧延迟。**这是延迟受限不是吞吐受限，堆硬件救不了。**
- **判据**：手感（单根连续体导丝能否 **60Hz 实时 + 接触稳定不穿透**），**不是画面**——画面流畅已由 doc/07 阶段 B 的客户端插值解决。
- **目标**：在 `feat/warp-xpbd-guidewire` 分支上，先做离线可行性 demo 验证判据；通过后实现满足现有 `PhysicsEngine` 协议的新引擎，灰度替换，**上层零改写**。
- **硬件 / 平台**：开发机 RTX 3060 Laptop（够开发），**部署在 Linux 服务器 + A6000 48G**（顶配级，且 Isaac Sim/Newton 的一等平台）。两者跑同一套 Warp/CUDA 代码，环境一致，硬件与平台均不构成约束。
- **路线图（关键）**：本项目后续要做 **RTX 实时渲染 / 拟真传感器、RL 训练、ROS 桥 / 数字孪生**——这些正是完整 Isaac Sim 的强项。因此选型不仅看当下，还要**为演进到 Isaac Sim 预留路径**（见 §三.4）。结论先行：**Newton 是当下与未来的公共底座**，选 Newton 独立版即是 Isaac Sim 的"上匝道"，不是岔路。

---

## 一、当前项目（cathsim）资产验收

迁移要"上层零改写"，前提是已有这些稳定接缝。**结论：阶段 A/B 已把迁移要用的接缝全部铺好，新引擎是"加法"不是"改造"。**

### 1.1 物理引擎抽象接缝（已就绪，核心资产）

`services/physics/`（doc/05 §29.2 阶段 A 产出）：

| 文件 | 作用 | 对迁移的意义 |
|---|---|---|
| `base.py` | `PhysicsEngine` 协议 + `RawPose` + `PlannedPath` + `quat_from_direction` | **新引擎实现这个协议即可接入**，是迁移的唯一接缝 |
| `factory.py` | `make_engine(...)` 构造期二选一（guided→Kinematic 否则 MuJoCo） | 加一条分支返回 `NewtonEngine`/`WarpEngine` |
| `mujoco_engine.py` | MuJoCo 专有逻辑（make_dm_env / 预穿线 IK / 接触壁距） | 新引擎的参照实现 |
| `kinematic_engine.py` | guided 循线 | 不受影响 |

**`PhysicsEngine` 协议（新引擎必须满足的全部接口）**：

```python
def reset(self) -> RawPose            # 复位，返回首帧原始读数
def step(self, push, rotate) -> RawPose   # 推进一控制步（输入 clamp 到 [-1,1]）
def render_bodies(self) -> list[dict]     # 每节 {"pos":[x,y,z], "quat":[x,y,z,w]}
def close(self) -> None
@property control_timestep(self) -> float
```

`RawPose` 跨引擎原始量：`tip_position/direction/quaternion`(xyzw)、`contact_force`、`wall_distance`、`target_position`、`joint_positions/velocities`、`reward`、`done`、`arclen`(可选精确弧长)。**派生量（progress/deviation/curvature/risk）留在 `NavigationEngine` 编排层，新引擎不用管。**

### 1.2 实时链路（已就绪，引擎无关）

- `services/realtime/physics_worker.py`：`PhysicsWorker` 后台线程 `while running: engine.step(latest_input)` 自主步进 + 双缓冲发布 `Frame`。**它包的是 `NavigationEngine`，对底层引擎完全无关**——换 Newton/Warp 后 worker/sender 一行不改。
- `websocket_handler.py`：25Hz sender + `control` 只覆盖 `latest_input`。
- **迁移收益**：GPU 引擎 60Hz 后，把 `guidewire_renderer.gd` 的插值时长调短/关掉（那 ~350ms 延迟本是为 3Hz 加的），手感即回。

### 1.3 导丝物理模型（现状，作为 XPBD 杆参数来源）

`src/cathsim/dm/components/guidewire.py`：导丝 = N 节 body 链，每节经 `J0`(x轴) / `J1`(y轴) 两个铰链构成 2-DOF 万向节，子节在父系 `[0,0,seg_len]`（段方向 = body 局部 +z），每节带 joint stiffness（`stiffness_scale` 可调）。亚毫米半径。长度随 `n_bodies`：segment_part ~0.58m、aorta_trunk ~0.256m。

→ **映射到 XPBD**：粒子链 + 距离约束（拉伸刚度）+ 弯曲约束（由现 joint stiffness 标定弯曲刚度）。扭转（cosserat）如需要再加扭转约束。

### 1.4 血管几何 / 碰撞数据（已就绪，是迁移最关键的输入）

| 资产 | 大小 | 对迁移的用途 |
|---|---|---|
| `meshes/segment_part/visual.stl` | 8.6M | 表面网格 → 体素化成 **SDF**（碰撞场）或直接 mesh 碰撞 |
| `meshes/segment_part/simplified.stl` | 1.5M | 轻量碰撞代理 |
| `meshes/segment_part/centerline.json` | — | 已重平滑 B 样条路径（最大转角 12.9°），渲染 + 预穿线 |
| `meshes/segment_part/{centerlines,graph}.json` | — | 分支图（导航逻辑） |
| `data/aorta_centerline/Segmentation.seg.nrrd` | 384K | **二值分割体 → 距离变换直接得 SDF**（segment_part 是其子段） |
| `data/aorta_centerline/Centerline curve (N).mrk.json` | 30 条 | **带半径**中心线（2.1–13.1mm），可做管约束/SDF |
| `meshes/aorta_trunk/` | 520 砖 | 现有密封壁砖管腔（MuJoCo 用） |

**关键**：segment_part 当年卡死 MuJoCo 的根因是 V-HACD 凸包管腔不密封（doc/05 §28.5）。**SDF 碰撞天然密封**（腔内距离<0），这个卡点在 XPBD 下不存在。SDF 两条现成来源：`visual.stl` 体素化，或 `Segmentation.seg.nrrd` 距离变换。

### 1.5 前端 / 协议不变量（迁移不得破坏）

`NavigationState` 字段、`state_batch` 协议（含 `seq`/`t_phys`）、Godot 渲染契约（`bodies[].pos/quat`、`path.waypoints`、`tip`）。Godot 端 `guidewire_renderer.gd` 按固定体数插值——**新引擎只要 body 数恒定就自动平滑**。

---

## 二、Isaac Sim / Newton 资产验收

验收对象：`E:\文档\Project\IsaacSim`（Isaac Sim 6.0.1-rc.7 全量源码）。

### 2.1 关键发现：Isaac Sim 自带的 Newton 就是"Warp + XPBD"

- `isaacsim.physics.newton` 扩展 Overview 原文：Newton 提供 **"advanced solvers including XPBD and MuJoCo backends"**，**CUDA graph capture** 优化，tensor 接口。
- `XPBDSolverConfig` 文档直引 **Macklin & Müller 2016 XPBD** 论文——和"手写 Warp+XPBD"是同一套理论。
- 物理技能表（`skills/physics-simulation/SKILL.md`）明确：

  | Newton 求解器 | 坐标 | 最适合 |
  |---|---|---|
  | SolverFeatherstone | 广义 | 关节机器人 |
  | SolverMuJoCo | 广义 | 验证过的运动控制（mujoco-warp） |
  | **SolverXPBD** | 最大坐标 | **软约束、cables、ropes** ← 导丝就是"cable/rope" |
  | SolverVBD | — | 软体/可变形 |

  → **导丝（细长柔性连续体）正是 SolverXPBD 的标称用例。**

### 2.2 Newton 可独立部署（决定性事实）

- pip 依赖（`python_packages.toml`）：`newton[sim]==1.2.1`、`mujoco-warp==3.8.0.3`、`newton-actuators`、`newton-usd-schemas`，底层 `omni.warp.core`。
- **Newton 是 pip 包，可脱离 Isaac Sim / Omniverse 单独装用**。Isaac Sim 里的 `isaacsim.physics.newton` 只是把它接进 USD/Fabric 的 wrapper。
- 后端 Warp/CUDA、开放 JIT，3060 与 A6000 同套代码。

### 2.3 完整 Isaac Sim 是"目的地"，但不是"起点"——分期而非否定

更正前一版的判断：本项目**部署在 Linux 服务器**（Isaac Sim 一等平台，无 Win 架构错配问题），且下列 Isaac Sim 强项**都在路线图上**。所以不是"用不上"，而是**现阶段还没到、先别付平台税**：

| Isaac Sim 强项 | 现阶段 | 路线图 | B（Newton 独立）如何预留 |
|---|---|---|---|
| RTX 实时渲染 / 拟真传感器 | Godot 够用 | ✅ 要做 | 渲染与物理解耦；RTX 是"再加一个渲染端"，物理(Newton)不变 |
| Isaac Lab（RL 训练 / 千环境并行） | 非当前目标 | ✅ 要做 | **Isaac Lab 底座就是 Newton**；Newton 原生模型可直接进 Isaac Lab 并行环境 |
| ROS 桥 / 数字孪生 | 非当前目标 | ✅ 要做 | Isaac Sim 基建；Newton 场景导入 Isaac Sim 后即接 ROS |
| USD 场景 / Omniverse 运行时 | 不需要 | 随上面三项 | 模型尽量 USD 可表达（`newton-usd-schemas`），保留导入通道 |

**结论修正**：完整 Isaac Sim 是**目的地**，不是错误方向。但当前判据（"单根导丝能否 60Hz 跟手"）是一个**物理可行性问题**，在 Newton 独立版里验证**最快**（pip 装、跑脚本、秒级迭代），不必先扛 Kit 启动 + USD 场景搭建的重迭代。**先用同一个引擎(Newton)以最低成本回答物理问题，再在需要 RTX/RL/ROS 时把场景"提升(promote)"进 Isaac Sim**——因为底层是同一个 Newton，这是提升不是重写。

---

## 三、选型结论：Newton 独立版（B）优先，手写 Warp（A）兜底

### 3.1 三方对比

| | A. 手写 Warp+XPBD | **B. Newton 独立（推荐）** | C. 完整 Isaac Sim |
|---|---|---|---|
| 装什么 | `pip install warp-lang`（几十 MB） | `pip install newton[sim]`（+mujoco-warp） | Omniverse 全家桶，多 GB |
| XPBD 求解器 | 自己写（积分/约束/碰撞/接触） | **现成**（SolverXPBD，标称 cables/ropes） | 现成（同 Newton） |
| 硬件 / 平台 | 任意 CUDA GPU | 任意 CUDA GPU | RTX 4080+，**Linux 一等**（部署正是 Linux+A6000，无错配） |
| 嵌进 FastAPI 后端 | ✅ 库调用 | ✅ 库调用 | ❌ 自成运行时（另起 sim 进程） |
| 接进 `PhysicsEngine` 协议 | ✅ | ✅ | ❌（平台级，非库级） |
| **承接路线图（RTX/RL/ROS）** | ❌ **死路**：手写求解器无法导入 Isaac Sim，将来全废 | ✅ **同一个 Newton**，可提升进 Isaac Sim/Isaac Lab | ✅ 本身就是 |
| 迭代速度（验证物理判据） | 快 | **快**（pip+脚本，秒级） | 慢（Kit 启动 + USD 场景） |
| 主要成本 | 写+调求解器（耗时大头）+ 对路线图是沉没成本 | 学 Newton API + 验证细杆/碰撞 | 现阶段平台税过早 |

### 3.2 为什么选 B：当下最快、未来不废

加入路线图维度后，三者关系更清晰：

1. **B vs A**：A 和 B 的工作量差异在于那块最难、最吃调试的 **XPBD 积分 + 约束投影 + 碰撞 broadphase + 接触 + CUDA graph**——A 自写，B 现成（且 SolverXPBD 标称用例就是 cables/ropes）。**更要命的是 A 对路线图是死路**：手写 Warp 求解器无法导入 Isaac Sim，将来做 RTX/RL/ROS 时整个废弃重来。**A 因此从"轻量之选"降为"纯应急兜底"。**
2. **B vs C**：底层是**同一个 Newton**。当前判据是物理可行性问题，B 里验证最快（pip 装、跑脚本、秒级迭代），不必先扛 Kit + USD 重迭代。等 RTX/RL/ROS 成为现行目标时，把 Newton 场景**提升**进 Isaac Sim，而非重写。
3. **所以 B 是唯一"当下最快 + 未来不废"的选项**：现在以最低成本回答物理判据，同时站在通往 Isaac Sim 的同一条 Newton 轨道上。

### 3.3 必过的验证关口（gate，0.5–1 天，不可跳过）

Newton 仍年轻（1.2.1，API 会变），全押 B 前用最小脚本验证两件事：

1. **细弹性杆能否搭**：Newton 是否有绳/杆原语，或用粒子链 + 距离约束 + 弯曲约束拼出"有弯曲刚度的 0.16–0.58m 细杆"（cosserat 扭转可能要手加约束）。
2. **杆能否撞血管不穿墙**：碰撞走 mesh 还是 SDF，细腔快推不穿隧。

**gate 通过 → 一路 B。gate 受阻 → 当天切 A。退回成本低**：因为 §四的目标架构、SDF 数据准备、协议接入对 A/B 完全共用，丢的只是这一天的 Newton spike。

### 3.4 预留 Isaac Sim 演进路径（RTX / RL / ROS / 数字孪生）

选 B 的同时，用三条设计原则保证将来"提升进 Isaac Sim"是加法不是重写：

1. **物理与渲染解耦**：物理始终是 Newton（`NewtonEngine` 实现 `PhysicsEngine`），渲染端可插拔。现在 Godot 流式渲染；将来要 RTX 拟真，是"再接一个渲染/传感器端"消费同一份物理状态，**物理代码不动**。
2. **场景 USD 可表达**：导丝/血管尽量用 Newton 原生模型 + **`newton-usd-schemas`** 表达（依赖里已有 `newton-usd-schemas==0.2.0`、`mujoco-usd-converter`）。血管 mesh、中心线、SDF 都保留为 USD/标准格式可导入的形态，保住进 Isaac Sim 的通道。
3. **数据格式向上兼容**：碰撞用 `visual.stl`/`Segmentation.seg.nrrd`（标准网格/体），不是引擎私有结构；将来 Isaac Sim 直接吃。

**各路线图能力的承接方式**：

| 未来能力 | 触发时机 | 承接动作（基于 B） |
|---|---|---|
| RTX 渲染 / 拟真传感器 | 需要拟真画面/合成数据 | Newton 物理不变，把场景加载进 Isaac Sim 用 RTX 渲染；或 Isaac Sim 出图、仍走流式 |
| RL 训练（Isaac Lab） | 训练导航策略 | Isaac Lab 底座即 Newton；Newton 模型进 Isaac Lab 起千环境并行（`make_instanceable`） |
| ROS 桥 / 数字孪生 | 接真实机器人/孪生 | 场景进 Isaac Sim 后用其 ROS2 桥；物理仍 Newton |

**判据触发点（何时真正切到完整 Isaac Sim）**：当"RTX 合成数据 / RL 训练 / ROS"中任意一项成为**现行开发目标**时，启动"Newton 场景 → USD → `isaacsim.physics.newton`"的提升，而非现在。

---

## 四、目标架构（迁移后）

```
Godot (60fps 插值, 关/短插值)  ◀── WebSocket(state_batch: bodies/path/seq) ──  FastAPI
                                                                                  │
                                                              PhysicsWorker(自主步进, 引擎无关)
                                                                                  │
                                                                   NavigationEngine(编排, 派生量)
                                                                                  │
                                                  factory.make_engine ── 选 ── NewtonEngine / WarpEngine
                                                                                  │
                                              ┌───────────────────────────────────┼─────────────────┐
                                          XPBD 杆(粒子链+弯曲)         SDF/mesh 碰撞(visual.stl/seg.nrrd)     A6000 GPU
```

- **新增**：`services/physics/newton_engine.py`（或 `warp_engine.py`）实现 `PhysicsEngine` 协议；`factory.make_engine` 加分支（如 `engine="newton"` 或按 phantom 配置）。内部用 Newton 原生模型 + USD 可表达的场景（见 §3.4）。
- **复用零改**：`NavigationEngine` 编排、`PhysicsWorker`/sender、`websocket_handler`、`PathPlanner`、Godot 全部渲染。
- **不变量**：`NavigationState` / `state_batch` / Godot 契约一字不动（doc/07 §不变量）。
- **演进预留**（虚线，未来）：`NewtonEngine` 的场景 ──导出 USD──▶ `isaacsim.physics.newton`（RTX 渲染 / Isaac Lab RL / ROS 桥）。底层同一个 Newton，提升非重写。

---

## 五、分阶段实施（可回退，判据驱动）

| 阶段 | 做什么 | 验收判据 | 跑在 |
|---|---|---|---|
| **D0 gate** | `pip install newton[sim]`；最小脚本：粒子链+弯曲约束搭一根杆，自由落+自碰 | 60Hz 不爆；细杆有合理弯曲刚度 | 3060 本地 |
| **D1 碰撞** | 杆 vs 一段直/弯管（mesh 或 SDF），施加 push | 不穿墙、被接住、快推不穿隧 | 3060 本地 |
| **D2 真腔** | segment_part 真腔：`Segmentation.seg.nrrd`→SDF 或 `visual.stl`→SDF；push/rotate 驱动 | 细腔稳定推进、接触力合理、**60Hz** | 3060 调 / A6000 跑量 |
| **D3 接线** | 包成 `newton_engine.py` 实现 `PhysicsEngine`；`factory` 接入；前端关插值 | 端到端可控演示；上层零改；回归绿 | 部署 A6000 |

- **判据贯穿**：手感（60Hz + 接触稳定），不是画面（doc/07 §判据）。
- **D0–D2 全程离线 demo**，不碰主线，不改协议；D3 才接入。
- **里程碑对齐 doc/07**：D2 完成 = **M3（手感判据出结论）**；D3 完成 = **M4（WarpEngine/NewtonEngine 实现协议、灰度替换、上层零改）**。

---

## 六、本地开发 + 服务器 Docker 部署

> Newton/Warp 是 CUDA + Linux 一等公民，Docker + NVIDIA Container Toolkit 是其标准部署形态。cathsim 本就是「前端(Godot) / 后端(Python 物理) 分离 + WebSocket」架构，正好把"本地交互"与"容器化算力"切开。

### 6.1 组件分布：谁本地、谁容器

| 组件 | 跑在哪 | 理由 |
|---|---|---|
| **Godot 客户端**（渲染 + 键盘） | **本地原生**（Win，开发者机） | 交互前端，要显示器/输入；经 WebSocket 连后端 |
| **Python 后端**（Newton 物理 + FastAPI） | **Docker 容器**（Linux + GPU） | Newton/Warp 是 CUDA+Linux 一等公民；容器化消除 Win 轮子问题 |

后端**同一个镜像**既可跑本地 3060（开发）、又可跑服务器 A6000（部署）；Godot 只改 WebSocket 地址（`ws://localhost:9000` ↔ `ws://server:9000`）。这与现有客户端/服务端切分天然对齐。

### 6.2 两种本地开发工作流（都收敛到同一 Docker 镜像）

**① 远程开发（推荐）**：VS Code **Remote-SSH** 进 Linux 服务器，代码在服务器容器内跑（直接用 A6000），编辑器 UI 在本地；Godot 本地连服务器 WS。
- 优点：**零 Windows-CUDA 折腾、开发即部署同环境、直用 A6000 大显存**。
- 代价：依赖到服务器的网络。

**② 本地容器（离线 / 纯本地时）**：本地 **WSL2 + Docker Desktop + NVIDIA GPU 直通**，3060 上跑同一镜像，源码 bind-mount 热改。
- 优点：真本地、离线可用、与服务器同镜像。
- 代价：WSL2+GPU 直通需一次性配置；3060 显存小（单根导丝开发够，RL/大批量上服务器）。

两者用**同一个 `Dockerfile`**，差异仅：dev 挂源码 + `uvicorn --reload`；deploy 用 compose 固化。

### 6.3 镜像与部署形态

```
Dockerfile:  nvidia/cuda:12.x-runtime-ubuntu22.04
             + python3.x + 项目依赖(fastapi/mujoco/dm_control/scipy/numpy...)
             + newton[sim]  ← D0 gate 验证通过后再加入
运行:        docker run --gpus all -p 9000:9000 ...   (或 docker compose up)
前置:        宿主机 NVIDIA 驱动 + nvidia-container-toolkit（WSL2 本地同理）
GPU 自检:    python -c "import warp; warp.init(); print(warp.get_cuda_devices())"
```

- **后端不出图**：渲染在 Godot；MuJoCo(现有 guided)走 headless/EGL，容器无需 X 依赖，干净。
- **唯一一次性配置**：宿主机驱动 + nvidia-container-toolkit；配好后 `--gpus all` 即用。

### 6.4 落地顺序（风险分段）

1. **最小可跑版**：先把现有 **FastAPI + MuJoCo** 后端容器化，跑通「本地编辑 → 服务器容器 → Godot 连上」这条链（不含 newton）。
2. **加 Newton**：D0 gate 验证 `newton[sim]` 通过后，再写进 `Dockerfile`，避免把"环境容器化"和"Newton 可行性"两个风险耦在一起。
3. 产出物：`Dockerfile`、`docker-compose.yml`（部署）、`.devcontainer/`（VS Code 一键进容器）；细化步骤并入 [04-部署与开发指南.md](04-部署与开发指南.md)。

**与 Isaac Sim 预留一致**（§3.4）：将来 RTX/RL 用的 Isaac Sim 官方亦走 Docker + Linux 分发，容器化后端是同一条演进路。

---

## 七、风险与回退

| 风险 | 应对 |
|---|---|
| Newton 1.2.1 API 不稳 / 细杆支持弱（B 的主要不确定性） | D0 gate 先验证；受阻当天切 A（手写 Warp），共用资产不白做 |
| cosserat 扭转刚度不足 | 先做无扭转（粒子+弯曲）验证导航；需要时再加扭转约束 |
| 细腔快推穿隧 | XPBD 多 substep + 连续碰撞/SDF 梯度约束；限推进速率 |
| SDF 分辨率 vs 显存 | 单血管，A6000 48G 充裕；3060 用降采样 SDF 调试 |
| 导航（主动转向到目标）仍是控制问题 | 与物理解耦；demo 阶段用户 push/rotate 驱动即可；autopilot 另议 |
| `import torch` 与 Newton CUDA 上下文冲突（若引入 torch） | 延后 torch 导入到物理 settle 之后（Isaac 技能 §Part5 警告） |

**A↔B 可互换**：目标架构、SDF 数据准备、`PhysicsEngine` 接入、Godot/实时链路对二者完全相同，差异只在"求解器自写 vs Newton 提供"。

---

## 八、当前状态与下一步

- **分支**：`feat/warp-xpbd-guidewire`（基于 `f19954c` = 全部 aorta/实时/PhysicsEngine 抽象 + segment_part 平滑）。
- **未开工**：`warp-lang`/`newton[sim]` 均未安装（`import warp` 当前失败）。
- **下一步**：执行 **D0 gate** —— 装 `newton[sim]`，写最小"粒子链弯曲杆"脚本验证 60Hz + 细杆刚度。这是 B 成不成立的第一个关口；通过即一路 B，受阻当天切 A。
- **gate 跑在哪**：Newton 是 Linux 一等公民，**建议 D0–D2 直接在 Linux + A6000 服务器上跑**（pip 轮子/CUDA 最顺、显存充裕），3060 本地用于改代码；避免 Win11 上趟 Newton 轮子兼容。这也与最终部署环境一致。

---

> 关联文档：[05-开发进度记录.md](05-开发进度记录.md)（§28 物理攻关、§29 抽象+实时）、[07-物理引擎抽象与实时性能架构.md](07-物理引擎抽象与实时性能架构.md)（阶段 A–D 规划）。
