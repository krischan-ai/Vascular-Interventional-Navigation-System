# 强化学习训练教程（一）：从环境检查到首次 PPO 实验

> 适用对象：第一次接触强化学习训练的开发者
>
> 适用代码：`reinforcement_learning/` 工作区与 `src/cathsim/rl/navigation_train.py`
>
> 默认环境：Python 3.10、Newton/Warp、Gymnasium、Stable-Baselines3、单张 CUDA GPU
>
> 上一篇：[11-强化学习训练设计方案.md](11-强化学习训练设计方案.md)
>
> 工作区入口：[reinforcement_learning/README.md](../../reinforcement_learning/README.md)

本文带你完成一条最小但完整的学习闭环：

```text
确认解释器
  → 预览训练参数
  → 依赖 smoke test
  → 环境与训练契约测试
  → 先启动 TensorBoard
  → 运行短程 PPO
  → 检查产物
  → 独立评估
  → 写下“证明了什么、没有证明什么”
```

本教程不会把“命令成功退出”称为“模型训练成功”。第一次实验的目标是验证训练管线，而不是获得可用于介入操作的策略。

---

## 1. 先理解正在训练什么

CathSim 当前训练链路是：

```text
PPO policy
  │ 输出 [target_direction_x, target_direction_y, target_direction_z, intensity]
  ▼
NavigationGymEnv
  │ 封装为 ShapeIntent
  ▼
NavigationEngine / ShapeIntentController
  │ 解算真实 push / rotate
  ▼
Newton/Warp 物理引擎
  │ 返回导丝状态、接触和路径进度
  ▼
observation + reward + termination
```

需要先认识五个词：

| 术语 | 在本项目中的含义 |
|---|---|
| Observation（观测） | 策略每一步看到的状态字典，包括尖端方向、路径进度、壁距、接触力、风险等 |
| Action（动作） | 默认是 4 维 ShapeIntent：目标方向 3 维加推进强度 1 维 |
| Reward（奖励） | 用路径进度作为主信号，同时惩罚偏离、接触、风险和动作抖动 |
| Episode（回合） | 从环境 `reset()` 到成功、碰撞、引擎结束或超时的一段交互 |
| Checkpoint（检查点） | 训练过程中保存的模型快照，用于评估或中断后续训 |

PPO 是本教程的第一条基线。它比较稳定、训练日志容易观察，但属于 on-policy 算法，Newton 仿真较慢时样本成本会比较高。现有代码也支持 SAC，后续应把两者作为对照实验，而不是只保留表现最好的一次结果。

---

## 2. 认识当前工作区

先进入强化学习工作区：

```bash
cd /home/ps/cathsim-warp/reinforcement_learning
```

当前关键文件如下：

```text
reinforcement_learning/
├── README.md
├── configs/
│   ├── curriculum/stage0.yaml
│   ├── env/navigation.yaml
│   ├── randomization/disabled.yaml
│   ├── reward/default.yaml
│   └── train/ppo_stage0.yaml
├── scripts/
│   ├── setup.sh
│   ├── smoke_test.sh
│   ├── smoke_test.py
│   ├── train_ppo.sh
│   └── train_navigation.py
└── runs/
```

真正执行训练的核心代码不在 YAML 中，而在：

```text
src/cathsim/gym/envs/navigation.py
src/cathsim/rl/navigation_train.py
src/cathsim/rl/navigation_evaluate.py
```

### 2.1 当前配置边界

截至本文版本，`configs/*.yaml` 是训练设计和参数记录模板，`train_navigation.py` **不会自动读取这些 YAML**。当前真正生效的是命令行参数以及代码默认值。

这会带来三个容易误判的问题：

1. 修改 `configs/train/ppo_stage0.yaml` 不会自动改变下一次训练。
2. `physics_hz`、`action_repeat`、curriculum 路线晋级和 domain randomization 尚未由当前训练入口接线。
3. `configs/reward/default.yaml` 把惩罚项写成负数，而环境代码采用“正权重后再做减法”的约定；在没有转换层之前，不能把 YAML 原样传入 `reward_weights`。

因此，每次实验都应以输出目录里的 `run_config.json` 为最终参数证据。

---

## 3. 第一步：确认解释器和环境

目的：确认后续命令使用项目自己的 Python 3.10 环境，而不是系统 Python。

执行：

```bash
pwd
.venv310/bin/python --version
.venv310/bin/python -c "import sys; print(sys.executable)"
```

你应该观察：

- 当前目录是 `/home/ps/cathsim-warp/reinforcement_learning`；
- Python 版本是 `3.10.x`；
- 解释器路径以 `reinforcement_learning/.venv310/bin/python` 结尾。

如果 `.venv310/bin/python` 不存在，再执行安装：

```bash
bash scripts/setup.sh
```

`setup.sh` 会下载和安装依赖，可能耗时并占用磁盘。不要用 `sudo` 执行，否则生成的环境或缓存可能归 root 所有，之后普通用户会遇到 `Permission denied`。

---

## 4. 第二步：先预览参数，不启动训练

目的：确认训练入口支持哪些参数，并在消耗 GPU 时间前发现拼写或路径错误。

执行：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src \
  .venv310/bin/python scripts/train_navigation.py --help
```

重点观察：

- `--algorithm` 支持 `ppo` 和 `sac`；
- `--action-mode` 支持 `shape_intent` 和 `direct`；
- `--physics-engine` 支持 `newton`、`mujoco` 和 `guided`；
- 可以独立设置 `--eval-freq` 与 `--checkpoint-freq`；
- 可以用 `--resume-model` 续训；
- SAC 续训时还可用 `--resume-replay-buffer` 恢复经验回放池。

这一步只证明命令行能够解析，不证明依赖、环境和物理仿真可运行。

---

## 5. 第三步：运行依赖 smoke test

目的：快速确认训练依赖可导入，并检查训练配置对象能否构造。

执行：

```bash
bash scripts/smoke_test.sh
```

成功时，末尾应出现：

```text
RL_IMPORT_SMOKE_OK
```

同时会打印 `torch`、`gymnasium`、`stable_baselines3`、`tensorboard` 和 `cathsim` 的版本。

这一步证明：

- 当前解释器能导入训练依赖；
- CathSim Gym 环境完成注册；
- `NavigationTrainConfig` 可以创建。

这一步没有证明：

- Newton 环境能完成 `reset/step`；
- GPU 物理稳定；
- reward 正确；
- PPO 能学习；
- 模型能到达目标。

---

## 6. 第四步：检查环境与训练契约

目的：在正式采样前验证 Gym API、有限数值、奖励分量、训练产物和评估统计的代码契约。

从 `reinforcement_learning/` 执行：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src \
  .venv310/bin/python -m pytest -q \
  ../tests/test_navigation_gym_env.py \
  ../tests/test_navigation_train.py \
  ../tests/test_navigation_evaluate.py
```

重点观察：

- 所有测试通过；
- 没有 `NaN`、`Inf` 或 observation space 越界；
- ShapeIntent 与 direct action 两种模式的基本契约都成立；
- PPO/SAC 模型、checkpoint、运行状态和 SAC replay buffer 的保存路径符合预期；
- 独立评估能统计 success、progress、contact 和 termination reason。

这些测试大量使用可控的 fake engine，适合检查接口回归，但不能代替真实 Newton rollout。

---

## 7. 第五步：训练前启动 TensorBoard

目的：从训练第一步起保留可观测性。TensorBoard 不是训练结束后的装饰，它用来判断训练是否还在运行、奖励是否异常以及本次结果对应哪个 run。

另开一个终端，执行：

```bash
cd /home/ps/cathsim-warp/reinforcement_learning
.venv310/bin/tensorboard \
  --logdir runs \
  --host 127.0.0.1 \
  --port 6006
```

然后在同一台机器访问 `http://127.0.0.1:6006`。

如果服务器在远程机器上，不要直接把 TensorBoard 暴露到公网；应使用 SSH 端口转发或受控的内网访问方式。

训练开始后优先观察：

| 指标 | 先问什么 |
|---|---|
| `rollout/ep_rew_mean` | 平均回合奖励是否持续变化，还是突然爆炸或变成非有限数 |
| `rollout/ep_len_mean` | 回合是否总在固定上限超时 |
| `train/approx_kl` | PPO 每次更新是否过大 |
| `train/clip_fraction` | 有多少更新被裁剪；长期极高可能意味着更新过猛 |
| `train/entropy_loss` | 策略探索性是否快速消失 |
| `train/explained_variance` | value network 是否开始解释回报变化 |
| `train/value_loss` | 价值估计是否不稳定 |

单个指标不能独立证明策略质量。尤其是平均奖励升高，可能来自奖励投机，而不是更安全地到达目标。

---

## 8. 第六步：运行第一次短程 PPO 实验

### 8.1 为什么使用 4096 timesteps

Stable-Baselines3 的 PPO 默认每轮先收集一批 rollout。过小的 `total_timesteps` 仍可能向上完成整批采样，而且只跑一轮时很难看到更新后的趋势。这里用 4096 步，目的是得到至少一个“采样—更新—再次采样”的短闭环。

它仍然只是工程实验，不是正式训练。

### 8.2 使用唯一的 run name

不要复用已有输出目录。复用相同 `run_name` 会混合 TensorBoard 日志，并可能覆盖 `run_config.json` 和 `run_status.json`。

本教程使用：

```text
tutorial_ppo_stage0_seed0_4k
```

如果该目录已经存在，请先换一个新名字，例如在末尾加入日期或递增编号；不要为了重跑教程直接删除旧实验。

### 8.3 启动训练

执行：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src \
CUDA_VISIBLE_DEVICES=0 \
CATHSIM_PHYSICS_ENGINE=newton \
  .venv310/bin/python scripts/train_navigation.py \
  --algorithm ppo \
  --run-name tutorial_ppo_stage0_seed0_4k \
  --output-dir runs \
  --phantom aorta_tree \
  --route-target endpoint_0 \
  --action-mode shape_intent \
  --physics-engine newton \
  --total-timesteps 4096 \
  --seed 0 \
  --eval-episodes 5 \
  --eval-freq 2048 \
  --checkpoint-freq 2048
```

你应该观察：

- 控制台打印 PPO rollout 和 train 指标；
- `runs/tutorial_ppo_stage0_seed0_4k/` 出现；
- 训练过程中出现 checkpoint 与 evaluation 产物；
- 正常结束后出现 `models/final_model.zip`；
- `run_status.json` 从 `running` 变成 `completed`。

如果训练报错，先保留完整终端输出，不要立刻删除 run 目录。失败时 `run_status.json` 会记录异常摘要，终端中的第一条有意义错误通常更接近根因。

---

## 9. 第七步：检查训练产物

执行：

```bash
find runs/tutorial_ppo_stage0_seed0_4k -maxdepth 3 -type f | sort
sed -n '1,240p' runs/tutorial_ppo_stage0_seed0_4k/run_config.json
sed -n '1,120p' runs/tutorial_ppo_stage0_seed0_4k/run_status.json
```

期望目录结构：

```text
runs/tutorial_ppo_stage0_seed0_4k/
├── eval/
│   └── evaluations.npz
├── models/
│   ├── best_model.zip
│   ├── ppo_checkpoint_..._steps.zip
│   └── final_model.zip
├── monitor/
│   ├── eval.csv.monitor.csv
│   └── train.csv.monitor.csv
├── tb/
├── run_config.json
└── run_status.json
```

逐项解释：

| 产物 | 用途 |
|---|---|
| `run_config.json` | 复现实验的参数证据 |
| `run_status.json` | 判断运行中、完成或失败 |
| `monitor/train.csv.monitor.csv` | 回合奖励和长度记录 |
| `tb/` | TensorBoard event 文件 |
| `eval/evaluations.npz` | 训练期间由 EvalCallback 产生的评估 |
| `models/best_model.zip` | 按训练期评估 reward 选择的最佳模型 |
| `models/final_model.zip` | 最后一步保存的模型，不保证优于 best model |

不要看到文件存在就立即判断“训练成功”。检查产物时应分成三层：

1. **完整性**：训练是否正常结束，预期文件是否都存在。
2. **可复现性**：参数、seed、环境和模型是否属于同一次实验。
3. **学习质量**：训练前后是否出现稳定、可解释的改进。

### 9.1 先检查完整性和实验身份

逐项确认：

- `run_status.json` 是 `completed`，并同时包含开始时间、结束时间和 final model 路径；
- `run_config.json` 中的算法、总步数、seed、phantom、route、动作模式和 Newton 参数与实际命令一致；
- checkpoint 的步数覆盖了预期节点，例如本实验应有 2048 和 4096；
- Monitor、TensorBoard 和 `evaluations.npz` 不是空文件；
- 当前 run 目录没有混入旧实验产物。

可用时间和大小辅助检查：

```bash
find runs/tutorial_ppo_stage0_seed0_4k \
  -maxdepth 4 -type f -printf '%TY-%Tm-%Td %TH:%TM:%TS  %10s  %p\n' | sort
```

如果 `tb/` 下出现同名的 `_1`、`_2`、`_3` 等多个目录，通常表示相同 `run_name` 被多次启动。Stable-Baselines3 会为 TensorBoard 自动增加后缀，但 config、status、Monitor 和模型仍可能被覆盖或来自不同次启动。此时不要把多个 event 文件拼成一条训练曲线；应找出真正有标量数据的 event，并把本次实验记为“可审计性受损”。下一次使用新的 run name。

### 9.2 从 Monitor 和训练期评估中提取数字

先看原始 CSV 的开头和末尾，确认格式和数据确实存在：

```bash
sed -n '1,12p' runs/tutorial_ppo_stage0_seed0_4k/monitor/train.csv.monitor.csv
tail -n 12 runs/tutorial_ppo_stage0_seed0_4k/monitor/train.csv.monitor.csv
```

下面的脚本比较训练早期和晚期各最多 100 个已完成回合，并读取 `evaluations.npz`。它只读文件，不修改训练结果：

```bash
.venv310/bin/python - <<'PY'
from pathlib import Path
import csv
import numpy as np

run = Path("runs/tutorial_ppo_stage0_seed0_4k")
with (run / "monitor/train.csv.monitor.csv").open() as file:
    next(file)  # Monitor 的 JSON 元数据行
    rows = list(csv.DictReader(file))

if not rows:
    raise SystemExit("Monitor 中没有已完成回合")

rewards = np.asarray([float(row["r"]) for row in rows])
lengths = np.asarray([int(row["l"]) for row in rows])
window = min(100, max(1, len(rows) // 2))

for name, index in (("early", slice(0, window)), ("late", slice(-window, None))):
    print(
        name,
        "episodes=", len(rewards[index]),
        "reward_mean=", rewards[index].mean(),
        "reward_std=", rewards[index].std(),
        "length_mean=", lengths[index].mean(),
    )

print(
    "all",
    "episodes=", len(rows),
    "reward_mean=", rewards.mean(),
    "reward_min=", rewards.min(),
    "reward_max=", rewards.max(),
    "length_mean=", lengths.mean(),
    "finite=", bool(np.isfinite(rewards).all()),
)

with np.load(run / "eval/evaluations.npz") as data:
    print("eval_timesteps=", data["timesteps"].tolist())
    print("eval_rewards=", data["results"].tolist())
    print("eval_lengths=", data["ep_lengths"].tolist())
PY
```

判断时不要只比较全程平均值。至少同时检查：

| 现象 | 可能含义 | 下一步 |
|---|---|---|
| 晚期 reward 高于早期，训练期 eval 也提高 | 可能出现学习信号 | 再看 success、progress 和安全指标，排除奖励投机 |
| 训练 reward 提高，但 eval 不提高 | 可能过拟合、训练噪声或环境不一致 | 核对 train/eval 环境参数并增加评估点 |
| reward 基本不变 | 训练过短、奖励信号弱或策略没有学习 | 先检查回合和环境是否有效，再决定是否延长训练 |
| reward 突然爆炸或出现非有限数 | 数值、物理或奖励实现异常 | 停止用该模型下结论，定位首次异常步 |
| 回合长度始终很短且以 `engine_done` 结束 | 环境可能在策略发挥作用前结束 | 检查 Newton 几何参数、初态和终止条件 |
| 回合长度总等于上限 | 策略可能一直超时 | 结合 progress 判断是缓慢前进还是完全停滞 |

回合长度变长或变短本身都不是好坏指标：成功后更快结束是好事，碰撞或引擎提前结束则是坏事，必须结合 `termination_reason` 判断。

### 9.3 判断 PPO 更新是否健康

在 TensorBoard 中把 rollout、eval 和 train 指标放在同一时间轴上。以下范围只能作为排查启发，不是通用合格线：

| 指标 | 初学者判断方法 |
|---|---|
| `rollout/ep_rew_mean` | 应与独立 eval 大方向一致；只升高这一项不能证明任务完成 |
| `train/approx_kl` | 观察是否突然增大；长期高于约 `0.05` 值得检查学习率，但应优先与配置的 `target_kl` 比较 |
| `train/clip_fraction` | 接近 0 可能表示更新很小；长期高于约 `0.3` 可能表示更新过猛；不要把某个固定值当训练目标 |
| `train/explained_variance` | 接近 1 表示 value network 更能解释回报；接近 0 表示解释力弱；负值表示比预测常数均值还差 |
| `train/value_loss` | 绝对大小依赖 reward 尺度，只能在相同配置下看趋势；持续增长或非有限值是警报 |
| `train/entropy_loss`、`train/std` | 看探索是否突然坍缩；绝对值依赖动作维度，不能跨任务硬比较 |

至少需要多个更新点才能谈趋势。如果 TensorBoard 只有一个 `train/*` 点，只能说明进行过更新，不能据此声称稳定或收敛。

还要注意 PPO 的回调时序：默认 `n_steps=2048`，本教程又在 2048 步触发第一次评估。第一次 EvalCallback 可能发生在首轮 rollout 收集完成、第一次参数更新之前。因此，首次生成的 `best_model.zip` 可能仍接近初始策略。“best”只表示在当前评估环境和当前 reward 下最好，不表示训练取得了正收益，更不表示临床最好、安全最好或跨几何最好。

---

## 10. 第八步：独立评估模型

训练期评估和独立评估要分开。独立评估应使用新的 seed 范围，并把结果保存成可审查的 JSON。

先评估 `best_model.zip`：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src \
CUDA_VISIBLE_DEVICES=0 \
CATHSIM_PHYSICS_ENGINE=newton \
  .venv310/bin/python -m cathsim.rl.navigation_evaluate \
  runs/tutorial_ppo_stage0_seed0_4k/models/best_model.zip \
  --algorithm ppo \
  --episodes 5 \
  --seed 1000 \
  --phantom aorta_tree \
  --route-target endpoint_0 \
  --physics-engine newton \
  --max-episode-steps 300 \
  --output runs/tutorial_ppo_stage0_seed0_4k/eval/independent_best_5ep_seed1000.json
```

再评估 final model，必须使用完全相同的环境参数和 seed 范围：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src \
CUDA_VISIBLE_DEVICES=0 \
CATHSIM_PHYSICS_ENGINE=newton \
  .venv310/bin/python -m cathsim.rl.navigation_evaluate \
  runs/tutorial_ppo_stage0_seed0_4k/models/final_model.zip \
  --algorithm ppo \
  --episodes 5 \
  --seed 1000 \
  --phantom aorta_tree \
  --route-target endpoint_0 \
  --physics-engine newton \
  --max-episode-steps 300 \
  --output runs/tutorial_ppo_stage0_seed0_4k/eval/independent_final_5ep_seed1000.json
```

如果训练时使用了 `--newton-rod-length`、`--newton-free-len`、`--newton-max-slack` 或 `--newton-insertion-margin`，评估时必须传入同样的值。否则改变的不只是模型，而是环境本身，比较没有意义。

重点读取：

| 字段 | 如何解释 |
|---|---|
| `success_rate` | 到达 `success_progress` 的回合比例 |
| `termination_reasons` | 成功、超时、碰撞停止或引擎结束各有多少 |
| `final_progress` | 回合结束时的路径进度 |
| `max_progress` | 回合中曾达到的最大进度，可发现“前进后回退” |
| `contact_integral` | 全回合接触代价累计 |
| `max_contact` | 单步最大接触 |
| `steps` | 到达或终止所需策略步数 |

### 10.1 按固定优先级判读

不要先挑最好看的 reward。按以下顺序判断：

1. **评估有效性**：模型能否加载，数值是否有限，环境参数是否与训练一致。
2. **任务完成度**：先看 `success_rate`，再看 `final_progress` 和 `max_progress`。
3. **终止质量**：区分 `success`、`timeout`、`collision_stop` 和 `engine_done`。
4. **安全代价**：比较 contact、penetration、buckling、wrong branch 和 shield intervention；当前为零的占位字段不能当成安全证据。
5. **效率**：只在成功回合之间比较 `steps`。失败得更快不是效率更高。
6. **reward**：只在相同 reward、环境和终止规则下比较，作为前述任务指标的补充。

若 reward 较高但 `success_rate=0`，模型仍未完成任务。若 success 较高但 contact 或 Safety Shield 干预很高，也不能称为安全策略。

### 10.2 比较 best 与 final

使用一张表记录同 seed、同环境下的差异：

| 模型 | success rate | reward mean | final/max progress | termination reasons | contact | steps |
|---|---:|---:|---|---|---|---:|
| best |  |  |  |  |  |  |
| final |  |  |  |  |  |  |

解释规则：

- best 与 final 都失败：不能声称 PPO 学会了任务；
- best 明显优于 final：后期训练可能退化或不稳定，应检查评估曲线和学习率；
- final 优于 best：训练期评估样本可能过少，或最后一次更新发生在最后一个 EvalCallback 之后；
- 两者几乎相同：可能训练太短、学习信号太弱，或策略输出对环境影响很小；
- 不同 seed 的所有 reward、steps、progress 都完全相同：优先记为“随机化覆盖不足”，不能把它们当成多个独立样本。

五回合评估的成功率只能以 20% 为步长变化，适合 smoke test，不适合给出稳定性结论。正式比较应在冻结的测试场景上增加 episode 数，报告离散程度或置信区间，并使用多个独立训练 seed。

---

## 11. 如何判定第一次实验

### 11.1 管线通过

第一次 4096 步实验满足以下条件，可记为“训练管线通过”：

- smoke test 出现 `RL_IMPORT_SMOKE_OK`；
- 三组契约测试通过；
- 训练没有 NaN/Inf；
- `run_config.json` 与实际命令一致；
- `run_status.json` 为 `completed`；
- checkpoint、final model、monitor 和 TensorBoard 日志存在；
- 独立评估能完成并输出 JSON。

这一级只叫“管线通过”，不叫“模型训练成功”。

### 11.2 质量分级标准

初学者可以用以下分级避免过度下结论：

| 级别 | 最低证据 | 允许的结论 |
|---|---|---|
| A：管线通过 | 11.1 的条件全部满足 | 训练、保存和评估链路可运行 |
| B：出现学习信号 | 至少 3 个训练期评估点总体改善；独立评估的 progress 或 success 优于未训练策略/固定基线；没有靠异常终止获得奖励 | PPO 可能开始学习，值得延长或复现实验 |
| C：初步任务能力 | 多个独立训练 seed 均有成功；冻结测试集上优于 baseline；安全代价没有恶化；人工轨迹检查合理 | 在当前受限场景中具备初步能力 |
| D：稳定候选策略 | 未见 route/几何、多 seed 统计、置信区间、安全压力测试和 baseline 对照均通过 | 可以进入下一阶段研究验证 |

B 级不是用某个固定 reward 阈值定义，因为 reward 尺度会随权重改变。它要求训练曲线、任务指标和独立评估给出方向一致的证据。C、D 级也不是医疗或生产认证。

### 11.3 本教程 4k PPO 结果的判读示例

对 `tutorial_ppo_stage0_seed0_4k` 的实际产物应用上述方法，可得到：

- `run_status.json` 为 `completed`，4096 步约耗时 17 分 18 秒，checkpoint、best/final、Monitor、TensorBoard 和训练期评估均存在；
- 693 个已完成训练回合的平均 reward 为 `-7.499`，平均长度只有 `5.91` 步；
- TensorBoard 的 rollout reward 从约 `-7.517` 变为 `-7.538`，训练期 eval reward 从 `-6.246` 降到 `-6.586`，没有一致的改善趋势；
- `approx_kl=0.009`、`clip_fraction=0.080` 没有显示更新爆炸，但 `explained_variance=-0.180` 表明 value network 尚不能可靠解释回报，而且只有一个 `train/*` 记录点；
- 使用 seed 1000–1004 独立复核时，best 和 final 都是 `0/5` 成功，全部在第 8 步以 `engine_done` 结束；
- best/final 的平均 reward 分别约为 `-6.246` 和 `-6.341`，最终 progress 分别约为 `0.96172` 和 `0.96130`，final 没有超过 best；
- 五个 seed 的结果完全相同，说明当前 reset 没有提供可观察的评估多样性；
- contact 为 0 不能证明安全，因为所有回合都异常短，并且部分安全代价仍是占位字段；
- `tb/` 中有两次早期启动留下的空 event 文件，说明该 run name 曾被复用，实验身份的可审计性受损。

因此，这次实验属于 **A：管线通过**，不满足 **B：出现学习信号**。最优先的问题不是盲目增加 timesteps，而是先解释为什么 `max_episode_steps=300` 的环境总在约 8 步 `engine_done`，检查 Newton 杆长、自由长度、插入余量、初始状态和终止条件；修复后再用唯一 run name、至少 3 个训练期评估点和真正不同的 reset 做对照。

### 11.4 不能据此声称

即使以上全部通过，也不能据此声称：

- PPO 已收敛；
- 模型具备稳定导航能力；
- 模型优于 autopilot；
- 模型能泛化到其他 route 或其他血管；
- 接触统计已具有临床意义；
- 模型可以进入 `RL_ASSIST`；
- 系统已达到生产或医疗使用标准。

正式结论至少需要：多个训练 seed、冻结的独立测试集、未见几何、明确的安全指标、人工回放检查，以及与 direct baseline 和 autopilot 的对照。

---

## 12. 常见问题与最小排查

### 12.1 `ModuleNotFoundError: cathsim`

含义：当前 Python 找不到仓库源码。

先检查：

```bash
pwd
.venv310/bin/python -c "import sys; print(sys.executable)"
```

然后确认训练命令包含：

```bash
PYTHONPATH=/home/ps/cathsim-warp:/home/ps/cathsim-warp/src
```

### 12.2 `Permission denied`

含义：环境、缓存或输出目录可能由另一个用户创建，常见原因是曾经用 `sudo` 执行安装或训练。

先检查目标所有权：

```bash
ls -ld .venv310 runs
ls -l runs/tutorial_ppo_stage0_seed0_4k
```

不要直接对整个仓库执行宽泛的 `chmod -R 777`。先确认具体错误路径和所有者，再只修复必要目标。

### 12.3 CUDA、Warp 或显存错误

先确认训练使用哪张卡：

```bash
nvidia-smi
.venv310/bin/python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

如果是显存不足，先停止无关 GPU 任务或降低单次实验规模。当前 Newton 训练只支持 `n_envs=1`，不要先通过增加并行环境解决速度问题。

### 12.4 TensorBoard 看不到曲线

依次检查：

```bash
find runs/tutorial_ppo_stage0_seed0_4k/tb -type f
sed -n '1,120p' runs/tutorial_ppo_stage0_seed0_4k/run_status.json
```

如果 event 文件存在，确认 TensorBoard 的 `--logdir` 指向 `reinforcement_learning/runs`。如果训练尚未写完第一批 rollout，曲线可能暂时为空。

### 12.5 没有 `best_model.zip`

先比较：

- `total_timesteps`
- `eval_freq`

只有训练步数达到评估频率后才会触发 EvalCallback。极短 smoke run 可能只有 `final_model.zip`。

### 12.6 多个评估 seed 得到完全相同结果

这不一定代表模型极其稳定，也可能说明当前 reset 随机化尚未真正改变物理初态。检查每个 `episode_results` 是否步数、reward、progress 都完全相同，并把它记录为“随机化覆盖不足”，不能把重复的确定性回放当成多个独立样本。

### 12.7 `run_status.json` 长期为 `running`

先确认进程是否仍存在，再查看终端或日志。进程被强制终止时，Python 可能没有机会把状态改为 `failed`，所以 `running` 不是进程仍存活的充分证据。

---

## 13. 实验记录模板

每次运行后至少写下：

```text
实验名称：
代码 commit：
日期：
操作者：

目标：
算法与 seed：
phantom / route：
动作模式：
Newton 参数：
总 timesteps：
评估 episodes 与 seed：

训练是否完成：
success_rate：
termination_reasons：
final/max progress：
contact integral / max contact：
TensorBoard 异常：
人工检查的预测或回放：

本次证明了什么：
本次没有证明什么：
下一次只修改哪个变量：
```

一次只改变一个主要变量。例如先固定环境与 seed，只比较 PPO 和 SAC；或固定算法，只比较默认杆长与 short-rod 参数。多项同时变化时，无法判断结果由哪一项造成。

---

## 14. 当前实现与后续教程路线

当前代码已经具备：

- `NavigationGymEnv` 的 Dict observation；
- ShapeIntent 4 维动作与 direct 2 维基线；
- progress、alignment、deviation、contact、risk、smoothness 奖励；
- PPO/SAC 训练；
- TensorBoard、Monitor、checkpoint、best/final model；
- SAC replay buffer 保存与续训；
- 独立评估 JSON。

尚需继续落地并在后续教程中覆盖：

1. 把 YAML 配置真正接入训练入口，并增加 dry-run/config preview。
2. 让 `physics_hz` 与 `action_repeat` 生效。
3. 实现 curriculum 的 route 晋级与回滚。
4. 实现可验证的 reset/domain randomization。
5. 补全 buckling、wrong branch、penetration 和 shield intervention cost。
6. 建立 autopilot、direct、PPO、SAC 的统一评估矩阵。
7. 增加多 seed、未见几何和冻结测试集。
8. 建立轨迹记录、行为克隆、恢复学习与 Godot `RL_ASSIST` 回放。

下一篇：[13-人机交互与强化学习一体化平台高质量数据集制作教程.md](13-人机交互与强化学习一体化平台高质量数据集制作教程.md)，先建立可追踪、可复核的数据闭环，再进入行为克隆、DAgger 和离线强化学习。
