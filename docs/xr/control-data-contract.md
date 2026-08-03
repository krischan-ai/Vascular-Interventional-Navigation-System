# XR 状态、控制、时钟与 Freshness 契约

> M0 冻结版：2026-08-03。安全优先；后端未兼容目标控制字段前，XR 非零控制保持禁用。

## 权威与时钟

- `navigation_visual_v3` 是导航显示契约，`data.safety` 是安全唯一权威。
- `timestamp_ms` 是后端 Unix ms，用于审计、排序与旧 Session 过滤。
- Freshness 使用客户端收到有效帧时的单调时钟计算，避免系统时钟跳变。
- 缺失字段保持 `null / unknown / stale`，禁止填假 0；空 `risk_regions` 保持空。

| 状态 | 最后有效帧年龄 | UI | 控制 |
|---|---:|---|---|
| Fresh | < 200 ms | 正常 | 继续检查全部门控 |
| Delayed | 200–499 ms | 黄色、显示年龄 | 阻断并归零 |
| Stale | ≥ 500 ms | 灰色过期、路径灰化 | 阻断并归零 |
| Disconnected | 无有效连接 | 重连提示 | 阻断并归零 |

Delayed 首版采用“阻断”而非限速。阈值修改必须同时更新验收用例与真机证据。

## 全部门控

仅当以下条件全部成立才允许非零值进入 ControlRouter：

1. XR Session 为 `FOCUSED`；
2. Actions active；
3. 必要控制器跟踪有效；
4. Deadman 当前按住；
5. WebSocket ready；
6. 后端 Session ready；
7. 数据为 Fresh；
8. 后端急停未锁存；
9. 无本地控制故障；
10. Reference Space 有效且未发生待确认重定位。

任一条件失效，同一客户端更新周期设置 `push=0`、`rotate=0`、`controls_blocked=true`。恢复后必须释放并重新按下 Deadman，不复用旧摇杆值。

## 目标控制帧

```json
{
  "control_source": "pico_openxr",
  "input_sequence": 42,
  "client_timestamp_ms": 1785753600123,
  "deadman_active": true,
  "push": 0.25,
  "rotate": -0.10
}
```

| 字段 | 规则 |
|---|---|
| `control_source` | 固定枚举；首版为 `pico_openxr` |
| `input_sequence` | Session 内单调递增；重连新 Session 归零 |
| `client_timestamp_ms` | Unix ms，仅审计，不代替单调 Freshness |
| `deadman_active` | 必须显式传递，不由非零轴推断 |
| `push` / `rotate` | 门控后的实际发送值，范围 `[-1,1]` |

当前后端只接受 `delta_push`/`delta_rotate`。M3 兼容期由 ControlRouter 映射字段；在后端完成 source/sequence/timestamp/deadman 校验前，XR 非零发送保持阻断。

## 输入与发送参数

| 参数 | 冻结值 |
|---|---:|
| 摇杆死区 | 0.08，越界后重映射 |
| 轴限幅 | [-1.0, 1.0] |
| 连续发送频率 | 20 Hz |
| 客户端门控周期 | 每 XR 帧 |
| 后端控制看门狗 | 150 ms；超时必须中性锁定 |
| 在途命令 | 最多 1 条；沿用桌面 lock-step |

后端看门狗当前尚未实现，是进入 M7 非零闭环的 P0 阻断。

## 急停与恢复

- 急停输入先在客户端阻断，再发送 `emergency_stop`。
- 收到 `emergency_stop_confirmed` 后显示“后端已锁存”。
- `resume` 只发请求；收到 `resume_confirmed` 且锁存为 false 后仍保持中性。
- 用户必须重新按 Deadman，全部门控重新成立后才可控制。
- 断线、失焦、Action inactive、追踪丢失、Reference Space 变化、Delayed/Stale 均走同周期归零。

## Session 与顺序

- 只接受当前后端 Session ID 的帧；旧 Session 帧丢弃。
- 同一 Session 中拒绝时间戳/序号倒退的控制确认。
- 重连创建新 WebSocketPeer、新 Session 和新 sequence 域。
- 网络线程只写共享状态；UI 和控制每帧消费同一不可变快照。
