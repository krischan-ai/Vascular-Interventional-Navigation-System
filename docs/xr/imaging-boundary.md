# DSA 与腔镜真实性边界

> M0 冻结版：2026-08-03。两路影像状态独立，禁止用一个 `CONNECTED` 概括。

## DSA

第一版仅使用固定演示图/占位图，界面必须同时显示：

```text
DSA
占位图
数据源：Demo Image
实时状态：未接入
```

禁止显示 `LIVE`、帧率、网络延迟或任何会让用户误认为实时造影的数据。

| 工具 | 第一版状态 |
|---|---|
| 放大/平移/复位 | 可选，仅操作占位图 |
| 测量 | 禁用 |
| 旋转 | 禁用或隐藏 |
| 截图 | 禁用；如保留，只能标为界面截图 |
| 窗宽窗位 | 隐藏 |

本地状态固定为 `dsa_source_mode=placeholder`、`status=unavailable_for_live`，不写入 `navigation_visual_v3` 伪装后端数据。

## 腔镜

腔镜是 Godot 三维血管场景的本地动态相机，不是外部视频：

```text
Vessel 3D → EndoscopeCamera3D → SubViewport → XR 影像面板
```

界面可显示“腔镜 / 三维相机视角 / 本地实时渲染”，不得显示伪网络延迟。状态独立为：

- `source=local_3d_camera`；
- `status=live_rendered / invalid_camera / paused`；
- 摄像机定位、渲染档位、实际分辨率和刷新率。

## 摄像机与性能试验

| 参数 | 初始值 | 允许范围 |
|---|---:|---:|
| back offset | 12 mm | 8–15 mm |
| vertical offset | 4 mm | 3–6 mm |
| lateral offset | 3 mm | 2–5 mm |
| FOV | 79° | 70°–90° |
| near plane | 0.5 mm | 尽可能小且稳定 |
| 动态 SubViewport | 1 个 | 不得按布局复制 |
| 分辨率长边 | 768 | 640 / 768 / 1024 |
| 刷新率 | 45 Hz | 30 / 45 / 60 Hz |

当前桌面代码使用单一 `UPDATE_ALWAYS` SubViewport、0.5mm near plane 和 78°–85° 质量档。XR 必须通过节流试验实现 30/45/60Hz，不得在未实现前伪报。

## 布局

- 默认上下双画面：DSA 56%–60%，腔镜 40%–44%。
- DSA 主视图：DSA 80%，腔镜画中画且仍可见。
- 腔镜主视图：腔镜 80%，DSA 画中画且仍明确为占位。
- 放大任一路不得遮挡急停、整体控制门控与另一画面的来源/状态标签。

## 无效与降级

- 无有效 tip/相机定位：腔镜显示 `invalid_camera`，不保留旧帧为 LIVE。
- XR Session 非可渲染态：腔镜 `paused`，控制门控独立归零。
- 性能不足：先降到 30Hz/640、降低 MSAA/材质，再保持单视口；不得删除真实性标签。
- DSA 资源缺失：灰色占位，不生成模拟血管图冒充数据。

## 验收

腔镜需证明导丝可见但不遮挡中央、前方分叉可辨、相机不穿壁、曲线处无剧烈 Roll、路径重规划平滑；所有性能结论以 PICO 真机为准。
