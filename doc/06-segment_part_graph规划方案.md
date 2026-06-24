# Segment_part 中心线图规划方案（方案 A）

**目标**：从单条 B 样条中心线（入口→根部）扩展为完整的中心线图（graph.json），支持任意两点 A* 路径规划，适用于 sim2real 的平滑导航。

**主要问题**：当前中心线过于曲折（骨架化→下采样→B样条三阶段堆积误差），不适合真实导丝物理推进。完整的分支拓扑图可以支持选择性导航到特定分支，路径更优化。

---

## I. 整体流程

```
segment_part.stl (源体膜)
    ↓
[Step 1] 体素化 + skeletonize + 拓扑保留
    ↓ 保留完整分支骨架（当前只取端点，现在保存完整骨架图）
[Step 2] 提取分支拓扑关系（邻接表、分支点、端点）
    ↓
[Step 3] 分支聚类与分割
    ↓ 将连续骨架分成 N 条独立分支
[Step 4] 下采样 + B样条平滑（每条分支独立）
    ↓ 降低曲折度，适合 sim2real
[Step 5] 生成 graph.json （MuJoCo 米制，adjacency map + edge weights）
    ↓
[Step 6] 更新 NavigationEngine 以支持图规划
    ↓
graph.json + centerline.json (多条)
    + 客户端 A* 规划任意起终点
    + sim2real 友好的平滑路径
```

---

## II. 详细步骤

### Step 1: 体素化与拓扑保留骨架提取

**输入**：segment_part.stl （16M，180k 面）

**处理**：
```python
# 参数（需根据实测调整）
voxel_pitch = 0.0015  # 1.5mm，足够捕捉细血管
volume = mesh.voxelized(pitch=voxel_pitch).fill()  # 二值化
skeleton = skeletonize(volume)  # scikit-image，保持拓扑
```

**输出**：
- 骨架体素点云（稀疏）
- 拓扑完整（所有分支连通）

**验证**：
- 骨架点数 < 原始顶点数 1000 倍（粗度可接受）
- 视觉检查：所有血管分支都在骨架中

---

### Step 2: 提取分支拓扑与关键点

**输入**：骨架体素点云（NetworkX 图）

**处理**：
```python
# 构建体素图（26-connectivity）
G = nx.Graph()
# 迭代所有体素点，相邻点连边

# 关键点识别
junction_pts = [v for v in G.nodes() if G.degree(v) >= 3]  # 分岔点
endpoint_pts = [v for v in G.nodes() if G.degree(v) == 1]   # 末端
```

**输出**：
- 分岔点坐标（m）：N_junc 个
- 末端坐标（m）：N_end 个（~13 个）
- 邻接关系：dict[node] → [neighbor, ...]

**验证**：
- 分岔点数 >> 末端数（典型血管分支）
- 邻接表连通性：单个连通分量

---

### Step 3: 分支聚类与分割

**目标**：将连续骨架分成 N 条"分支"，每条从一个分岔点/入口出发到某个末端或其他分岔点。

**处理**：
```python
# DFS 从入口开始，沿最长路径走到末端
entry_node = skeleton_node_nearest_to([-0.824, -0.223, 0.263])

branches = []
visited = set()

def dfs_branch(start, parent=None):
    """递归提取以 start 为起点的分支"""
    path = [start]
    current = start
    while True:
        neighbors = [n for n in G[current] if n != parent and n not in visited]
        if not neighbors:
            break  # 到达末端或分岔点
        if len(neighbors) > 1:
            # 分岔点：递归处理各子分支
            for neighbor in neighbors:
                sub_branch = dfs_branch(neighbor, current)
                branches.append(sub_branch)
            break
        else:
            current = neighbors[0]
            path.append(current)
    visited.update(path)
    return path

# 从入口出发
main_branch = dfs_branch(entry_node)
branches.append(main_branch)
```

**输出**：
- `branches`：list[list[node]]，每个分支是体素点序列
- 数量：typically 10~20 条分支

**验证**：
- 所有骨架点都被分配到某个分支（`len(visited) ≈ len(skeleton)`）
- 分支间无重叠（except 分岔点）

---

### Step 4: 下采样 + B样条平滑

**目标**：降低骨架的离散锯齿感，生成光滑、sim2real 友好的路径。

**处理**（每条分支独立）：
```python
for branch_idx, voxel_path in enumerate(branches):
    # 转换到世界坐标
    world_pts = voxel_grid.indices_to_points(np.array(voxel_path))
    
    # 下采样（保留关键点）
    downsampled = []
    for i, pt in enumerate(world_pts):
        if i == 0 or i == len(world_pts) - 1:
            downsampled.append(pt)  # 保留起终点
        elif i % 5 == 0 or np.linalg.norm(pt - downsampled[-1]) > 0.003:
            downsampled.append(pt)  # 每 5 点或 3mm 距离保留一个
    
    # B样条平滑（复用 PathPlanner.smooth_path）
    smooth_result = planner.smooth_path(
        waypoints=[tuple(p) for p in downsampled],
        smoothing_factor=0.25e-6,  # 米制，调参值
        num_points=len(downsampled) * 2,
        degree=3
    )
    
    centerlines[f"branch_{branch_idx}"] = {
        "waypoints": smooth_result["waypoints"],
        "length_m": smooth_result["length_m"],
        "max_curvature": smooth_result["max_curvature"]
    }
```

**输出**：
- `centerlines`：dict，每条分支 → waypoints 数组
- 质量指标：平均曲率、总长、偏离骨架距离

**验证**：
- 光滑性检查：相邻点距离均匀（无尖刺）
- 偏离度检查：B样条与原骨架距离 < 1mm

---

### Step 5: 生成 graph.json

**目标**：按 GraphLoader 的 adjacency map 格式导出，支持 PathPlanner.plan()。

**格式**（参考 VPP data/vpp_assets/case_001/graph/graph.json）：
```json
{
  "x1,y1,z1": [
    ["x2,y2,z2", edge_weight_1],
    ["x3,y3,z3", edge_weight_2]
  ],
  "x2,y2,z2": [
    ["x1,y1,z1", edge_weight_1],
    ...
  ],
  ...
}
```

**处理**：
```python
# 构建节点与边
graph_dict = {}

# 分岔点和末端作为图节点
all_key_points = list(junctions) + list(endpoints)

for branch_idx, (waypoints, metadata) in enumerate(centerlines.items()):
    branch_wpts = [tuple(p) for p in metadata["waypoints"]]
    
    # 分支的起点和终点连接关键点
    start_pt = branch_wpts[0]
    end_pt = branch_wpts[-1]
    
    # 找最近的关键点
    start_key = nearest_junction_or_endpoint(start_pt, all_key_points)
    end_key = nearest_junction_or_endpoint(end_pt, all_key_points)
    
    # 边权（可用距离或其他指标）
    edge_weight = np.linalg.norm(np.array(end_key) - np.array(start_key))
    
    # 添加到邻接表
    if start_key not in graph_dict:
        graph_dict[start_key] = []
    graph_dict[start_key].append([end_key, edge_weight])
    
    if end_key not in graph_dict:
        graph_dict[end_key] = []
    graph_dict[end_key].append([start_key, edge_weight])  # 双向

# 导出为 JSON
output_path = Path("src/cathsim/dm/components/phantom_assets/meshes/segment_part/graph.json")
output_path.write_text(
    json.dumps(graph_dict, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8"
)
```

**输出**：
- `graph.json`：adjacency map，格式与 PathPlanner 兼容
- 节点数：~13（分岔 + 末端）
- 边数：~20（分支连接）

**验证**：
- JSON 可解析，无格式错误
- 所有节点度数 ≥ 1（连通）
- 试用 GraphLoader 加载成功

---

### Step 6: NavigationEngine 集成与 A* 规划支持

**改动**（services/navigation_engine.py）：

```python
# 检测并加载 graph.json
def _load_phantom_graph(self) -> Graph | None:
    """加载内置体膜的中心线图（如果存在）"""
    graph_path = (
        _SRC_DIR / "cathsim/dm/components/phantom_assets/meshes"
        / self.phantom / "graph.json"
    )
    if graph_path.is_file():
        try:
            from services.graph_loader import GraphLoader
            loader = GraphLoader(graph_path)
            return loader.graph
        except Exception:
            return None
    return None

# 在 __init__ 中调用
self._graph = self._load_phantom_graph()

# 支持 start/end 参数进行 A* 规划
def plan_to_target(self, start_pos, end_pos):
    """规划从 start_pos 到 end_pos 的路径"""
    if self._graph is None:
        raise ValueError(f"{self.phantom} 无中心线图")
    
    planner = PathPlanner(graph=self._graph)
    result = planner.plan(
        start=start_pos,
        end=end_pos,
        algorithm="astar",
        smooth=True,
        smooth_factor=0.25e-6
    )
    self.set_planned_path(result.smooth_waypoints or result.waypoints)
    return result
```

**客户端/后端使用**：
```python
# 方式 1：指定具体起终点
engine = NavigationEngine(phantom="segment_part")
result = engine.plan_to_target(
    start_pos=[-0.824, -0.223, 0.263],  # 入口
    end_pos=[-1.028, -0.260, 0.138]      # 根部
)

# 方式 2：通过 WebSocket 后端
# 客户端发送 path_request: {start, end, phantom}
# 后端回复 path_response: {waypoints, length_m, ...}
```

**验证**：
- 不同起终点都能规划成功
- 路径光滑度与原 B 样条一致
- 执行时间 < 100ms（A* + 平滑）

---

## III. 预期产出

| 文件 | 内容 | 用途 |
|------|------|------|
| `graph.json` | 分支邻接表 + 边权 | PathPlanner 输入 |
| 多条 `centerline_branch_*.json` | 各分支 B 样条路径 | 备用 / 直接导航 |
| `segment_part_graph_gen.py` | 完整生成脚本 | 重新计算 / 参数调优 |
| 本文档（doc/06-...） | 方案步骤与验证标准 | 维护与扩展参考 |

---

## IV. 质量验收标准

- [ ] graph.json 能被 GraphLoader 正常加载
- [ ] 所有节点连通（单个连通分量）
- [ ] A* 规划任意两点成功
- [ ] 路径光滑度：相邻点距离均匀，无尖刺（视觉检查）
- [ ] 偏离骨架 < 1.5mm（精度要求）
- [ ] 规划时间 < 100ms
- [ ] sim2real 实测：导丝能沿规划路径平滑推进（需要硬件）

---

## V. 风险与回滚

| 风险 | 缓解措施 |
|------|------|
| 骨架提取失败（拓扑断裂）| 调整 voxel_pitch，重跑；或回退到单条 centerline |
| 分支聚类错误（重叠或遗漏）| DFS 算法验证，visualize 骨架与分支重叠 |
| B样条过平滑（失去血管细节）| 降低 smoothing_factor，增加 num_points |
| graph.json 格式错误 | 用 GraphLoader 单元测试验证 |

---

## VI. 后续优化方向

1. **拓扑识别**：自动标记"优先"分支（粗分支优于细分支），支持临床偏好
2. **多目标**：在多个末端/分岔点间优化（弧长 vs 曲率 vs 安全性）
3. **动态重规划**：若接触压力过高，实时切换到备选路径
4. **医学注释**：关联解剖学名称（如"左冠"、"右冠"）

