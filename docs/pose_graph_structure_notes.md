# Pose Graph 结构说明

这份文档专门解释 `simple_slam` 里为什么同时需要：

- `nodes`
- `submaps`
- `constraints`

以及它们之间到底是什么关系。

配套图示见：

- `docs/pose_graph_structure_diagram.svg`
- `docs/pose_graph_structure_diagram.md`

## 1. 先看整张图长什么样

在 `simple_slam` 当前这条设计路线里，位姿图不是“只有一串轨迹点”，而是一个二部图：

```text
submap_7 -------- node_42
    |               |
    |               |
    |----------- node_43
    |
submap_8 -------- node_43
    |
    |----------- node_44
```

你可以把它理解成：

- `node`
  表示一个关键帧
- `submap`
  表示一个局部子图
- 连线
  表示这个关键帧和这个子图之间存在一个位姿约束

所以这张图里：

- 点有两类：`node` 和 `submap`
- 边只有一类：`constraint`

## 2. 三种对象分别保存什么

### 2.1 `nodes`

`nodes` 保存的是关键帧节点。

在当前代码里对应：

- `TrajectoryNode2D`
- `PoseGraph2D::nodes_`

它表示：

```text
T_map_node
```

也就是：

某个关键帧节点在 `map` 坐标系下的位姿。

可以把它近似理解成：

- 当前关键帧对应的机器人位姿
- 或当前关键帧对应的 scan 位姿

### 2.2 `submaps`

`submaps` 保存的是局部子图。

在当前代码里对应：

- `Submap2D`
- `PoseGraph2D::submaps_`

每个 `submap` 也有自己的全局位姿：

```text
T_map_submap
```

也就是：

这个子图坐标系在 `map` 坐标系下的位置和朝向。

### 2.3 `constraints`

`constraints` 保存的是节点和子图之间的相对位姿测量。

在当前代码里对应：

- `Constraint2D`
- `PoseGraph2D::constraints_`

它里面最关键的量是：

```text
relative_pose = T_submap_node
```

也就是：

当前节点在对应子图坐标系下的相对位姿。

## 3. 为什么必须有三层

如果系统里只有：

- `T_map_node`
- `T_map_submap`

那你只是保存了“各自在哪里”，并没有保存“它们之间应该满足什么关系”。

而后端优化最需要的其实是：

```text
node 和 submap 之间的相对关系
```

也就是约束。

所以：

- `nodes` 和 `submaps`
  是优化变量
- `constraints`
  是测量关系

这就是为什么不能只有两层，必须有三层。

## 4. 一条约束到底表示什么

假设当前关键帧 `node_42` 被插入到了 `submap_7`。

那么系统里就应该保存一条约束：

```text
constraint(node_42, submap_7)
```

它可以画成：

```text
submap_7 ----[ T_submap7_node42 ]---- node_42
```

这里的边上写的：

```text
T_submap7_node42
```

就是这条约束的测量值。

它表示：

从 `submap_7` 坐标系看，`node_42` 应该在什么位置。

## 5. 当前系统里的三个位姿量

现在最容易混的就是下面这三个：

### 5.1 `T_map_node`

这是当前节点在 `map` 下的位姿。

在代码里大致对应：

- `result.local_pose`
- `TrajectoryNode2D.local_pose`

### 5.2 `T_map_submap`

这是当前子图在 `map` 下的位姿。

在代码里对应：

- `Submap2D::global_pose()`

### 5.3 `T_submap_node`

这是当前节点在子图坐标系下的相对位姿。

在代码里对应：

- `Constraint2D::relative_pose`

这三个量之间满足：

```text
T_map_node = T_map_submap * T_submap_node
```

反过来也可以写成：

```text
T_submap_node = (T_map_submap)^-1 * T_map_node
```

这正是当前代码里：

```cpp
RelativePose(submap_pose, node_pose)
```

在做的事情。

## 6. 为什么 `MatchToActiveSubmap()` 返回的不是 `T_submap_node`

当前前端里有这样一段逻辑：

```cpp
result.local_pose = MatchToActiveSubmap(result.range_data, lidar_odom_pose);
```

虽然函数名字里有 `submap`，但它返回的不是：

```text
T_submap_node
```

而是：

```text
T_map_node
```

原因是：

前端做的是：

1. 给当前帧一个全局预测位姿
2. 拿 active submap 当参考地图
3. 在预测位姿附近搜索
4. 找到一个更好的“当前帧全局位姿”

所以：

- `submap` 是匹配参考对象
- 返回值仍然是 `map` 下的节点位姿

真正的：

```text
T_submap_node
```

是在后面建约束时，单独算出来的。

## 7. 后端优化到底优化什么

后端不是只优化 `submap`，也不是只优化 `node`。

后端优化的是：

- `T_map_node`
- `T_map_submap`

而 `Constraint2D::relative_pose` 是固定测量：

```text
T_submap_node(measured)
```

后端做的事情可以概括成：

```text
让优化后的 RelativePose(T_map_submap, T_map_node)
尽量接近 constraint.relative_pose
```

也就是说：

- `node` 和 `submap` 都是变量
- `constraint` 是观测

## 8. 为什么 active submap 还没 finished 也要进 pose graph

很多人第一次做这里都会疑惑：

“submap 不是还在长吗，为什么现在就要登记进 pose graph？”

原因是：

一旦当前关键帧被插入某个 active submap，
这个 keyframe 和这个 submap 之间的关系就已经存在了。

也就是说：

```text
node ---- constraint ---- submap
```

这条边从那一刻就成立，不需要等 submap finished。

`finished` 主要影响的是：

- 是否继续插入
- 是否更适合做回环候选

而不是：

- 能不能进入 pose graph

## 9. 对照当前代码怎么理解

你现在这套代码链路可以这样看：

### 前端

负责：

- 处理 scan
- 估计 `result.local_pose`
- 判断关键帧
- 插入 active submaps
- 返回 `insertion_submap_ids`

### 节点层

负责：

- `pose_graph_->AddNode(result)`
- `pose_graph_->RegisterSubmaps(...)`
- 根据 `insertion_submap_ids` 构造 `Constraint2D`

### 后端

负责：

- 读取 `nodes`
- 读取 `submaps`
- 读取 `constraints`
- 后续进行优化

## 10. 最后一张总图

把上面全部合起来，就是这样：

```text
                 Pose Graph

       T_map_submap7                 T_map_node42
           submap_7  ------------------- node_42
               |         constraint:
               |         T_submap7_node42
               |
               |
               |         constraint:
               |         T_submap7_node43
               |
               ------------------------- node_43
                                         T_map_node43


       T_map_submap8
           submap_8  ------------------- node_43
               |         constraint:
               |         T_submap8_node43
               |
               ------------------------- node_44
                                         T_map_node44
```

这里可以一眼看出：

- `submap_7` 和 `submap_8` 是图中的一类顶点
- `node_42`、`node_43`、`node_44` 是另一类顶点
- 每条边保存的是一个 `T_submap_node`

而后端做的事就是：

让这些顶点的位置整体调整之后，仍然尽量满足这些边给出的相对关系。

## 11. 一句话总结

`nodes / submaps / constraints` 的分工是：

- `nodes`
  存关键帧的全局位姿变量
- `submaps`
  存子图的全局位姿变量
- `constraints`
  存 node 和 submap 之间的相对位姿测量

这三者合起来，才是一张真正可优化的 pose graph。
