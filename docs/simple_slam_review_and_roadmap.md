# simple_slam 代码评审与开发路线

这份文档面向当前 `simple_slam` 的代码状态，重点回答四件事：

1. 前端目前写得怎么样
2. `Submap2D` 的定义现在是否合理
3. 后端图结构当前是否成立
4. 下一步应该先做什么、后做什么

另外补充一个常见设计问题：

- 当前系统使用 `x, y, yaw` 表示二维位姿是否合适
- 是否应该直接统一改成 `4x4` 齐次矩阵

---

## 1. 总体判断

当前 `simple_slam` 已经不是“只有接口的空壳”，而是一条能跑通的局部 SLAM 主链路：

- `LaserScan` 预处理
- 位姿预测
- 帧间激光里程计
- scan-to-submap 匹配
- 关键帧判定
- 子图插入
- node / submap / constraint 登记到 pose graph

也就是说，你现在最有价值的地方不是“再补更多前端小技巧”，而是把现有数据流固化成稳定的系统边界，再把后端真正接起来。

一句话评价：

- 前端：骨架清楚，已经具备原型价值
- `submap`：方向对，但定义还更偏“显示地图”而不是“优化友好”
- 后端图结构：建模是对的，求解器还没落地

---

## 2. 前端评价

涉及的核心文件：

- `src/simple-slam/src/frontend/local_slam_frontend.cpp`
- `src/simple-slam/include/simple_slam/frontend/local_slam_frontend.hpp`
- `src/simple-slam/src/simple_slam_node.cpp`

### 2.1 做得好的地方

当前前端主流程是清楚的：

1. `FilterScan()` 做量测过滤
2. `VoxelFilter()` 做轻量降采样
3. `PredictPose()` 生成初值
4. 无外部里程计时用 `MatchToPreviousScan()` 做激光里程计
5. 用 `MatchToActiveSubmap()` 做 scan-to-submap 匹配
6. 用 `ShouldCreateKeyframe()` 判关键帧
7. 用 `InsertIntoActiveSubmaps()` 插入活动子图

这个顺序本身是合理的，而且你已经把下面几个概念区分开了：

- `predicted_pose`
- `lidar_odom_pose`
- `result.local_pose`
- `last_keyframe_pose_`

这是很好的设计意识。很多早期 SLAM 原型最容易犯的问题就是把“预测位姿”“匹配位姿”“关键帧位姿”混在一起，后面一接后端就会很乱。

### 2.2 当前前端的主要问题

#### 问题 1：`scan` 和 `odom` 没有时间对齐

位置：

- `src/simple-slam/src/simple_slam_node.cpp:218`
- `src/simple-slam/src/simple_slam_node.cpp:221`

当前 `HandleScan()` 直接拿 `latest_odom_` 给前端用。这样做在静态场景或低速情况下也许还能工作，但严格来说这是不可靠的，因为：

- `scan` 时间戳和 `odom` 时间戳可能不一致
- bag 回放时更容易出现“最新消息不是同一时刻消息”
- 快速转弯时，预测初值会明显偏掉

这会直接影响：

- scan-to-scan 初值
- scan-to-submap 搜索中心
- 关键帧轨迹稳定性

这个问题属于高优先级。

#### 问题 2：新 submap 的创建时机不够稳

位置：

- `src/simple-slam/src/frontend/local_slam_frontend.cpp:115`
- `src/simple-slam/src/frontend/local_slam_frontend.cpp:563`
- `src/simple-slam/src/frontend/local_slam_frontend.cpp:584`

当前 `AddScan()` 在匹配前先调用一次：

```cpp
MaybeGrowActiveSubmaps(predicted_pose);
```

这意味着：

- 新子图可能绑定在预测位姿上
- 但真正更可信的是匹配后的 `matched_pose`

虽然插入前又调了一次 `MaybeGrowActiveSubmaps(matched_pose)`，但如果前面已经创建过了，第二次不会修正已有子图初始位姿。

这会把“预测误差”直接写进子图锚点。

#### 问题 3：两个 active submap 已经存在，但匹配只用最后一个

位置：

- `src/simple-slam/src/frontend/local_slam_frontend.cpp:490`

当前 scan-to-submap 匹配只选：

```cpp
const auto& matching_submap = active_submaps_.back();
```

但你实际上已经有了两个重叠活动子图的生命周期设计。这样一来，前面那个“成熟子图”的稳定信息没有被用于匹配，等于生命周期设计和匹配设计没有完全闭环。

更合理的方向是：

- 对两个 active submap 联合打分
- 或者至少选分数更高的那个

#### 问题 4：有效 scan 全部注册成 node，但约束只给关键帧

位置：

- `src/simple-slam/src/simple_slam_node.cpp:229`

当前策略是：

- 每帧有效 scan 都 `AddNode`
- 只有关键帧真正插入子图时才 `AddConstraint`

这会让图里存在很多没有边的 node。

如果当前 `PoseGraph2D` 只是调试容器，这没关系；但一旦进入真正优化，这会带来语义问题：

- 哪些 node 是优化变量
- 哪些只是前端轨迹缓存
- 未连接节点是否参与后端

所以后续最好做二选一：

1. 只对关键帧建图
2. 区分 `all_nodes` 和 `optimization_nodes`

---

## 3. `Submap2D` 评价

涉及文件：

- `src/simple-slam/include/simple_slam/mapping/submap_2d.hpp`
- `src/simple-slam/src/mapping/submap_2d.cpp`

### 3.1 做得好的地方

当前 `Submap2D` 已经不是一个简单数组，而是具备完整系统语义：

- 有 `id`
- 有 `global_pose`
- 有局部地图坐标系
- 有 ray casting 更新
- 有 `ToOccupancyGridData()` 导出
- 有 `GetProbability()` 供 scan matcher 查询

这说明你已经把 `submap` 当作图优化里的一级对象了，而不是临时地图块。这一点很重要，方向是对的。

### 3.2 当前定义的优点

当前版本使用固定大小局部栅格，在工程上有几个明显优点：

- 实现简单
- 坐标关系清楚
- 发布地图方便
- 调试直观
- 很适合作为第一版 2D SLAM 原型

对你现在这个阶段来说，这种设计完全合理，不属于“写错方向”。

### 3.3 当前定义的不足

#### 问题 1：未知栅格和已观测栅格没有在匹配时区分

位置：

- `src/simple-slam/src/mapping/submap_2d.cpp:99`
- `src/simple-slam/src/mapping/submap_2d.cpp:155`
- `src/simple-slam/src/mapping/submap_2d.cpp:198`

当前 `known_cells_` 只在导出 OccupancyGrid 时使用，但 `GetProbability()` 查询概率时没有利用它。

这意味着：

- 未知格内部默认 `log_odds = 0`
- 查询时变成 `p = 0.5`

在 scan matching 里，这通常不是理想行为。因为未知不应该天然给“半命中”的中性奖励，否则评分会偏平，约束会变弱。

更合理的做法是：

- unknown：接近 `0.0` 分或很小权重
- known occupied：高分
- known free：低分甚至负分

#### 问题 2：当前 submap 更偏“占据图展示”，不够“匹配与优化友好”

你现在的 `Submap2D` 很适合：

- 发布局部地图
- 合并全局地图
- 做 RViz 可视化

但如果后面要继续向 Cartographer 风格靠近，`submap` 通常还需要更强的“匹配目标”属性，例如：

- 更明确的概率查询语义
- 已知空闲区对评分的显式惩罚
- 更好的边界处理
- 后续可扩展到预计算匹配缓存

所以当前 `submap` 不是错，而是“第一版地图载体已经成型，但还没长成优化器喜欢的样子”。

#### 问题 3：固定尺寸地图后续会遇到边界问题

当前 `LocalToGrid()` 出界直接失败，`CastRay()` 也会直接跳过整个射线。

这在第一版里完全可以接受，但后面会遇到两个问题：

- 大场景下单个子图可能边缘裁剪明显
- 边缘附近的约束质量不稳定

这个问题优先级没有前两个高，但要在架构上提前知道。

---

## 4. 后端图结构评价

涉及文件：

- `src/simple-slam/include/simple_slam/types.hpp`
- `src/simple-slam/include/simple_slam/optimization/pose_graph_2d.hpp`
- `src/simple-slam/src/optimization/pose_graph_2d.cpp`
- `src/simple-slam/src/backend/pose_graph_backend.cpp`
- `src/simple-slam/src/simple_slam_node.cpp`

### 4.1 图结构本身是对的

当前你采用的是：

- `node`
- `submap`
- `constraint`

三层建模。

这是正确方向，而且是值得保留的核心设计。

你已经把最重要的约束语义写清楚了：

```text
T_map_node = T_map_submap * T_submap_node
```

其中：

- `node` 和 `submap` 是变量
- `constraint.relative_pose` 是测量

这正是标准 2D pose graph 的可扩展建模方式。

### 4.2 当前后端的优点

#### 优点 1：前后端边界已经清楚

前端负责：

- 给出 `result.local_pose`
- 插关键帧
- 决定当前关键帧进入哪些 active submap

图层负责：

- 记录节点
- 登记子图
- 保存 node-submap 约束

后端负责：

- 读取图
- 做优化
- 回写 pose

这种职责划分非常好，后面继续演进不会太痛苦。

#### 优点 2：约束定义已经具备后续扩展性

`Constraint2D` 当前虽然简单，但已经有：

- `node_id`
- `submap_id`
- `relative_pose`
- 权重
- `tag`

这意味着以后你可以继续扩展：

- `kLoopClosure`
- 不同来源约束不同权重
- 回环约束和子图内约束共用同一图结构

### 4.3 当前后端的核心问题

#### 问题 1：图结构正确，但还没有真正“求解”

位置：

- `src/simple-slam/src/backend/pose_graph_backend.cpp:51`

当前 `RunOptimization()` 做的事情是：

- 遍历约束
- 计算预测相对位姿
- 计算误差
- 但不更新 node / submap pose

所以当前后端还不能叫“优化器”，只能叫“图数据检查器”。

#### 问题 2：图里的变量状态还没有拆成“初值 / 优化值”

当前 `TrajectoryNode2D` 里保存的是：

- `local_pose`

这个名字在当前阶段可以工作，但后面会逐渐不够用，因为：

- 前端 pose 是初值
- 后端 pose 是优化值

后面建议明确拆成：

- `initial_pose`
- `optimized_pose`

子图也一样。

#### 问题 3：当前只支持 node-submap 边，尚未进入真正全局一致性阶段

这是正常状态，不算错误，但意味着：

- 现在图的作用主要是“把局部建图结果结构化保存起来”
- 还没有进入“回环修正全局地图”的阶段

所以你接下来的主线一定不是继续堆前端参数，而是把图从“存起来”变成“能求解”。

---

## 5. 下一步先做什么，后做什么

这里给你一个明确的开发顺序。

不要先去写回环检测。当前最优路径是先把局部图优化最小版接通。

### 第一阶段：先把图结构变成真正可优化的图

优先级最高。

#### 先做 1：统一 node 语义

目标：

- 明确 pose graph 里到底存哪些 node

推荐方案：

- 只把关键帧注册成图节点

这样会立刻带来三个好处：

- 图更干净
- 约束更自然
- 后端实现更简单

#### 先做 2：修正 submap 匹配评分语义

目标：

- 不让 unknown cell 在匹配里提供虚假的“0.5 概率奖励”

建议：

- `unknown` 给 0 分
- `known occupied` 加分
- `known free` 降分

这是前端稳定性提升里性价比很高的一步。

#### 先做 3：修正新 submap 创建时机

目标：

- 新子图使用 `matched_pose` 作为初始锚点

建议：

- 不要在匹配前创建新 submap
- 或者创建后在首次真正插入时完成锚定

#### 先做 4：让 scan-to-submap 支持双 active submap

目标：

- 让重叠子图设计真正参与前端匹配

推荐做法：

- 对两个 active submap 联合打分

---

### 第二阶段：做“最小可用后端”

当前最值得做的后端最小版是：

- 固定 node pose
- 只优化 submap pose

为什么这样做：

- 实现最简单
- 可以快速验证 constraint 链路是否正确
- 可以先把子图之间的漂移关系拉平

这个阶段建议做下面几步：

#### 后做 5：给 node 和 submap 增加“初值 / 优化值”分离

目标：

- 不再用一个 pose 字段同时承担前端值和后端值

#### 后做 6：在后端里最小化约束残差并回写 submap pose

目标：

- 让 `RunOptimization()` 真正改图

哪怕第一版只是简单迭代法、Gauss-Newton、小规模 Ceres，都比现在“只算误差不更新”前进一步。

#### 后做 7：地图发布改为使用优化后 submap pose

目标：

- 让 `/global_map` 真正体现后端修正结果

---

### 第三阶段：再进入回环

只有前两阶段稳定之后，才建议开始做：

- finished submap 候选检索
- node-to-submap 或 submap-to-submap 回环约束
- 全局优化

不建议现在就做回环，原因很简单：

- 如果当前图变量定义还没稳定
- 如果局部后端都没先跑通

那回环一接进来，问题会很难排查。

---

## 6. `x, y, yaw` 会不会有问题

短答案：

- 对你当前这个 2D SLAM，`x, y, yaw` 是正确选择
- 不建议现在把内部统一改成 `4x4` 矩阵

### 6.1 为什么 `x, y, yaw` 在 2D SLAM 里是合理的

你当前系统是严格的 2D 场景：

- 地图是二维占据栅格
- 激光点是二维点
- 优化变量是平面位姿
- 约束也是二维相对位姿

所以系统的自由度本来就是 3 个：

- `x`
- `y`
- `yaw`

这时候直接用：

```cpp
struct Pose2D {
  double x;
  double y;
  double yaw;
};
```

优点很多：

- 语义直接
- 内存小
- 计算便宜
- 调试方便
- 优化器残差定义简单
- 不容易把 2D 问题做成伪 3D 问题

### 6.2 为什么不建议现在统一换成 `4x4`

`4x4` 齐次矩阵更适合：

- 3D 变换链
- 点云库接口
- 通用图形学/机器人变换统一表示

但在纯 2D SLAM 里，把所有内部状态都换成 `4x4` 往往会带来这些问题：

- 信息冗余
- 数值上更容易漂离正交约束
- 调试时不直观
- 你明明只优化 3 个自由度，却存成 16 个数
- 后端残差和雅可比写起来反而更绕

所以对你现在这个工程，不建议把“状态表示”改成 `4x4`。

### 6.3 更好的工程做法是什么

建议采用下面这种分层方式：

#### 内部核心状态

继续使用：

- `Pose2D {x, y, yaw}`

用于：

- node pose
- submap pose
- constraint relative pose
- 前端预测和匹配结果

#### 与库交互时

按需转换为：

- `Eigen::Matrix3d`
- `Eigen::Isometry2d`
- 或必要时 `Eigen::Matrix4f`

例如：

- PCL ICP 接口时可以临时构造 `4x4`
- 但系统里的真值状态仍保持 `Pose2D`

这才是最适合你当前项目的做法。

### 6.4 如果以后升级到 3D 怎么办

如果以后你要做 3D SLAM，再考虑统一升级状态表示：

- 平移 `x, y, z`
- 旋转四元数 / SO(3) / 旋转矩阵
- 必要时再用 `4x4` 齐次变换做传输和接口

但那是 3D 系统的设计问题，不应该反向影响当前 2D 系统。

---

## 7. 最终建议

如果只给一个最核心判断，那就是：

当前 `simple_slam` 最值得继续投入的主线不是“再补一点前端小功能”，而是尽快把下面四件事做扎实：

1. 关键帧建图语义统一
2. `submap` 匹配评分语义修正
3. 新子图创建时机修正
4. 最小可用后端优化接通

建议执行顺序：

1. 先修 `submap` 打分和新子图创建时机
2. 再统一 pose graph 中 node 的语义
3. 然后实现“固定 node、优化 submap”的第一版后端
4. 最后再考虑 loop closure

---

## 8. 当前测试结论

本次检查时，`simple_slam` 的单元测试可通过：

- `test_local_slam_frontend`
- `test_ceres_scan_matcher_2d`

`colcon test --packages-select simple_slam` 的结果里：

- 功能测试通过
- `cpplint` 失败

当前失败主要是格式问题，不是功能问题：

- 尾随空格
- 注释空格格式

所以目前真正需要优先解决的，不是 lint，而是上面列出的架构与数据流问题。
