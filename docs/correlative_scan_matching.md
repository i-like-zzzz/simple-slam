# simple_slam 中的 Correlative 匹配说明

这份文档只解释 `LocalSlamFrontend::MatchToPreviousScanCorrelative()` 当前这版代码到底怎么工作，不把它写成“通用扫描匹配综述”。

对应实现主要在：

- `src/frontend/local_slam_frontend.cpp`
- `MatchToPreviousScan()`
- `MatchToPreviousScanCorrelative()`
- `ScoreScanToScanCandidate()`

## 1. 这个匹配在系统里处于哪一层

`simple_slam` 里这段 `correlative` 代码属于“相邻两帧激光之间的里程计匹配”，不是 scan-to-submap，也不是后端优化。

当前链路是：

1. 新激光帧进入前端
2. 先做滤波和降采样
3. 用 `/odom` 或上一帧激光里程计结果，给出一个初始相对位姿 `initial_relative_pose`
4. 在当前帧点云和上一帧点云之间做匹配
5. 得到本帧相对上一帧的运动增量

当 `lidar_odom_matcher` 配成 `correlative` 时，步骤 4 走的是离散搜索，不是 ICP。

## 2. 入口是怎么切进去的

`MatchToPreviousScan()` 会先把当前帧和上一帧点云做降采样，然后按 `lidar_odom_matcher` 分发：

- `point_to_point_icp`
- `generalized_icp`
- `correlative`

切到 `correlative` 后，会调用 `MatchToPreviousScanCorrelative(current_points, previous_points, initial_relative_pose)`。

这里的三个输入含义分别是：

- `current_points`
  当前帧激光点
- `previous_points`
  上一帧激光点
- `initial_relative_pose`
  一个先验相对位姿，通常来自 `/odom` 预测，或者来自上一帧激光里程计增量

这说明 `correlative` 不是“从全局零开始搜”，而是围绕先验解附近做局部搜索。

## 3. 核心思想

这套方法可以概括成一句话：

在预测位姿附近枚举一批候选位姿，对每个候选位姿计算“当前帧和上一帧贴得有多好”，最后选分数最高的那个。

它不是通过求导、线性化、迭代最小二乘来优化，而是：

1. 先定义一个搜索窗口
2. 在窗口内按固定步长枚举候选解
3. 给每个候选解打分
4. 选最大分数对应的位姿

所以它更接近“离散搜索 + 打分”，而不是 Ceres 那种连续变量非线性优化。

## 4. 搜索是怎么做的

代码会先检查两个前提：

- 平移和旋转搜索窗口必须大于 0
- 搜索步长必须大于 0

然后把初始位姿本身当作当前最优解：

- `best_pose = initial_relative_pose`
- `best_score = ScoreScanToScanCandidate(...)`

接着做三重循环，分别搜索：

- `yaw_delta`
  从 `-lidar_odom_angular_window` 到 `+lidar_odom_angular_window`
- `dx`
  从 `-lidar_odom_linear_window` 到 `+lidar_odom_linear_window`
- `dy`
  从 `-lidar_odom_linear_window` 到 `+lidar_odom_linear_window`

每个候选位姿都在 `initial_relative_pose` 基础上加一个扰动：

```text
candidate.x   = initial.x   + dx
candidate.y   = initial.y   + dy
candidate.yaw = initial.yaw + yaw_delta
```

然后算分，谁分数更高就更新 `best_pose`。

这意味着搜索空间是一个局部 3D 网格：

- x 平移
- y 平移
- yaw 旋转

它的优点是直接、稳定，不依赖梯度。
它的缺点也很明确：窗口越大、步长越细，计算量越高。

## 5. 打分函数到底在算什么

打分函数在 `ScoreScanToScanCandidate()`。

可以把它拆成两部分：

`总分 = 点云对齐分数 - 偏离先验的惩罚`

### 5.1 点云对齐分数

对当前帧的每一个点，先用候选位姿把它变换到上一帧坐标系下：

```text
point_in_previous = TransformPoint(point_in_current, candidate_relative_pose)
```

然后在 `previous_points` 里找最近点，得到最近距离平方 `min_distance_sq`。

这个点的得分是：

```text
exp(-0.5 * min_distance_sq / sigma^2)
```

这个式子的含义很直接：

- 距离很小，分数接近 1
- 距离变大，分数按高斯形式衰减
- 没有硬阈值截断，而是“越近越高”

所有点的分数累加后，再除以当前点数，得到平均匹配分数。

所以这部分本质上是在问：

“如果我相信这个候选位姿，那么当前帧的点变换过去后，是否普遍贴近上一帧点云？”

### 5.2 先验惩罚项

如果只看最近点距离，搜索结果可能会因为局部重复结构而跑到一个虽然能对上、但偏离预测太多的位置。

所以代码又加了两项惩罚：

- 平移惩罚
  候选位姿与初始位姿之间的平移差
- 旋转惩罚
  候选位姿与初始位姿之间的角度差

最终返回：

```text
score
- lidar_odom_translation_weight * translation_penalty
- lidar_odom_rotation_weight * rotation_penalty
```

这等价于在说：

- 候选解要尽量让两帧点云对得上
- 但也不能无约束地偏离里程计预测

因此它不是单纯靠点云几何，也显式融合了运动先验。

## 6. 它和 ICP 的区别

虽然 `correlative`、ICP、GICP 都是在做两帧配准，但求解方式完全不同。

### `correlative`

- 在固定窗口内枚举候选位姿
- 每个候选位姿都独立打分
- 最后直接选最高分
- 不求导，不线性化，不做迭代最小二乘

### `point_to_point_icp` / `generalized_icp`

- 先根据当前估计建立点对应
- 再通过连续优化更新位姿
- 反复迭代直到收敛或达到迭代上限
- 依赖一个足够接近的初始值

工程上可以这么理解：

- `correlative` 更像“粗搜索”
- ICP / GICP 更像“连续精配准”

## 7. 它和 Ceres 非线性优化的关系

严格说，这里不是 Ceres 风格的非线性优化。

原因有三点：

1. 没有把位姿当成连续变量去求最优
2. 没有定义残差块和雅可比
3. 没有用 Gauss-Newton、LM 这类方法迭代更新

但它们也不是完全没关系。

从“目标”看，二者都在寻找一个最优位姿，使“匹配更好、偏离先验更小”。

区别在“怎么求”：

- `correlative`
  靠枚举和评分找最优离散格点
- Ceres
  靠连续优化在参数空间里迭代逼近局部最优

所以更准确的说法是：

`correlative` 有一个目标函数，但它的求解器不是 Ceres 式非线性最小二乘，而是离散网格搜索。

## 8. 这版实现的代价和局限

这段代码能工作，但要清楚它是“先跑通链路”的工程实现，不是高性能版本。

当前实现的代价主要有：

- 每个候选位姿都要遍历当前帧所有点
- 每个点又要线性扫描上一帧所有点找最近点
- 总复杂度会随搜索窗口和点数快速上涨

也就是说，它现在没有做：

- KD-tree 最近邻加速
- 分层搜索
- 预计算概率栅格
- branch-and-bound
- Ceres 精配准收敛

这也是为什么配置里默认还是 `point_to_point_icp`，而 `correlative` 更像一个对初值更不敏感的备选方案。

## 9. 关键参数如何影响结果

和这段匹配直接相关的参数有：

- `lidar_odom_linear_window`
  平移搜索范围。太小可能搜不到真值，太大计算量会上升。
- `lidar_odom_angular_window`
  角度搜索范围。道理相同。
- `scan_matcher.linear_step`
  平移搜索步长。越小越细，越慢。
- `scan_matcher.angular_step`
  旋转搜索步长。越小越细，越慢。
- `lidar_odom_point_sigma`
  控制距离分数衰减速度。越小越苛刻，越大越宽松。
- `lidar_odom_translation_weight`
  偏离预测平移越远，罚得越重。
- `lidar_odom_rotation_weight`
  偏离预测角度越远，罚得越重。

一个实用判断是：

- 如果匹配老是贴着预测解不动，通常是惩罚权重过大或者搜索窗口太小
- 如果匹配经常跳到奇怪位置，通常是搜索窗口太大、权重太小，或者场景里重复结构太强

## 10. 用一句话总结

`simple_slam` 里的 `correlative` 匹配，本质上是在预测位姿附近做一个三维离散搜索，用“点到上一帧最近点的高斯相似度”加上“偏离先验的惩罚”作为评分，最后取最高分位姿。

它可以理解成一种简单直接的局部 scan-to-scan 搜索器，不是 Ceres 式非线性优化，也不是 ICP 的迭代收敛路线。
