# Submap Position Issue Notes

日期: 2026-04-08

## 背景

在 `ros_simulation` 中使用以下命令回放 bag 运行 `simple_slam` 时，观察到子图位置显示异常，怀疑“子图位置有问题”：

```bash
ros2 launch bringup main.launch.py \
  simulator:=none \
  play_bag:=true \
  slam_system:=simple_slam \
  start_stage:=false \
  start_rviz:=false \
  rviz_config:=/home/zwc/ros_simulation/install/simple_slam/share/simple_slam/rviz/simple_slam.rviz \
  bag_file:=/home/zwc/ros_simulation/bag/2026-04-01-12-48-23_ros2 \
  simple_slam_config_file:=/home/zwc/ros_simulation/install/simple_slam/share/simple_slam/config/simple_slam_bag_laser_link.yaml
```

## 先看了哪些提交

在 `src/simple-slam` 中检查了最近几次提交：

```text
010ed6a add intra-submap constraints to pose graph
4152cb3 refactor submap indexing into local frame
7302a56 feat: expand local slam frontend and submap outputs
```

其中最关键的是：

- `4152cb3 refactor submap indexing into local frame`
- 这次提交把 `Submap2D` 从“世界坐标轴对齐子图”改成了“拥有局部坐标系、可带 yaw 的子图”

而后续发布和地图合并逻辑没有同步完成旋转子图适配。

## 问题根因

### 1. `Submap2D` 已经支持旋转

`Submap2D` 在当前实现中保存的是：

- `global_pose_`
- `local_map_origin_`

并通过：

- `WorldToLocal()`
- `LocalToWorld()`

完成世界坐标与子图局部坐标之间的变换。

这意味着子图已经不是简单的“世界坐标里一块轴对齐矩形栅格”，而是“局部栅格 + 世界位姿”。

### 2. `active_submap` 发布仍然按老逻辑处理

在修复前，`PublishActiveSubmap()` 里：

- `origin.position` 使用左下角世界坐标
- 但 `origin.orientation` 固定为单位四元数

这样 RViz 会把子图当成“无旋转的 OccupancyGrid”显示。

结果：

- 只要子图创建时 `yaw != 0`
- 子图在 RViz 中的位置和朝向就会不对

### 3. `local_map/global_map` 合并也仍然按老逻辑处理

在修复前，`BuildMergedMap()` 的逻辑是：

- 取每个子图左下角 `lower_left`
- 直接用 `offset_x/offset_y` 把整张子图贴到全局栅格

这个做法只在“子图不旋转”时成立。

一旦子图有旋转，正确做法应该是：

- 先计算子图每个栅格中心在世界坐标中的位置
- 再把这个世界坐标投影到全局合并地图的栅格

否则会出现：

- 子图边界框不准确
- 合并地图位置偏移
- 旋转后的子图被错误地当作未旋转图块平移拼接

## 这次做了哪些修改

### 修改 1: 给 `Submap2D` 增加几何辅助接口

文件：

- `include/simple_slam/mapping/submap_2d.hpp`
- `src/mapping/submap_2d.cpp`

新增接口：

- `GetCellCenterInWorld(int cell_x, int cell_y)`
- `GetWorldCorners()`

作用：

- 用于得到子图某个栅格中心的世界坐标
- 用于得到旋转后子图四个角点的世界坐标

### 修改 2: 修正 `active_submap` 的姿态发布

文件：

- `src/simple_slam_node.cpp`

在 `PublishActiveSubmap()` 中：

- 保留 `origin.position = submap 左下角世界坐标`
- 将 `origin.orientation` 改为 `submap->global_pose().yaw`

这样 RViz 在显示单张活动子图时，会按真实子图姿态旋转显示。

### 修改 3: 修正多子图合并逻辑

文件：

- `src/simple_slam_node.cpp`

在 `BuildMergedMap()` 中：

- 全局边界不再通过 `lower_left + width/height` 推导
- 改为遍历 `GetWorldCorners()` 的四个角点来构建包围盒

在写入全局地图时：

- 不再使用整张子图统一 `offset_x/offset_y` 平移
- 改为对每个有效栅格：
  - 计算其世界坐标
  - 再投到全局 merged map 的栅格索引

这样 `local_map` 和 `global_map` 可以正确接纳旋转子图。

## 实际验证

做了两类验证：

### 1. 编译验证

执行：

```bash
colcon build --packages-select simple_slam
```

结果：

- 编译通过

### 2. 启动命令复现

使用你提供的命令实际启动。

由于当前环境中 ROS 2 默认日志目录 `~/.ros/log` 不可写，运行时额外设置了：

```bash
export ROS_LOG_DIR=/tmp/ros_logs
```

然后成功启动：

- `simple_slam_node`
- `ros2 bag play`

并看到前端持续处理 scan 的日志输出。

说明：

- 修改没有破坏当前 bag 回放链路
- 节点和话题发布逻辑仍可正常运行

## 这次没有动的内容

本次没有继续深入修改下面这些部分：

- `pose_graph` 的真实后端优化
- `backend_->RunOptimization()` 的约束求解
- `Submap2D::SetGlobalPose()` 的优化后回写链路

也就是说，这次修复只针对：

- 子图显示位置
- 子图姿态发布
- 旋转子图的地图合并

不涉及真正的后端位姿图优化。

## 相关文件

本次实际改动涉及：

- `include/simple_slam/mapping/submap_2d.hpp`
- `src/mapping/submap_2d.cpp`
- `src/simple_slam_node.cpp`

当前工作区中还保留了你自己的未提交修改：

- `include/simple_slam/backend/pose_graph_backend.hpp`
- `src/backend/pose_graph_backend.cpp`
- `src/simple_slam_node.cpp`

因此如果后续继续整理提交，建议把“显示修复”和“pose graph 试验代码”分开提交，便于回溯。

## 建议

建议下一步在本机开 RViz 再观察以下话题：

- `/active_submap`
- `/local_map`
- `/global_map`

重点确认：

- 单张活动子图是否随 yaw 正确旋转
- 相邻子图是否仍然能平滑衔接
- 合并图是否不再出现整体偏移或错位

如果后面要继续接后端优化，建议下一步把“优化后的 `submap global pose` 回写 + map 重新发布”这条链补完整。
