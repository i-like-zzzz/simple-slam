# simple_slam

面向 ROS 2 的渐进式 2D SLAM 系统。

这个包的目标不是一次性复制 Cartographer 的全部能力，而是按工程化顺序逐步构建一套可演进的 2D SLAM 系统：

1. 数据流和模块边界
2. 2D 激光局部前端
3. 子图构建
4. 回环检测与位姿图优化
5. 地图发布与调试工具

当前代码状态已经完成了前 3 步里的第一版局部实现：

- `simple_slam_node`
  负责 ROS 2 接口、话题订阅发布、TF 和调试输出。
- `LocalSlamFrontend`
  已具备 scan 预处理、位姿预测、帧间激光里程计、scan-to-submap 匹配、关键帧判定。
- `Submap2D`
  已具备固定大小局部占据栅格、活动子图生命周期和关键帧插入。
- `PoseGraph2D`
  当前还是节点和子图容器，尚未接约束。
- `PoseGraphBackend`
  当前只保留接口，尚未实现约束构建、优化和回环。

换句话说，当前系统已经不是“只有接口”，而是一套可运行的局部前端 SLAM 原型；接下来的主线工作是：

1. 重构子图表示与子图管理
2. 定义节点-子图约束
3. 接后端优化
4. 再做回环检测和全局一致性

## 文档

- `docs/config_reference.md`
  当前参数说明。
- `docs/development_notes.md`
  当前实现边界、已完成能力和下一阶段工程判断。
- `docs/correlative_scan_matching.md`
  解释 `lidar_odom_matcher=correlative` 在 `simple_slam` 里的匹配流程、打分原理，以及它和 ICP / Ceres 的区别。
- `docs/submap_refactor_plan.md`
  子图重构目标、与 Cartographer 风格设计的差异，以及建议的重构路线。

## 当前可观察话题

- `/trajectory`
- `/laser_odom`
- `/current_scan_cloud`
- `/keyframes`

## 启动示例

直接启动 `simple_slam` 并打开 RViz：

```bash
source /home/zwc/ros_simulation/install/setup.bash
ros2 launch simple_slam simple_slam.launch.py
```

用 `bring_up` 走 bag 回放验证：

```bash
source /home/zwc/ros_simulation/install/setup.bash
ros2 launch bringup main.launch.py \
  simulator:=none \
  play_bag:=true \
  slam_system:=simple_slam \
  start_stage:=false \
  start_rviz:=true \
  rviz_config:=/home/zwc/ros_simulation/install/simple_slam/share/simple_slam/rviz/simple_slam.rviz \
  bag_file:=/home/zwc/ros_simulation/bag/1111
```
