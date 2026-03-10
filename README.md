# simple_slam

面向 ROS 2 的渐进式 2D SLAM 系统。

这个包的目标不是一次性复制 Cartographer 的全部能力，而是按工程化顺序逐步构建：

1. 数据流和模块边界
2. 2D 激光局部前端
3. 子图构建
4. 回环检测与位姿图优化
5. 地图发布与调试工具

当前阶段实现的是第 1 步：

- `simple_slam_node`：主节点和 ROS 接口
- `LocalSlamFrontend`：局部前端接口
- `Submap2D`：子图状态容器
- `PoseGraph2D`：位姿图接口与状态缓存

后续迭代将逐步把 scan matching、约束构建和非线性优化填充进去。
