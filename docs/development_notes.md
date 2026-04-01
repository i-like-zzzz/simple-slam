# simple_slam 开发说明

这份说明只记录当前实现真实做到哪里、哪些边界已经稳定、下一步应该怎么推进。

## 当前目录怎么分

- `include/simple_slam/frontend`
  放局部前端。现在已经有 scan 预处理、位姿预测、帧间激光里程计、scan-to-submap 匹配和关键帧判定。
- `include/simple_slam/mapping`
  放子图和地图表达。当前 `Submap2D` 用的是固定大小、world 轴对齐的局部占据栅格。
- `include/simple_slam/optimization`
  当前主要是位姿图数据容器，后面会补约束和优化结果表达。
- `include/simple_slam/backend`
  当前先放后端接口，边界已经有了，实现还没有填进去。
- `include/simple_slam/system`
  放系统级别的模式和调度相关定义。现在已经把 `mapping` / `localization` 模式抽出来了。

## 当前前端实际做了什么

现在的前端链路是：

1. 接 `/scan`
2. 过滤无效量测并做简单体素降采样
3. 用 `/odom` 增量或上一帧激光里程计结果做位姿预测
4. 在无 `/odom` 时执行帧间激光里程计匹配
5. 在当前活动子图上做 scan-to-submap 匹配
6. 按位移和角度阈值决定是否生成关键帧
7. 把关键帧插入活动子图
8. 发布 `/trajectory`、`/laser_odom`、`/current_scan_cloud`、`/keyframes` 和 `map -> odom`

这说明当前代码已经不是“只有激光里程计”，而是第一版局部前端 SLAM。

## 当前已经稳定下来的边界

- 前端入口是 `LocalSlamFrontend::AddScan()`
- 子图入口是 `Submap2D::InsertRangeData()`
- 节点层负责 ROS 话题和 TF
- 位姿图当前只登记节点和子图，不负责优化
- 后端当前只保留接口，不改变前端轨迹

这些边界已经足够稳定，可以开始往后端扩展，不需要继续把所有逻辑都塞回前端类里。

## 当前 `Submap2D` 的定位

当前 `Submap2D` 不是 Cartographer 那种成熟子图实现，它更像一个最小可运行版本：

- 固定大小
- world 轴对齐
- 不做动态扩容
- 不使用旋转子图局部坐标系
- 用简单的 log-odds 栅格表达占据概率
- 用两个重叠活动子图跑通数据流

它已经够用来支撑当前前端，但还不是最终版本。下一阶段最值得投入的不是“继续堆 matcher 参数”，而是把子图表示和位姿图约束补成熟。

## 运行模式

- `mapping`
  正常建图，前端会更新活动子图。
- `localization`
  当前只是预留接口，本质上只是不再更新地图。

## 当前还没做的关键能力

- 没有节点-子图约束结构
- 没有后端优化线程
- 没有回环检测
- 没有优化结果回写前端和子图位姿
- 没有子图序列化和地图持久化

## 工程判断

到当前阶段，不建议继续把主要精力放在“再加一个前端 matcher”上。

更合理的主线是：

1. 重构 `Submap2D`
2. 把子图管理从 `LocalSlamFrontend` 里拆清楚
3. 定义位姿图约束
4. 接后端优化
5. 最后再做回环检测
