# simple_slam 参数说明

这份文档只解释 `config/simple_slam_2d.yaml` 里当前真的会用到的参数，不写泛泛而谈的“可以根据场景调整”。

## 1. 时间与坐标系

- `use_sim_time`
  仿真、bag 回放时一般设成 `true`。
- `map_frame`
  局部建图和路径发布使用的固定坐标系。
- `odom_frame`
  当前节点会发布 `map -> odom_frame` 变换；如果接入了外部 `/odom` 或已有 TF 树，
  这里决定地图坐标系最终挂到哪一层底盘树上。
- `published_frame`
  当前局部位姿、`laser_odom` 和关键帧可视化都会以这个 frame 为主。

## 2. 系统模式与调试

- `system_mode`
  当前建议用 `mapping`。`localization` 还只是预留接口。
- `enable_backend`
  现在后端还没有真正接约束和优化线程，所以默认关掉。
- `publish_keyframe_markers`
  打开后会发布 `/keyframes`，RViz 调试很有用。
- `keyframe_marker_scale`
  关键帧箭头尺寸。
- `debug_log_every_n_scans`
  设成 `100` 时，每处理 100 帧输出一次前端状态。排查 bag 时很有用。

## 3. 激光预处理

- `min_range`
  裁掉过近的量测，避免机器人本体或异常近点干扰。
- `max_range`
  裁掉过远点，先让前端速度和稳定性可控。
- `voxel_filter_size`
  点云降采样尺度。默认 `0.05` 比较稳妥。

如果你发现前端明显太慢，通常先调大 `voxel_filter_size`，而不是先去改子图参数。

## 4. 关键帧与活动子图

- `scans_per_accumulation`
  最少隔多少帧才允许插一次关键帧。
- `active_submap_num_range_data`
  单个活动子图最多插多少帧。
- `min_range_points_for_match`
  当前帧有效点太少时，前端直接跳过。
- `keyframe_translation_threshold`
  位移达到这个阈值就允许插关键帧。
- `keyframe_rotation_threshold`
  旋转达到这个阈值也允许插关键帧。

如果你看到关键帧太密，一般先加大这两个阈值；如果关键帧很久不出，再往小调。

## 5. 激光里程计参数

这一组在没有外部 `/odom` 时最重要。

- `lidar_odom_point_sigma`
  影响 ICP 的对应点距离阈值。这个值太小，容易不收敛；太大，容易对上错误结构。
- `lidar_odom_max_points`
  送进 ICP 的最大点数。默认 `48` 是为了 bag 回放时控制算力。
- `lidar_odom_linear_window`
- `lidar_odom_angular_window`
- `lidar_odom_translation_weight`
- `lidar_odom_rotation_weight`

这四个参数目前保留下来，是为了后面继续强化“预测项约束”时不用改配置结构。当前 ICP 版本真正最敏感的还是 `lidar_odom_point_sigma` 和 `lidar_odom_max_points`。

## 6. scan-to-submap 匹配参数

- `linear_search_window`
- `angular_search_window`
- `linear_search_step`
- `angular_search_step`

这一组决定了局部 scan-to-submap 粗匹配搜索范围和粒度。

- 搜索窗口大：更不容易丢，但更慢
- 搜索步长小：更细，但更慢

- `translation_delta_cost_weight`
- `rotation_delta_cost_weight`

这两个参数会惩罚偏离预测位姿太远的候选解。

## 7. 子图栅格参数

- `submap_resolution`
  单个栅格的米数。
- `submap_width`
- `submap_height`
  栅格数量，不是米。实际尺寸要乘上 `resolution`。
- `submap_hit_probability`
- `submap_miss_probability`
  控制占据/空闲更新力度。

## 8. 建议的调参顺序

如果你现在要针对一包新 bag 调试，建议按这个顺序来：

1. 先确认 `/scan`、`/laser_odom`、`/trajectory`、`/keyframes` 都有输出
2. 再调 `voxel_filter_size` 和 `lidar_odom_max_points`，先把实时性跑顺
3. 再调 `lidar_odom_point_sigma`，让激光里程计不要明显发散
4. 再调关键帧阈值
5. 最后才去改 scan-to-submap 的搜索窗口和步长

如果一上来就去大改子图窗口，通常会把问题越调越乱。
