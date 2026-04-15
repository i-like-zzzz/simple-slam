# ROS 2 时间体系笔记

这份笔记专门对应当前工程里的 `simple_slam + bring_up + rosbag/gazebo/stage` 用法，不是泛泛的 ROS 教程。

## 1. 先分清两套时间

ROS 2 里最重要的是区分下面两套时间：

- 墙上时间 / 系统时间
  就是当前机器自己的真实时间。`use_sim_time=false` 时，节点默认用它。
- ROS 时间 / 仿真时间
  就是节点从 `/clock` 订阅到的时间。`use_sim_time=true` 时，节点用它。

一句话说：

- `use_sim_time=false`：节点跟着电脑当前时间走
- `use_sim_time=true`：节点跟着 `/clock` 走

## 2. 仿真时间从哪来

仿真时间本身不是节点凭空生成的，必须有一个时间源持续发布 `/clock`。

常见来源有三种：

- Gazebo
  Gazebo 根据仿真世界的推进发布 `/clock`
- Stage
  Stage 根据自己的仿真步进发布 `/clock`
- rosbag 回放
  `ros2 bag play --clock ...` 会按 bag 的播放进度发布 `/clock`

如果节点开了 `use_sim_time=true`，但系统里没人发 `/clock`，那时间体系就是错的。

## 3. Gazebo / Stage 的时间是什么

Gazebo 和 Stage 的“时间”都不是你电脑右下角那个现实时间，而是：

- 仿真世界已经推进了多少秒
- 这个推进速度可以和真实时间一样，也可以更快、更慢
- 仿真暂停时，`/clock` 也会停

所以在仿真里：

- 地图、TF、传感器、导航、SLAM
- 只要都设置成 `use_sim_time=true`

它们就会共享同一套仿真时间。

这也是“仿真共用时间”的意思：不是大家时间差不多，而是大家实际都在读同一个 `/clock`。

## 4. bag 的时间是什么

bag 里有两层容易混的时间：

- 消息自己的 `header.stamp`
- 回放时发布出来的 `/clock`

### 4.1 `header.stamp`

这是录包时每条消息自带的时间戳，比如某帧 `/scan` 在录制当天的某个时间被打了戳。

这部分是写死在 bag 里的，回放时不会自动变成“今天的当前时间”。

### 4.2 回放 `/clock`

只有在执行：

```bash
ros2 bag play <bag> --clock 50.0
```

时，播放器才会额外发布 `/clock`。

这个 `/clock` 表示：

- 现在 bag 播到什么时刻了
- 节点如果 `use_sim_time=true`，就会跟着这个回放时间走

所以 bag 回放时，真正要同步的是：

- 消息时间戳
- TF 时间戳
- 节点内部 `now()`

它们都落在同一条 bag 时间线上。

## 5. 实车时间是什么

实车通常不用 `/clock`，而是直接用系统时间，也就是：

- `use_sim_time=false`

这时所有节点都直接拿本机时间戳工作。

如果系统是多机部署，还要保证不同设备时间尽量同步，比如：

- NTP
- PTP

否则会出现：

- 激光时间和里程计时间不一致
- TF 外推失败
- 传感器融合时间错位

所以实车的关键不是“开 `/clock`”，而是“所有设备用同一套真实世界时间”。

## 6. 三种场景该怎么配

### 6.1 Gazebo / Stage 仿真

- 仿真器发 `/clock`
- 所有算法节点 `use_sim_time=true`

这时的时间源是仿真世界。

### 6.2 bag 回放

- `ros2 bag play --clock ...`
- 所有消费 bag 数据的节点 `use_sim_time=true`

这时的时间源是 bag 播放器。

### 6.3 实车在线跑

- 不发布 `/clock`
- 所有节点 `use_sim_time=false`

这时的时间源是机器系统时间。

## 7. 不能混着用

下面这几种混法都会出问题：

- 节点 `use_sim_time=true`，但系统里没有 `/clock`
- 一部分节点用仿真时间，一部分节点用系统时间
- bag 在放旧时间戳数据，但节点内部 `now()` 还是现实时间

典型症状就是：

- `TF_OLD_DATA`
- `Lookup would require extrapolation`
- TF 明明有，但总说在过去或未来
- RViz 里地图、点云、TF 看起来在乱跳

## 8. 这次 simple_slam 的实际问题

你这次的问题就在这里：

- [simple_slam_bag_2d.yaml](/home/zwc/ros_simulation/src/simple-slam/config/simple_slam_bag_2d.yaml) 里 `use_sim_time: true`
- 但之前 [bag_play.launch.py](/home/zwc/ros_simulation/src/bring_up/launch/bag_play.launch.py) 播 bag 时没有加 `--clock`

结果就变成：

- `/scan`、`/tf` 的时间戳来自 bag 录制时刻
- `simple_slam_node` 又要求自己走 ROS 时间
- 但系统里没有有效 `/clock`

于是同一系统里混进了两条时间线：

- bag 的旧时间线
- 当前机器的现实时间线

这正是之前大量 `TF_OLD_DATA` 的根因。

## 9. 这次修复做了什么

现在 [bag_play.launch.py](/home/zwc/ros_simulation/src/bring_up/launch/bag_play.launch.py) 已经改成：

```python
cmd=[
    'ros2', 'bag', 'play',
    bag_file,
    '--rate', bag_rate,
    '--clock', '50.0'
],
```

循环播放分支也一样加了 `--clock`。

这样 bag 回放时会真正发布 `/clock`，于是：

- `simple_slam_node` 的 `now()`
- TF buffer 用的时间
- bag 里的消息时间

终于落回同一条时间线上。

## 10. 为什么修完后还有启动初期 TF 警告

这和时间主问题不是一回事。

当前启动顺序是：

1. `simple_slam_node` 先起来
2. bag 延迟几秒后开始放
3. bag 里前几条 `/tf` 还没到达时，节点已经开始尝试查变换

所以会看到少量类似警告：

- `failed to lookup transform base_link -> lidar_top`
- `failed to lookup transform base_footprint -> base_link`

这更像“启动初期 TF 尚未就绪”，不是“时间体系再次错了”。

## 11. 这个工程里怎么判断时间是不是对的

最实用的检查顺序：

1. 看节点参数

```bash
ros2 param get /simple_slam_node use_sim_time
```

2. 看 `/clock` 有没有发布者

```bash
ros2 topic info /clock -v
```

3. 看 `map -> base_footprint` 是否连续

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

4. 看日志里是否持续刷下面这些错误

- `TF_OLD_DATA`
- `extrapolation into the past`
- `extrapolation into the future`

## 12. 当前工程里的推荐规则

你后面开发时，直接按这三条记：

- 跑 Gazebo / Stage：`use_sim_time=true`
- 放 bag：`use_sim_time=true`，并且 `ros2 bag play` 必须带 `--clock`
- 跑实车：`use_sim_time=false`

再压缩成一句话：

谁提供时间，所有节点就统一跟谁走；不要让一部分节点活在 bag/仿真时间里，另一部分活在现实时间里。

## 13. 一个容易混淆但不冲突的点

在 [local_slam_frontend.cpp](/home/zwc/ros_simulation/src/simple-slam/src/frontend/local_slam_frontend.cpp) 里你还能看到：

```cpp
std::chrono::steady_clock::now()
```

这个不是 ROS 时间，不拿来做 TF 和消息对齐，只是用来统计算法耗时。

所以：

- `steady_clock` 用于性能计时
- `/clock` / ROS time 用于消息时间、TF 时间、仿真同步

两者职责不同，不冲突。
