# Log-Odds 与 Occupancy Probability 公式说明

这份文档只解释 `simple_slam` 里这两个公式为什么这么写，以及它们在栅格建图中的意义：

```cpp
double ProbabilityToLogOdds(const double probability) {
  return std::log(probability / (1.0 - probability));
}

double LogOddsToProbability(const double log_odds) {
  const double odds = std::exp(log_odds);
  return odds / (1.0 + odds);
}
```

## 1. 基本定义

在占据栅格地图里，我们通常关心某个格子被占据的概率：

\[
p = P(\text{occupied})
\]

其中：

- `p = 0.5` 表示未知
- `p > 0.5` 表示更偏向“被占据”
- `p < 0.5` 表示更偏向“空闲”

但在实际更新时，通常不会直接在线性概率空间里反复加减，而是先变换到 `log-odds` 空间。

## 2. odds 是什么

先定义 `odds`：

\[
\text{odds} = \frac{p}{1-p}
\]

它表示：

“这个格子被占据”的可能性，相对于“这个格子不被占据”的可能性之比。

例如：

- 当 `p = 0.5` 时，

\[
\text{odds} = \frac{0.5}{0.5} = 1
\]

- 当 `p = 0.8` 时，

\[
\text{odds} = \frac{0.8}{0.2} = 4
\]

表示“占据”大约是“空闲”的 4 倍。

## 3. log-odds 是什么

再对 `odds` 取自然对数，也就是 `ln`：

\[
l = \ln \frac{p}{1-p}
\]

这就是 `log-odds`。

这正对应代码里的：

```cpp
double ProbabilityToLogOdds(const double probability) {
  return std::log(probability / (1.0 - probability));
}
```

记号上通常写成：

\[
l = \ln \frac{p}{1-p}
\]

它有几个很重要的性质：

- 当 `p = 0.5` 时，`l = 0`
- 当 `p > 0.5` 时，`l > 0`
- 当 `p < 0.5` 时，`l < 0`

所以：

- 正数表示更倾向占据
- 负数表示更倾向空闲
- 0 表示未知

## 4. 为什么建图喜欢用 log-odds

核心原因是：更新更方便。

如果你直接在概率空间里更新，一个格子被多次命中、多次穿过，组合起来会比较别扭。

而在 `log-odds` 空间中，新的观测可以近似写成“累加”：

\[
l_t = l_{t-1} + \Delta l
\]

在你的 `Submap2D` 里就是这种思想：

- 命中栅格时，加上 `hit_probability` 对应的 log-odds 增量
- 穿过栅格时，加上 `miss_probability` 对应的 log-odds 增量

于是一个格子如果被反复命中，它的值会越来越大；
如果反复被射线穿过，它的值会越来越小。

这就是为什么代码里内部不直接存概率，而是存：

```cpp
std::vector<double> log_odds_cells_;
```

## 5. 从 log-odds 反推回 probability

如果有：

\[
l = \ln \frac{p}{1-p}
\]

那么先两边取指数：

\[
e^l = \frac{p}{1-p}
\]

整理可得：

\[
p = \frac{e^l}{1 + e^l}
\]

这就是代码里的逆变换：

```cpp
double LogOddsToProbability(const double log_odds) {
  const double odds = std::exp(log_odds);
  return odds / (1.0 + odds);
}
```

也可以写成等价形式：

\[
p = \frac{1}{1 + e^{-l}}
\]

这就是一个标准的 sigmoid 形式。

## 6. 两个公式互为逆变换

正向：

\[
p \rightarrow l = \log \frac{p}{1-p}
\]

反向：

\[
l \rightarrow p = \frac{e^l}{1 + e^l}
\]

所以：

- 建图内部用 `log-odds`
- 发布 OccupancyGrid 或查询概率时再转回 `probability`

这是非常常见的一套做法。

## 7. 在你当前 simple_slam 里的具体作用

在当前 `Submap2D` 实现中，这两个函数的角色是：

1. `ProbabilityToLogOdds()`
   把参数里的 `hit_probability`、`miss_probability` 转成可累加的更新量

2. `UpdateCell()`
   把这些增量加到 `log_odds_cells_` 上

3. `LogOddsToProbability()`
   在 `GetProbability()` 和 `ToOccupancyGridData()` 里，把内部值恢复成直观概率

也就是说：

- 内部表示是 `log-odds`
- 对外接口表现为 `occupancy probability`

## 8. 一个简单数值例子

假设：

\[
p_{\text{hit}} = 0.7
\]

那么对应的 log-odds 增量是：

\[
\ln\frac{0.7}{0.3} \approx 0.847
\]

如果某个栅格连续两次被命中，大致会累计成：

\[
l \approx 0.847 + 0.847 = 1.694
\]

再转回概率：

\[
p = \frac{e^{1.694}}{1 + e^{1.694}} \approx 0.845
\]

可以看到，多次命中后，占据概率会上升。

如果多次被 miss 更新，`l` 会走向负值，概率就会下降到 `0.5` 以下。

## 9. 为什么还要做限幅

如果一个格子被长期反复更新，`log-odds` 可能会非常大或非常小。

所以通常会做截断：

\[
l \in [l_{\min}, l_{\max}]
\]

你当前代码里就是：

```cpp
log_odds = std::clamp(log_odds + delta, -4.0, 4.0);
```

这样做的目的是：

- 防止数值过饱和
- 防止某个格子被单边观测永久“锁死”
- 给后续观测留下修正空间

## 10. 新一帧雷达进来后，栅格是怎么更新的

每来一帧新的雷达数据，栅格地图都会按类似思路更新。流程可以理解成下面这几步：

### 第 1 步：先把 LaserScan 变成一组激光命中点

前端会先把每个量测 `range` 转成二维点：

\[
(x, y) = (r\cos\theta,\ r\sin\theta)
\]

这一步之后，一帧扫描就变成了一组 `returns`。

### 第 2 步：确定这一帧激光在地图里的位姿

前端先估计当前帧位姿 `local_pose`，这个位姿来自：

- 里程计预测
- 帧间激光匹配
- scan-to-submap 匹配修正

然后在插图阶段，会把这一帧扫描按这个位姿投到当前活动子图里。

### 第 3 步：对这一帧里的每一束激光做 ray casting

在 `Submap2D::InsertRangeData()` 里，会对每个 hit 点做：

1. 传感器原点 `sensor_origin`
2. 命中点 `hit_in_world`
3. 从原点到命中点画一条射线

也就是：

- 射线穿过的格子，认为更可能是空闲
- 射线终点命中的格子，认为更可能是占据

这就是占据栅格建图里最经典的 inverse sensor model。

### 第 4 步：穿过的格子做 miss 更新

如果激光从传感器出发，经过一些格子后才打到障碍物，
那么中间被穿过的那些格子更应该是“空的”。

所以这些格子会加上：

\[
\Delta l_{\text{miss}} = \ln \frac{p_{\text{miss}}}{1-p_{\text{miss}}}
\]

注意：

- 如果 `p_miss < 0.5`，那么这个值是负的
- 所以它会把该格子的 log-odds 往负方向推
- 概率也就更偏向“空闲”

例如你现在默认：

\[
p_{\text{miss}} = 0.49
\]

那么：

\[
\Delta l_{\text{miss}} = \ln\frac{0.49}{0.51} < 0
\]

所以每次被激光穿过，这个格子的占据概率都会轻微下降。

### 第 5 步：命中的终点格子做 hit 更新

射线终点打到障碍物的那个格子，会加上：

\[
\Delta l_{\text{hit}} = \ln \frac{p_{\text{hit}}}{1-p_{\text{hit}}}
\]

因为通常 `p_hit > 0.5`，所以：

\[
\Delta l_{\text{hit}} > 0
\]

它会把这个格子的 log-odds 往正方向推，让它越来越像“被占据”。

例如默认：

\[
p_{\text{hit}} = 0.7
\]

那么：

\[
\Delta l_{\text{hit}} = \ln\frac{0.7}{0.3} \approx 0.847
\]

### 第 6 步：多帧累积，概率逐渐稳定

假设某个格子连续多次被命中，那么它的内部值会变成：

\[
l_t = l_0 + n \cdot \Delta l_{\text{hit}}
\]

如果某个格子多次被射线穿过，那么会变成：

\[
l_t = l_0 + n \cdot \Delta l_{\text{miss}}
\]

所以：

- 经常被打中的地方，越来越像障碍物
- 经常被穿过的地方，越来越像空闲区域
- 没怎么观测到的地方，仍然接近未知

### 第 7 步：查询或发布地图时，再把 log-odds 变回概率

内部累计完成后，对外查询时用：

\[
p = \frac{e^l}{1 + e^l}
\]

再映射成 OccupancyGrid 的 `0~100` 或未知 `-1`。

所以你可以把整件事记成一句话：

一帧雷达进来之后，不是“直接覆盖旧地图”，而是“沿着每一束激光对经过格子做 miss 更新、对终点格子做 hit 更新，然后在 log-odds 空间里累积证据”。

## 11. 一句话总结

这两个公式的本质是：

- 用 `probability` 表达“占据的直觉意义”
- 用 `log-odds` 表达“适合反复累加更新的内部数值形式”

因此在占据栅格建图里，最常见的模式就是：

\[
\text{probability} \leftrightarrow \text{log-odds}
\]

对外看概率，对内做 log-odds 累加。
