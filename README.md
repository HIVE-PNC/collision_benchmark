# 碰撞检测基准测试

这是一个用 CMake 组织的 C++ 单线程碰撞检测基准项目。项目会基于固定随机数种子生成栅格地图和车辆位置姿态，保证每次运行的数据一致，便于比较不同碰撞检测方法的耗时。

## 场景参数

- 地图尺寸：100 m x 100 m
- 栅格分辨率：0.25 m
- 栅格数量：400 x 400
- 车体尺寸：长 4 m，宽 2 m
- 障碍物生成：每个栅格按固定概率随机生成障碍物
- 障碍物概率：0.05
- 地图随机种子：20260602
- 位姿随机种子：20260603
- 测试位姿数量：100000

## 算法说明

- `AABB`：根据车辆当前 `yaw` 计算 4 个角点，再取 `x/y` 最大最小值构造紧 AABB。只要该 AABB 范围内存在障碍物栅格，就认为碰撞。这是保守检测，会有误报。
- `AABB + OBB`：先用当前 `yaw` 对应的紧 AABB 做粗筛，再把障碍物栅格中心点转换到车体坐标系，用 OBB 判断是否落在车体矩形内。
- `AABB + triangle-area`：先用当前 `yaw` 对应的紧 AABB 做粗筛，再用三角形面积和方法判断障碍物栅格中心点是否在车体矩形内。
- `fixed AABB + OBB`：不根据 `yaw` 计算角点最大最小值，而是使用车体半对角线作为固定最大 `x/y` 偏移构造粗筛 AABB，然后再做 OBB 精确判断。
- `fixed AABB + triangle-area`：使用同样的固定最大 `x/y` 偏移构造粗筛 AABB，然后再做三角形面积法精确判断。
- `circle`：使用整个车体矩形的外接圆进行检测。圆心为车辆中心，半径为车体半对角线。这是保守近似，会把车体四角外、圆内的区域也当作碰撞区域。
- `two circles`：沿车体长度方向把 4 m x 2 m 矩形分成前后两个 2 m x 2 m 小矩形，每个小矩形使用一个外接圆。两个圆心沿车体朝向分别偏移 `+1 m` 和 `-1 m`，半径均为 `sqrt(2) = 1.414 m`。这是比单个大外接圆更贴近车体的保守近似。

对于 4 m x 2 m 的车体，固定 AABB 的最大偏移为：

```text
sqrt((4 / 2)^2 + (2 / 2)^2) = sqrt(5) = 2.236 m
```

因此固定 AABB 直接使用：

```text
x: [pose.x - 2.236, pose.x + 2.236]
y: [pose.y - 2.236, pose.y + 2.236]
```

这种方式省掉了粗筛阶段按 `yaw` 计算角点和最大最小值的开销，但粗筛窗口比紧 AABB 更松，会扫描更多栅格。实际是否更快取决于障碍物密度、车辆尺寸、早退出概率和 CPU 缓存表现。

单个外接圆的半径和固定 AABB 的最大偏移相同，都是车体半对角线：

```text
radius = sqrt((4 / 2)^2 + (2 / 2)^2) = sqrt(5) = 2.236 m
```

两个外接圆把车辆切成前后两个 2 m x 2 m 区域：

```text
circle_center_1 = pose_center + heading * 1.0
circle_center_2 = pose_center - heading * 1.0
radius = sqrt((2 / 2)^2 + (2 / 2)^2) = sqrt(2) = 1.414 m
```

## 构建和运行

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/collision_benchmark
```

程序会生成确定性的地图文件：

- `data/grid_map.csv`：完整栅格地图
- `data/obstacles_xy.csv`：障碍物栅格中心点坐标

## 当前测试结果

测试环境：本项目在 Release 模式下编译并运行一次，测试 100000 个固定随机位姿。

```text
Map: 100m x 100m, resolution 0.25m, cells 400 x 400, occupied 8048, map_seed 20260602, pose_seed 20260603
Vehicle: length 4m, width 2m, poses 100000

algorithm                            poses    collisions        total_ms          avg_us
----------------------------------------------------------------------------------------
AABB                                100000         99992           8.188           0.082
AABB + OBB                          100000         99771          20.709           0.207
AABB + triangle-area                100000         99771          18.870           0.189
fixed AABB + OBB                    100000         99771          24.664           0.247
fixed AABB + triangle-area          100000         99771          25.605           0.256
circle                              100000         99998          12.429           0.124
two circles                         100000         99973          15.705           0.157
```

从这组结果看，`fixed AABB` 两个版本没有更快。主要原因是固定 AABB 使用半对角线构造正方形粗筛区域，虽然少算了角点最大最小值，但会多遍历一些栅格。当前参数下，多遍历栅格的代价超过了省掉的 AABB 构造代价。

`circle` 和 `two circles` 都比精确矩形检测更快，但它们是保守近似，碰撞数量比 `AABB + OBB` 和 `AABB + triangle-area` 更多。`two circles` 比单圆更贴近车体形状，误报更少，但需要计算两个圆心并做两次圆内判断，因此耗时高于单圆。
