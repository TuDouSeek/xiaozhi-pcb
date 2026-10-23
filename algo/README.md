# Navigation 算法目录

本目录专门写**电机运行算法、小智控制算法、自动驾驶、雷达避障**，与 `main/boards/xiaozhi-pcb-arm/` 的硬件驱动解耦。

## 目录结构

```text
algo/
├── README.md               本文件
├── common/                 公共数学/滤波工具（与硬件无关，可在 PC 单测）
│   ├── pid.h               通用 PID（位置式+增量式）
│   ├── filters.h           一阶低通/EMA/中值滤波
│   └── math_utils.h        限幅/角度归一化/插值
├── motion/                 电机运行算法（差速运动学、速度闭环、轨迹跟踪）
│   ├── diff_drive.h        差速底盘正逆运动学 v/w ↔ l/r
│   ├── speed_loop.h        双轮速度闭环（基于 MotorController 接口）
│   └── trajectory.h        梯形速度规划/定点/定速巡航
├── perception/             雷达感知（扫描、聚类、障碍物栅格）
│   ├── scan.h              点云预处理（去噪/下采样/极坐标→直角）
│   ├── obstacle.h          扇区碰撞检测/最近障碍距离
│   └── grid_map.h          简易占据栅格（后续建图用）
├── navigation/             自动驾驶（避障、漫游、回充预留）
│   ├── avoid.h             VFH/动态窗口简化版避障
│   ├── wander.h            随机漫游+碰障转向
│   └── autopilot.h         总调度：perception → decision → motion
└── sim/                    PC 端仿真/单测（不烧板也能跑算法）
    └── test_drive.cc       diff_drive + avoid 的最小可跑示例
```

## 与固件的对接约定

- `motion` 只调 `MotorController::SetSpeed(l, r)`（-100~100），不直接碰 GPIO/LEDC。
- `perception` 只消费雷达点云 `[(angle°, distMm)]`（来自 `ld14` / UDP :8888），不碰 UART。
- `navigation` 输出 `(v, w)` 线速度/角速度，经 `diff_drive` 转成 `(l, r)` 再下发。
- 所有算法头文件保持"无 ESP 依赖"，方便在电脑上 `g++ sim/test_drive.cc` 直接验证。

## 上机流程（ESP32-S3）

1. 先在 `sim/` 把算法跑通；
2. 在 `xiaozhi-pcb-arm.cc` 里 `motors_->SetSpeed(l, r)` 的调用处，把 `(l, r)` 换成 `autopilot` 输出；
3. 雷达点云从 `LidarDriver` 回调取，喂给 `perception/scan.h`。
