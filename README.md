# 小智 AI 履带机器人 · xiaozhi-pcb-arm

基于 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 的定制分支，把语音助手落到一台**带雷达的履带底盘**上：ESP32-S3 主控、LD14 激光雷达、双电机编码器闭环、本地网页点云与云端 MQTT 遥控。

上游的语音链路（唤醒词、ASR/LLM/TTS、MCP）保持不动，本仓库的改动集中在板级驱动、运动控制和调试工具链。

## 功能一览

| 模块 | 能力 |
| --- | --- |
| 语音交互 | 唤醒词 `nihaoxiaozhi`，接入 xiaozhi.me 官方服务；MCP 工具可被模型直接调用（前进/后退/转向/停止/刹车/雷达状态/眼睛表情） |
| 激光雷达 | LD14 走 UART2（RX=39），PWM 调速（40）；设备侧 HTTP + WebSocket 推点云，同时 UDP 广播给本地桥接 |
| 电机控制 | 2× AT8870 + 正交编码器；开环占空比 / 直行 trim / 速度闭环 PID 三种模式，参数存 NVS |
| 本地调试 | 设备自带网页：`/motor` 电机测试台、`/lidar` 雷达点云、`/api/motor/tune` 调参接口 |
| 云端遥控 | MQTT `robot/cmd` 下发、`robot/telemetry` 200ms 遥测，2s 无指令自动刹车 |
| Web 控制台 | `fwq/` 静态前端：摇杆、履带滑块、双轮配平（含遥测自动标定）、速度曲线、雷达 3D 视图 |
| 显示 | SSD1306 OLED 机器人 UI（信号/时间/电池/语音柱）+ 双 GC9A01 圆屏眼睛表情 |
| 网络 | Wi-Fi 为主；ML307 4G 硬件已预留（当前变体未启用） |

## 硬件与引脚

主控建议 **ESP32-S3-WROOM-1-N16R8**（16MB Flash + 8MB OPI PSRAM），PSRAM 为唤醒词所必需。

| 外设 | 信号 | GPIO |
| --- | --- | --- |
| 电机 A（AT8870 逻辑输入） | IN1 / IN2 | 17 / 18 |
| 电机 B | IN1 / IN2 | 38 / 47 |
| 编码器 A | A 相 / B 相 | 1 / 2 |
| 编码器 B | A 相 / B 相 | 21 / 48 |
| 激光雷达 | UART2 RX / PWM | 39 / 40 |
| OLED（SSD1306） | SDA / SCL | 41 / 42 |
| 双圆屏 GC9A01 | DC / RST / SCK / MOSI / CS-L / CS-R | 13 / 14 / 11 / 12 / 10 / 9 |
| 麦克风 INMP441 | SD / WS / SCK | 4 / 5 / 6 |
| 功放 MAX98357 | DIN / BCLK / LRC | 16 / 15 / 7 |
| BOOT 按键 | — | 0 |
| 4G 模组（预留） | TX / RX | 20 / 19 |

> B 轮在固件里配置了 `invert_b = true`，接线按现板执行即可；改线后同步修改 `config.h` 与 `xiaozhi-pcb-arm.cc` 中的极性。

## 目录结构

```
main/boards/xiaozhi-pcb-arm/   板级代码
  motor_controller.*           电机驱动 + 编码器测速 + trim + PID 闭环 + NVS 调参
  motor_mqtt.*                 robot/cmd、robot/telemetry 云桥与看门狗
  ld14_driver.* / ld14_system.* 雷达驱动与数据流
  ld14_web.*                   /motor、/lidar 页面和 /api/* 接口
  eye_display.* / eye_emotions.h  双圆屏表情
  robot_oled_display.*         机器人 UI
  xiaozhi-pcb-arm.cc           板级装配、MCP 工具注册
fwq/                           Web 控制台 + UDP→WS 雷达桥接（node server.js）
algo/                          运动/导航/仿真骨架
docs/                          协议与开发文档
scripts/build.py               带板卡选择的构建脚本
```

## 编译与烧录

环境：ESP-IDF **v6.1**（本仓库实测），上游建议 v6.0.2 及以上。`config.json` 会自动追加 `CONFIG_HTTPD_WS_SUPPORT=y`，雷达 WebSocket 依赖它。

```bash
source ~/.espressif/tools/activate_idf_v6.1.sh        # Windows 换成 export.bat
python scripts/build.py xiaozhi-pcb-arm
```

产物：

- `build/merged-binary.bin` —— 合并固件，烧到 `0x0` 即可
- `build/xiaozhi.bin` + bootloader + 分区表 —— 手动分地址烧录

```bash
# 一键烧录（推荐）
python -m esptool --chip esp32s3 -p /dev/cu.usbmodem21401 -b 460800 \
  --before default-reset --after hard-reset write-flash 0x0 build/merged-binary.bin

# 开发迭代
idf.py -p /dev/cu.usbmodem21401 flash monitor
```

首次上电进入热点配网 → 到 xiaozhi.me 用激活码绑定 → 说"你好小智"开始对话。

## 三种遥控方式

设备同时支持本地网页、局域网 HTTP 和云端 MQTT，**联调时只保留一种**，否则指令会互相覆盖。

### 1. 设备自带网页

| 地址 | 用途 |
| --- | --- |
| `http://<设备IP>/motor` | 电机测试台：单轮 A/B、双轮、刹车、trim 微调 |
| `http://<设备IP>/lidar` | 雷达点云与量程切换 |
| `ws://<设备IP>/ws/lidar` | 点云 JSON 流 |

### 2. HTTP 接口

| 接口 | 说明 |
| --- | --- |
| `GET /api/motor/control?motor=A\|B\|both&dir=forward\|backward\|stop\|brake&speed=0..100[&trim=0.2..2]` | 手动驱动 |
| `GET /api/motor/status` | 双轮方向/速度/RPM/脉冲/trim |
| `GET /api/motor/tune[?closed=0\|1&kp=&ki=&kd=&gear=&noload=&trim_a=&trim_b=][&save=1]` | 读写闭环与 PID 参数，`save=1` 写 NVS |
| `GET /api/lidar_status` | 雷达帧数、CRC 错误、转速 |
| `GET /api/lidar_points` | 最新一帧点云 |

### 3. 云端 MQTT

设备连 `mqtt://103.231.59.46:1883`，浏览器走 `ws://103.231.59.46:8083/mqtt`（EMQX 同时开 TCP 与 WS）。

**下发 `robot/cmd`**

```json
{"action":"manual","speed_l":60,"speed_r":60}
{"action":"brake"}
{"tune":{"closed":1,"gear":76,"kp":1.0,"ki":2.0},"save":1}
```

`action` 支持 `forward / backward / left / right / stop / brake / manual`；`speed_l`、`speed_r` 范围 -100~100。

**遥测 `robot/telemetry`（200ms）**

```json
{"rpm_l":96.2,"rpm_r":95.8,"cnt_l":12345,"cnt_r":12300,
 "tgt_l":96.0,"tgt_r":96.0,"closed":1}
```

**看门狗**：2s 收不到 `robot/cmd` 自动抱死刹车；本地 WS 摇杆另有 300ms 看门狗。

## 两轮不同步怎么修

同一占空比下两轮转速天然不一致。固件提供三级手段，建议按顺序标定：

**第 1 步 · 校准减速比**（默认 30，必须实测）

悬空一轮，用 `/api/motor/status` 读 `cnt`，手转输出轴一圈再读一次，差值 = `11 × 减速比 × 4`（3344 → 76）。

```bash
curl "http://<IP>/api/motor/tune?gear=76&save=1"
```

**第 2 步 · 量空载满速**

开环 100% 空转，读稳定后的 RPM，写入 `noload`：

```bash
curl "http://<IP>/api/motor/tune?noload=195&save=1"
```

**第 3 步 · 开启闭环**

```bash
curl "http://<IP>/api/motor/tune?closed=1&kp=1.0&ki=2.0&kd=0&save=1"
```

闭环后 `percent` 会被映射成目标 RPM，PID 直接跟踪编码器实测值，两轮差值应稳定在 2% 以内。抖动就降 `kp`，跟随偏慢就加 `ki`。

**回滚**：`curl "http://<IP>/api/motor/tune?closed=0&save=1"`。

参数全部存 NVS（命名空间 `motor`），断电重启自动恢复。前端 `fwq` 的右轮配平系数在检测到 `closed=1` 后会自动停用，避免和 PID 相互打架。

## Web 控制台（fwq）

```bash
cd fwq
npm install          # 仅依赖 ws
node server.js 8080  # 静态页面 + UDP 8888 雷达桥 + WS 8889
```

打开 `http://localhost:8080/`，在顶部填 Broker IP / 端口（WebSocket 8083）后连接。

雷达链路：ESP32 每帧通过 UDP 8888 广播 254 字节点云 → `server.js` 转 JSON → WS 8889 → 前端 3D 视图。防火墙需放行 UDP 8888。

> 仓库里的 `index.html` 引用 `unpkg.com` 的 mqtt/three。部署到无外网的服务器时，把这两行换成本地 `vendor/mqtt.min.js`、`vendor/three.min.js`。

## 安全设计

- MQTT 2s / 本地 WS 300ms 双看门狗，失联自动刹车
- `Brake()` 会同时清零 PID 目标并暂停闭环（修复了"急停后 20ms 又被旧目标加速"的问题）
- BOOT 长按：立即刹停电机并关闭雷达 PWM
- 急停指令 `{"action":"brake"}` 与页面急停按钮等价

## 常见问题

| 现象 | 排查 |
| --- | --- |
| 雷达无点云 | 确认 UDP 8888 放行、云台电机 PWM 已启动、`/api/lidar_status` 帧数是否递增 |
| 电机不转 | 检查 AT8870 供电、GPIO 17/18/38/47 接线、`invert_b` 配置 |
| 两轮反向 | 单轮测试确认极性；必要时只改 `invert_b` 或换相线 |
| 急停后车又动 | 旧固件闭环遗留 bug，本分支已修，确认烧的是最新固件 |
| WebSocket 连不上 | 编译配置需 `CONFIG_HTTPD_WS_SUPPORT=y`（`build.py` 已自动追加） |
| 参数改了没生效 | 加 `&save=1` 才会写 NVS；读回 `GET /api/motor/tune` 确认当前值 |
| 想恢复出厂参数 | `idf.py erase-flash`（会同时清 Wi-Fi 配置） |

## 分支与版本

- `main`：稳定分支，已合并固件闭环、调参接口与前端改动（PR #1）
- `test`：调试分支，日常改动先落这里
- `v1.1.0-test`：闭环版本的冻结标签

## 许可证

MIT License，详见 [LICENSE](LICENSE)。上游项目：[78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)。

硬件与开发细节见 [docs/custom-board.md](docs/custom-board.md)、[docs/esp-idf-6-migration.md](docs/esp-idf-6-migration.md)；**自主导航（避障/航点/安全层）的架构与接口设计见 [docs/autonomy-design.md](docs/autonomy-design.md)**；板级说明见 [main/boards/xiaozhi-pcb-arm/README.md](main/boards/xiaozhi-pcb-arm/README.md)（部分章节写于骨架阶段，以本文件为准）。
