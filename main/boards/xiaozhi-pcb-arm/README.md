# xiaozhi-pcb-arm 板卡说明（烧录版）

小智 AI 履带机器人板卡（ESP32-S3）。本文档面向"拿到固件 → 烧录 → 设备跑起来"的完整流程，同时给出从源码自行构建的全部命令。

> 固件上报标识：`xiaozhi-pcb-arm`（OTA 升级通道依赖此唯一名称，发布后不要修改 `config.json` 中的 `type`）。

---

## 1. 硬件需求

| 项目 | 要求 |
| --- | --- |
| 主控 | ESP32-S3 模组，建议 ESP32-S3-WROOM-1-**N16R8**（16MB Flash + 8MB OPI PSRAM） |
| Flash | 16MB（项目对 S3 默认 16MB + `partitions/v2/16m.csv`，config.json 无需额外 sdkconfig） |
| PSRAM | 必须（默认 sdkconfig 启用 OPI PSRAM，AFE 唤醒词依赖） |
| 显示 | I2C OLED（SSD1306 128×64，地址 0x3C）为主显示，运行自定义机器人 UI（信号格 + 状态/时间 + 电池 + 离散语音柱）；两块 1.28" GC9A01 圆屏为机器人眼睛（骨架阶段未接入渲染） |
| 音频 | INMP441 数字麦克风 + MAX98357 I2S 功放（SD_MODE 接 3.3V 使能） |
| 电机 | 2× AT8870 驱动 + 6 线正交编码器减速电机（逻辑脚已按原理图定稿：17/18/38/47） |
| 雷达 | LD-06/LD-19 协议 UART 雷达（骨架按 230400bps + 0x54 0x2C 帧头解析） |
| 网络 | Wi-Fi（ML307R 4G 硬件已预留，当前变体未启用） |

### 引脚映射表（与 `config.h` 一致）

| 外设 | 信号 | GPIO |
| --- | --- | --- |
| INMP441 麦克风 | SD / WS / SCK | 4 / 5 / 6 |
| MAX98357 功放 | DIN / BCLK / LRC | 16 / 15 / 7 |
| OLED (I2C) | SDA / SCL | 41 / 42 |
| 双圆屏 (SPI 共线) | DC / RST / SCK / MOSI | 13 / 14 / 11 / 12 |
| 双圆屏 | CS 左眼 / CS 右眼 | 10 / 9 |
| 电机 A 编码器 | A 相 / B 相 | 1 / 2 |
| 电机 B 编码器 | A 相 / B 相 | 21 / 48 |
| AT8870 逻辑输入（原理图定稿） | A-IN1 / A-IN2 / B-IN1 / B-IN2 | 17 / 18 / 38 / 47 |
| 激光雷达 | UART RX / PWM 调速 | 39 / 40 |
| BOOT 按键 | — | 0 |
| 4G 模组（预留未启用） | MCU TX / MCU RX | 20 / 19 |

---

## 2. 获取固件（两条路线）

### 路线 A：直接使用已构建的合并固件（推荐，无需 IDF 环境）

构建产物为单文件合并固件，包含 bootloader、分区表、应用和资源分区：

```text
build/merged-binary.bin
```

烧录方式见第 4 节（只需这一条：`merged-binary.bin` 写入地址 `0x0`）。

### 路线 B：从源码构建

#### 2.1 环境准备（本仓库实测配置）

- ESP-IDF：**v5.5.5**（`E:\App\Espressif\frameworks\esp-idf-v5.5.5`，与 `.vscode/settings.json` 一致；上游 AGENTS.md 推荐 v6.0.2，升级前先不要混用）
- Python 虚拟环境：`E:\App\Espressif\python_env\idf5.5_py3.11_env`（本机实际存在的是 **py3.11** 版本）

两个实测踩坑点（务必按此操作）：

1. **必须用 IDF venv 里的 python 运行 `build.py`**。用系统 Python 直接跑会报
   `No module named 'esp_idf_monitor'`，因为 `build.py` 会用它自己的解释器去拉起 `idf.py`。
2. **构建前确认没有残留的 `kconfserver`/`idf.py` 后台进程**（VS Code ESP-IDF 插件常见残留），否则
   `fullclean` 会报 `PermissionError: build\.ninja_log 被占用`。

PowerShell 推荐写法（已在本仓库实测到 reconfigure 阶段）：

```powershell
cd D:\Project\XIAOZHI\xiaozhi-esp32-main\xiaozhi-esp32-main-V2

& "E:\App\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" scripts/build.py xiaozhi-pcb-arm
```

预期输出开头：

```text
Project Version: 2.4.2 (main\boards\xiaozhi-pcb-arm\config.json)
name: xiaozhi-pcb-arm
reported_type: xiaozhi-pcb-arm
target: esp32s3
sdkconfig_append: CONFIG_BOARD_TYPE_XIAOZHI_PCB_ARM=y
```

构建成功后得到 `build\merged-binary.bin`（同时可用 `build\xiaozhi.bin`、`build\bootloader\bootloader.bin`、`build\partition_table\partition-table.bin` 单独烧录）。

可选项：

```powershell
# 指定语言与唤醒词（默认 zh-CN；S3 默认 AFE 唤醒词 "nihaoxiaozhi"）
& "E:\App\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" scripts/build.py xiaozhi-pcb-arm --language zh-CN --wake-word nihaoxiaozhi

# 额外生成 releases/v<版本>_xiaozhi-pcb-arm.zip
... scripts/build.py xiaozhi-pcb-arm --zip
```

#### 2.2 分区布局（v2/16m.csv，供手动烧录参考）

| 分区 | 偏移 | 大小 |
| --- | --- | --- |
| bootloader (S3) | `0x0` | — |
| partition table | `0x8000` | 0x1000 |
| nvs | `0x9000` | 0x4000 |
| otadata | `0xd000` | 0x2000 |
| phy_init | `0xf000` | 0x1000 |
| ota_0 (app) | `0x20000` | 0x3f0000 |
| ota_1 (app) | `0x410000` | 0x3f0000 |
| assets（表情/字体/语音资源） | `0x800000` | 8MB |

---

## 3. 首次烧录后的启动流程

1. 上电 → OLED 出现启动画面 → 日志打印 `UUID=... SKU=xiaozhi-pcb-arm`。
2. 首次无 Wi-Fi 配置时设备进入配网模式（热点配网，`78/esp-wifi-connect` 组件），按 BOOT 键也可在启动阶段手动进入。
3. 联网后设备到 `https://api.tenclass.net/xiaozhi/ota/`（默认 OTA URL）校验版本并获取激活码 → 登录 [xiaozhi.me](https://xiaozhi.me) 控制台用激活码绑定设备。
4. 激活完成进入 Idle，说"你好小智"即可唤醒对话。

BOOT 按键行为（板级实现）：

| 动作 | 效果 |
| --- | --- |
| 单击（Starting 状态） | 进入 Wi-Fi 配网模式 |
| 单击（其他状态） | 开始/结束对话（ToggleChatState） |
| 长按 | 立即刹车电机 + 关闭雷达电机 PWM（安全停止） |

---

## 4. 烧录方法

### 方式 1：Flash Download Tool（图形界面，最省事）

1. 下载乐鑫官方 [Flash Download Tool](https://www.espressif.com/zh-hans/support/download/other-tools)。
2. 芯片类型选 `ESP32-S3`，工作模式 `Develop`。
3. 加载文件：`build/merged-binary.bin`，烧录地址填 `0x0`。
4. SPI Speed：40MHz；SPI Mode：DIO；勾选"DoNotChgBin"。
5. 选择 COM 口（设备管理器确认，波特率可 921600），点 `ERASE` → `START`。

### 方式 2：esptool 命令行

```powershell
& "E:\App\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" -m esptool `
  --chip esp32s3 --port COM5 --baud 921600 write_flash 0x0 build\merged-binary.bin
```

### 方式 3：idf.py（开发迭代用）

```powershell
# 在已正确 export 的 ESP-IDF 终端中（VS Code ESP-IDF 插件底部栏可直接点 Build/Flash/Monitor）
idf.py -p COM5 flash monitor
```

串口监视器波特率 **115200**。

---

## 5. 烧录后验证清单

| 检查项 | 预期 |
| --- | --- |
| 上电日志 | `I (xxx) main: UUID=... SKU=xiaozhi-pcb-arm` |
| OLED | `SSD1306 driver installed`，显示机器人 UI：顶部信号格/状态或时钟/电池，下方语音柱动画 |
| 音频 | 对设备说话，日志出现 AFE 唤醒词命中（需先完成配网激活） |
| 电机工具 | 在 xiaozhi.me 对话中让 AI 调用 `robot.motion.forward` 等（见第 6 节） |
| 雷达 | `robot.lidar.status` 返回 `frames_received` 递增（雷达供电与波特率匹配时） |

---

## 6. 设备端 MCP 工具（AI 可调用）

### OLED 自定义 UI（`robot_oled_display.cc`）

本板 OLED 不使用框架默认的"表情 + 聊天字幕"布局，改为按《OLED显示参考.md》实现的机器人 UI（`RobotOledDisplay`，继承 `OledDisplay`）：

| 区域 | 内容 | 行为 |
| --- | --- | --- |
| 顶部状态栏（0..15 行） | 左：4 格 Wi-Fi 信号；中：状态文本；右：电池图标 | 状态文本/通知复用框架 `SetStatus` / `ShowNotification`；Idle 且系统时间有效时显示 `HH:MM` 时钟 |
| 分隔线（第 15 行） | 横线 | 与参考图一致 |
| 语音波形区（16..63 行） | 60 根 1px 离散柱（两端各 4 根固定基线） | LVGL 33ms 定时器驱动：汉宁窗 + 双频谐波 + 随机毛刺；Listening/Speaking/Notifying 状态模拟"吐字节奏"（50~130ms 随机切换能量，20% 停顿），Idle 时回落基线微点 |

语音能量是状态模拟（不是真实音频采样）：与 AFE 的说话状态联动，说话时跳动、空闲时静止，视觉与参考工程一致且不占用音频/CPU 资源。聊天字幕被有意省略；如需恢复字幕可在 `SetupUI()` 中参照 `OledDisplay::SetupUI_128x64()` 添加。

| 工具名 | 功能 | 状态 |
| --- | --- | --- |
| `robot.motion.forward` | 双电机前进（60%） | 可用 |
| `robot.motion.backward` | 双电机后退（-60%） | 可用 |
| `robot.motion.turn_left` | 左转（左 -45% / 右 45%） | 可用 |
| `robot.motion.turn_right` | 右转（左 45% / 右 -45%） | 可用 |
| `robot.motion.stop` | 停止（滑行） | 可用 |
| `robot.motion.brake` | 双路刹车（IN1=IN2=高） | 可用 |
| `robot.lidar.status` | 雷达状态（帧数/字节/CRC 错误/电机占空比） | 可用 |

编码器计数通过 `GetDeviceStatusJson()` 上报（`robot.left_encoder` / `robot.right_encoder`）。

---

## 8. 安全警告与已知 TODO（骨架阶段）

- **AT8870 逻辑输入脚已按原理图定稿：17/18/38/47**（`config.h` 生效中）。首次上电机验证前仍建议：
  1. 确认这四个 GPIO 没有接到其他外设；
  2. 建议先断开电机动力电（AT8870 VCC），只验证语音/显示/雷达链路；
  3. 如原理图再次变更，修改 `config.h` 中 `MOTOR_A_IN1_PIN` 等宏并重新编译。
- 双 GC9A01 圆屏：骨架仅在日志中提示未接入，不影响 OLED 主显示；后续按 LVGL 双显示或镜像刷新实现。
- 雷达解析默认 LD-06/LD-19 协议（47 字节帧、12 点、末字节累加和校验），实际雷达型号不一致时改 `lidar_driver.cc` 的 `ParseFrame()`。
- 电机当前为开环 PWM，速度闭环（编码器 PID）与里程计累计尚未实现。
- 本工作区在文档撰写时 build 目录被后台 kconfserver 进程占用，**完整编译尚未跑通**；`--list-boards` 已验证能枚举到本板卡，第 2.1 节命令为实测到 reconfigure 阶段的版本。

---

## 9. 故障排查

| 现象 | 处理 |
| --- | --- |
| `No module named 'esp_idf_monitor'` | 没用 IDF venv 的 python 运行 `build.py`，见 2.1 节第 1 条 |
| `PermissionError: build\.ninja_log` | 关闭 VS Code ESP-IDF 插件构建/结束残留 kconfserver 进程后重试 |
| `Failed to detect modem` 循环 | 当前变体为 Wi-Fi 版，未启用 4G；确认没选错固件变体 |
| OLED 无显示 | 检查 I2C 地址 0x3C、GPIO41/42 接线；日志出现 `OLED init failed` 时框架自动回退 NoDisplay |
| 唤醒词不生效 | 确认模组为 N16R8（OPI PSRAM）；`menuconfig` 中 `WAKE_WORD_TYPE` 默认 AFE，不要关闭 |
| OTA 升级覆盖风险 | 保持 `config.json` 的 `type` 唯一且不变，避免与其他板卡固件互相覆盖 |
