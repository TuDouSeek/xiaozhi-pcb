#ifndef __LD14_DRIVER_H__
#define __LD14_DRIVER_H__

// LD14 激光雷达独立驱动（与既有 LidarDriver 解耦）。
//   - UART2 RX=GPIO39（经原理图核对），固定 115200 8N1，独立 FreeRTOS 任务解析
//   - 47 字节定帧：0x54 帧头 + 转速 + 起止角度 + 12 点(距离+置信度) + 时间戳 + CRC8
//   - 360° 环形点云缓存（1°/bin，时间衰减 + 置信度滤波）
//   - GPIO40 LEDC PWM 电机调速（Timer3/Channel7，不占用既有 Timer1/Channel4）
//
// 注意：GPIO40 与既有 lidar PWM 复用同一引脚，两者只能启用其一。
// 若已接入 LD14，请停用既有 InitializeLidar()，改由 ld14_system_init() 接管。

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 供电机闭环/规划使用的结构化特征（角度单位均为度，距离单位均为 mm）。
typedef struct {
    uint16_t front_min_mm;    // 正前方 345°~15° 最小距离（前向速度规划/目标距离闭环）
    uint16_t left_mm;         // 左侧 75°~105° 均值（差速居中）
    uint16_t right_mm;        // 右侧 255°~285° 均值（差速居中）
    uint16_t gmin_mm;         // 全周最近障碍距离（急停保护）
    uint16_t gmin_angle;      // 全周最近障碍方位角
    uint16_t escape_angle;    // 扇区可穿行度最优避障航向角
    uint16_t rpm;             // 雷达实际转速（转/分）
    uint32_t stamp_ms;        // 特征计算时刻
} ld14_features_t;

void ld14_driver_init(void);   // 建 UART + 起解析任务 + 电机软启动（默认 60%）
void ld14_driver_deinit(void);

// 电机占空比 0~100（0=停转）。维持 6Hz 扫描约需 55~70，请实测微调。
void ld14_motor_set_duty(int percent);

// 取最新特征快照；无有效数据时返回 false。
bool ld14_get_features(ld14_features_t* out);

// 取 360° 距离快照（dist_mm[360]，0=未知/过期；conf 可为 NULL）。
// 返回有效 bin 数量。
int ld14_snapshot(uint16_t* dist_mm, uint8_t* conf);

// 取 120 点融合极坐标快照（3°/bin，索引 i 对应角度 i*3°，0=未知/过期）。
// 融合缓冲由串口解析持续写入，单帧 12 点不会整圈清零；返回有效点数。
int ld14_polar120_snapshot(uint16_t* out120);

#ifdef __cplusplus
}
#endif

#endif  // __LD14_DRIVER_H__
