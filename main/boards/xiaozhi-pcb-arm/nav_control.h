#ifndef __NAV_CONTROL_H__
#define __NAV_CONTROL_H__

#include <cstdint>

#include <esp_err.h>

class MotorController;

// 闭环动作：按编码器走直线距离、按 IMU 航向转固定角度。
// 由 50Hz 后台任务执行，HTTP(/api/nav)、MQTT、本地 WS 均可触发/取消。
enum class NavState : uint8_t { kIdle = 0, kMove, kTurn };

struct NavStatus {
    NavState state = NavState::kIdle;
    float target = 0.0f;      // move: 目标 mm；turn: 目标角度（相对）
    float progress = 0.0f;    // move: 已走 mm；turn: 已转角
    float yaw = 0.0f;         // 当前航向（deg，已应用 yaw_sign）
    float dist_mm = 0.0f;     // 本次动作的编码器里程（带符号）
    uint32_t elapsed_ms = 0;
    const char* phase = "idle";   // idle/moving/turning/done/cancelled
    const char* reason = "";      // 取消原因
    bool imu_ok = false;
};

esp_err_t nav_control_init(MotorController* motors);

// 距离 mm 正=前进、负=后退；speed 为百分比 10..100。
bool nav_start_move(float dist_mm, int speed_percent);
// 角度正=左转(CCW)、负=右转；需要 IMU 就绪。
bool nav_start_turn(float angle_deg, int speed_percent);
void nav_cancel(const char* reason);
bool nav_is_active();

void nav_get_status(NavStatus* out);

// counts_per_m = 每米编码器计数（默认按 68mm 轮径估算）。
bool nav_get_calibration(float* counts_per_m, float* yaw_sign);
bool nav_set_calibration(float counts_per_m, float yaw_sign, bool save);

#endif  // __NAV_CONTROL_H__
