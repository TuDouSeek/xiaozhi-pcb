#include "nav_control.h"

#include <math.h>
#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>

#include "ld14_driver.h"
#include "motor_controller.h"
#include "robot_imu.h"

#define TAG "NavControl"

namespace {

constexpr int kTickMs = 20;
constexpr float kMoveToleranceMm = 12.0f;
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kSlowDownMm = 150.0f;
constexpr float kMinSpeedPercent = 12.0f;
constexpr float kMinTurnPercent = 10.0f;
constexpr uint32_t kMoveTimeoutMs = 20000;
constexpr uint32_t kTurnTimeoutMs = 15000;
constexpr uint16_t kObstacleStopMm = 250;

constexpr const char* kNvsNamespace = "nav";
constexpr const char* kNvsKeyCpm = "cpm";
constexpr const char* kNvsKeyYawSign = "ysign";

// 68mm 轮径：3344 脉冲/圈 ÷ (π × 0.068m) ≈ 15655 脉冲/米。
constexpr float kDefaultCountsPerMeter = 15655.0f;

MotorController* s_motors = nullptr;
SemaphoreHandle_t s_lock = nullptr;

NavState s_state = NavState::kIdle;
const char* s_phase = "idle";
const char* s_reason = "";
float s_target = 0.0f;
float s_progress = 0.0f;
float s_yaw = 0.0f;
float s_dist_mm = 0.0f;
uint32_t s_start_ms = 0;
uint32_t s_elapsed_ms = 0;
int s_speed = 35;

int32_t s_start_left = 0;
int32_t s_start_right = 0;
float s_target_yaw = 0.0f;
uint32_t s_stall_since_ms = 0;

float s_counts_per_meter = kDefaultCountsPerMeter;
float s_yaw_sign = 1.0f;

uint32_t NowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

float Wrap180(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

int ClampSpeed(int speed_percent) {
    if (speed_percent < 10) {
        return 10;
    }
    return speed_percent > 100 ? 100 : speed_percent;
}

void LoadCalibration() {
    nvs_handle_t handle;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    float value = 0.0f;
    size_t len = sizeof(value);
    if (nvs_get_blob(handle, kNvsKeyCpm, &value, &len) == ESP_OK &&
        value > 1000.0f && value < 200000.0f) {
        s_counts_per_meter = value;
    }
    int8_t sign = 1;
    len = sizeof(sign);
    if (nvs_get_blob(handle, kNvsKeyYawSign, &sign, &len) == ESP_OK &&
        (sign == 1 || sign == -1)) {
        s_yaw_sign = (float)sign;
    }
    nvs_close(handle);
}

void SaveCalibration() {
    nvs_handle_t handle;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "打开 NVS 失败，标定未持久化");
        return;
    }
    nvs_set_blob(handle, kNvsKeyCpm, &s_counts_per_meter, sizeof(float));
    const int8_t sign = s_yaw_sign >= 0.0f ? 1 : -1;
    nvs_set_blob(handle, kNvsKeyYawSign, &sign, sizeof(sign));
    nvs_commit(handle);
    nvs_close(handle);
}

// 调用方需持有 s_lock。
void FinishLocked(const char* phase, const char* reason) {
    if (s_motors != nullptr) {
        s_motors->Brake();
    }
    s_phase = phase;
    s_reason = reason != nullptr ? reason : "";
    s_elapsed_ms = NowMs() - s_start_ms;
    s_state = NavState::kIdle;
    s_stall_since_ms = 0;
    ESP_LOGI(TAG, "动作结束: %s %s (%.1f, %ums)", phase, s_reason,
             (double)s_progress, (unsigned)s_elapsed_ms);
}

void MoveTickLocked() {
    const int32_t left = s_motors->GetLeftCount();
    const int32_t right = s_motors->GetRightCount();
    const float counts =
        0.5f * (float)((left - s_start_left) + (right - s_start_right));
    s_dist_mm = counts / s_counts_per_meter * 1000.0f;
    s_progress = s_dist_mm;
    s_elapsed_ms = NowMs() - s_start_ms;

    if (s_elapsed_ms > kMoveTimeoutMs) {
        FinishLocked("cancelled", "timeout");
        return;
    }
    const float error_mm = s_target - s_dist_mm;
    if (fabsf(error_mm) <= kMoveToleranceMm) {
        FinishLocked("done", "");
        return;
    }
    // 冲过目标（误差方向与目标相反）且幅度不大时不再倒车。
    if ((error_mm > 0.0f) != (s_target > 0.0f) && fabsf(error_mm) < 40.0f) {
        FinishLocked("done", "");
        return;
    }
    if (s_target > 0.0f) {
        ld14_features_t features = {};
        if (ld14_get_features(&features) && features.front_min_mm > 0 &&
            features.front_min_mm < kObstacleStopMm) {
            FinishLocked("cancelled", "obstacle");
            return;
        }
    }

    float speed = (float)s_speed;
    if (fabsf(error_mm) < kSlowDownMm) {
        speed = fmaxf(kMinSpeedPercent, (float)s_speed * fabsf(error_mm) / kSlowDownMm);
    }
    speed = copysignf(speed, error_mm);
    const int output = (int)lroundf(speed);

    // 堵转保护：给了速度却轮子不转，1.2s 内判定为卡住。
    const float rpm = fmaxf(fabsf(s_motors->GetLeftRpm()),
                            fabsf(s_motors->GetRightRpm()));
    if (fabsf(speed) > 25.0f && rpm < 8.0f) {
        if (s_stall_since_ms == 0) {
            s_stall_since_ms = s_elapsed_ms;
        } else if (s_elapsed_ms - s_stall_since_ms > 1200) {
            FinishLocked("cancelled", "stall");
            return;
        }
    } else {
        s_stall_since_ms = 0;
    }

    s_motors->SetSpeed(output, output);
}

void TurnTickLocked() {
    float yaw = 0.0f;
    float gz = 0.0f;
    robot_imu_read(&yaw, &gz, nullptr, nullptr);
    s_yaw = yaw * s_yaw_sign;
    gz *= s_yaw_sign;
    s_progress = Wrap180(s_yaw - (s_target_yaw - s_target));
    s_elapsed_ms = NowMs() - s_start_ms;

    if (!robot_imu_is_ready()) {
        FinishLocked("cancelled", "imu");
        return;
    }
    if (s_elapsed_ms > kTurnTimeoutMs) {
        FinishLocked("cancelled", "timeout");
        return;
    }
    const float error = Wrap180(s_target_yaw - s_yaw);
    if (fabsf(error) <= kTurnToleranceDeg) {
        FinishLocked("done", "");
        return;
    }

    float speed = 0.55f * error - 0.08f * gz;
    if (fabsf(speed) < kMinTurnPercent) {
        speed = copysignf(kMinTurnPercent, error);
    }
    const float limit = (float)s_speed;
    if (speed > limit) {
        speed = limit;
    } else if (speed < -limit) {
        speed = -limit;
    }
    const int output = (int)lroundf(speed);
    // 正角度 = 逆时针（左转）：左轮后退、右轮前进。
    s_motors->SetSpeed(-output, output);
}

void NavTask(void* arg) {
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kTickMs));
        if (s_motors == nullptr) {
            continue;
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_state == NavState::kMove) {
            MoveTickLocked();
        } else if (s_state == NavState::kTurn) {
            TurnTickLocked();
        }
        xSemaphoreGive(s_lock);
    }
}

}  // namespace

esp_err_t nav_control_init(MotorController* motors) {
    if (motors == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_motors != nullptr) {
        return ESP_OK;
    }
    s_motors = motors;
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    LoadCalibration();
    xTaskCreatePinnedToCore(NavTask, "nav_control", 4096, nullptr, 2, nullptr, 1);
    ESP_LOGI(TAG, "闭环动作就绪: %.0f 脉冲/米, yaw_sign=%.0f",
             (double)s_counts_per_meter, (double)s_yaw_sign);
    return ESP_OK;
}

bool nav_start_move(float dist_mm, int speed_percent) {
    if (s_motors == nullptr) {
        return false;
    }
    if (fabsf(dist_mm) < 5.0f) {
        return false;
    }
    if (fabsf(dist_mm) > 20000.0f) {
        ESP_LOGW(TAG, "距离 %.0fmm 超限", (double)dist_mm);
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = NavState::kMove;
    s_phase = "moving";
    s_reason = "";
    s_target = dist_mm;
    s_speed = ClampSpeed(speed_percent);
    s_start_left = s_motors->GetLeftCount();
    s_start_right = s_motors->GetRightCount();
    s_start_ms = NowMs();
    s_elapsed_ms = 0;
    s_progress = 0.0f;
    s_dist_mm = 0.0f;
    s_stall_since_ms = 0;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "开始走直线: %.0fmm，%d%%", (double)dist_mm, s_speed);
    return true;
}

bool nav_start_turn(float angle_deg, int speed_percent) {
    if (s_motors == nullptr) {
        return false;
    }
    if (!robot_imu_is_ready()) {
        ESP_LOGW(TAG, "IMU 未就绪，无法按角度转向");
        return false;
    }
    if (fabsf(angle_deg) < 1.0f || fabsf(angle_deg) > 3600.0f) {
        return false;
    }
    float yaw = 0.0f;
    robot_imu_read(&yaw, nullptr, nullptr, nullptr);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = NavState::kTurn;
    s_phase = "turning";
    s_reason = "";
    s_target = angle_deg;
    s_speed = ClampSpeed(speed_percent);
    s_yaw = yaw * s_yaw_sign;
    s_target_yaw = s_yaw + angle_deg;
    s_start_ms = NowMs();
    s_elapsed_ms = 0;
    s_progress = 0.0f;
    s_dist_mm = 0.0f;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "开始转向: %.0f°，%d%%", (double)angle_deg, s_speed);
    return true;
}

void nav_cancel(const char* reason) {
    if (s_lock == nullptr) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state != NavState::kIdle) {
        FinishLocked("cancelled", reason != nullptr ? reason : "user");
    }
    xSemaphoreGive(s_lock);
}

bool nav_is_active() {
    if (s_lock == nullptr) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool active = s_state != NavState::kIdle;
    xSemaphoreGive(s_lock);
    return active;
}

void nav_get_status(NavStatus* out) {
    if (out == nullptr) {
        return;
    }
    if (s_lock == nullptr) {
        out->imu_ok = robot_imu_is_ready();
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->state = s_state;
    out->target = s_target;
    out->progress = s_progress;
    out->yaw = s_yaw;
    out->dist_mm = s_dist_mm;
    out->elapsed_ms = s_state == NavState::kIdle ? s_elapsed_ms
                                                 : NowMs() - s_start_ms;
    out->phase = s_phase;
    out->reason = s_reason;
    out->imu_ok = robot_imu_is_ready();
    xSemaphoreGive(s_lock);
}

bool nav_get_calibration(float* counts_per_m, float* yaw_sign) {
    if (counts_per_m == nullptr || yaw_sign == nullptr) {
        return false;
    }
    *counts_per_m = s_counts_per_meter;
    *yaw_sign = s_yaw_sign;
    return true;
}

bool nav_set_calibration(float counts_per_m, float yaw_sign, bool save) {
    if (counts_per_m < 1000.0f || counts_per_m > 200000.0f) {
        return false;
    }
    if (yaw_sign != 1.0f && yaw_sign != -1.0f) {
        return false;
    }
    s_counts_per_meter = counts_per_m;
    s_yaw_sign = yaw_sign;
    if (save) {
        SaveCalibration();
    }
    return true;
}
