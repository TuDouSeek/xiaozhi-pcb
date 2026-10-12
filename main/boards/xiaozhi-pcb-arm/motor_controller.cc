#include "motor_controller.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cmath>
#include <initializer_list>

// PCNT quadrature decoding driver (driver/pcnt.h) is part of esp_driver_pcnt.
// PRIV_REQUIRES in main/CMakeLists.txt already lists the core esp drivers; if
// the build reports a missing pcnt.h, add esp_driver_pcnt there.

#define TAG "MotorCtrl"

namespace {

constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr int kDutyMax = (1 << 10) - 1;
constexpr ledc_channel_t kChannelA1 = LEDC_CHANNEL_0;
constexpr ledc_channel_t kChannelA2 = LEDC_CHANNEL_1;
constexpr ledc_channel_t kChannelB1 = LEDC_CHANNEL_2;
constexpr ledc_channel_t kChannelB2 = LEDC_CHANNEL_3;

bool IsValidPin(gpio_num_t pin) {
    return pin >= 0 && pin != GPIO_NUM_NC;
}

}  // namespace

MotorController::MotorController(const Config& config) : config_(config) {
    dir_sign_[0] = config_.invert_a ? -1 : 1;
    dir_sign_[1] = config_.invert_b ? -1 : 1;
    if (IsValidPin(config_.stby)) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << config_.stby,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(config_.stby, 1);
    }

    InitPwm();

    encoders_ready_ = InitEncoder(config_.enc_a1, config_.enc_a2, encoder_a_) &&
                      InitEncoder(config_.enc_b1, config_.enc_b2, encoder_b_);
}

MotorController::~MotorController() {
    Stop();
    for (auto* encoder : {&encoder_a_, &encoder_b_}) {
        if (encoder->unit != nullptr) {
            pcnt_unit_disable(encoder->unit);
            pcnt_del_unit(encoder->unit);
        }
    }
}

void MotorController::InitPwm() {
    if (!IsValidPin(config_.in_a1) || !IsValidPin(config_.in_a2) ||
        !IsValidPin(config_.in_b1) || !IsValidPin(config_.in_b2)) {
        ESP_LOGW(TAG, "AT8870 input pins not configured (GPIO_NUM_NC), motors disabled");
        return;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = kDutyResolution,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    const gpio_num_t pins[4] = {config_.in_a1, config_.in_a2, config_.in_b1, config_.in_b2};
    const ledc_channel_t channels[4] = {kChannelA1, kChannelA2, kChannelB1, kChannelB2};
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = {.output_invert = 0},
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
    }
    motors_ready_ = true;
    ESP_LOGI(TAG, "AT8870 PWM ready (A: IN1=%d IN2=%d, B: IN1=%d IN2=%d)", config_.in_a1,
             config_.in_a2, config_.in_b1, config_.in_b2);
}

bool MotorController::InitEncoder(gpio_num_t pin_a, gpio_num_t pin_b, EncoderUnit& unit) {
    if (!IsValidPin(pin_a) || !IsValidPin(pin_b)) {
        ESP_LOGW(TAG, "Encoder pins not configured (GPIO_NUM_NC), encoder disabled");
        return false;
    }

    pcnt_unit_config_t unit_conf = {
        .low_limit = -1000,
        .high_limit = 1000,
        .intr_priority = 0,
        .flags =
            {
                .accum_count = true,  // keep counting past low/high limit
            },
    };
    esp_err_t err = pcnt_new_unit(&unit_conf, &unit.unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_unit failed: %s", esp_err_to_name(err));
        return false;
    }

    pcnt_chan_config_t chan_a_conf = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
        .flags = {.invert_edge_input = 0, .invert_level_input = 0},
    };
    pcnt_chan_config_t chan_b_conf = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
        .flags = {.invert_edge_input = 0, .invert_level_input = 0},
    };
    pcnt_channel_handle_t chan_a = nullptr;
    pcnt_channel_handle_t chan_b = nullptr;
    err = pcnt_new_channel(unit.unit, &chan_a_conf, &chan_a);
    if (err == ESP_OK) {
        err = pcnt_new_channel(unit.unit, &chan_b_conf, &chan_b);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_channel failed: %s", esp_err_to_name(err));
        pcnt_del_unit(unit.unit);
        unit.unit = nullptr;
        return false;
    }

    // Standard quadrature decoding: A/B edges increment/decrement with level
    // direction detection, x4 counts per encoder line cycle.
    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_ERROR_CHECK(pcnt_unit_enable(unit.unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit.unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit.unit));
    unit.ready = true;
    return true;
}

void MotorController::ApplyChannel(int index, int percent) {
    if (!motors_ready_) {
        return;
    }
    current_percent_[index] = percent;
    // 相位取反轮：交换正反通道，等效物理方向与 A 轮对齐。
    const int p = percent * dir_sign_[index];

    const ledc_channel_t forward = (index == 0) ? kChannelA1 : kChannelB1;
    const ledc_channel_t reverse = (index == 0) ? kChannelA2 : kChannelB2;
    int duty = abs(p) * kDutyMax / 100;
    if (duty > 0 && duty < 80) {
        // Keep a small floor so the AT8870 actually starts driving the motor.
        duty = 80;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, forward, p > 0 ? duty : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, forward);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, reverse, p < 0 ? duty : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, reverse);
}

void MotorController::SetSpeed(int left_percent, int right_percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    left_percent = left_percent < -100 ? -100 : (left_percent > 100 ? 100 : left_percent);
    right_percent = right_percent < -100 ? -100 : (right_percent > 100 ? 100 : right_percent);
    if (closed_loop_.load()) {
        // 闭环模式：percent 按空载满速线性映射为目标 RPM，由 PID 跟踪。
        target_rpm_[0].store(left_percent * no_load_rpm_ / 100.0f);
        target_rpm_[1].store(right_percent * no_load_rpm_ / 100.0f);
        return;
    }
    ApplyChannel(0, left_percent);
    ApplyChannel(1, right_percent);
}

void MotorController::Stop() {
    SetSpeed(0, 0);
}

void MotorController::Brake() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motors_ready_) {
        return;
    }
    current_percent_[0] = 0;
    current_percent_[1] = 0;
    for (auto channel : {kChannelA1, kChannelA2, kChannelB1, kChannelB2}) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, kDutyMax);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }
}

void MotorController::BrakeChannel(int index) {    std::lock_guard<std::mutex> lock(mutex_);
    if (!motors_ready_ || (index != 0 && index != 1)) {
        return;
    }
    current_percent_[index] = 0;
    const ledc_channel_t c1 = (index == 0) ? kChannelA1 : kChannelB1;
    const ledc_channel_t c2 = (index == 0) ? kChannelA2 : kChannelB2;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, c1, kDutyMax);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, c1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, c2, kDutyMax);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, c2);
}

void MotorController::SetChannelSpeed(int index, int percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index != 0 && index != 1) {
        return;
    }
    percent = percent < -100 ? -100 : (percent > 100 ? 100 : percent);
    if (closed_loop_.load()) {
        target_rpm_[index].store(percent * no_load_rpm_ / 100.0f);
        return;
    }
    current_percent_[index] = percent;
    ApplyChannel(index, percent);
}

int32_t MotorController::GetLeftCount() const {
    int count = 0;
    if (encoder_a_.ready) {
        pcnt_unit_get_count(encoder_a_.unit, &count);
    }
    return static_cast<int32_t>(count * dir_sign_[0]);
}

int32_t MotorController::GetRightCount() const {
    int count = 0;
    if (encoder_b_.ready) {
        pcnt_unit_get_count(encoder_b_.unit, &count);
    }
    return static_cast<int32_t>(count * dir_sign_[1]);
}

void MotorController::ClearCounts() {
    if (encoder_a_.ready) {
        pcnt_unit_clear_count(encoder_a_.unit);
    }
    if (encoder_b_.ready) {
        pcnt_unit_clear_count(encoder_b_.unit);
    }
}

// ---- 测速与 PID 闭环 ----
void MotorController::SetGearRatio(float ratio) {
    if (ratio > 1.0f) {
        gear_ratio_ = ratio;
    }
}

float MotorController::GetLeftRpm() const { return rpm_[0]; }
float MotorController::GetRightRpm() const { return rpm_[1]; }

void MotorController::SetPidGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void MotorController::SetNoLoadRpm(float rpm) {
    if (rpm > 1.0f) {
        no_load_rpm_ = rpm;
    }
}

void MotorController::EnableClosedLoop(bool enable) {
    if (enable == closed_loop_.load()) {
        return;
    }
    // 切换模式时清 PID 状态与目标，防止积分残留造成冲击。
    integ_[0] = integ_[1] = 0.0f;
    prev_err_[0] = prev_err_[1] = 0.0f;
    target_rpm_[0].store(0.0f);
    target_rpm_[1].store(0.0f);
    closed_loop_.store(enable);
    if (!enable) {
        Stop();
    }
    ESP_LOGI(TAG, "Closed-loop %s", enable ? "ENABLED" : "disabled");
}

bool MotorController::IsClosedLoop() const { return closed_loop_.load(); }

void MotorController::SetTargetRpm(float left_rpm, float right_rpm) {
    target_rpm_[0].store(left_rpm);
    target_rpm_[1].store(right_rpm);
}

float MotorController::CountsPerRev() const {
    return 11.0f * gear_ratio_ * 4.0f;  // PPR × 减速比 × 正交四倍频
}

void MotorController::UpdateSpeed(uint32_t dt_ms) {
    if (dt_ms == 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const float cpr = CountsPerRev();
    for (int i = 0; i < 2; ++i) {
        int count = 0;
        const EncoderUnit& enc = (i == 0) ? encoder_a_ : encoder_b_;
        if (enc.ready) {
            pcnt_unit_get_count(enc.unit, &count);
        }
        // 差分 → RPM：dN/CPR 转 × (60000/dt_ms)，取反轮同步反转符号。
        const int signed_count = count * dir_sign_[i];
        float inst = (signed_count - last_count_[i]) * 60000.0f / (cpr * dt_ms);
        last_count_[i] = signed_count;
        // 一阶 EMA 平滑，抑制量化抖动（20ms 周期下 α=0.35 约 60ms 响应）。
        rpm_[i] += (inst - rpm_[i]) * 0.35f;
        if (closed_loop_.load() && motors_ready_) {
            const float err = target_rpm_[i].load() - rpm_[i];
            const float dt = dt_ms / 1000.0f;
            // 位置式 PID + 条件积分（输出饱和时停积，抗 windup）。
            float up = kp_ * err;
            float ud = kd_ * (err - prev_err_[i]) / dt;
            float out = up + integ_[i] + ud;
            float clamped = out < -100.0f ? -100.0f : (out > 100.0f ? 100.0f : out);
            if (clamped == out) {
                integ_[i] += ki_ * err * dt;
                const float i_max = 60.0f;
                if (integ_[i] > i_max) {
                    integ_[i] = i_max;
                } else if (integ_[i] < -i_max) {
                    integ_[i] = -i_max;
                }
                out = up + integ_[i] + ud;
                clamped = out < -100.0f ? -100.0f : (out > 100.0f ? 100.0f : out);
            }
            prev_err_[i] = err;
            ApplyChannel(i, (int)lroundf(clamped));
        }
    }
}

void MotorController::SpeedTask(void* arg) {
    auto* self = static_cast<MotorController*>(arg);
    uint32_t last = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t stat = last;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(self->speed_period_ms_));
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        const uint32_t dt = now - last;
        last = now;
        self->UpdateSpeed(dt);
        if (now - stat >= 2000) {
            stat = now;
            ESP_LOGI(TAG, "RPM L=%.0f R=%.0f cnt=(%ld,%ld)%s", self->rpm_[0],
                     self->rpm_[1], (long)self->last_count_[0],
                     (long)self->last_count_[1],
                     self->closed_loop_.load() ? " [CLOSED]" : "");
        }
    }
}

bool MotorController::StartSpeedTask(uint32_t period_ms) {
    if (speed_task_started_) {
        return true;
    }
    if (period_ms < 5) {
        period_ms = 5;
    } else if (period_ms > 200) {
        period_ms = 200;
    }
    speed_period_ms_ = period_ms;
    if (xTaskCreatePinnedToCore(SpeedTask, "motor_speed", 3072, this, 2, nullptr, 1) !=
        pdPASS) {
        ESP_LOGE(TAG, "Speed task create failed");
        return false;
    }
    speed_task_started_ = true;
    ESP_LOGI(TAG, "Speed task @%ums on core 1", (unsigned)period_ms);
    return true;
}
