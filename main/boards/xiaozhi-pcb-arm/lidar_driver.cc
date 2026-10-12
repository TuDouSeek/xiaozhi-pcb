#include "lidar_driver.h"

#include <cmath>
#include <cstring>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>

#define TAG "Lidar"

namespace {

// LD-06 style frame: 0x54 0x2C header + speed + start_angle, 12 points
// (2-byte distance + 1-byte quality each), end_angle, timestamp, checksum.
constexpr size_t kFrameSize = 47;
constexpr uint8_t kFrameHeader0 = 0x54;
constexpr uint8_t kFrameHeader1 = 0x2C;
constexpr int kPointsPerFrame = 12;
constexpr size_t kReadChunk = 256;
constexpr int kTaskStack = 4096;
constexpr int kTaskPriority = 4;

}  // namespace

LidarDriver::LidarDriver(gpio_num_t uart_rx_pin, int baudrate, gpio_num_t pwm_pin)
    : uart_rx_pin_(uart_rx_pin), baudrate_(baudrate), pwm_pin_(pwm_pin) {
    uart_num_ = UART_NUM_1;
    scan_.resize(360);
}

LidarDriver::~LidarDriver() {
    Stop();
}

bool LidarDriver::Start() {
    if (running_.load()) {
        return true;
    }
    if (uart_rx_pin_ == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "Lidar RX pin not configured (GPIO_NUM_NC), lidar disabled");
        return false;
    }

    uart_config_t uart_config = {
        .baud_rate = baudrate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
#if SOC_UART_SUPPORT_REF_CLK
        .source_clk = UART_SCLK_REF_TICK,
#else
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    if (uart_driver_install(uart_num_, kReadChunk * 2, 0, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(uart_num_, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install lidar UART driver");
        return false;
    }
    uart_set_pin(uart_num_, UART_PIN_NO_CHANGE, uart_rx_pin_, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    status_.uart_ready = true;

    if (pwm_pin_ != GPIO_NUM_NC) {
        ledc_timer_config_t timer_conf = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = LEDC_TIMER_1,
            .freq_hz = 25000,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
        ledc_channel_config_t channel_conf = {
            .gpio_num = pwm_pin_,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_4,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = 0,
            .hpoint = 0,
            .flags = {.output_invert = 0},
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
    }

    running_.store(true);
    BaseType_t created = xTaskCreate(TaskEntry, "lidar_task", kTaskStack, this, kTaskPriority,
                                     nullptr);
    if (created != pdPASS) {
        running_.store(false);
        status_.uart_ready = false;
        ESP_LOGE(TAG, "Failed to create lidar task");
        return false;
    }
    ESP_LOGI(TAG, "Lidar started (RX=%d, %d baud, PWM=%d)", uart_rx_pin_, baudrate_, pwm_pin_);
    return true;
}

void LidarDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if (pwm_pin_ != GPIO_NUM_NC) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
    }
    uart_driver_delete(uart_num_);
    std::lock_guard<std::mutex> lock(mutex_);
    status_.uart_ready = false;
    status_.task_running = false;
}

void LidarDriver::SetMotorSpeed(int duty_percent) {
    duty_percent = duty_percent < 0 ? 0 : (duty_percent > 100 ? 100 : duty_percent);
    motor_duty_.store(duty_percent);
    if (pwm_pin_ == GPIO_NUM_NC) {
        return;
    }
    // LD-series lidar motors expect ~30% idle duty before spinning up.
    const int kMinSpinDuty = 30;
    int effective = duty_percent == 0 ? 0 : (kMinSpinDuty + (100 - kMinSpinDuty) * duty_percent / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, effective * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    std::lock_guard<std::mutex> lock(mutex_);
    status_.motor_duty_percent = duty_percent;
}

void LidarDriver::TaskEntry(void* arg) {
    static_cast<LidarDriver*>(arg)->Task();
    vTaskDelete(nullptr);
}

void LidarDriver::Task() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.task_running = true;
    }

    uint8_t buffer[kReadChunk];
    uint8_t frame[kFrameSize];
    size_t frame_len = 0;

    while (running_.load()) {
        int len = uart_read_bytes(uart_num_, buffer, kReadChunk, pdMS_TO_TICKS(50));
        if (len <= 0) {
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_.bytes_received += len;
        }
        for (int i = 0; i < len; i++) {
            if (frame_len == 0) {
                if (buffer[i] != kFrameHeader0) {
                    continue;
                }
            } else if (frame_len == 1 && buffer[i] != kFrameHeader1) {
                frame_len = 0;
                if (buffer[i] == kFrameHeader0) {
                    frame[frame_len++] = buffer[i];
                }
                continue;
            }
            frame[frame_len++] = buffer[i];
            if (frame_len == kFrameSize) {
                ParseFrame(frame, kFrameSize);
                frame_len = 0;
            }
        }
    }
}

bool LidarDriver::ParseFrame(const uint8_t* frame, size_t len) {
    if (len != kFrameSize) {
        return false;
    }
    // Simple checksum: last byte is the sum of bytes 0..45.
    uint8_t checksum = 0;
    for (size_t i = 0; i < kFrameSize - 1; i++) {
        checksum += frame[i];
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (checksum != frame[kFrameSize - 1]) {
            status_.crc_errors++;
            return false;
        }
        status_.frames_received++;
    }

    // Angles are stored in 0.01-degree units; interpolate the 12 points
    // between the frame's start and end angles.
    const float start_angle = (frame[4] | (frame[5] << 8)) / 100.0f;
    const float end_angle = (frame[42] | (frame[43] << 8)) / 100.0f;
    float last_angle = 0.0f;
    for (int p = 0; p < kPointsPerFrame; p++) {
        const uint8_t* point = &frame[6 + p * 3];
        uint16_t distance_mm = point[0] | (point[1] << 8);
        uint8_t quality = point[2];
        if (distance_mm == 0) {
            continue;
        }
        const float t = static_cast<float>(p) / (kPointsPerFrame - 1);
        float angle = start_angle + (end_angle - start_angle) * t;
        angle = fmodf(angle, 360.0f);
        if (angle < 0) {
            angle += 360.0f;
        }
        last_angle = angle;
        std::lock_guard<std::mutex> lock(mutex_);
        scan_[static_cast<size_t>(angle)] = {angle, distance_mm, quality};
    }
    (void)last_angle;

    if (on_frame_received) {
        on_frame_received();
    }
    return true;
}

std::vector<LidarDriver::ScanPoint> LidarDriver::GetLatestScan() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ScanPoint> result;
    result.reserve(scan_.size());
    for (const auto& point : scan_) {
        if (point.distance_mm > 0) {
            result.push_back(point);
        }
    }
    return result;
}

LidarDriver::Status LidarDriver::GetStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}