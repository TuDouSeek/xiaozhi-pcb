#include "robot_imu.h"

#include <math.h>
#include <string.h>

#include <atomic>

#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Bosch 官方 8KB 初始化镜像（BMI270 必须上传后才能启动陀螺/加速度计）。
// 头文件由 main/CMakeLists.txt 追加的 firmware 目录提供。
#include "bmi270_image.h"

#define TAG "RobotImu"

namespace {

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegDataStart = 0x0C;  // acc_x lsb .. gyro_z msb，共 12 字节
constexpr uint8_t kRegInternalStatus = 0x21;
constexpr uint8_t kRegTempLsb = 0x22;
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyrConf = 0x42;
constexpr uint8_t kRegGyrRange = 0x43;
constexpr uint8_t kRegInitAddr0 = 0x59;
constexpr uint8_t kRegInitAddr1 = 0x5A;
constexpr uint8_t kRegInitCtrl = 0x5B;
constexpr uint8_t kRegInitData = 0x5E;
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;
constexpr uint8_t kRegCmd = 0x7E;

constexpr uint8_t kChipIdBmi270 = 0x24;
constexpr uint8_t kAccConf200Hz = 0xA9;   // perf filter + normal BW + 200Hz
constexpr uint8_t kAccRange8G = 0x02;
constexpr uint8_t kGyrConf200Hz = 0xA9;
constexpr uint8_t kGyrRange1000Dps = 0x01;
constexpr uint8_t kPwrCtrlAll = 0x0E;     // temp + gyro + acc

constexpr float kGyrLsbPerDps = 32.8f;    // ±1000 dps
constexpr float kAccLsbPerG = 4096.0f;    // ±8 g
constexpr int kSamplePeriodMs = 5;        // 200Hz
constexpr int kCalibSamples = 300;        // 1.5s

i2c_master_dev_handle_t s_dev = nullptr;
i2c_master_bus_handle_t s_bus = nullptr;
bool s_ready = false;

std::atomic<float> s_yaw{0.0f};
std::atomic<float> s_gz{0.0f};
std::atomic<float> s_pitch{0.0f};
std::atomic<float> s_roll{0.0f};
std::atomic<float> s_bias{0.0f};
std::atomic<float> s_temp{0.0f};
std::atomic<bool> s_calib_done{false};
std::atomic<bool> s_calib_request{true};  // 上电默认先标定
std::atomic<bool> s_reset_yaw_request{false};
std::atomic<float> s_pending_yaw{0.0f};

esp_err_t WriteReg(uint8_t reg, uint8_t value) {
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

esp_err_t ReadRegs(uint8_t reg, uint8_t* out, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100);
}

esp_err_t UploadConfig() {
    const uint8_t* image = bmi270_config_file;
    const size_t size = sizeof(bmi270_config_file);
    if (size % 32 != 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_RETURN_ON_ERROR(WriteReg(kRegInitAddr0, 0x00), TAG, "init addr0");
    ESP_RETURN_ON_ERROR(WriteReg(kRegInitAddr1, 0x00), TAG, "init addr1");
    ESP_RETURN_ON_ERROR(WriteReg(kRegInitCtrl, 0x00), TAG, "init ctrl");

    for (size_t offset = 0; offset < size; offset += 32) {
        uint8_t buf[33];
        buf[0] = kRegInitData;
        memcpy(buf + 1, image + offset, 32);
        esp_err_t err = i2c_master_transmit(s_dev, buf, sizeof(buf), 200);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "配置上传失败 offset=%u: %s", (unsigned)offset,
                     esp_err_to_name(err));
            return err;
        }
    }
    ESP_RETURN_ON_ERROR(WriteReg(kRegInitCtrl, 0x01), TAG, "init start");

    // INTERNAL_STATUS[3:0] == 0x01 表示初始化完成。
    for (int i = 0; i < 100; ++i) {
        uint8_t status = 0;
        if (ReadRegs(kRegInternalStatus, &status, 1) == ESP_OK &&
            (status & 0x0F) == 0x01) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGE(TAG, "配置上传后初始化超时");
    return ESP_ERR_TIMEOUT;
}

void CalibrationTick(int16_t raw_gz, float* sum, float* sum_sq, int* count) {
    const float dps = raw_gz / kGyrLsbPerDps;
    *sum += dps;
    *sum_sq += dps * dps;
    ++(*count);
}

void ImuTask(void* arg) {
    (void)arg;
    int64_t last_us = esp_timer_get_time();

    float calib_sum = 0.0f;
    float calib_sum_sq = 0.0f;
    int calib_count = 0;

    float gz_lp = 0.0f;
    float still_ms = 0.0f;

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.05f) {
            dt = kSamplePeriodMs * 0.001f;
        }

        uint8_t raw[12] = {};
        if (ReadRegs(kRegDataStart, raw, sizeof(raw)) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(kSamplePeriodMs));
            continue;
        }
        const int16_t ax = (int16_t)(raw[0] | (raw[1] << 8));
        const int16_t ay = (int16_t)(raw[2] | (raw[3] << 8));
        const int16_t az = (int16_t)(raw[4] | (raw[5] << 8));
        const int16_t gz = (int16_t)(raw[10] | (raw[11] << 8));

        uint8_t temp_raw[2] = {};
        if (ReadRegs(kRegTempLsb, temp_raw, sizeof(temp_raw)) == ESP_OK) {
            const int16_t t = (int16_t)(temp_raw[0] | (temp_raw[1] << 8));
            s_temp.store(23.0f + t / 512.0f);
        }

        const float gz_dps = gz / kGyrLsbPerDps;
        gz_lp += 0.3f * (gz_dps - gz_lp);

        if (s_calib_request.load()) {
            s_calib_done.store(false);
            CalibrationTick(gz, &calib_sum, &calib_sum_sq, &calib_count);
            if (calib_count >= kCalibSamples) {
                const float n = (float)calib_count;
                const float mean = calib_sum / n;
                const float var = calib_sum_sq / n - mean * mean;
                const float stddev = var > 0.0f ? sqrtf(var) : 0.0f;
                // 静止时标准差应远小于 2dps；机器人正在动则放弃本次标定。
                if (stddev < 2.0f) {
                    s_bias.store(mean);
                    s_calib_done.store(true);
                    s_yaw.store(0.0f);
                    ESP_LOGI(TAG, "陀螺零偏标定完成: %.3f dps (σ=%.3f)", (double)mean,
                             (double)stddev);
                } else {
                    s_bias.store(0.0f);
                    s_calib_done.store(true);
                    ESP_LOGW(TAG, "标定时检测到运动(σ=%.2f)，零偏按 0 处理",
                             (double)stddev);
                }
                s_calib_request.store(false);
                calib_sum = calib_sum_sq = 0.0f;
                calib_count = 0;
                gz_lp = 0.0f;
            }
        } else {
            // 长时间静止时缓慢跟踪零偏，抑制温漂。
            if (fabsf(gz_lp - s_bias.load()) < 0.6f) {
                still_ms += dt * 1000.0f;
            } else {
                still_ms = 0.0f;
            }
            if (still_ms > 2000.0f) {
                s_bias.store(s_bias.load() + 0.0002f * (gz_lp - s_bias.load()));
            }

            const float rate = gz_lp - s_bias.load();
            if (s_reset_yaw_request.exchange(false)) {
                s_yaw.store(s_pending_yaw.load());
            } else {
                s_yaw.store(s_yaw.load() + rate * dt);
            }
            s_gz.store(rate);
        }

        // 姿态角：静止/低速时用加速度计即可，颠簸时会短暂失真。
        const float fx = ax / kAccLsbPerG;
        const float fy = ay / kAccLsbPerG;
        const float fz = az / kAccLsbPerG;
        const float norm = sqrtf(fx * fx + fy * fy + fz * fz);
        if (norm > 0.5f && norm < 1.8f) {
            const float roll = atan2f(fy, fz) * 57.29578f;
            const float pitch = atan2f(-fx, sqrtf(fy * fy + fz * fz)) * 57.29578f;
            s_roll.store(roll);
            s_pitch.store(pitch);
        }

        vTaskDelay(pdMS_TO_TICKS(kSamplePeriodMs));
    }
}

}  // namespace

esp_err_t robot_imu_init(i2c_master_bus_handle_t bus, uint8_t addr) {
    if (s_ready) {
        return ESP_OK;
    }
    if (bus == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 400000;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "挂载 I2C 设备失败: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t chip_id = 0;
    if (ReadRegs(kRegChipId, &chip_id, 1) != ESP_OK || chip_id != kChipIdBmi270) {
        ESP_LOGW(TAG, "未找到 BMI270（chip_id=0x%02X，期望 0x24），检查 CS/SDO 接线",
                 chip_id);
        i2c_master_bus_rm_device(s_dev);
        s_dev = nullptr;
        return ESP_ERR_NOT_FOUND;
    }

    auto write_or_log = [](uint8_t reg, uint8_t value) {
        esp_err_t e = WriteReg(reg, value);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "写寄存器 0x%02X 失败: %s", reg, esp_err_to_name(e));
        }
        return e;
    };

    if ((err = write_or_log(kRegCmd, 0xB6)) != ESP_OK) {  // 软复位
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    if ((err = write_or_log(kRegPwrConf, 0x00)) != ESP_OK) {  // 关闭高级省电
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = UploadConfig();
    if (err != ESP_OK) {
        return err;
    }

    if ((err = write_or_log(kRegAccConf, kAccConf200Hz)) != ESP_OK ||
        (err = write_or_log(kRegAccRange, kAccRange8G)) != ESP_OK ||
        (err = write_or_log(kRegGyrConf, kGyrConf200Hz)) != ESP_OK ||
        (err = write_or_log(kRegGyrRange, kGyrRange1000Dps)) != ESP_OK ||
        (err = write_or_log(kRegPwrCtrl, kPwrCtrlAll)) != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(60));  // 陀螺启动时间 45ms

    s_ready = true;
    xTaskCreatePinnedToCore(ImuTask, "robot_imu", 4096, nullptr, 3, nullptr, 1);
    ESP_LOGI(TAG, "BMI270 就绪：200Hz 采样，I2C 0x%02X", addr);
    return ESP_OK;
}

bool robot_imu_is_ready() { return s_ready; }

bool robot_imu_calib_done() { return s_calib_done.load(); }

void robot_imu_read(float* yaw_deg, float* gz_dps, float* pitch_deg, float* roll_deg) {
    if (yaw_deg != nullptr) {
        *yaw_deg = s_yaw.load();
    }
    if (gz_dps != nullptr) {
        *gz_dps = s_gz.load();
    }
    if (pitch_deg != nullptr) {
        *pitch_deg = s_pitch.load();
    }
    if (roll_deg != nullptr) {
        *roll_deg = s_roll.load();
    }
}

float robot_imu_bias_dps() { return s_bias.load(); }

float robot_imu_temperature() { return s_temp.load(); }

int robot_imu_scan(uint8_t* addresses, int max_count) {
    if (s_bus == nullptr || addresses == nullptr || max_count <= 0) {
        return 0;
    }
    int count = 0;
    for (uint8_t addr = 0x08; addr < 0x78 && count < max_count; ++addr) {
        if (i2c_master_probe(s_bus, addr, 20) == ESP_OK) {
            addresses[count++] = addr;
        }
    }
    return count;
}

void robot_imu_request_calibration() { s_calib_request.store(true); }

void robot_imu_reset_yaw(float yaw_deg) {
    s_pending_yaw.store(yaw_deg);
    s_reset_yaw_request.store(true);
}
