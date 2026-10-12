#include "ld14_driver.h"

#include <string.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "LD14"

namespace {

// ---- 引脚与外设（与既有模块错开 UART/LEDC 编号，GPIO40 复用见头文件说明） ----
// 经原理图核对：LD14 TX -> GPIO39（UART RX），LD14 PWM -> GPIO40。
// GPIO39 原属旧 lidar 驱动（UART1），现旧驱动已禁用；本驱动用 UART2，无冲突。
constexpr uart_port_t kUart = UART_NUM_2;
constexpr gpio_num_t kRxPin = GPIO_NUM_39;
constexpr int kBaud = 115200;  // LD14 本机固定波特率，不做自适应切换
constexpr gpio_num_t kPwmPin = GPIO_NUM_40;
constexpr int kPwmFreqHz = 25000;

// ---- LD14 帧格式 ----
constexpr size_t kFrameLen = 47;
constexpr uint8_t kHeader0 = 0x54;
constexpr uint8_t kHeader1 = 0x2C;
constexpr int kPtsPerFrame = 12;
constexpr int kDefaultDuty = 60;

// ---- 点云缓存 ----
constexpr uint32_t kBinDecayMs = 1200;  // 超过此时长未更新的 bin 视为过期
constexpr uint8_t kMinConf = 10;        // 置信度门限

struct Bin {
    uint16_t dist_mm = 0;
    uint8_t conf = 0;
    uint32_t stamp_ms = 0;
};

Bin s_bins[360];

// ---- 120 点极坐标融合图（UDP 二进制流专用） ----
// 雷达单包只有 12 个点（约 10°~12°），直接快照会大面积为 0。
// 此处按 3°/bin 常驻融合：串口解包时写入，UDP 发送时按 400ms 过期淘汰。
// 与 s_bins 独立，避免改动既有特征链路。
constexpr int kPolar120 = 120;
constexpr uint32_t kPolarStaleMs = 400;  // 超期未更新视为丢失，置 0
constexpr uint16_t kMinValidDistMm = 80;  // 盲区误测过滤
uint16_t s_polar120[kPolar120] = {};
uint32_t s_polar_stamp[kPolar120] = {};
portMUX_TYPE s_spin = portMUX_INITIALIZER_UNLOCKED;
uint32_t s_frames = 0;
uint32_t s_crc_err = 0;
uint32_t s_bytes = 0;
float s_rpm_ema = 0.0f;
bool s_running = false;
bool s_first_synced = false;

uint32_t NowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

// CRC-8, poly 0x4D, MSB-first, init 0x00（LD 系列通用）。
uint8_t Crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void ParseFrame(const uint8_t* f) {
    // 校验：接受含帧头 / 不含帧头两种 CRC 口径（不同批次文档差异）。
    uint8_t expect = f[46];
    if (Crc8(f, 46) != expect && Crc8(f + 1, 45) != expect) {
        ++s_crc_err;
        return;
    }
    const uint16_t speed_ds = (uint16_t)(f[2] | (f[3] << 8));  // 度/秒
    const uint16_t start = (uint16_t)(f[4] | (f[5] << 8));     // 0.01°
    const uint16_t end = (uint16_t)(f[42] | (f[43] << 8));
    uint16_t diff = (end >= start) ? (uint16_t)(end - start) : (uint16_t)(end - start + 36000);
    const uint32_t now = NowMs();

    // 注意：临界区内严禁 ESP_LOGx（newlib/flash 锁 → abort）。首帧通知
    // 只在区内打标记，出区后打印。
    bool first = false;
    taskENTER_CRITICAL(&s_spin);
    for (int i = 0; i < kPtsPerFrame; ++i) {
        const uint16_t dist = (uint16_t)(f[6 + i * 3] | (f[7 + i * 3] << 8));
        const uint8_t conf = f[8 + i * 3];
        if (conf < kMinConf) {
            continue;
        }
        uint32_t angle = (start + (uint32_t)diff * i / kPtsPerFrame) / 100;  // 度
        angle %= 360;
        Bin& b = s_bins[angle];
        b.dist_mm = dist;
        b.conf = conf;
        b.stamp_ms = now;
        // 融合进 120 点极坐标图：conf/距离双门限，只写不覆盖为 0。
        if (conf > kMinConf && dist > kMinValidDistMm) {
            int idx = (int)(angle / 3) % kPolar120;
            s_polar120[idx] = dist;
            s_polar_stamp[idx] = now;
        }
    }
    if (speed_ds > 0) {
        float rpm = speed_ds / 6.0f;
        s_rpm_ema = (s_rpm_ema <= 0.01f) ? rpm : s_rpm_ema * 0.9f + rpm * 0.1f;
    }
    ++s_frames;
    if (!s_first_synced) {
        s_first_synced = true;
        first = true;
    }
    taskEXIT_CRITICAL(&s_spin);
    if (first) {
        ESP_LOGI(TAG, "雷达首帧同步 ok");
    }
}

// 滑动窗口流式解析：逐字节匹配 0x54 0x2C，攒够 47B 做 CRC8；
// 失败仅窗口前移 1 字节，禁止整包丢弃。
void ConsumeWindow(uint8_t* win, size_t* len) {
    while (*len >= kFrameLen) {
        // 在窗口内搜索帧头。
        size_t pos = 0;
        while (pos + 1 < *len && !(win[pos] == kHeader0 && win[pos + 1] == kHeader1)) {
            ++pos;
        }
        if (pos + 1 >= *len) {
            // 未找到完整帧头：只保留末尾可能残缺的 0x54。
            if (*len > 0 && win[*len - 1] == kHeader0) {
                win[0] = kHeader0;
                *len = 1;
            } else {
                *len = 0;
            }
            return;
        }
        if (pos > 0) {
            memmove(win, win + pos, *len - pos);
            *len -= pos;
        }
        if (*len < kFrameLen) {
            return;  // 等更多字节
        }
        // 候选帧 win[0..46]：CRC8（poly 0x4D，init 0x00），兼容两种口径。
        if (Crc8(win, 46) == win[46] || Crc8(win + 1, 45) == win[46]) {
            ParseFrame(win);
            memmove(win, win + kFrameLen, *len - kFrameLen);
            *len -= kFrameLen;
        } else {
            ++s_crc_err;
            // 仅前移 1 字节继续搜寻。
            memmove(win, win + 1, *len - 1);
            --(*len);
        }
    }
    // 窗口防爆：长期不同步时丢弃陈旧字节，只留最后 1 个。
    if (*len > 128) {
        win[0] = win[*len - 1];
        *len = 1;
    }
}

void ParserTask(void* arg) {
    (void)arg;
    uint8_t buf[256];
    static uint8_t win[256];
    size_t wlen = 0;
    uint32_t last_data_ms = NowMs();
    uint32_t last_stat_ms = NowMs();
    uint32_t stat_bytes = 0, stat_frames = 0, stat_crc = 0;
    while (s_running) {
        int n = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(100));
        const uint32_t now = NowMs();
        if (n > 0) {
            last_data_ms = now;
            s_bytes += (uint32_t)n;
            stat_bytes += (uint32_t)n;
            // 追加到滑动窗口（窗口满则丢最旧，保证不断流）。
            for (int i = 0; i < n; ++i) {
                if (wlen >= sizeof(win)) {
                    memmove(win, win + 1, sizeof(win) - 1);
                    wlen = sizeof(win) - 1;
                }
                win[wlen++] = buf[i];
            }
            ConsumeWindow(win, &wlen);
        } else if (now - last_data_ms >= 2000) {
            last_data_ms = now;  // 限流：至多 2s 一次
            ESP_LOGW(TAG, "No UART RX data on GPIO 39, check power & physical pin");
        }
        // 结构化统计：每 10 秒一次（bytes / frames / crc_err / rpm），平时静默。
        if (now - last_stat_ms >= 10000) {
            uint32_t frames = s_frames, crc = s_crc_err;
            float rpm = s_rpm_ema;
            ESP_LOGI(TAG,
                     "[LD14_STAT] +%uB frames=%u(+%u) crc_err=%u(+%u) "
                     "rpm=%.0f total=%uB",
                     (unsigned)stat_bytes, (unsigned)frames,
                     (unsigned)(frames - stat_frames), (unsigned)crc,
                     (unsigned)(crc - stat_crc), rpm, (unsigned)s_bytes);
            stat_bytes = 0;
            stat_frames = frames;
            stat_crc = crc;
            last_stat_ms = now;
        }
    }
    vTaskDelete(nullptr);
}

}  // namespace

void ld14_motor_set_duty(int percent) {
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, (uint32_t)(percent * 1023 / 100));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);
}

void ld14_driver_init(void) {
    if (s_running) {
        return;
    }
    uart_config_t cfg = {
        .baud_rate = kBaud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(kUart, 512, 0, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(kUart, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UART%d install failed", kUart);
        return;
    }
    uart_set_pin(kUart, UART_PIN_NO_CHANGE, kRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ledc_timer_config_t tconf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_3,
        .freq_hz = kPwmFreqHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ledc_timer_config(&tconf);
    ledc_channel_config_t cconf = {
        .gpio_num = kPwmPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_7,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_3,
        .duty = 0,
        .hpoint = 0,
        .flags = {},
    };
    ledc_channel_config(&cconf);

    s_running = true;
    xTaskCreate(ParserTask, "ld14_parse", 4096, nullptr, 4, nullptr);
    // 电机软启动：0 → 默认占空比，避免上电冲击电流。
    for (int d = 0; d <= kDefaultDuty && s_running; d += 10) {
        ld14_motor_set_duty(d);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    ESP_LOGI(TAG, "LD14 on UART%d RX=%d %dbaud, motor PWM=%d @%dHz duty=%d%%",
             kUart, kRxPin, kBaud, kPwmPin, kPwmFreqHz, kDefaultDuty);
}

void ld14_driver_deinit(void) {
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(150));
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, 0);
    uart_driver_delete(kUart);
}

static bool BinValid(const Bin& b, uint32_t now) {
    return b.dist_mm > 0 && (now - b.stamp_ms) < kBinDecayMs;
}

bool ld14_get_features(ld14_features_t* out) {
    if (out == nullptr) {
        return false;
    }
    const uint32_t now = NowMs();
    uint32_t front_min = UINT32_MAX;
    uint32_t left_sum = 0, left_n = 0, right_sum = 0, right_n = 0;
    uint32_t gmin = UINT32_MAX;
    uint16_t gangle = 0;
    uint32_t sector_min[12];
    for (int i = 0; i < 12; ++i) {
        sector_min[i] = UINT32_MAX;
    }

    taskENTER_CRITICAL(&s_spin);
    for (int a = 0; a < 360; ++a) {
        const Bin& b = s_bins[a];
        if (!BinValid(b, now)) {
            continue;
        }
        uint32_t d = b.dist_mm;
        if (a >= 345 || a <= 15) {
            if (d < front_min) {
                front_min = d;
            }
        }
        if (a >= 75 && a <= 105) {
            left_sum += d;
            ++left_n;
        }
        if (a >= 255 && a <= 285) {
            right_sum += d;
            ++right_n;
        }
        if (d < gmin) {
            gmin = d;
            gangle = (uint16_t)a;
        }
        uint32_t s = sector_min[a / 30];
        if (d < s) {
            sector_min[a / 30] = d;
        }
    }
    float rpm = s_rpm_ema;
    uint32_t frames = s_frames;
    taskEXIT_CRITICAL(&s_spin);

    if (frames == 0) {
        return false;
    }
    // 逃逸角：可穿行（扇区最小 >800mm）扇区中取最小距离最大者，并以前方为偏好。
    int best = -1;
    uint32_t best_score = 0;
    for (int i = 0; i < 12; ++i) {
        if (sector_min[i] == UINT32_MAX || sector_min[i] < 800) {
            continue;
        }
        uint32_t capped = sector_min[i] > 5000 ? 5000 : sector_min[i];
        int center = i * 30 + 15;
        int dev = center > 180 ? 360 - center : center;  // 距正前方的偏离
        uint32_t score = capped * 2 + (uint32_t)(180 - dev);
        if ((int)score > (int)best_score || best < 0) {
            best_score = score;
            best = i;
        }
    }
    out->front_min_mm = front_min == UINT32_MAX ? 0 : (uint16_t)front_min;
    out->left_mm = left_n ? (uint16_t)(left_sum / left_n) : 0;
    out->right_mm = right_n ? (uint16_t)(right_sum / right_n) : 0;
    out->gmin_mm = gmin == UINT32_MAX ? 0 : (uint16_t)gmin;
    out->gmin_angle = gangle;
    out->escape_angle = best < 0 ? 180 : (uint16_t)(best * 30 + 15);
    out->rpm = (uint16_t)(rpm + 0.5f);
    out->stamp_ms = now;
    return true;
}

int ld14_snapshot(uint16_t* dist_mm, uint8_t* conf) {
    if (dist_mm == nullptr) {
        return 0;
    }
    const uint32_t now = NowMs();
    int valid = 0;
    taskENTER_CRITICAL(&s_spin);
    for (int a = 0; a < 360; ++a) {
        if (BinValid(s_bins[a], now)) {
            dist_mm[a] = s_bins[a].dist_mm;
            ++valid;
        } else {
            dist_mm[a] = 0;
        }
        if (conf != nullptr) {
            conf[a] = s_bins[a].conf;
        }
    }
    taskEXIT_CRITICAL(&s_spin);
    return valid;
}

// 取 120 点融合极坐标快照（3°/bin，索引 i 对应角度 i*3°）。
// 超期未更新的点置 0，其余保留有效值；返回有效点数。
int ld14_polar120_snapshot(uint16_t* out120) {
    if (out120 == nullptr) {
        return 0;
    }
    const uint32_t now = NowMs();
    int valid = 0;
    taskENTER_CRITICAL(&s_spin);
    for (int i = 0; i < kPolar120; ++i) {
        if (s_polar120[i] > 0 && (now - s_polar_stamp[i]) < kPolarStaleMs) {
            out120[i] = s_polar120[i];
            ++valid;
        } else {
            out120[i] = 0;
        }
    }
    taskEXIT_CRITICAL(&s_spin);
    return valid;
}
