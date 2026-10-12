#include "robot_oled_display.h"

#include <cmath>
#include <cstdint>
#include <ctime>

#include <esp_log.h>
#include <lvgl.h>
#include <wifi_manager.h>

#include "application.h"
#include "assets/lang_config.h"
#include "board.h"
#include "lvgl_theme.h"

#define TAG "RobotOled"

namespace {

// Geometry follows the Arduino reference (OLED显示参考.md): a 16 px header
// holding Wi-Fi signal steps, centered status text and a battery icon, then a
// 44 px tall band of discrete voice bars between x=4 and x=124.
constexpr int kHeaderHeight = 16;
constexpr int kWaveTop = 16;
constexpr int kWaveHeight = 44;
constexpr int kWaveMidY = kWaveHeight / 2;
constexpr int kWaveLeft = 4;
constexpr int kBarPitch = 2;  // 1 px bar + 1 px gap
constexpr int kMinBarHeight = 2;
constexpr int kBatteryX = 104;
constexpr int kBatteryFillMax = 14;
constexpr int kTimerPeriodMs = 33;
// The Arduino reference ran at ~80 fps; these factors keep the same visual
// speed at the ~30 fps LVGL tick used here.
constexpr float kPhasePerTick = 0.6f;
constexpr float kEnergyDamping = 0.25f;
constexpr float kPi = 3.1415926f;

lv_color_t LitColor() { return lv_color_white(); }

lv_obj_t* CreateSolidRect(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* rect = lv_obj_create(parent);
    lv_obj_remove_style_all(rect);
    lv_obj_set_pos(rect, x, y);
    lv_obj_set_size(rect, w, h);
    lv_obj_set_style_bg_color(rect, LitColor(), 0);
    lv_obj_set_style_bg_opa(rect, LV_OPA_COVER, 0);
    return rect;
}

}  // namespace

RobotOledDisplay::RobotOledDisplay(esp_lcd_panel_io_handle_t panel_io,
                                   esp_lcd_panel_handle_t panel, int width, int height,
                                   bool mirror_x, bool mirror_y)
    : OledDisplay(panel_io, panel, width, height, mirror_x, mirror_y) {}

RobotOledDisplay::~RobotOledDisplay() {
    DisplayLockGuard lock(this);
    if (anim_timer_ != nullptr) {
        lv_timer_delete(anim_timer_);
        anim_timer_ = nullptr;
    }
    if (wave_container_ != nullptr) {
        lv_obj_del(wave_container_);
        wave_container_ = nullptr;
    }
    if (header_ != nullptr) {
        // The status/notification labels are children of the header.
        status_label_ = nullptr;
        notification_label_ = nullptr;
        lv_obj_del(header_);
        header_ = nullptr;
    }
}

void RobotOledDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI() called multiple times, skipping duplicate call");
        return;
    }
    Display::SetupUI();

    ESP_LOGI(TAG, "RobotOledDisplay::SetupUI() creating 128x64 widgets");
    DisplayLockGuard lock(this);

    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();

    auto screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(screen, text_font, 0);
    lv_obj_set_style_text_color(screen, LitColor(), 0);

    // --- Header: signal steps | status text | battery -----------------------
    header_ = lv_obj_create(screen);
    lv_obj_remove_style_all(header_);
    lv_obj_set_pos(header_, 0, 0);
    lv_obj_set_size(header_, width_, kHeaderHeight);

    for (int i = 0; i < kSignalBars; i++) {
        const int h = (i + 1) * 2;
        signal_bars_[i] = CreateSolidRect(header_, 2 + i * 4, 12 - h, 3, h);
        // Steps above the current level stay hollow.
        lv_obj_set_style_bg_opa(signal_bars_[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(signal_bars_[i], LitColor(), 0);
        lv_obj_set_style_border_width(signal_bars_[i], 1, 0);
    }

    lv_obj_t* battery_outline = lv_obj_create(header_);
    lv_obj_remove_style_all(battery_outline);
    lv_obj_set_pos(battery_outline, kBatteryX, 2);
    lv_obj_set_size(battery_outline, 18, 8);
    lv_obj_set_style_border_color(battery_outline, LitColor(), 0);
    lv_obj_set_style_border_width(battery_outline, 1, 0);

    battery_fill_ = CreateSolidRect(battery_outline, 1, 1, 0, 4);
    battery_fill_width_ = 0;
    CreateSolidRect(header_, kBatteryX + 18, 4, 2, 4);

    CreateSolidRect(screen, 0, kHeaderHeight - 1, width_, 1);

    status_label_ = lv_label_create(header_);
    lv_obj_set_width(status_label_, width_);
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);
    lv_obj_align(status_label_, LV_ALIGN_CENTER, 0, -1);

    notification_label_ = lv_label_create(header_);
    lv_obj_set_width(notification_label_, width_);
    lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(notification_label_, "");
    lv_obj_align(notification_label_, LV_ALIGN_CENTER, 0, -1);
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    // --- Voice bar band ------------------------------------------------------
    wave_container_ = lv_obj_create(screen);
    lv_obj_remove_style_all(wave_container_);
    lv_obj_set_pos(wave_container_, 0, kWaveTop);
    lv_obj_set_size(wave_container_, width_, 48);

    for (int b = 0; b < kWaveBars; b++) {
        wave_bars_[b] = CreateSolidRect(wave_container_, kWaveLeft + b * kBarPitch,
                                        kWaveMidY - kMinBarHeight / 2, 1, kMinBarHeight);
        bar_heights_[b] = kMinBarHeight;
    }

    // --- Low battery popup (parity with the stock layout) --------------------
    low_battery_popup_ = lv_obj_create(screen);
    lv_obj_set_scrollbar_mode(low_battery_popup_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(low_battery_popup_, width_ * 0.9, text_font->line_height * 2);
    lv_obj_align(low_battery_popup_, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(low_battery_popup_, LitColor(), 0);
    lv_obj_set_style_radius(low_battery_popup_, 10, 0);
    low_battery_label_ = lv_label_create(low_battery_popup_);
    lv_label_set_text(low_battery_label_, Lang::Strings::BATTERY_NEED_CHARGE);
    lv_obj_set_style_text_color(low_battery_label_, lv_color_white(), 0);
    lv_obj_center(low_battery_label_);
    lv_obj_add_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);

    anim_timer_ = lv_timer_create(AnimTimerCallback, kTimerPeriodMs, this);

    // Force one full framebuffer flush while the LVGL lock is held, so the
    // first frame reaches the panel even if the LVGL task was delayed.
    lv_display_t* disp = lv_display_get_default();
    if (disp != nullptr) {
        lv_refr_now(disp);
        ESP_LOGI(TAG, "SetupUI done, first frame flushed");
    } else {
        ESP_LOGW(TAG, "SetupUI done, but no default display for first flush");
    }
}

void RobotOledDisplay::UpdateStatusBar(bool update_all) {
    UpdateIndicators(update_all);
    UpdateClock(update_all);
}

void RobotOledDisplay::AnimTimerCallback(lv_timer_t* timer) {
    // Runs inside the LVGL task: never take the display lock here.
    auto* display = static_cast<RobotOledDisplay*>(lv_timer_get_user_data(timer));
    if (display != nullptr) {
        display->UpdateWaveform();
    }
}

void RobotOledDisplay::UpdateWaveform() {
    if (wave_container_ == nullptr) {
        return;
    }

    // Syllable simulation from the reference sketch, gated by device state so
    // the bars stay on the flat baseline while idle.
    const auto state = Application::GetInstance().GetDeviceState();
    const bool talking = state == kDeviceStateListening || state == kDeviceStateSpeaking ||
                         state == kDeviceStateNotifying;

    const int64_t now = lv_tick_get();
    if (talking) {
        if (now - last_switch_ms_ >= switch_duration_ms_) {
            last_switch_ms_ = now;
            switch_duration_ms_ = 50 + static_cast<int>(lv_rand(0, 79));
            // 20% short pause, otherwise a random volume jump.
            target_energy_ = lv_rand(0, 99) < 20
                                   ? 0.08f
                                   : 0.45f + static_cast<float>(lv_rand(0, 54)) / 100.0f;
        }
    } else {
        target_energy_ = 0.02f;
        last_switch_ms_ = now;
        switch_duration_ms_ = 80;
    }

    voice_energy_ += (target_energy_ - voice_energy_) * kEnergyDamping;
    phase_ += kPhasePerTick;

    const int active_total = kWaveBars - kFlatBars * 2;
    for (int b = 0; b < kWaveBars; b++) {
        int height = kMinBarHeight;
        if (b >= kFlatBars && b < kWaveBars - kFlatBars) {
            const float t =
                static_cast<float>(b - kFlatBars) / static_cast<float>(active_total - 1);
            // Hanning window tapers the bar heights towards both edges.
            const float window = 0.5f * (1.0f - cosf(2.0f * kPi * t));
            const float f1 = sinf(t * 8.0f + phase_ * 2.5f);
            const float f2 = cosf(t * 16.0f - phase_ * 3.8f) * 0.5f;
            const float envelope = (f1 + f2 + 1.5f) * 0.35f;
            const float jitter = static_cast<float>(lv_rand(40, 99)) / 100.0f;
            const float dynamic_h =
                envelope * jitter * voice_energy_ * (kWaveHeight * 0.85f) * window;
            height = static_cast<int>(dynamic_h);
            if (height < kMinBarHeight) {
                height = kMinBarHeight;
            }
        }
        if (height > kWaveHeight) {
            height = kWaveHeight;
        }
        if (height != bar_heights_[b]) {
            bar_heights_[b] = height;
            lv_obj_t* bar = wave_bars_[b];
            lv_obj_set_height(bar, height);
            int y = kWaveMidY - height / 2;
            if (y < 0) {
                y = 0;
            } else if (y > 48 - height) {
                y = 48 - height;
            }
            lv_obj_set_y(bar, y);
        }
    }
}

void RobotOledDisplay::UpdateIndicators(bool update_all) {
    if (header_ == nullptr) {
        return;
    }
    DisplayLockGuard lock(this);

    // Throttle RSSI queries; UpdateStatusBar is called once per second.
    static int rssi_divider = 0;
    if (!update_all && rssi_divider++ % 5 != 0) {
        return;
    }

    int level = 0;
    auto& wifi = WifiManager::GetInstance();
    if (wifi.IsConnected()) {
        const int rssi = wifi.GetRssi();
        if (rssi >= -65) {
            level = 4;
        } else if (rssi >= -75) {
            level = 3;
        } else if (rssi >= -85) {
            level = 2;
        } else {
            level = 1;
        }
    }
    if (level != signal_level_) {
        signal_level_ = level;
        for (int i = 0; i < kSignalBars; i++) {
            lv_obj_set_style_bg_opa(signal_bars_[i], i < level ? LV_OPA_COVER : LV_OPA_TRANSP,
                                    0);
        }
    }

    int level_pct = 0;
    bool charging = false;
    bool discharging = false;
    if (Board::GetInstance().GetBatteryLevel(level_pct, charging, discharging)) {
        level_pct = level_pct < 0 ? 0 : (level_pct > 100 ? 100 : level_pct);
        int fill_w = level_pct * kBatteryFillMax / 100;
        if (charging) {
            fill_w = kBatteryFillMax;
        }
        if (fill_w != battery_fill_width_) {
            battery_fill_width_ = fill_w;
            lv_obj_set_width(battery_fill_, fill_w);
        }
        if (low_battery_popup_ != nullptr) {
            const bool show_popup = level_pct == 0 && discharging;
            if (show_popup && lv_obj_has_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN)) {
                lv_obj_remove_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);
            } else if (!show_popup &&
                       !lv_obj_has_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN)) {
                lv_obj_add_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

void RobotOledDisplay::UpdateClock(bool update_all) {
    // Same idle-clock behavior as the stock status bar: show HH:MM while idle
    // once the system time is valid, and let the app states own the text
    // otherwise.
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() != kDeviceStateIdle) {
        last_displayed_clock_min_ = -1;
        return;
    }
    time_t now = time(NULL);
    struct tm* tm_now = localtime(&now);
    if (tm_now == nullptr || tm_now->tm_year < 2025 - 1900) {
        return;
    }
    const int cur_min = tm_now->tm_hour * 60 + tm_now->tm_min;
    if (update_all || cur_min != last_displayed_clock_min_) {
        last_displayed_clock_min_ = cur_min;
        char time_str[16];
        strftime(time_str, sizeof(time_str), "%H:%M", tm_now);
        SetStatus(time_str);
    }
}
