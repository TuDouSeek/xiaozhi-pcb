#ifndef __ROBOT_OLED_DISPLAY_H__
#define __ROBOT_OLED_DISPLAY_H__

#include "display/oled_display.h"

#include <lvgl.h>

// Custom 128x64 OLED UI for the xiaozhi-pcb-arm tracked robot.
//
// Layout follows the Arduino reference (OLED显示参考.md): a 16 px header with
// Wi-Fi signal steps, centered status text and a battery icon, plus a 44 px
// band of 60 discrete voice bars driven by an LVGL timer. Chat subtitles are
// intentionally dropped; status text and notifications carry user feedback.
class RobotOledDisplay : public OledDisplay {
public:
    RobotOledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                     int width, int height, bool mirror_x, bool mirror_y);
    ~RobotOledDisplay() override;

    void SetupUI() override;
    void UpdateStatusBar(bool update_all = false) override;

private:
    static constexpr int kSignalBars = 4;
    static constexpr int kWaveBars = 60;
    static constexpr int kFlatBars = 4;

    static void AnimTimerCallback(lv_timer_t* timer);
    void UpdateWaveform();
    void UpdateIndicators(bool update_all);
    void UpdateClock(bool update_all);

    lv_obj_t* header_ = nullptr;
    lv_obj_t* signal_bars_[kSignalBars] = {};
    lv_obj_t* battery_fill_ = nullptr;
    lv_obj_t* wave_container_ = nullptr;
    lv_obj_t* wave_bars_[kWaveBars] = {};
    int bar_heights_[kWaveBars] = {};
    lv_timer_t* anim_timer_ = nullptr;

    int signal_level_ = -1;
    int battery_fill_width_ = -1;
    float voice_energy_ = 0.0f;
    float target_energy_ = 0.0f;
    float phase_ = 0.0f;
    int64_t last_switch_ms_ = 0;
    int switch_duration_ms_ = 80;
};

#endif  // __ROBOT_OLED_DISPLAY_H__
