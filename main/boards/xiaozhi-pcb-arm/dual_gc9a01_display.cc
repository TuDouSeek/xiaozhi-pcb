#include "dual_gc9a01_display.h"

#include <esp_log.h>

#define TAG "DualGC9A01"

bool DualGc9a01Display::Create(const Config& config, esp_lcd_panel_handle_t& out_panel) {
    (void)config;
    out_panel = nullptr;
    ESP_LOGW(TAG,
             "GC9A01 dual-screen skeleton is present, but the final panel driver is still TODO");
    return false;
}