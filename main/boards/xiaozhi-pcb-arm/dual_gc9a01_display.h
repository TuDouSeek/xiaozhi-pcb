#ifndef __DUAL_GC9A01_DISPLAY_H__
#define __DUAL_GC9A01_DISPLAY_H__

#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

// Two GC9A01 round panels sharing one SPI bus with independent CS lines.
// The LVGL display registered with esp_lvgl_port renders once; the mirror
// panel callback replays every flushed area to the second panel so both
// eyes always show identical content.
class DualGc9a01Display {
public:
    struct Config {
        spi_host_device_t spi_host;
        gpio_num_t sck;
        gpio_num_t mosi;
        gpio_num_t dc;
        gpio_num_t rst;
        gpio_num_t cs_left;
        gpio_num_t cs_right;
        int width;
        int height;
    };

    // Returns false when either panel fails to initialize; in that case the
    // caller should fall back to a NoDisplay-style setup.
    static bool Create(const Config& config, esp_lcd_panel_handle_t& out_panel);

private:
    DualGc9a01Display() = default;
};

#endif  // __DUAL_GC9A01_DISPLAY_H__