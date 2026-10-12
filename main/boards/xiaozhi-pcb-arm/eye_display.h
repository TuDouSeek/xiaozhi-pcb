#ifndef __EYE_DISPLAY_H__
#define __EYE_DISPLAY_H__

#include <cstdint>

#include <driver/gpio.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_io.h>
#include <esp_err.h>

struct gfx_disp;
typedef struct gfx_disp gfx_disp_t;
struct gfx_obj;
typedef struct gfx_obj gfx_obj_t;
typedef void* gfx_handle_t;

// 双 GC9A01 表情屏：左 CS=GPIO10，右 CS=GPIO9，共享 SPI2。
// 表情资源见 eye_emotions.h：`<范围>-<表情>-<版本>.eaf`，
// L/R = 左右屏分别播放，Public = 双屏同播同一份。
// AI/上层只调 PlayEmotion(id)；空闲随机逻辑在内部任务里跑。
class EyeDisplay {
public:
    EyeDisplay();
    ~EyeDisplay();

    bool Initialize();
    bool IsReady() const { return ready_; }

    // 延迟启动(等 OLED/LVGL 首帧后由板卡定时器调用)，默认播 zhayan。
    void BeginAnimation();
    // AI/状态机入口：id 见 eye_emotions.h (shangxin/shengqi/weixiao/...)。
    // 返回 false = 未知 id 或播放器尚未就绪。
    bool PlayEmotion(const char* id);
    const char* current_emotion() const { return current_emotion_; }

private:
    struct EmbeddedEaf {
        const uint8_t* start;
        const uint8_t* end;
    };

    bool InitializePanel(gpio_num_t cs_pin, esp_lcd_panel_io_handle_t& io,
                         esp_lcd_panel_handle_t& panel);
    static void EmoteStartTask(void* arg);
    static void IdleRandomTask(void* arg);
    bool StartEmoteLocked();
    bool SwitchEmotionLocked(const char* id);
    EmbeddedEaf ResolveEaf(const char* alias);
    static void FlushToPanel(gfx_disp_t* disp, int x1, int y1, int x2, int y2,
                             const void* data);
    static bool OnColorTransDone(esp_lcd_panel_io_handle_t panel_io,
                                 esp_lcd_panel_io_event_data_t* edata, void* user_ctx);

    esp_lcd_panel_io_handle_t left_io_ = nullptr;
    esp_lcd_panel_io_handle_t right_io_ = nullptr;
    esp_lcd_panel_handle_t left_panel_ = nullptr;
    esp_lcd_panel_handle_t right_panel_ = nullptr;
    bool spi_bus_initialized_ = false;
    bool ready_ = false;

    bool EnsureEafImageMapped();

    gfx_handle_t emote_handle_ = nullptr;
    gfx_disp_t* disp_left_ = nullptr;
    gfx_disp_t* disp_right_ = nullptr;
    gfx_obj_t* anim_left_ = nullptr;
    gfx_obj_t* anim_right_ = nullptr;
    const uint8_t* eaf_image_ = nullptr;
    uint32_t eaf_entry_count_ = 0;
    bool emote_started_ = false;
    const char* current_emotion_ = "zhayan";
    const char* pending_emotion_ = nullptr;
    bool remote_hold_ = false;
};

#endif  // __EYE_DISPLAY_H__
