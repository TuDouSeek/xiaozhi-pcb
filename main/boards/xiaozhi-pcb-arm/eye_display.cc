#include "eye_display.h"

#include <driver/spi_master.h>
#include <esp_partition.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_gc9a01.h>
#include <esp_log.h>
#include <spi_flash_mmap.h>

#include <cstring>

#include <gfx.h>

#include "config.h"
#include "eye_emotions.h"

#define TAG "EyeDisplay"

namespace {

constexpr uint32_t kEmoteFps = 30;
constexpr uint32_t kEafFps = 25;
// 空闲随机：眨眼 2~5 次后，若无外部指令则播一次左右滚动。
constexpr int kIdleBlinkMin = 2;
constexpr int kIdleBlinkMax = 5;
constexpr uint32_t kEmoteLoopMs = 2600;

// `eyeeaf` partition image layout (see gen_eye_eaf.py):
//   magic "EYEA", entry count u32,
//   per entry: alias[32] NUL-padded, data offset u32, data size u32,
//   then raw EAF blobs.
constexpr uint32_t kEyeEafMagic = 0x41455945;  // "EYEA" little-endian
constexpr size_t kEyeEafAliasLen = 32;
constexpr size_t kEyeEafEntryLen = 32 + 4 + 4;

}  // namespace

EyeDisplay::EyeDisplay() = default;

EyeDisplay::~EyeDisplay() {
    if (anim_left_ != nullptr) {
        gfx_obj_delete(anim_left_);
    }
    if (anim_right_ != nullptr) {
        gfx_obj_delete(anim_right_);
    }
    if (disp_left_ != nullptr) {
        gfx_disp_del(disp_left_);
    }
    if (disp_right_ != nullptr) {
        gfx_disp_del(disp_right_);
    }
    if (emote_handle_ != nullptr) {
        gfx_emote_deinit(emote_handle_);
    }
    if (left_panel_ != nullptr) {
        esp_lcd_panel_del(left_panel_);
    }
    if (right_panel_ != nullptr) {
        esp_lcd_panel_del(right_panel_);
    }
    if (left_io_ != nullptr) {
        esp_lcd_panel_io_del(left_io_);
    }
    if (right_io_ != nullptr) {
        esp_lcd_panel_io_del(right_io_);
    }
    if (spi_bus_initialized_) {
        spi_bus_free(DISPLAY_GC9A01_SPI_HOST);
    }
}

bool EyeDisplay::InitializePanel(gpio_num_t cs_pin, esp_lcd_panel_io_handle_t& io,
                                 esp_lcd_panel_handle_t& panel) {
    esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(
        cs_pin, DISPLAY_GC9A01_DC_PIN, nullptr, nullptr);
    io_config.pclk_hz = 40 * 1000 * 1000;
    io_config.trans_queue_depth = 1;

    esp_err_t err = esp_lcd_new_panel_io_spi(
        static_cast<esp_lcd_spi_bus_handle_t>(DISPLAY_GC9A01_SPI_HOST), &io_config, &io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GC9A01 IO creation failed on CS=%d: %s", cs_pin, esp_err_to_name(err));
        return false;
    }

    const esp_lcd_panel_dev_config_t panel_config = {
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
        .reset_gpio_num = GPIO_NUM_NC,
        .vendor_config = nullptr,
        .flags = {},
    };
    err = esp_lcd_new_panel_gc9a01(io, &panel_config, &panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GC9A01 driver creation failed on CS=%d: %s", cs_pin, esp_err_to_name(err));
        return false;
    }

    if ((err = esp_lcd_panel_reset(panel)) != ESP_OK ||
        (err = esp_lcd_panel_init(panel)) != ESP_OK ||
        (err = esp_lcd_panel_invert_color(panel, true)) != ESP_OK ||
        (err = esp_lcd_panel_disp_on_off(panel, true)) != ESP_OK) {
        ESP_LOGE(TAG, "GC9A01 initialization failed on CS=%d: %s", cs_pin, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool EyeDisplay::Initialize() {
    const spi_bus_config_t bus_config = GC9A01_PANEL_BUS_SPI_CONFIG(
        DISPLAY_GC9A01_SCK_PIN, DISPLAY_GC9A01_MOSI_PIN,
        DISPLAY_GC9A01_WIDTH * 10 * sizeof(uint16_t));
    esp_err_t err = spi_bus_initialize(DISPLAY_GC9A01_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GC9A01 SPI bus initialization failed: %s", esp_err_to_name(err));
        return false;
    }
    spi_bus_initialized_ = true;

    gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << DISPLAY_GC9A01_RST_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&reset_config));
    gpio_set_level(DISPLAY_GC9A01_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DISPLAY_GC9A01_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    if (!InitializePanel(DISPLAY_GC9A01_CS_EYE_L, left_io_, left_panel_) ||
        !InitializePanel(DISPLAY_GC9A01_CS_EYE_R, right_io_, right_panel_)) {
        return false;
    }

    ready_ = true;
    ESP_LOGI(TAG, "GC9A01 IO ready (left CS=%d, right CS=%d), EAF playback deferred",
             DISPLAY_GC9A01_CS_EYE_L, DISPLAY_GC9A01_CS_EYE_R);
    return true;
}

void EyeDisplay::FlushToPanel(gfx_disp_t* disp, int x1, int y1, int x2, int y2,
                              const void* data) {
    auto* panel = static_cast<esp_lcd_panel_handle_t>(gfx_disp_get_user_data(disp));
    if (panel != nullptr) {
        esp_lcd_panel_draw_bitmap(panel, x1, y1, x2, y2, data);
    }
}

bool EyeDisplay::OnColorTransDone(esp_lcd_panel_io_handle_t panel_io,
                                  esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    auto* disp = static_cast<gfx_disp_t*>(user_ctx);
    if (disp != nullptr) {
        gfx_disp_flush_ready(disp, true);
    }
    return true;
}

bool EyeDisplay::EnsureEafImageMapped() {
    if (eaf_image_ != nullptr) {
        return true;
    }
    const esp_partition_t* part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "eyeeaf");
    if (part == nullptr) {
        ESP_LOGE(TAG, "eyeeaf partition not found (use partitions/v2/16m-arm.csv)");
        return false;
    }
    esp_partition_mmap_handle_t handle = 0;
    const void* mapped = nullptr;
    if (esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &mapped, &handle) !=
        ESP_OK) {
        ESP_LOGE(TAG, "eyeeaf mmap failed");
        return false;
    }
    const auto* base = static_cast<const uint8_t*>(mapped);
    uint32_t magic = 0;
    uint32_t count = 0;
    memcpy(&magic, base, 4);
    memcpy(&count, base + 4, 4);
    if (magic != kEyeEafMagic || count == 0 || count > 64) {
        ESP_LOGE(TAG, "eyeeaf image invalid (magic=0x%08X count=%u), reflash eye_eaf.bin?",
                 static_cast<unsigned>(magic), static_cast<unsigned>(count));
        esp_partition_munmap(handle);
        return false;
    }
    eaf_image_ = base;
    eaf_entry_count_ = count;
    // Keep mapped for the lifetime of the process; EAF decoder reads in place.
    (void)handle;
    ESP_LOGI(TAG, "eyeeaf image mapped: %u entries", static_cast<unsigned>(count));
    return true;
}

EyeDisplay::EmbeddedEaf EyeDisplay::ResolveEaf(const char* alias) {
    if (!EnsureEafImageMapped()) {
        return {nullptr, nullptr};
    }
    const uint8_t* table = eaf_image_ + 8;
    for (uint32_t i = 0; i < eaf_entry_count_; ++i) {
        const uint8_t* entry = table + i * kEyeEafEntryLen;
        char name[kEyeEafAliasLen + 1];
        memcpy(name, entry, kEyeEafAliasLen);
        name[kEyeEafAliasLen] = '\0';
        const char* a = name;
        const char* b = alias;
        while (*a && *b && *a == *b) {
            ++a;
            ++b;
        }
        if (*a != '\0' || *b != '\0') {
            continue;
        }
        uint32_t offset = 0;
        uint32_t size = 0;
        memcpy(&offset, entry + 32, 4);
        memcpy(&size, entry + 36, 4);
        if (size == 0) {
            return {nullptr, nullptr};
        }
        return {eaf_image_ + offset, eaf_image_ + offset + size};
    }
    ESP_LOGW(TAG, "EAF alias not in eyeeaf image: %s", alias ? alias : "(null)");
    return {nullptr, nullptr};
}

bool EyeDisplay::SwitchEmotionLocked(const char* id) {
    const eye_cfg::EyeEmotion* em = eye_cfg::FindEmotion(id);
    if (em == nullptr) {
        ESP_LOGW(TAG, "Unknown emotion id: %s", id ? id : "(null)");
        return false;
    }
    EmbeddedEaf left = ResolveEaf(em->left_alias);
    EmbeddedEaf right = ResolveEaf(em->right_alias);
    if (left.start == nullptr || right.start == nullptr) {
        ESP_LOGE(TAG, "EAF alias not embedded for %s", id);
        return false;
    }
    const size_t left_len = static_cast<size_t>(left.end - left.start);
    const size_t right_len = static_cast<size_t>(right.end - right.start);
    if (left_len == 0 || right_len == 0) {
        ESP_LOGE(TAG, "Embedded EAF empty for %s", id);
        return false;
    }

    gfx_emote_lock(emote_handle_);
    gfx_anim_stop(anim_left_);
    gfx_anim_stop(anim_right_);
    const gfx_anim_src_t left_src = {
        .type = GFX_ANIM_SRC_TYPE_MEMORY, .data = left.start, .data_len = left_len};
    const gfx_anim_src_t right_src = {
        .type = GFX_ANIM_SRC_TYPE_MEMORY, .data = right.start, .data_len = right_len};
    bool ok = gfx_anim_set_src_desc(anim_left_, &left_src) == ESP_OK &&
              gfx_anim_set_src_desc(anim_right_, &right_src) == ESP_OK &&
              gfx_anim_set_segment(anim_left_, 0, 0xFFFFFFFF, kEafFps, true) == ESP_OK &&
              gfx_anim_set_segment(anim_right_, 0, 0xFFFFFFFF, kEafFps, true) == ESP_OK &&
              gfx_anim_start(anim_left_) == ESP_OK && gfx_anim_start(anim_right_) == ESP_OK;
    gfx_emote_unlock(emote_handle_);
    if (ok) {
        current_emotion_ = em->id;
        ESP_LOGI(TAG, "Emotion -> %s (%s), L=%uB R=%uB", em->id, em->label,
                 static_cast<unsigned>(left_len), static_cast<unsigned>(right_len));
    } else {
        ESP_LOGE(TAG, "EAF switch failed for %s", id);
    }
    return ok;
}

bool EyeDisplay::StartEmoteLocked() {
    gfx_core_config_t core_config = {
        .fps = kEmoteFps,
        .task = GFX_EMOTE_INIT_CONFIG(),
    };
    emote_handle_ = gfx_emote_init(&core_config);
    if (emote_handle_ == nullptr) {
        ESP_LOGE(TAG, "gfx_emote_init failed");
        return false;
    }

    const gfx_disp_config_t left_config = {
        .h_res = DISPLAY_GC9A01_WIDTH,
        .v_res = DISPLAY_GC9A01_HEIGHT,
        .flush_cb = FlushToPanel,
        .update_cb = nullptr,
        .user_data = left_panel_,
        .flags = {.swap = true, .buff_dma = true, .buff_spiram = false, .double_buffer = true},
        .buffers = {.buf1 = nullptr,
                    .buf2 = nullptr,
                    .buf_pixels = DISPLAY_GC9A01_WIDTH * 10},
    };
    disp_left_ = gfx_disp_add(emote_handle_, &left_config);
    if (disp_left_ == nullptr) {
        ESP_LOGE(TAG, "gfx_disp_add(left) failed");
        return false;
    }

    const gfx_disp_config_t right_config = {
        .h_res = DISPLAY_GC9A01_WIDTH,
        .v_res = DISPLAY_GC9A01_HEIGHT,
        .flush_cb = FlushToPanel,
        .update_cb = nullptr,
        .user_data = right_panel_,
        .flags = {.swap = true, .buff_dma = true, .buff_spiram = false, .double_buffer = true},
        .buffers = {.buf1 = nullptr,
                    .buf2 = nullptr,
                    .buf_pixels = DISPLAY_GC9A01_WIDTH * 10},
    };
    disp_right_ = gfx_disp_add(emote_handle_, &right_config);
    if (disp_right_ == nullptr) {
        ESP_LOGE(TAG, "gfx_disp_add(right) failed");
        return false;
    }

    const esp_lcd_panel_io_callbacks_t callbacks = {
        .on_color_trans_done = OnColorTransDone,
    };
    ESP_ERROR_CHECK(
        esp_lcd_panel_io_register_event_callbacks(left_io_, &callbacks, disp_left_));
    ESP_ERROR_CHECK(
        esp_lcd_panel_io_register_event_callbacks(right_io_, &callbacks, disp_right_));

    gfx_disp_set_bg_color(disp_left_, GFX_COLOR_HEX(0x000000));
    gfx_disp_set_bg_color(disp_right_, GFX_COLOR_HEX(0x000000));

    anim_left_ = gfx_anim_create(disp_left_);
    anim_right_ = gfx_anim_create(disp_right_);
    if (anim_left_ == nullptr || anim_right_ == nullptr) {
        ESP_LOGE(TAG, "gfx_anim_create failed");
        return false;
    }
    gfx_obj_align(anim_left_, GFX_ALIGN_CENTER, 0, 0);
    gfx_obj_align(anim_right_, GFX_ALIGN_CENTER, 0, 0);
    gfx_obj_set_size(anim_left_, DISPLAY_GC9A01_WIDTH, DISPLAY_GC9A01_HEIGHT);
    gfx_obj_set_size(anim_right_, DISPLAY_GC9A01_WIDTH, DISPLAY_GC9A01_HEIGHT);

    emote_started_ = true;
    const char* first = pending_emotion_ != nullptr ? pending_emotion_ : "zhayan";
    pending_emotion_ = nullptr;
    if (!SwitchEmotionLocked(first)) {
        return false;
    }
    if (xTaskCreate(IdleRandomTask, "eye_idle", 2048, this, 2, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Idle random task not started");
    }
    ESP_LOGI(TAG, "EAF emote engine ready, %u emotions",
             static_cast<unsigned>(eye_cfg::kEyeEmotionCount));
    return true;
}

void EyeDisplay::EmoteStartTask(void* arg) {
    auto* eyes = static_cast<EyeDisplay*>(arg);
    if (!eyes->StartEmoteLocked()) {
        ESP_LOGE(TAG, "EAF playback failed; panels stay dark");
    }
    vTaskDelete(nullptr);
}

void EyeDisplay::IdleRandomTask(void* arg) {
    auto* eyes = static_cast<EyeDisplay*>(arg);
    int blinks_left =
        kIdleBlinkMin + static_cast<int>(esp_random() % (kIdleBlinkMax - kIdleBlinkMin + 1));
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kEmoteLoopMs));
        // 外部(AI/状态机)刚切过表情：重新计数，不打断它。
        if (eyes->remote_hold_) {
            eyes->remote_hold_ = false;
            blinks_left = kIdleBlinkMin +
                          static_cast<int>(esp_random() % (kIdleBlinkMax - kIdleBlinkMin + 1));
            continue;
        }
        if (blinks_left > 0) {
            eyes->SwitchEmotionLocked("zhayan");
            --blinks_left;
        } else {
            // 随机眨完一轮、无其他指令：滚一眼，保持“活着”。
            eyes->SwitchEmotionLocked("zuoyougundong");
            blinks_left = kIdleBlinkMin +
                          static_cast<int>(esp_random() % (kIdleBlinkMax - kIdleBlinkMin + 1));
        }
    }
}

bool EyeDisplay::PlayEmotion(const char* id) {
    const eye_cfg::EyeEmotion* em = eye_cfg::FindEmotion(id);
    if (em == nullptr) {
        return false;
    }
    if (!emote_started_) {
        // 播放器还没起来：记住，启动后首播它。
        pending_emotion_ = em->id;
        current_emotion_ = em->id;
        remote_hold_ = true;
        return true;
    }
    remote_hold_ = true;
    return SwitchEmotionLocked(em->id);
}

void EyeDisplay::BeginAnimation() {
    if (!ready_ || emote_started_) {
        return;
    }
    emote_started_ = true;
    if (xTaskCreate(EmoteStartTask, "eye_eaf", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create EAF playback task");
        emote_started_ = false;
    }
}
