#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "eye_display.h"
#include "ld14_system.h"
#include "ld14_web.h"
#include "web_emotion.h"
#include "mcp_server.h"
#include "assets/lang_config.h"
#include "motor_controller.h"
#include "motor_mqtt.h"
#include "lidar_driver.h"
#include "robot_oled_display.h"

#include <memory>

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>

#define TAG "XIAOZHI_PCB_ARM"

namespace {

class MotionTools {
public:
    MotionTools(MotorController& motors, LidarDriver& lidar) : motors_(motors), lidar_(lidar) {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("robot.motion.forward", "Drive both motors forward", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.SetSpeed(60, 60);
                        return true;
                    });
        mcp.AddTool("robot.motion.backward", "Drive both motors backward", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.SetSpeed(-60, -60);
                        return true;
                    });
        mcp.AddTool("robot.motion.turn_left", "Turn the robot left", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.SetSpeed(-45, 45);
                        return true;
                    });
        mcp.AddTool("robot.motion.turn_right", "Turn the robot right", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.SetSpeed(45, -45);
                        return true;
                    });
        mcp.AddTool("robot.motion.stop", "Stop both motors", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.Stop();
                        return true;
                    });
        mcp.AddTool("robot.motion.brake", "Brake both motors", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        motors_.Brake();
                        return true;
                    });
        mcp.AddTool("robot.lidar.status", "Get lidar acquisition status", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        auto status = lidar_.GetStatus();
                        cJSON* root = cJSON_CreateObject();
                        cJSON_AddBoolToObject(root, "uart_ready", status.uart_ready);
                        cJSON_AddBoolToObject(root, "task_running", status.task_running);
                        cJSON_AddNumberToObject(root, "frames_received", status.frames_received);
                        cJSON_AddNumberToObject(root, "bytes_received", status.bytes_received);
                        cJSON_AddNumberToObject(root, "crc_errors", status.crc_errors);
                        cJSON_AddNumberToObject(root, "motor_duty_percent", status.motor_duty_percent);
                        return root;
                    });
    }

private:
    MotorController& motors_;
    LidarDriver& lidar_;
};

}  // namespace

class XiaoZhiPcbArmBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    std::unique_ptr<Display> display_;
    std::unique_ptr<EyeDisplay> eyes_;
    WebEmotionServer web_emotion_;
    NetworkEventCallback app_network_cb_ = nullptr;

    // Chained network-event handler: starts the web console once an IP is
    // available, prints the console banner, then forwards to Application.
    void OnBoardNetworkEvent(NetworkEvent event, const std::string& data) {
        if (event == NetworkEvent::Connected) {
            char ip_str[16] = "";
            esp_netif_t* netif = esp_netif_get_default_netif();
            if (netif != nullptr) {
                esp_netif_ip_info_t ip_info = {};
                if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
                }
            }
            if (!web_emotion_.Start(eyes_.get())) {
                ESP_LOGW(TAG, "Web emotion console failed to start");
            }
            ESP_LOGI(TAG,
                     "\n================================================================\n"
                     "  [Web Console Ready]\n"
                     "  Local IP Address : %s\n"
                     "  Emotion Control  : http://%s/\n"
                     "================================================================",
                     ip_str[0] != '\0' ? ip_str : "<unknown>", ip_str[0] != '\0' ? ip_str : "<unknown>");
        }
        if (app_network_cb_) {
            app_network_cb_(event, data);
        }
    }
    Button boot_button_;
    std::unique_ptr<MotorController> motors_;
    std::unique_ptr<LidarDriver> lidar_;
    std::unique_ptr<MotionTools> motion_tools_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeOledDisplay() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = DISPLAY_I2C_ADDRESS,
            .scl_speed_hz = 100 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {.dc_low_on_data = 0, .disable_control_phase = 0},
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {.height = static_cast<uint8_t>(DISPLAY_HEIGHT)};
        panel_config.vendor_config = &ssd1306_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));

        esp_err_t probe = i2c_master_probe(display_i2c_bus_, DISPLAY_I2C_ADDRESS, 100);
        ESP_LOGI(TAG, "OLED probe 0x%02X: %s", DISPLAY_I2C_ADDRESS, esp_err_to_name(probe));

        esp_err_t init_err = esp_lcd_panel_init(panel_);
        if (init_err != ESP_OK) {
            ESP_LOGE(TAG, "OLED init failed: %s", esp_err_to_name(init_err));
            display_ = std::make_unique<NoDisplay>();
            return;
        }
        // Explicitly enable the internal charge pump (0x8D,0x14). Some
        // SSD1306/SH1106 lots power up with the pump off, which leaves the
        // panel dark even though I2C probe + init report ESP_OK.
        esp_err_t pump_err = esp_lcd_panel_io_tx_param(panel_io_, 0x8D,
                                                       (const uint8_t[]){0x14}, 1);
        if (pump_err != ESP_OK) {
            ESP_LOGW(TAG, "OLED charge pump enable failed: %s", esp_err_to_name(pump_err));
        }
        // Pre-charge period (0xD9,0xF1) + VCOMH deselect level (0xDB,0x40):
        // wrong/missing values here also cause a dark panel on some lots.
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0xD9,
                                                  (const uint8_t[]){0xF1}, 1));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0xDB,
                                                  (const uint8_t[]){0x40}, 1));
        // Display ON (0xAF).
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        // ---- Hardware lamp test: force Entire Display ON (0xA5) ----
        // 0xA5 ignores GDDRAM and lights every pixel. If the panel turns
        // fully white, power/I2C/panel are fine and the fault is in the
        // software frame path; if it stays dark, suspect power supply or
        // wiring (this board wires no OLED RST pin, see config.h).
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0x81,
                                                  (const uint8_t[]){0xCF}, 1));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0xA5, nullptr, 0));
        ESP_LOGI(TAG, "[TEST] Sent Entire Display ON (0xA5) command to OLED - "
                      "panel should be FULLY WHITE now");
        vTaskDelay(pdMS_TO_TICKS(1500));
        // Resume normal GDDRAM display (0xA4) so LVGL frames show afterwards.
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0xA4, nullptr, 0));
        ESP_LOGI(TAG, "[TEST] Sent Resume Display (0xA4), back to normal mode");

        ESP_LOGI(TAG, "OLED initialized: SDA=%d SCL=%d address=0x%02X", DISPLAY_SDA_PIN,
                 DISPLAY_SCL_PIN, DISPLAY_I2C_ADDRESS);
        display_ = std::make_unique<RobotOledDisplay>(panel_io_, panel_, DISPLAY_WIDTH,
                                                      DISPLAY_HEIGHT, DISPLAY_MIRROR_X,
                                                      DISPLAY_MIRROR_Y);
    }

    void InitializeMotors() {
        MotorController::Config cfg;
        cfg.in_a1 = MOTOR_A_IN1_PIN;
        cfg.in_a2 = MOTOR_A_IN2_PIN;
        cfg.in_b1 = MOTOR_B_IN1_PIN;
        cfg.in_b2 = MOTOR_B_IN2_PIN;
        cfg.enc_a1 = ENCODER_A_A_PIN;
        cfg.enc_a2 = ENCODER_A_B_PIN;
        cfg.enc_b1 = ENCODER_B_A_PIN;
        cfg.enc_b2 = ENCODER_B_B_PIN;
        cfg.stby = MOTOR_STBY_PIN;
        // 右轮（B 路）电机极性反接：实测"前进"时右轮倒转、车身原地右转，
        // 而"右转"指令反而双轮同向直行。这里翻转 B 路相位，驱动方向与编码器
        // 符号一起取反，rpm/cnt/闭环 PID 仍自洽，不需要拆电机线。
        cfg.invert_b = true;
        motors_ = std::make_unique<MotorController>(cfg);
        // 后台测速 20ms（Core 1）：刷新 RPM + 2s 一次遥测日志；
        // 默认开环，闭环需显式 EnableClosedLoop(true)。
        motors_->StartSpeedTask(20);
    }

    void InitializeLidar() {
        lidar_ = std::make_unique<LidarDriver>(LIDAR_UART_RX_PIN, LIDAR_UART_BAUDRATE,
                                               LIDAR_PWM_PIN);
        lidar_->Start();
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        boot_button_.OnLongPress([this]() {
            if (motors_) {
                motors_->Brake();
            }
            if (lidar_) {
                lidar_->SetMotorSpeed(0);
            }
        });
    }

    void InitializeMcpTools() {
        motion_tools_ = std::make_unique<MotionTools>(*motors_, *lidar_);
        // 小智 AI 可调用：按 id 切换双圆屏表情。
        auto* eyes = eyes_.get();
        McpServer::GetInstance().AddTool(
            "robot.face.set_emotion", "Set robot eye emotion on both round displays. "
                                      "id: shangxin/danzuoyanbi/shengqi/zuoyougundong/"
                                      "weixiao/wuyu/zhayan/biyan.",
            PropertyList({Property("id", kPropertyTypeString)}),
            [eyes](const PropertyList& properties) -> ReturnValue {
                if (eyes == nullptr) {
                    throw std::runtime_error("eyes not ready");
                }
                auto id = properties["id"].value<std::string>();
                if (!eyes->PlayEmotion(id.c_str())) {
                    throw std::runtime_error("unknown emotion: " + id);
                }
                return true;
            });
    }

public:
    XiaoZhiPcbArmBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeOledDisplay();
        eyes_ = std::make_unique<EyeDisplay>();
        if (!eyes_->Initialize()) {
            ESP_LOGE(TAG, "GC9A01 eye display initialization failed");
        } else {
            esp_timer_handle_t eyes_timer = nullptr;
            esp_timer_create_args_t args = {
                .callback =
                    [](void* arg) {
                        static_cast<EyeDisplay*>(arg)->BeginAnimation();
                    },
                .arg = eyes_.get(),
                .dispatch_method = ESP_TIMER_TASK,
                .name = "eyes_start",
                .skip_unhandled_events = false,
            };
            if (esp_timer_create(&args, &eyes_timer) == ESP_OK) {
                esp_timer_start_once(eyes_timer, 8000 * 1000);
            } else {
                eyes_->BeginAnimation();
            }
        }
        InitializeMotors();
        // LD14 独立子系统（UART2/GPIO32 解析 + /lidar 可视化），一行接入。
        // 注意：LD14 与既有 lidar 共用 GPIO40(PWM)。若 LD14 已上机，
        // 请注释掉下一行 InitializeLidar()，避免双驱动抢占同一 PWM 引脚。
        ld14_system_init();
        // 已由 LD14 接管（UART2/GPIO32 + GPIO40 PWM）。旧驱动（UART1/GPIO39，
        // 与 LD14 争抢 GPIO40 并覆写 LEDC 配置）保持禁用，切勿解开注释。
        // InitializeLidar();
        // Web 中控台遥控绑定：v/w(±100) → 差速 left=v-w, right=v+w。
        if (motors_) {
            auto* motors = motors_.get();
            ld14_web_set_motion_sink(
                [motors](int v, int w) {
                    int l = v - w, r = v + w;
                    l = l < -100 ? -100 : (l > 100 ? 100 : l);
                    r = r < -100 ? -100 : (r > 100 ? 100 : r);
                    motors->SetSpeed(l, r);
                },
                [motors]() { motors->Brake(); });
            // 电机测试台绑定（/motor 页面）：单轮 A/B 或双轮 X，dir 0止/1正/2反/3刹。
            // 注意与 WS 差速遥控互斥：测试台直接写 PWM，若同时用摇杆遥控，
            // WS 看门狗（300ms 无心跳即刹车）可能覆盖测试台指令，联调时只用其一。
            // 电机云端 MQTT 桥：robot/cmd 下发 + robot/telemetry 200ms 遥测。
            // 与本地 HTTP/WS 遥控互斥，云端联调时不要再用摇杆/测试台页面。
            motor_mqtt_init(motors);
            ld14_web_set_motor_sinks(
                [motors](char m, int dir, int speed) {
                    int signed_sp = (dir == 2) ? -speed : (dir == 1 ? speed : 0);
                    if (m == 'X') {
                        if (dir == 3) {
                            motors->Brake();
                        } else {
                            motors->SetSpeed(signed_sp, signed_sp);
                        }
                        return;
                    }
                    const int idx = (m == 'B') ? 1 : 0;
                    if (dir == 3) {
                        motors->BrakeChannel(idx);
                    } else {
                        motors->SetChannelSpeed(idx, signed_sp);
                    }
                },
                [motors](char m, float& rpm, int32_t& cnt) -> bool {
                    if (m == 'B') {
                        rpm = motors->GetRightRpm();
                        cnt = motors->GetRightCount();
                    } else {
                        rpm = motors->GetLeftRpm();
                        cnt = motors->GetLeftCount();
                    }
                    return true;
                });
        }
        InitializeButtons();
        InitializeMcpTools();
        // NOTE: WebEmotionServer must NOT start here: the TCP/IP stack is not
        // up yet in the constructor (httpd_start would hit an LwIP
        // "Invalid mbox" assert). It is started from OnBoardNetworkEvent()
        // after Wi-Fi reports Connected (= GOT_IP).
    }

    void SetNetworkEventCallback(NetworkEventCallback callback) override {
        app_network_cb_ = std::move(callback);
        WifiBoard::SetNetworkEventCallback(
            [this](NetworkEvent event, const std::string& data) { OnBoardNetworkEvent(event, data); });
    }

    std::string GetBoardType() override { return "xiaozhi-pcb-arm"; }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    Display* GetDisplay() override { return display_.get(); }

    void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            if (lidar_) {
                lidar_->SetMotorSpeed(70);
            }
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    std::string GetDeviceStatusJson() override {
        auto json = WifiBoard::GetDeviceStatusJson();
        if (motors_ && lidar_) {
            // Replace trailing '}' with robot section.
            if (!json.empty() && json.back() == '}') {
                json.pop_back();
                json += R"(,"robot":{)";
                json += R"("left_encoder":)" + std::to_string(motors_->GetLeftCount()) + R"(,)";
                json += R"("right_encoder":)" + std::to_string(motors_->GetRightCount()) + R"(,)";
                auto lidar_status = lidar_->GetStatus();
                json += R"("lidar_frames":)" + std::to_string(lidar_status.frames_received) + R"(,)";
                json += R"("lidar_crc_errors":)" + std::to_string(lidar_status.crc_errors);
                json += R"(})";
            }
        }
        return json;
    }
};

DECLARE_BOARD(XiaoZhiPcbArmBoard);
