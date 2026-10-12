#include "motor_mqtt.h"

#include <string.h>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mqtt_client.h>

#include <cJSON.h>

#define TAG "MotorMqtt"

namespace {

// event_handler 回调（mqtt 任务线程）只做解析 + 下发，禁止阻塞；
// telemetry 独立任务 200ms 发布；看门狗在 telemetry 循环内顺带检查。
MotorController* s_motors = nullptr;
MotorMqttConfig s_cfg;
esp_mqtt_client_handle_t s_client = nullptr;
uint32_t s_last_cmd_ms = 0;
bool s_started = false;
bool s_mqtt_started = false;  // client_start 只允许执行一次（防 Wi-Fi 重连反复触发）
TaskHandle_t s_tel_task = nullptr;

uint32_t NowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

int Clamp100(int v) { return v < -100 ? -100 : (v > 100 ? 100 : v); }

void ApplyCmd(int l, int r) {
    if (s_motors == nullptr) {
        return;
    }
    s_motors->SetSpeed(Clamp100(l), Clamp100(r));
    s_last_cmd_ms = NowMs();
}

// robot/cmd 负载：{"speed_l":int,"speed_r":int} 或 {"action":"stop"}
void HandleCmd(const char* data, int len) {
    if (data == nullptr || len <= 0 || len > 256) {
        return;
    }
    char tmp[257];
    memcpy(tmp, data, len);
    tmp[len] = '\0';
    cJSON* root = cJSON_Parse(tmp);
    if (root == nullptr) {
        ESP_LOGW(TAG, "cmd JSON 解析失败");
        return;
    }
    const cJSON* action = cJSON_GetObjectItem(root, "action");
    if (cJSON_IsString(action)) {
        const char* a = action->valuestring;
        // 动作指令：缺 speed 时取默认 60；manual 透传 speed_l/speed_r。
        const cJSON* sl0 = cJSON_GetObjectItem(root, "speed_l");
        const cJSON* sr0 = cJSON_GetObjectItem(root, "speed_r");
        int mag = 60;
        if (cJSON_IsNumber(sl0)) {
            mag = sl0->valueint >= 0 ? sl0->valueint : -sl0->valueint;
        } else if (cJSON_IsNumber(sr0)) {
            mag = sr0->valueint >= 0 ? sr0->valueint : -sr0->valueint;
        }
        if (strcmp(a, "stop") == 0) {
            ApplyCmd(0, 0);
        } else if (strcmp(a, "brake") == 0) {
            if (s_motors != nullptr) {
                s_motors->Brake();
                s_last_cmd_ms = NowMs();
            }
        } else if (strcmp(a, "forward") == 0) {
            ApplyCmd(mag, mag);
        } else if (strcmp(a, "backward") == 0) {
            ApplyCmd(-mag, -mag);
        } else if (strcmp(a, "left") == 0) {
            ApplyCmd(-mag, mag);
        } else if (strcmp(a, "right") == 0) {
            ApplyCmd(mag, -mag);
        } else if (strcmp(a, "manual") == 0) {
            int l = cJSON_IsNumber(sl0) ? sl0->valueint : 0;
            const cJSON* sr1 = cJSON_GetObjectItem(root, "speed_r");
            int r = cJSON_IsNumber(sr1) ? sr1->valueint : 0;
            ApplyCmd(l, r);
        } else {
            ESP_LOGW(TAG, "未知 action: %s", a);
        }
        cJSON_Delete(root);
        return;
    }
    const cJSON* sl = cJSON_GetObjectItem(root, "speed_l");
    const cJSON* sr = cJSON_GetObjectItem(root, "speed_r");
    if (cJSON_IsNumber(sl) || cJSON_IsNumber(sr)) {
        int l = cJSON_IsNumber(sl) ? sl->valueint : 0;
        int r = cJSON_IsNumber(sr) ? sr->valueint : 0;
        ApplyCmd(l, r);
        ESP_LOGI(TAG, "cmd L=%d R=%d", Clamp100(l), Clamp100(r));
    } else {
        ESP_LOGW(TAG, "cmd 缺 speed_l/speed_r");
    }
    cJSON_Delete(root);
}

void TelemetryTask(void* arg) {
    (void)arg;
    char body[96];
    while (s_client != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(s_cfg.telemetry_period_ms));
        if (s_client == nullptr) {
            break;
        }
        // 2s 看门狗：失联自动抱死刹车（Brake 双轮 HIGH，比滑行停更短距离，
        // 与本地 WS 的 300ms 看门狗独立，云端链路延迟大，取 2s）。
        static bool braked = false;
        if (s_motors != nullptr && NowMs() - s_last_cmd_ms > s_cfg.cmd_timeout_ms) {
            if (!braked) {
                braked = true;
                s_motors->Brake();
                ESP_LOGW(TAG, "cmd 超时 %ums，无指令自动刹车",
                         (unsigned)s_cfg.cmd_timeout_ms);
            }
        } else {
            braked = false;  // 链路恢复：解除闩锁，下次失联可再次触发
        }
        if (s_motors == nullptr) {
            continue;
        }
        float rpm_l = s_motors->GetLeftRpm();
        float rpm_r = s_motors->GetRightRpm();
        int32_t cnt_l = s_motors->GetLeftCount();
        int32_t cnt_r = s_motors->GetRightCount();
        int n = snprintf(body, sizeof(body),
                         "{\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"cnt_l\":%ld,\"cnt_r\":%ld}",
                         rpm_l, rpm_r, (long)cnt_l, (long)cnt_r);
        esp_mqtt_client_publish(s_client, s_cfg.topic_telemetry, body, n, 0, 0);
    }
    s_tel_task = nullptr;
    vTaskDelete(nullptr);
}

void MqttEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data) {    (void)arg;
    (void)base;
    auto* event = static_cast<esp_mqtt_event_handle_t>(data);
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "云端已连接，订阅 %s", s_cfg.topic_cmd);
            esp_mqtt_client_subscribe_single(event->client, s_cfg.topic_cmd, 0);
            s_last_cmd_ms = NowMs();  // 连接瞬间视为有心跳，避免刚连上就触发看门狗
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "云端断开，自动重连中…");
            break;
        case MQTT_EVENT_DATA:
            if (event->topic_len == (int)strlen(s_cfg.topic_cmd) &&
                strncmp(event->topic, s_cfg.topic_cmd, event->topic_len) == 0) {
                HandleCmd(event->data, event->data_len);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT 错误（检查 Broker IP/端口/网络）");
            break;
        default:
            break;
    }
}

// 板级构造阶段 default event loop 尚未创建，esp_event_handler_register
// 会报 ESP_ERR_INVALID_STATE 被 ERROR_CHECK 复位。因此不用事件，
// 改用轻量轮询任务等有效 IP（WIFI_STA_DEF 非 0.0.0.0）后再 start。
void MqttStartTask(void* arg) {
    (void)arg;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip = {};
    while (true) {
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
            if (ip.ip.addr != 0) {
                break;  // 已联网
            }
        } else {
            netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "网络就绪（IP: " IPSTR "），启动 MQTT…", IP2STR(&ip.ip));
    s_last_cmd_ms = NowMs();  // 从联网起算 2s 看门狗窗口
    if (esp_mqtt_client_start(s_client) != ESP_OK) {
        ESP_LOGE(TAG, "mqtt client 启动失败，500ms 后由本任务重试");
        // start 失败极少见：回退等待，IP 仍有效则下轮直接重试
        vTaskDelay(pdMS_TO_TICKS(500));
        if (esp_mqtt_client_start(s_client) != ESP_OK) {
            ESP_LOGE(TAG, "mqtt client 二次启动失败，等待下次重启");
            vTaskDelete(nullptr);
            return;
        }
    }
    s_mqtt_started = true;
    if (s_tel_task == nullptr) {
        xTaskCreate(TelemetryTask, "motor_mqtt_tel", 3072, nullptr, 2, &s_tel_task);
    }
    vTaskDelete(nullptr);
}

}  // namespace

void motor_mqtt_init(MotorController* motors, const MotorMqttConfig& config) {
    if (motors == nullptr) {
        ESP_LOGE(TAG, "motors 为空，云端桥未启动");
        return;
    }
    if (s_started) {
        return;
    }
    s_motors = motors;
    s_cfg = config;
    s_last_cmd_ms = NowMs();

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = s_cfg.broker_uri;
    cfg.credentials.client_id = s_cfg.client_id;
    cfg.session.keepalive = 30;
    cfg.session.disable_clean_session = false;
    cfg.network.disable_auto_reconnect = false;
    cfg.network.reconnect_timeout_ms = 5000;
    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == nullptr) {
        ESP_LOGE(TAG, "mqtt client 创建失败");
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client,
                                                   (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                                   MqttEventHandler, nullptr));
    // 关键：此时 default event loop / LwIP 都未就绪，不注册事件、不 start。
    // 起轻量轮询任务等有效 IP 后再 client_start（见 MqttStartTask），
    // 否则 ERROR_CHECK(0x103) 或 tcpip mbox 断言直接复位。
    xTaskCreate(MqttStartTask, "motor_mqtt_start", 3072, nullptr, 2, nullptr);
    s_started = true;
    ESP_LOGI(TAG, "电机云端桥就绪（待联网后启动）: %s cmd=%s tel=%s",
             s_cfg.broker_uri, s_cfg.topic_cmd, s_cfg.topic_telemetry);
}

void motor_mqtt_deinit(void) {
    if (!s_started) {
        return;
    }
    s_started = false;
    if (s_client != nullptr) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = nullptr;
    }
    // TelemetryTask 检测到 s_client 为空后自杀。
}
