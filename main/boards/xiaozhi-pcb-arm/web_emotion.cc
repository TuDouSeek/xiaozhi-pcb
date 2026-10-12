#include "web_emotion.h"

#include <string>

#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "eye_display.h"
#include "eye_emotions.h"

#define TAG "WebEmotion"

namespace {

constexpr uint16_t kWebPort = 80;

std::string BuildIndexPage(const char* current) {
    std::string html =
        "<!DOCTYPE html><html lang=\"zh\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>表情控制台</title>"
        "<style>body{font-family:sans-serif;max-width:480px;margin:24px auto;padding:0 16px}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
        "button{font-size:18px;padding:16px;border-radius:12px;border:1px solid #888;cursor:pointer}"
        "button.on{background:#222;color:#fff}"
        "#st{margin:12px 0;color:#555}</style></head><body>"
        "<h2>机器人表情</h2><div id=\"st\">当前: ";
    html += (current != nullptr ? current : "-");
    html += "</div><div class=\"grid\">";
    for (size_t i = 0; i < eye_cfg::kEyeEmotionCount; ++i) {
        const auto& em = eye_cfg::kEyeEmotions[i];
        html += "<button data-id=\"";
        html += em.id;
        html += "\"";
        if (current != nullptr && std::string(current) == em.id) {
            html += " class=\"on\"";
        }
        html += ">";
        html += em.label;
        html += "</button>";
    }
    html +=
        "</div><script>"
        "async function setEmotion(id){await fetch('/api/emotion?id='+id);refresh();}"
        "async function refresh(){try{const r=await fetch('/api/state');const j=await r.json();"
        "document.getElementById('st').textContent='当前: '+j.current;"
        "document.querySelectorAll('button').forEach(b=>b.classList.toggle('on',b.dataset.id===j.current));"
        "}catch(e){}}"
        "document.querySelectorAll('button').forEach(b=>b.onclick=()=>setEmotion(b.dataset.id));"
        "setInterval(refresh,2000);</script></body></html>";
    return html;
}

}  // namespace

WebEmotionServer* WebEmotionServer::s_instance_ = nullptr;

bool WebEmotionServer::Start(EyeDisplay* eyes) {
    s_instance_ = this;
    if (server_ != nullptr || starter_running_) {
        return true;
    }
    eyes_ = eyes;
    // httpd_start() crashes (lwip assert) if called before the TCP/IP stack is
    // up. The board constructor runs before esp_netif_init(), so defer the
    // real start to a task that waits for the first netif to appear.
    starter_running_ = true;
    if (xTaskCreate(StarterTask, "web_emotion", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create starter task");
        starter_running_ = false;
        return false;
    }
    return true;
}

void WebEmotionServer::StarterTask(void* arg) {
    auto* self = static_cast<WebEmotionServer*>(arg);
    // Wait until esp_netif_init() has run (first netif => tcpip thread alive).
    for (int i = 0; i < 120 && esp_netif_get_nr_of_ifs() == 0; ++i) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (esp_netif_get_nr_of_ifs() == 0) {
        ESP_LOGE(TAG, "No netif after 60s, web console disabled");
        self->starter_running_ = false;
        vTaskDelete(nullptr);
        return;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kWebPort;
    // WS worker 跑 cJSON 解析 + 800B 级收发，4096 在开启 WS 后偏紧，
    // 栈溢出会直接 abort 并掐断连接。SRAM 已省出 60KB+，给到 6144。
    // task_priority=3：低于 Wi-Fi 驱动/AFE 音频任务(5)，避免饿死音频环形缓冲。
    config.stack_size = 6144;
    config.task_priority = 3;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    // WS 长连接每个都会独占一个 httpd 会话槽：httpd 内部已占 3 个，
    // max_open_sockets 上限为 7，超配会直接 httpd_start failed。
    // keep-alive + LRU：轮询短连接复用不堆积，槽满时淘汰最久未用，
    // 杜绝 accept ENFILE(23) 挤爆。
    // max_uri_handlers 16：默认 8 个 URI 槽被 emotion(3)+资源/OTA 路由占去大半，
    // lidar 系 7 条（/lidar,/ws/lidar,/favicon,/api/lidar_status×2,/api/lidar_points×2）
    // 否则最后两条注册报 ESP_ERR_HTTPD_HANDLERS_FULL，前端拿 404。
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.keep_alive_enable = true;
    config.max_uri_handlers = 16;
    if (httpd_start(&self->server_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        self->server_ = nullptr;
        self->starter_running_ = false;
        vTaskDelete(nullptr);
        return;
    }
    const httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = OnIndex,
                                   .user_ctx = self};
    const httpd_uri_t set_uri = {.uri = "/api/emotion", .method = HTTP_GET,
                                 .handler = OnSetEmotion, .user_ctx = self};
    const httpd_uri_t state_uri = {.uri = "/api/state", .method = HTTP_GET, .handler = OnState,
                                   .user_ctx = self};
    httpd_register_uri_handler(self->server_, &index_uri);
    httpd_register_uri_handler(self->server_, &set_uri);
    httpd_register_uri_handler(self->server_, &state_uri);
    self->starter_running_ = false;
    ESP_LOGI(TAG, "Emotion console at http://<device-ip>/");
    vTaskDelete(nullptr);
}

void WebEmotionServer::Stop() {
    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
}

esp_err_t WebEmotionServer::OnIndex(httpd_req_t* req) {
    auto* self = static_cast<WebEmotionServer*>(req->user_ctx);
    const char* current =
        (self != nullptr && self->eyes_ != nullptr) ? self->eyes_->current_emotion() : nullptr;
    std::string page = BuildIndexPage(current);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page.c_str(), page.size());
}

esp_err_t WebEmotionServer::OnSetEmotion(httpd_req_t* req) {
    auto* self = static_cast<WebEmotionServer*>(req->user_ctx);
    if (self == nullptr || self->eyes_ == nullptr) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "eyes not ready");
    }
    char query[64] = {};
    char id[32] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "id", id, sizeof(id)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing id");
    }
    if (!self->eyes_->PlayEmotion(id)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown emotion");
    }
    httpd_resp_set_type(req, "application/json");
    std::string body = std::string("{\"ok\":true,\"current\":\"") + id + "\"}";
    return httpd_resp_send(req, body.c_str(), body.size());
}

esp_err_t WebEmotionServer::OnState(httpd_req_t* req) {
    auto* self = static_cast<WebEmotionServer*>(req->user_ctx);
    const char* current =
        (self != nullptr && self->eyes_ != nullptr) ? self->eyes_->current_emotion() : "";
    httpd_resp_set_type(req, "application/json");
    std::string body = std::string("{\"current\":\"") + current + "\"}";
    return httpd_resp_send(req, body.c_str(), body.size());
}
