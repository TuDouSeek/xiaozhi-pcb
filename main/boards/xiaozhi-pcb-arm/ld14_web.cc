#include "ld14_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include "ld14_driver.h"
#include "web_emotion.h"

#include <atomic>
#include <functional>

#include <cJSON.h>
#include <esp_timer.h>

#define TAG "LD14Web"

namespace {

constexpr int kUdpPeriodMs = 50;  // 雷达 UDP 广播 20Hz：254B 小包对 Wi-Fi/CPU 近零负担
constexpr uint16_t kUdpPort = 8888;
constexpr int kUdpPoints = 120;  // 3°/bin，与驱动融合图一致
constexpr uint32_t kDriveTimeoutMs = 300;  // 心跳看门狗：失联即刹车

// ---- 紧凑二进制协议（254B/包，远小于 MTU，零分片） ----
// 索引 i ↔ 角度 i*3°，只传距离，不传 angle/conf。
#pragma pack(push, 1)
struct LidarUdpPacket {
    uint16_t magic;        // 0xAA55 帧对齐校验
    uint16_t seq;          // 自增包序号
    uint16_t rpm;
    uint16_t front_dist;   // mm
    uint16_t left_dist;    // mm
    uint16_t right_dist;   // mm
    uint16_t point_count;  // 固定 120
    uint16_t distances[kUdpPoints];
};
#pragma pack(pop)
static_assert(sizeof(LidarUdpPacket) == 254, "UDP packet must be 254B");
constexpr uint16_t kUdpMagic = 0xAA55;

// ---- 底盘运动下沉（板级绑定 MotorController） ----
Ld14DriveSink s_drive;
Ld14BrakeSink s_brake;
std::atomic<uint32_t> s_last_drive_ms{0};
std::atomic<bool> s_driving{false};
uint32_t s_last_cmd_ms = 0;

int Clamp100(int v) { return v < -100 ? -100 : (v > 100 ? 100 : v); }

// 解析上行 {"cmd":"drive","v":..,"w":..} / {"cmd":"stop"} / {"cmd":"ping"}
void HandleDriveFrame(const uint8_t* payload, size_t len) {
    if (len == 0 || len > 256 || !s_drive) {
        return;
    }
    char tmp[257];
    memcpy(tmp, payload, len);
    tmp[len] = '\0';
    cJSON* root = cJSON_Parse(tmp);
    if (root == nullptr) {
        return;
    }
    const cJSON* cmd = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "drive") == 0) {
        const cJSON* v = cJSON_GetObjectItem(root, "v");
        const cJSON* w = cJSON_GetObjectItem(root, "w");
        int iv = Clamp100(cJSON_IsNumber(v) ? v->valueint : 0);
        int iw = Clamp100(cJSON_IsNumber(w) ? w->valueint : 0);
        // 30ms 节流：保护电机任务不被 WS 洪泛。
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - s_last_cmd_ms >= 30 || (iv == 0 && iw == 0)) {
            s_last_cmd_ms = now;
            s_last_drive_ms.store(now);
            s_driving.store(iv != 0 || iw != 0);
            s_drive(iv, iw);
        }
    } else if (cJSON_IsString(cmd) && (strcmp(cmd->valuestring, "stop") == 0 ||
                                       strcmp(cmd->valuestring, "ping") == 0)) {
        if (strcmp(cmd->valuestring, "stop") == 0 && s_brake) {
            s_driving.store(false);
            s_brake();
        }
        // ping 仅保活不断言，由前端 RTT 计时使用。
        s_last_drive_ms.store((uint32_t)(esp_timer_get_time() / 1000));
    }
    cJSON_Delete(root);
}

// ---- 数据流分离 ----
// 数据流 A（控制/状态）：OnWs 只收 drive/stop/ping，立刻 return；状态经
//   GET /api/lidar_status 按需拉取（~100B），无任何周期性 JSON 推送。
// 数据流 B（雷达高速流）：独立 LidarUdpTask 以 UDP 广播 254B 二进制包。

// 数据流 B：雷达 UDP 广播任务（Core 1，prio 3，10Hz，254B/包）。
// 只做"拷贝快照 → sendto → delay"，禁止任何耗时计算，禁止 JSON。
void LidarUdpTask(void* arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket 创建失败，雷达广播 disabled");
        vTaskDelete(nullptr);
        return;
    }
    int bc = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

    // 目标：本机网段广播地址；取不到则退化为全网广播。
    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(kUdpPort);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != nullptr) {
        esp_netif_ip_info_t ip = {};
        if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0) {
            dest.sin_addr.s_addr = ip.ip.addr | ~(ip.netmask.addr);
        }
    }
    ESP_LOGI(TAG, "雷达 UDP 广播 → %s:%u @%dms/包", inet_ntoa(dest.sin_addr),
             (unsigned)kUdpPort, kUdpPeriodMs);

    LidarUdpPacket pkt = {};
    uint16_t seq = 0;
    uint32_t sent = 0;
    while (true) {
        // 底盘看门狗：每 100ms 跑一次，300ms 无心跳即刹车。
        if (s_driving.load() && s_brake) {
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            if (now - s_last_drive_ms.load() > kDriveTimeoutMs) {
                s_driving.store(false);
                s_brake();
                ESP_LOGW(TAG, "Drive watchdog: no heartbeat for %ums, BRAKE",
                         (unsigned)(now - s_last_drive_ms.load()));
            }
        }
        ld14_features_t f = {};
        const bool has = ld14_get_features(&f);
        uint16_t polar[kUdpPoints] = {};
        const int nvalid = ld14_polar120_snapshot(polar);

        pkt.magic = kUdpMagic;
        pkt.seq = seq++;
        pkt.rpm = has ? f.rpm : 0;
        pkt.front_dist = has ? f.front_min_mm : 0;
        pkt.left_dist = has ? f.left_mm : 0;
        pkt.right_dist = has ? f.right_mm : 0;
        pkt.point_count = kUdpPoints;
        memcpy(pkt.distances, polar, sizeof(pkt.distances));

        ssize_t r = sendto(sock, &pkt, sizeof(pkt), 0,
                           (struct sockaddr*)&dest, sizeof(dest));
        if (r != (ssize_t)sizeof(pkt)) {
            ESP_LOGW(TAG, "UDP sendto 短写 r=%d（Wi-Fi 拥塞？）", (int)r);
        }
        if (++sent == 1) {
            ESP_LOGI(TAG, "UDP 首包已发出 %uB valid=%d front=%umm",
                     (unsigned)sizeof(pkt), nvalid,
                     (unsigned)(has ? f.front_min_mm : 0));
        }
        if (sent % 600 == 0) {
            ESP_LOGI(TAG, "UDP 心跳 seq=%u valid=%d/120 rpm=%u",
                     (unsigned)seq, nvalid, (unsigned)(has ? f.rpm : 0));
        }
        vTaskDelay(pdMS_TO_TICKS(kUdpPeriodMs));
    }
}

// 数据流 A：点云 HTTP 降级通道（仅桥接断开时 1Hz 拉取，~600B）。
// 桥接正常时绝不调用，零额外负载；桥接未启动时保证 Canvas 仍有画面。
static void SetCors(httpd_req_t* req);  // 前置声明：定义见下方，OnLidarPoints 先用
esp_err_t OnLidarPoints(httpd_req_t* req) {
    SetCors(req);
    uint16_t polar[kUdpPoints] = {};
    ld14_polar120_snapshot(polar);
    char body[768];
    int n = snprintf(body, sizeof(body), "{\"pts\":[");
    for (int i = 0; i < kUdpPoints && n > 0 && (size_t)n < sizeof(body) - 8; ++i) {
        n += snprintf(body + n, sizeof(body) - n, "%s%u", i ? "," : "", polar[i]);
    }
    if ((size_t)n < sizeof(body) - 3) {
        n += snprintf(body + n, sizeof(body) - n, "]}");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}
// CORS：浏览器跨域拉取不报错。注意绝不能加 Connection:close——Vite 代理
// 与浏览器都用 keep-alive 复用连接，强制 close 会让代理复用已关连接，
// 反而报 ECONNRESET/socket hang up；fd 占用由低频轮询 + LRU 保证。
static void SetCors(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

// ---- 电机测试台（/motor + /api/motor/*） ----
// Web 层记录每轮最后指令（dir/speed）用于状态回显；遥测（rpm/cnt）
// 由板级 tel sink 实时读取。httpd 工作线程与测速任务并发，
// MotorController 内部持 mutex，本层只做轻量查询解析，无共享可变状态之外的竞态。
Ld14MotorCmdSink s_motor_cmd;
Ld14MotorTelSink s_motor_tel;
int s_motor_dir[2] = {0, 0};    // 0=stop 1=forward 2=backward 3=brake
int s_motor_speed[2] = {0, 0};  // 占空比 0~100

static int MotorIndex(char m) {
    if (m == 'A' || m == 'a') {
        return 0;
    }
    if (m == 'B' || m == 'b') {
        return 1;
    }
    return -1;
}

// GET /api/motor/control?motor=A|B|both&dir=forward|backward|stop|brake&speed=0~100
esp_err_t OnMotorControl(httpd_req_t* req) {
    SetCors(req);
    char query[128] = {};
    char motor[8] = {};
    char dir[16] = {};
    char speed[8] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query");
    }
    httpd_query_key_value(query, "motor", motor, sizeof(motor));
    httpd_query_key_value(query, "dir", dir, sizeof(dir));
    httpd_query_key_value(query, "speed", speed, sizeof(speed));

    int d = 0;  // 0=stop 1=forward 2=backward 3=brake
    if (strcmp(dir, "forward") == 0) {
        d = 1;
    } else if (strcmp(dir, "backward") == 0) {
        d = 2;
    } else if (strcmp(dir, "brake") == 0) {
        d = 3;
    } else if (dir[0] != '\0' && strcmp(dir, "stop") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad dir");
    }
    int sp = atoi(speed);
    if (sp < 0) {
        sp = 0;
    } else if (sp > 100) {
        sp = 100;
    }
    // dir=stop/brake 时 speed 归零回显，避免面板显示残留转速。
    if (d == 0 || d == 3) {
        sp = 0;
    }

    if (!s_motor_cmd) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "motor sink not bound");
    }
    char m = motor[0] ? (char)toupper(motor[0]) : 'X';
    if (m == 'X' || strcasecmp(motor, "both") == 0) {
        m = 'X';
        s_motor_cmd('X', d, sp);
        s_motor_dir[0] = s_motor_dir[1] = d;
        s_motor_speed[0] = s_motor_speed[1] = sp;
    } else {
        int idx = MotorIndex(m);
        if (idx < 0) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad motor");
        }
        s_motor_cmd(m, d, sp);
        s_motor_dir[idx] = d;
        s_motor_speed[idx] = sp;
    }
    char body[96];
    int n = snprintf(body, sizeof(body),
                     "{\"ok\":true,\"motor\":\"%c\",\"dir\":%d,\"speed\":%d}", m, d, sp);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

// GET /api/motor/status → {a:{dir,speed,rpm,cnt},b:{...}}
esp_err_t OnMotorStatus(httpd_req_t* req) {
    SetCors(req);
    float rpm_a = 0, rpm_b = 0;
    int32_t cnt_a = 0, cnt_b = 0;
    if (s_motor_tel) {
        s_motor_tel('A', rpm_a, cnt_a);
        s_motor_tel('B', rpm_b, cnt_b);
    }
    char body[192];
    int n = snprintf(body, sizeof(body),
                     "{\"a\":{\"dir\":%d,\"speed\":%d,\"rpm\":%.1f,\"cnt\":%ld},"
                     "\"b\":{\"dir\":%d,\"speed\":%d,\"rpm\":%.1f,\"cnt\":%ld}}",
                     s_motor_dir[0], s_motor_speed[0], (double)rpm_a, (long)cnt_a,
                     s_motor_dir[1], s_motor_speed[1], (double)rpm_b, (long)cnt_b);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

esp_err_t OnLidarStatus(httpd_req_t* req) {
    SetCors(req);
    ld14_features_t f = {};
    const bool has = ld14_get_features(&f);
    char body[160];
    int n = snprintf(body, sizeof(body),
                     "{\"ok\":%s,\"rpm\":%u,\"front\":%u,\"left\":%u,\"right\":%u,"
                     "\"gmin\":%u,\"gangle\":%u,\"escape\":%u}",
                     has ? "true" : "false", f.rpm, f.front_min_mm, f.left_mm,
                     f.right_mm, f.gmin_mm, f.gmin_angle, f.escape_angle);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

// CORS 预检：浏览器跨域 fetch 前的 OPTIONS 直接 204 放行，不进业务逻辑。
esp_err_t OnOptions(httpd_req_t* req) {
    SetCors(req);
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t OnFavicon(httpd_req_t* req) {
    return httpd_resp_send(req, nullptr, 0);  // 204 空回，消除浏览器 favicon 404 噪声
}

esp_err_t OnWs(httpd_req_t* req) {
    // 非阻塞标准模板：运行在 httpd 工作线程，处理完手头这一帧必须立刻 return。
    // 点云下行走独立 LidarUdpTask（UDP 广播），绝不在此阻塞。
    if (req->method == HTTP_GET) {
        // 握手请求：不阻塞，交给框架完成升级。
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        return ESP_OK;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        return ESP_OK;
    }
    if (ws_pkt.len > 0 && ws_pkt.len <= 1024) {
        uint8_t* buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf != nullptr) {
            ws_pkt.payload = buf;
            if (httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len) == ESP_OK) {
                if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.len > 0) {
                    HandleDriveFrame(buf, ws_pkt.len);
                }
            }
            free(buf);
        }
    }
    return ESP_OK;  // 立刻返回，保证 HTTPD 任务正常轮转与网络包 flush
}

esp_err_t OnPage(httpd_req_t* req);
esp_err_t OnFavicon(httpd_req_t* req);
esp_err_t OnMotorPage(httpd_req_t* req);  // 实现在下方电机页面区，此处前置声明
extern const char kPage[];

void WaiterTask(void* arg) {
    (void)arg;
    // 等既有 Web 后台把 httpd 建好（连网后），再挂自己的路由。
    httpd_handle_t hd = nullptr;
    for (int i = 0; i < 120; ++i) {
        WebEmotionServer* web = WebEmotionServer::instance();
        if (web != nullptr) {
            hd = web->server_handle();
        }
        if (hd != nullptr) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (hd == nullptr) {
        ESP_LOGE(TAG, "httpd not ready, lidar web disabled");
        vTaskDelete(nullptr);
        return;
    }
    const httpd_uri_t page = {.uri = "/lidar",
                              .method = HTTP_GET,
                              .handler = OnPage,
                              .user_ctx = nullptr};
    const httpd_uri_t ws = {.uri = "/ws/lidar",
                            .method = HTTP_GET,
                            .handler = OnWs,
                            .user_ctx = nullptr,
                            .is_websocket = true};
    const httpd_uri_t status = {.uri = "/api/lidar_status",
                                  .method = HTTP_GET,
                                  .handler = OnLidarStatus,
                                  .user_ctx = nullptr};
    const httpd_uri_t status_opt = {.uri = "/api/lidar_status",
                                    .method = HTTP_OPTIONS,
                                    .handler = OnOptions,
                                    .user_ctx = nullptr};
    const httpd_uri_t points = {.uri = "/api/lidar_points",
                                .method = HTTP_GET,
                                .handler = OnLidarPoints,
                                .user_ctx = nullptr};
    const httpd_uri_t points_opt = {.uri = "/api/lidar_points",
                                    .method = HTTP_OPTIONS,
                                    .handler = OnOptions,
                                    .user_ctx = nullptr};
    const httpd_uri_t motor = {.uri = "/motor",
                               .method = HTTP_GET,
                               .handler = OnMotorPage,
                               .user_ctx = nullptr};
    const httpd_uri_t motor_ctrl = {.uri = "/api/motor/control",
                                    .method = HTTP_GET,
                                    .handler = OnMotorControl,
                                    .user_ctx = nullptr};
    const httpd_uri_t motor_status = {.uri = "/api/motor/status",
                                      .method = HTTP_GET,
                                      .handler = OnMotorStatus,
                                      .user_ctx = nullptr};
    const httpd_uri_t favicon = {.uri = "/favicon.ico",
                                 .method = HTTP_GET,
                                 .handler = OnFavicon,
                                 .user_ctx = nullptr};
    esp_err_t r1 = httpd_register_uri_handler(hd, &page);
    esp_err_t r2 = httpd_register_uri_handler(hd, &ws);
    esp_err_t r3 = httpd_register_uri_handler(hd, &favicon);
    esp_err_t r4 = httpd_register_uri_handler(hd, &status);
    esp_err_t r5 = httpd_register_uri_handler(hd, &status_opt);
    esp_err_t r6 = httpd_register_uri_handler(hd, &points);
    esp_err_t r7 = httpd_register_uri_handler(hd, &points_opt);
    esp_err_t r8 = httpd_register_uri_handler(hd, &motor);
    esp_err_t r9 = httpd_register_uri_handler(hd, &motor_ctrl);
    esp_err_t r10 = httpd_register_uri_handler(hd, &motor_status);
    ESP_LOGI(TAG, "Route register: /lidar=%s /ws/lidar=%s /favicon.ico=%s /api/lidar_status=%s+%s /api/lidar_points=%s+%s /motor=%s /api/motor=%s+%s",
             esp_err_to_name(r1), esp_err_to_name(r2), esp_err_to_name(r3),
             esp_err_to_name(r4), esp_err_to_name(r5),
             esp_err_to_name(r6), esp_err_to_name(r7),
             esp_err_to_name(r8), esp_err_to_name(r9), esp_err_to_name(r10));
    if (r1 != ESP_OK || r2 != ESP_OK) {
        ESP_LOGE(TAG, "Lidar route registration failed, web disabled");
        vTaskDelete(nullptr);
        return;
    }
    // 数据流 B 独立任务：Core 1，prio 3，与音频/Wi-Fi(核0)隔离。
    xTaskCreatePinnedToCore(LidarUdpTask, "ld14_udp", 3072, nullptr, 3, nullptr, 1);
    ESP_LOGI(TAG, "Lidar console at http://<device-ip>/lidar, UDP radar on :8888");
    vTaskDelete(nullptr);
}

esp_err_t OnPage(httpd_req_t* req) {
    SetCors(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, kPage, strlen(kPage));
}

// ---- 电机测试台页面（/motor）：双轮独立 + 联动，滑块调速，1Hz 状态轮询 ----
const char kMotorPage[] = R"HTML(<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>电机测试台</title>
<style>body{background:#0b1118;color:#d7e3f0;font-family:sans-serif;max-width:560px;margin:12px auto;padding:0 12px}
h2{color:#58c4ff}h3{color:#7d93a8;margin:14px 0 6px}.card{background:#141d28;border:1px solid #24405c;border-radius:10px;padding:10px;margin-bottom:10px}
.row{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}
button{flex:1;min-width:64px;padding:10px;border-radius:10px;border:1px solid #24405c;background:#17324a;color:#fff;font-size:14px}
button.warn{background:#3a1414;border-color:#7a2b2b}button.on{outline:2px solid #58c4ff}
input[type=range]{width:100%;margin:8px 0}.tele{font-family:monospace;font-size:13px;color:#7dffb2}
#st{margin:8px 0;color:#7d93a8;font-size:13px}</style></head><body>
<h2>电机测试台</h2><div id="st">连接中…</div>
<h3>整体联控</h3><div class="card"><div class="row">
<button onclick="cmd('both','forward')">前进</button>
<button onclick="cmd('both','backward')">后退</button>
<button onclick="cmd('both','stop')" class="warn">停止</button>
<button onclick="cmd('both','brake')" class="warn">刹车</button>
</div></div>
<h3>电机 A（左轮）</h3><div class="card"><div class="row">
<button onclick="cmd('A','forward')">正转</button>
<button onclick="cmd('A','backward')">反转</button>
<button onclick="cmd('A','brake')" class="warn">刹车</button>
<button onclick="cmd('A','stop')" class="warn">停止</button>
</div><label>速度 <b id="vA">50</b>%</label>
<input type="range" id="sA" min="0" max="100" value="50" oninput="spd('A')">
<div class="tele" id="tA">rpm - · cnt -</div></div>
<h3>电机 B（右轮）</h3><div class="card"><div class="row">
<button onclick="cmd('B','forward')">正转</button>
<button onclick="cmd('B','backward')">反转</button>
<button onclick="cmd('B','brake')" class="warn">刹车</button>
<button onclick="cmd('B','stop')" class="warn">停止</button>
</div><label>速度 <b id="vB">50</b>%</label>
<input type="range" id="sB" min="0" max="100" value="50" oninput="spd('B')">
<div class="tele" id="tB">rpm - · cnt -</div></div>
<script>let dirA='forward',dirB='forward';
async function cmd(m,d){if(m==='A')dirA=d;if(m==='B')dirB=d;
const s=m==='A'?v('sA'):m==='B'?v('sB'):50;
try{await fetch('/api/motor/control?motor='+m+'&dir='+d+'&speed='+s);tick();}catch(e){}}
function v(id){return document.getElementById(id).value;}
async function spd(m){const id=m==='A'?'sA':'sB',t=m==='A'?'vA':'vB';
document.getElementById(t).textContent=v(id);
const d=m==='A'?dirA:dirB;if(d==='forward'||d==='backward')await cmd(m,d);}
const DN=['停止','正转','反转','刹车'];
async function tick(){try{const r=await fetch('/api/motor/status');const j=await r.json();
document.getElementById('tA').textContent='rpm '+j.a.rpm+' · cnt '+j.a.cnt+' · '+DN[j.a.dir]+' '+j.a.speed+'%';
document.getElementById('tB').textContent='rpm '+j.b.rpm+' · cnt '+j.b.cnt+' · '+DN[j.b.dir]+' '+j.b.speed+'%';
document.getElementById('st').textContent='在线 @1Hz';}catch(e){
document.getElementById('st').textContent='状态拉取失败…';}}
setInterval(tick,1000);tick();</script></body></html>)HTML";

esp_err_t OnMotorPage(httpd_req_t* req) {
    SetCors(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, kMotorPage, strlen(kMotorPage));
}

// ---- 内嵌暗色监控页（PROGMEM 常量，进 Flash 不占 SRAM） ----
const char kPage[] = R"HTML(<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD14 雷达监控</title>
<style>body{background:#0b1118;color:#d7e3f0;font-family:sans-serif;max-width:560px;margin:12px auto;padding:0 12px}
h2{color:#58c4ff}.cards{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:10px 0}
.card{background:#141d28;border:1px solid #24405c;border-radius:10px;padding:8px 10px}
.card b{font-size:20px;color:#7dffb2}.card small{color:#7d93a8}
canvas{background:#060a10;border:1px solid #24405c;border-radius:12px;width:100%}</style></head><body>
<h2>LD14 雷达监控</h2><canvas id="cv" width="360" height="360"></canvas>
<div class="cards">
<div class="card"><small>前方安全距离</small><br><b id="c_front">-</b> mm</div>
<div class="card"><small>左 / 右均值</small><br><b id="c_lr">-</b> mm</div>
<div class="card"><small>最近障碍</small><br><b id="c_g">-</b></div>
<div class="card"><small>逃逸航向 / RPM</small><br><b id="c_e">-</b></div>
<div class="card"><small>编码器目标 L（预留）</small><br><b id="c_el">-</b></div>
<div class="card"><small>编码器目标 R（预留）</small><br><b id="c_er">-</b></div>
</div><div id="st">连接中…（点云经 UDP :8888 广播，请用 PC 端桥接查看）</div>
<script>async function tick(){try{const r=await fetch('/api/lidar_status');const d=await r.json();
document.getElementById('c_front').textContent=d.front;
document.getElementById('c_lr').textContent=d.left+' / '+d.right;
document.getElementById('c_g').textContent=d.gmin+'mm @'+d.gangle+'°';
document.getElementById('c_e').textContent=d.escape+'° / '+d.rpm;
document.getElementById('st').textContent='状态 @0.5Hz · 点云走 UDP 广播 :8888';}catch(e){
document.getElementById('st').textContent='状态拉取失败…';}}
setInterval(tick,2000);tick();</script></body></html>)HTML";

}  // namespace

// 注意：必须定义在匿名 namespace 之外（外部链接），否则板级代码链接不到。
void ld14_web_set_motion_sink(Ld14DriveSink drive, Ld14BrakeSink brake) {
    s_drive = std::move(drive);
    s_brake = std::move(brake);
}

// 同上：匿名 namespace 内定义会变成内部链接，别的 .cc 链接时报
// undefined reference。此处与 s_motor_cmd/s_motor_tel 同一 TU，
// 隐式 using 使内部变量在此可见，编译链接无碍。
void ld14_web_set_motor_sinks(Ld14MotorCmdSink cmd, Ld14MotorTelSink tel) {
    s_motor_cmd = std::move(cmd);
    s_motor_tel = std::move(tel);
}

void ld14_web_init(void) {
    xTaskCreate(WaiterTask, "ld14_web", 3072, nullptr, 3, nullptr);
}
