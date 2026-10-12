#ifndef __WEB_EMOTION_H__
#define __WEB_EMOTION_H__

#include <esp_http_server.h>

class EyeDisplay;

// 网页表情控制台：浏览器打开 http://<设备IP>/ 点选表情。
// 路由：GET /（控制台页面）、GET /api/emotion?id=xxx（切换）、
//       GET /api/state（当前状态 JSON）。
class WebEmotionServer {
public:
    WebEmotionServer() = default;
    ~WebEmotionServer() { Stop(); }

    bool Start(EyeDisplay* eyes);
    void Stop();

    // 供 LD14 等独立模块复用同一 httpd 实例（连网前返回 nullptr）。
    static WebEmotionServer* instance() { return s_instance_; }
    httpd_handle_t server_handle() const { return server_; }

private:
    static esp_err_t OnIndex(httpd_req_t* req);
    static esp_err_t OnSetEmotion(httpd_req_t* req);
    static esp_err_t OnState(httpd_req_t* req);
    static void StarterTask(void* arg);

    EyeDisplay* eyes_ = nullptr;
    httpd_handle_t server_ = nullptr;
    bool starter_running_ = false;
    static WebEmotionServer* s_instance_;
};

#endif  // __WEB_EMOTION_H__
