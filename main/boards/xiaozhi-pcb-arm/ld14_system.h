#ifndef __LD14_SYSTEM_H__
#define __LD14_SYSTEM_H__

// LD14 子系统无侵入式接入点。主入口仅需调用一行：
//
//     ld14_system_init();
//
// 它按序完成：雷达驱动（UART2 解析任务 + GPIO40 PWM 软启动）
// 与 Web 可视化挂载（/lidar 页面 + /ws/lidar 推送，复用既有 httpd）。
// 不改动麦克风/音频/语音/网络底层任何业务逻辑与引脚。

#ifdef __cplusplus
extern "C" {
#endif

void ld14_system_init(void);

#ifdef __cplusplus
}
#endif

#endif  // __LD14_SYSTEM_H__
