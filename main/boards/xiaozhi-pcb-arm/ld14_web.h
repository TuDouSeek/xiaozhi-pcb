#ifndef __LD14_WEB_H__
#define __LD14_WEB_H__

// LD14 Web 可视化：复用既有 WebEmotionServer 的 httpd 实例（等其就绪后）
// 注册独立路由，不干扰既有 / 与 /api/emotion。
//   GET /lidar      暗色雷达监控页（Canvas 极坐标点云 + 特征卡片）
//   GET /ws/lidar   WebSocket 推送（10Hz 紧凑 JSON：特征 + 120 点点云）
// 由 ld14_system_init() 间接启动，无需业务方调用。

#include <cstdint>
#include <functional>

void ld14_web_init(void);

// 底盘运动接入：drive(v,w) 取值 ±100（v 线速度，w 角速度，正=前进/右转）；
// brake() 为急停。由板级代码绑定到 MotorController。
// 看门狗：超过 300ms 无上位机心跳自动调 brake() 防跑飞。
using Ld14DriveSink = std::function<void(int v, int w)>;
using Ld14BrakeSink = std::function<void()>;
void ld14_web_set_motion_sink(Ld14DriveSink drive, Ld14BrakeSink brake);

// 电机测试台接入：单轮独立控制（调试/标定用）。
// motor: 'A'(左) / 'B'(右) / 'X'(双轮联动)；dir: 0=滑行停止 1=正转 2=反转 3=刹车；
// speed: 占空比 0~100。由板级代码绑定到 MotorController。
using Ld14MotorCmdSink = std::function<void(char motor, int dir, int speed)>;
// 回传某单轮遥测：rpm(转/分，带方向符号) 与累计脉冲 cnt；无绑定时返回 false。
using Ld14MotorTelSink = std::function<bool(char motor, float& rpm, int32_t& cnt)>;
void ld14_web_set_motor_sinks(Ld14MotorCmdSink cmd, Ld14MotorTelSink tel);

// 直行配平接入：index 0=A(左) / 1=B(右)，trim 为开环占空比缩放系数
// （0.2~2.0，1.0 不修正）。用于抵消两轮机械差异导致的同占空比转速不同。
using Ld14TrimSetSink = std::function<void(int index, float trim)>;
using Ld14TrimGetSink = std::function<float(int index)>;
void ld14_web_set_trim_sinks(Ld14TrimSetSink set_sink, Ld14TrimGetSink get_sink);

#endif  // __LD14_WEB_H__
