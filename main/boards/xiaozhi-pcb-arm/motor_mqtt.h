#ifndef __MOTOR_MQTT_H__
#define __MOTOR_MQTT_H__

// 电机云端 MQTT 桥：robot/cmd（下发）/ robot/telemetry（遥测）。
// 由板级代码在 MotorController 就绪后调用 motor_mqtt_init(&motors) 启动，
// 常驻后台；与本地 HTTP/WS 遥控互斥（联调时只用其一）。

#include <functional>

#include <driver/gpio.h>

#include "motor_controller.h"

// 下发通道抽象：默认直接绑 MotorController，也便于单元测试替换。
struct MotorMqttConfig {
    const char* broker_uri = "mqtt://103.231.59.46:1883";
    const char* client_id = "xiaozhi_robot_s3";
    const char* topic_cmd = "robot/cmd";
    const char* topic_telemetry = "robot/telemetry";
    uint32_t cmd_timeout_ms = 2000;    // 超过此时长无指令 → 自动刹车
    uint32_t telemetry_period_ms = 200;  // 遥测发布周期
};

// 启动云端桥（幂等，可重复调用）。motors 必须长存活（板级 unique_ptr）。
void motor_mqtt_init(MotorController* motors, const MotorMqttConfig& config = {});

// 停止桥并释放 client（一般不需要调，重启由整机负责）。
void motor_mqtt_deinit(void);

#endif  // __MOTOR_MQTT_H__
