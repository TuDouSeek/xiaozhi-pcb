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
    const char* broker_uri = "mqtt://103.236.92.193:60008";
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

// 运行时 broker 配置：URI 优先取 NVS（namespace "mqtt" / key "uri"），
// 未设置时回落到 MotorMqttConfig::broker_uri。换服务器不必重烧固件。
const char* motor_mqtt_broker_uri(void);

// 切换 broker 并立即重连；persist=true 同时写入 NVS。
// uri 支持 mqtt:// mqtts:// tcp:// ssl:// ws:// wss://，长度 8..95。
bool motor_mqtt_set_broker_uri(const char* uri, bool persist);

// 清除 NVS 覆盖值，恢复编译期默认 broker（重启生效）。
bool motor_mqtt_reset_broker_uri(void);

#endif  // __MOTOR_MQTT_H__
