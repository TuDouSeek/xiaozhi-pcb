#ifndef __ROBOT_IMU_H__
#define __ROBOT_IMU_H__

#include <cstdint>

#include <driver/i2c_master.h>
#include <esp_err.h>

// BMI270（I2C 0x68）挂在板级 I2C 总线上，与 OLED 共用 GPIO41/42。
// init 内部完成：软复位 → 上传 8KB 配置 → 200Hz 采样任务。
// 采样任务启动后先做 1.5s 静止零偏标定，此后 yaw 由陀螺积分得到。
esp_err_t robot_imu_init(i2c_master_bus_handle_t bus, uint8_t addr = 0x68);

bool robot_imu_is_ready();
bool robot_imu_calib_done();

// yaw / gz 单位 deg 与 deg/s；pitch/roll 由加速度计解算（deg）。
void robot_imu_read(float* yaw_deg, float* gz_dps, float* pitch_deg, float* roll_deg);

// 当前陀螺零偏（dps）与芯片温度（°C，未做温补，仅参考）。
float robot_imu_bias_dps();
float robot_imu_temperature();

// 扫描板级 I2C 总线（0x08..0x77），返回设备地址个数，用于接线自检。
int robot_imu_scan(uint8_t* addresses, int max_count);

// 异步触发重新标定：机器人必须静止，完成后 yaw 归零。
void robot_imu_request_calibration();

// 立即把 yaw 置为指定角度（默认 0），用于重设航向基准。
void robot_imu_reset_yaw(float yaw_deg = 0.0f);

#endif  // __ROBOT_IMU_H__
