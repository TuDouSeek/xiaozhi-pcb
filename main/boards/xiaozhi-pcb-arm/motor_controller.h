#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include <cstdint>

#include <atomic>
#include <mutex>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pulse_cnt.h>

// Dual AT8870 H-bridge controller with quadrature encoder feedback.
//
// Wiring per channel: IN1/IN2 are PWM inputs on the AT8870 (forward = IN1
// duty with IN2 low, reverse = IN2 duty with IN1 low, both high = brake).
// Encoders are 6-wire motors with A/B quadrature phases read by PCNT.
//
// Speed range is -100..100 percent per channel. PWM runs at 20 kHz.
class MotorController {
public:
    struct Config {
        // H-bridge inputs: motor A (left) and motor B (right)
        gpio_num_t in_a1 = GPIO_NUM_NC;
        gpio_num_t in_a2 = GPIO_NUM_NC;
        gpio_num_t in_b1 = GPIO_NUM_NC;
        gpio_num_t in_b2 = GPIO_NUM_NC;
        // Quadrature encoder inputs: motor A and motor B
        gpio_num_t enc_a1 = GPIO_NUM_NC;
        gpio_num_t enc_a2 = GPIO_NUM_NC;
        gpio_num_t enc_b1 = GPIO_NUM_NC;
        gpio_num_t enc_b2 = GPIO_NUM_NC;
        // Optional AT8870 standby pin, shared by both drivers
        gpio_num_t stby = GPIO_NUM_NC;
        // 相位取反：某轮若因安装镜像导致“正转反跑”，置 true 即可在软件层
        // 同时反转驱动方向与编码器符号，rpm/cnt/闭环全部自洽，无需改线。
        bool invert_a = false;
        bool invert_b = false;
    };

    explicit MotorController(const Config& config);
    ~MotorController();

    // left/right in percent, -100 (full reverse) .. 100 (full forward)
    // 开环直驱：关闭速度闭环（如曾使能），PWM 直接跟随输入占空比。
    void SetSpeed(int left_percent, int right_percent);
    void Stop();
    void Brake();
    // 单轮刹车（仅该路 IN1/IN2 置 HIGH，另一路不受影响）。
    void BrakeChannel(int index);
    // 单轮调速（保持另一轮当前值，常用于测试台单轮调试）。
    void SetChannelSpeed(int index, int percent);

    int32_t GetLeftCount() const;
    int32_t GetRightCount() const;
    void ClearCounts();

    bool IsMotorsReady() const { return motors_ready_; }
    bool IsEncodersReady() const { return encoders_ready_; }

    // ---- 编码器测速（20ms 级周期调用 UpdateSpeed，或起后台任务） ----
    // 单圈计数 = 11(PPR) × 减速比 × 4(正交四倍频)，默认减速比 30，
    // 实测减速比后用 SetGearRatio() 校准，否则 RPM 按比例偏差。
    void SetGearRatio(float ratio);
    // 最近一次 UpdateSpeed() 换算的轮速（电机轴 RPM，EMA 平滑后）。
    float GetLeftRpm() const;
    float GetRightRpm() const;
    // 起后台测速任务（Core 1，prio 2，默认 20ms 周期）：刷新 RPM；
    // 若闭环使能则同步跑 PID。幂等，可重复调用。
    bool StartSpeedTask(uint32_t period_ms = 20);
    // 手动刷新一次测速（未起后台任务时由外部周期调用，dt_ms 为距上次间隔）。
    void UpdateSpeed(uint32_t dt_ms);

    // ---- PID 速度闭环（位置式，50Hz 级调用效果最佳） ----
    // 使能后 SetSpeed() 切换为“目标转速模式”：输入 percent 按空载满速
    // 线性映射为目标 RPM，再由 PID 跟踪编码器实测 RPM。
    void EnableClosedLoop(bool enable);
    bool IsClosedLoop() const;
    void SetTargetRpm(float left_rpm, float right_rpm);
    void SetPidGains(float kp, float ki, float kd);
    void SetNoLoadRpm(float rpm);  // percent→RPM 映射满量程，默认 200

private:
    struct EncoderUnit {
        pcnt_unit_handle_t unit = nullptr;
        bool ready = false;
    };

    void InitPwm();
    bool InitEncoder(gpio_num_t pin_a, gpio_num_t pin_b, EncoderUnit& unit);
    void ApplyChannel(int index, int percent);
    float CountsPerRev() const;
    static void SpeedTask(void* arg);

    Config config_;
    bool motors_ready_ = false;
    bool encoders_ready_ = false;
    EncoderUnit encoder_a_;
    EncoderUnit encoder_b_;
    int current_percent_[2] = {0, 0};
    int dir_sign_[2] = {1, 1};  // 相位符号：invert 的轮为 -1，驱动与编码器同反
    // HTTP/WS/测速多任务并发调用 SetSpeed/Brake/UpdateSpeed 时的互斥保护。
    // LEDC/PCNT 寄存器操作本身短小，此锁只保 current_percent_/PID 状态一致性。
    mutable std::mutex mutex_;

    // ---- 测速与 PID 状态 ----
    float gear_ratio_ = 30.0f;  // 实测减速比后用 SetGearRatio() 校准
    float rpm_[2] = {0.0f, 0.0f};
    int last_count_[2] = {0, 0};
    float kp_ = 1.2f, ki_ = 3.0f, kd_ = 0.02f;  // 起步值，按实车整定
    float integ_[2] = {0.0f, 0.0f};
    float prev_err_[2] = {0.0f, 0.0f};
    float no_load_rpm_ = 200.0f;  // percent→RPM 满量程，按空载实测校准
    std::atomic<float> target_rpm_[2] = {0.0f, 0.0f};
    std::atomic<bool> closed_loop_ = false;
    bool speed_task_started_ = false;
    uint32_t speed_period_ms_ = 20;
};

#endif  // __MOTOR_CONTROLLER_H__
