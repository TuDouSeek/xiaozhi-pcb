#ifndef __LIDAR_DRIVER_H__
#define __LIDAR_DRIVER_H__

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>

// Skeleton driver for the LD-06 / LD-19 style UART lidar:
//  - 230400 baud 8N1, point-cloud frames of 47 bytes (0x54 0x2C header)
//  - PWM output controls the lidar motor speed (25 kHz)
//
// If the wired lidar uses a different frame format, adapt kFrameSize /
// ParseFrame() below; the UART plumbing stays the same.
class LidarDriver {
public:
    struct ScanPoint {
        float angle_deg;       // 0..360
        uint16_t distance_mm;  // 0 means no valid measurement
        uint8_t quality;       // 0..255
    };

    struct Status {
        bool uart_ready = false;
        bool task_running = false;
        uint32_t frames_received = 0;
        uint32_t bytes_received = 0;
        uint32_t crc_errors = 0;
        int motor_duty_percent = 0;
    };

    LidarDriver(gpio_num_t uart_rx_pin, int baudrate, gpio_num_t pwm_pin);
    ~LidarDriver();

    // Starts the UART + motor PWM; returns false if the UART cannot be opened.
    bool Start();
    void Stop();

    // PWM motor speed, 0..100 percent.
    void SetMotorSpeed(int duty_percent);

    // Latest partial scan (per-degree bins). Empty while no valid frame.
    std::vector<ScanPoint> GetLatestScan() const;
    Status GetStatus() const;

    // Invoked from the lidar task after each fully parsed frame.
    std::function<void()> on_frame_received;

private:
    static void TaskEntry(void* arg);
    void Task();

    bool ParseFrame(const uint8_t* frame, size_t len);

    gpio_num_t uart_rx_pin_;
    int baudrate_;
    gpio_num_t pwm_pin_;
    std::atomic<bool> running_{false};
    std::atomic<int> motor_duty_{0};
    uart_port_t uart_num_;

    mutable std::mutex mutex_;
    Status status_;
    // One bin per degree of rotation, in mm.
    std::vector<ScanPoint> scan_;
};

#endif  // __LIDAR_DRIVER_H__