#pragma once
// 通用 PID（位置式），无 ESP 依赖，可在 PC/ESP 通用。
namespace algo {
class Pid {
public:
    float kp = 1.2f, ki = 3.0f, kd = 0.02f;
    float i_max = 100.0f, out_max = 100.0f;
    float Update(float err, float dt) {
        if (dt <= 0) return out_;
        integ_ += err * dt;
        if (integ_ > i_max) integ_ = i_max;
        if (integ_ < -i_max) integ_ = -i_max;
        float d = (err - prev_) / dt;
        prev_ = err;
        out_ = kp * err + ki * integ_ + kd * d;
        if (out_ > out_max) out_ = out_max;
        if (out_ < -out_max) out_ = -out_max;
        return out_;
    }
    void Reset() { integ_ = 0; prev_ = 0; out_ = 0; }
private:
    float integ_ = 0, prev_ = 0, out_ = 0;
};
}  // namespace algo
