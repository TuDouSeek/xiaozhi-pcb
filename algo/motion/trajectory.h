#pragma once
// 梯形速度规划：给定目标速度，按加速度斜坡逼近，防起步抖/急停前冲。
#include "common/filters.h"
namespace algo {
class Ramp {
public:
    float accel = 120.0f;  // 每秒变化量
    float Update(float target, float dt) {
        float step = accel * dt;
        if (cur_ < target) { cur_ += step; if (cur_ > target) cur_ = target; }
        else if (cur_ > target) { cur_ -= step; if (cur_ < target) cur_ = target; }
        return Clamp(cur_, -100.0f, 100.0f);
    }
    void Reset(float v = 0) { cur_ = v; }
private:
    float cur_ = 0;
};
}  // namespace algo
