#pragma once
// 差速底盘正逆运动学：(v,w) <-> (l,r)，单位统一为 -100~100。
#include "common/filters.h"
namespace algo {
struct Vel { float v = 0; float w = 0; };   // 线速度/角速度（归一化）
struct Wheel { float l = 0; float r = 0; }; // 左右轮
// 逆解：v/w -> l/r，wheel_base 为轮距系数(越大转向越缓)
inline Wheel VelToWheel(Vel vw, float wheel_base = 1.0f) {
    Wheel wh;
    wh.l = Clamp(vw.v - vw.w * wheel_base * 0.5f, -100.0f, 100.0f);
    wh.r = Clamp(vw.v + vw.w * wheel_base * 0.5f, -100.0f, 100.0f);
    return wh;
}
// 正解：l/r -> v/w
inline Vel WheelToVel(Wheel wh, float wheel_base = 1.0f) {
    Vel vw;
    vw.v = (wh.l + wh.r) * 0.5f;
    vw.w = wheel_base > 0 ? (wh.r - wh.l) / wheel_base : 0;
    return vw;
}
}  // namespace algo
