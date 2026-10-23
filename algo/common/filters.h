#pragma once
namespace algo {
// 一阶低通 / EMA
inline float LowPass(float prev, float x, float alpha) {
    return prev + alpha * (x - prev);
}
inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
// 角度归一化到 [-180, 180)
inline float NormAngle(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
}  // namespace algo
