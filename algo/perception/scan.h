#pragma once
// 雷达点云预处理：过滤无效点，输出最近障碍。
// 输入 pts: [angle°, distMm]；输出各扇区最小距离。
#include <vector>
namespace algo {
struct Point { float angle = 0; float dist_mm = 0; };
// 前向扇区(-fov/2~fov/2)最小距离，超量程按 max_mm 计
inline float FrontClearance(const std::vector<Point>& pts, float fov = 60.0f, float max_mm = 6000.0f) {
    float best = max_mm;
    for (const auto& p : pts) {
        float a = p.angle;
        if (a > 180.0f) a -= 360.0f;
        if (a < -fov * 0.5f || a > fov * 0.5f) continue;
        if (p.dist_mm < 50.0f) continue;  // 过近盲区丢弃
        if (p.dist_mm < best) best = p.dist_mm;
    }
    return best;
}
// 左右扇区最小距离（左转决策用）：left=true 取 30°~150°
inline float SideClearance(const std::vector<Point>& pts, bool left, float max_mm = 6000.0f) {
    float best = max_mm;
    for (const auto& p : pts) {
        float a = p.angle;
        if (a < 0) a += 360.0f;
        bool in = left ? (a >= 30.0f && a <= 150.0f) : (a >= 210.0f && a <= 330.0f);
        if (!in) continue;
        if (p.dist_mm < 50.0f) continue;
        if (p.dist_mm < best) best = p.dist_mm;
    }
    return best;
}
}  // namespace algo
