#pragma once
// 极简避障：前方有障碍则减速+向宽敞侧转向，否则直行。
#include "../perception/scan.h"
#include "../motion/diff_drive.h"
namespace algo {
// stop_mm: 小于此距离刹车；slow_mm: 小于此距离减速
inline Vel Avoid(const std::vector<Point>& pts, float cruise = 60.0f,
                 float stop_mm = 400.0f, float slow_mm = 1200.0f) {
    Vel out;
    float front = FrontClearance(pts);
    if (front < stop_mm) {
        float l = SideClearance(pts, true);
        float r = SideClearance(pts, false);
        out.v = 0;
        out.w = (l > r) ? 60.0f : -60.0f;  // 原地转向，向宽敞侧
        return out;
    }
    out.v = cruise;
    if (front < slow_mm) out.v = cruise * (front / slow_mm);
    float l = SideClearance(pts, true);
    float r = SideClearance(pts, false);
    out.w = Clamp((r - l) * 0.02f, -40.0f, 40.0f);  // 轻微避让
    return out;
}
}  // namespace algo
