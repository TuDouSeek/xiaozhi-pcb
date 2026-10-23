// algo PC 仿真最小示例：g++ -std=c++17 -I. sim/test_drive.cc -o /tmp/t && /tmp/t
#include <cstdio>
#include <vector>
#include "motion/diff_drive.h"
#include "navigation/avoid.h"
int main() {
    // 模拟：正前方 0° 处 300mm 有障碍
    std::vector<algo::Point> pts;
    for (int a = 0; a < 360; a += 3) pts.push_back({(float)a, 3000.0f});
    pts.push_back({0.0f, 300.0f});
    algo::Vel vw = algo::Avoid(pts, 60.0f);
    algo::Wheel wh = algo::VelToWheel(vw);
    printf("v=%.1f w=%.1f -> l=%.1f r=%.1f\n", vw.v, vw.w, wh.l, wh.r);
    return 0;
}
