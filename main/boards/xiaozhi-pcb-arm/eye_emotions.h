#pragma once

// 表情配置文件：main/assets/EAF/*.eaf -> 双圆屏播放映射。
// 命名规则：`<范围>-<表情>-<版本>.eaf`
//   范围: L = 左屏(KS10), R = 右屏(CS9), Public = 双屏同播
//   表情: 中文状态名；版本: 迭代号，配置文件只关心最新版。
//
// 后续添加新表情：
//   1. 把 .eaf 丢进 main/assets/EAF/，沿用命名规则；
//   2. 在 kEyeEmotions 末尾加一行；
//   3. 在 main/CMakeLists.txt 的 EYE_EAF_FILES 列表加一行(ASCII 别名会自动嵌入)；
//   4. 重新 idf.py build 即可。小智AI/随机逻辑直接调 PlayEmotion(id)。

#include <stddef.h>

namespace eye_cfg {

enum class EyeScope : unsigned char { LEFT = 0, RIGHT = 1, PUBLIC = 2 };

struct EyeEmotion {
    const char* id;       // ASCII id，供代码/MCP/AI调用，如 "shangxin"
    const char* label;    // 中文状态名，如 "伤心"
    const char* left_src;   // main/assets/EAF 下左屏源文件名(UTF-8)
    const char* right_src;  // 右屏源文件名；Public 表情与 left_src 相同
    const char* left_alias;   // ASCII 构建别名(嵌入用)
    const char* right_alias;
    EyeScope scope;         // PUBLIC = 双屏同播；LEFT/RIGHT 成对由两行组成
    unsigned version;       // 当前版本
    bool idle_random;       // 是否参与无指令时的随机播放(眨眼/滚动等)
};

// 当前 11 个文件去重后的 8 种状态：
//   伤心/单左眼闭/生气 = L+R 成对；左右滚动/微笑/无语/眨眼/闭眼 = Public 双屏同播。
static constexpr EyeEmotion kEyeEmotions[] = {
    {"shangxin", "伤心", "L-shangxin-1.eaf", "R-shangxin-1.eaf",
     "eye_shangxin_L.eaf", "eye_shangxin_R.eaf", EyeScope::LEFT, 1, false},
    {"danzuoyanbi", "单左眼闭", "L-danzuoyanbi-1.eaf", "R-danzuoyanbi-1.eaf",
     "eye_danzuoyanbi_L.eaf", "eye_danzuoyanbi_R.eaf", EyeScope::LEFT, 1, false},
    {"shengqi", "生气", "L-shengqi-1.eaf", "R-shengqi-1.eaf",
     "eye_shengqi_L.eaf", "eye_shengqi_R.eaf", EyeScope::LEFT, 1, false},
    {"zuoyougundong", "左右滚动", "Public-zuoyougundong-1.eaf", "Public-zuoyougundong-1.eaf",
     "eye_zuoyougundong.eaf", "eye_zuoyougundong.eaf", EyeScope::PUBLIC, 1, true},
    {"weixiao", "微笑", "Public-weixiao-1.eaf", "Public-weixiao-1.eaf",
     "eye_weixiao.eaf", "eye_weixiao.eaf", EyeScope::PUBLIC, 1, false},
    {"wuyu", "无语", "Public-wuyu-1.eaf", "Public-wuyu-1.eaf",
     "eye_wuyu.eaf", "eye_wuyu.eaf", EyeScope::PUBLIC, 1, false},
    {"zhayan", "眨眼", "Public-zhayan-1.eaf", "Public-zhayan-1.eaf",
     "eye_zhayan.eaf", "eye_zhayan.eaf", EyeScope::PUBLIC, 1, true},
    {"biyan", "闭眼", "Public-biyan-1.eaf", "Public-biyan-1.eaf",
     "eye_biyan.eaf", "eye_biyan.eaf", EyeScope::PUBLIC, 1, false},
};

static constexpr size_t kEyeEmotionCount = sizeof(kEyeEmotions) / sizeof(kEyeEmotions[0]);

inline const EyeEmotion* FindEmotion(const char* id) {
    if (id == nullptr) {
        return nullptr;
    }
    for (size_t i = 0; i < kEyeEmotionCount; ++i) {
        const char* a = kEyeEmotions[i].id;
        const char* b = id;
        while (*a && *b && *a == *b) {
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0') {
            return &kEyeEmotions[i];
        }
    }
    return nullptr;
}

}  // namespace eye_cfg
