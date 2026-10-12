#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

// ESP32-S3 指定引脚
#define I2C_SDA_PIN 41
#define I2C_SCL_PIN 42

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 波形视窗参数
#define WAVE_X 4
#define WAVE_Y 16
#define WAVE_W 120
#define WAVE_H 44

// 离散柱状条参数
#define BAR_WIDTH 1   // 每根柱子宽 1 像素
#define BAR_GAP   1   // 柱子之间间隙 1 像素
#define TOTAL_BARS (WAVE_W / (BAR_WIDTH + BAR_GAP)) // 60 根柱子

// 两侧各固定静止的柱子数量（对应约 10 像素宽度）
#define FLAT_BAR_COUNT 5 

// 状态栏：居中显示格式化时间字符串
void drawStatusBar(int signalBars, const char* timeStr, int batteryPct) {
  // 1. 信号格
  for (int i = 0; i < 4; i++) {
    int h = (i + 1) * 2;
    int bx = 2 + i * 4;
    int by = 10 - h;
    if (i < signalBars) display.fillRect(bx, by, 3, h, SSD1306_WHITE);
    else display.drawRect(bx, by, 3, h, SSD1306_WHITE);
  }

  // 2. 居中时间（8 个字符宽 48px，在 128 宽屏幕上居中起点为 (128-48)/2 = 40）
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 2);
  display.print(timeStr);

  // 3. 电池图标
  display.drawRect(104, 2, 18, 8, SSD1306_WHITE);
  display.fillRect(122, 4, 2, 4, SSD1306_WHITE);
  int fillW = map(constrain(batteryPct, 0, 100), 0, 100, 0, 14);
  if (fillW > 0) display.fillRect(106, 4, fillW, 4, SSD1306_WHITE);

  display.drawFastHLine(0, 13, 128, SSD1306_WHITE);
}

// 核心：离散语音音条绘制
void renderDiscreteBars(float voiceEnergy, float phase) {
  int midY = WAVE_Y + (WAVE_H / 2);

  for (int b = 0; b < TOTAL_BARS; b++) {
    int barX = WAVE_X + b * (BAR_WIDTH + BAR_GAP);
    int barHeight = 2; // 默认平线微点高度

    // 判断是否在中间活跃跳动区
    if (b >= FLAT_BAR_COUNT && b < (TOTAL_BARS - FLAT_BAR_COUNT)) {
      int activeIdx = b - FLAT_BAR_COUNT;
      int activeTotal = TOTAL_BARS - (FLAT_BAR_COUNT * 2);
      float t = (float)activeIdx / (float)(activeTotal - 1); // 0.0 ~ 1.0

      // 汉宁窗两端压制：保证跳动到左右边界时高度自然衰减到基线
      float window = 0.5f * (1.0f - cos(2.0f * 3.1415926f * t));

      // 叠加多频谐波产生起伏包络
      float f1 = sin((t * 8.0f) + phase * 2.5f);
      float f2 = cos((t * 16.0f) - phase * 3.8f) * 0.5f;
      float envelope = (f1 + f2 + 1.5f) * 0.35f;

      // 随机毛刺因子
      float jitter = (float)random(40, 100) / 100.0f;

      // 计算最终高度
      float dynamicH = envelope * jitter * voiceEnergy * (WAVE_H * 0.85f) * window;
      barHeight = (int)dynamicH;
      if (barHeight < 2) barHeight = 2;
    }

    // 上下对称居中展开绘制
    int barY = midY - (barHeight / 2);
    barY = constrain(barY, WAVE_Y, WAVE_Y + WAVE_H - barHeight);

    display.fillRect(barX, barY, BAR_WIDTH, barHeight, SSD1306_WHITE);
  }
}

void setup() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
}

void loop() {
  display.clearDisplay();

  // --- 时间计算逻辑（初始时间设为 12:00:00，开机自动递增） ---
  static const unsigned long START_HOURS = 12;
  static const unsigned long START_MINS  = 0;
  static const unsigned long START_SECS  = 0;
  static const unsigned long START_TOTAL_SECS = START_HOURS * 3600 + START_MINS * 60 + START_SECS;

  unsigned long totalSeconds = START_TOTAL_SECS + (millis() / 1000);
  int hours   = (totalSeconds / 3600) % 24;
  int minutes = (totalSeconds / 60) % 60;
  int seconds = totalSeconds % 60;

  char timeStr[9];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, seconds);

  // --- 模拟机器人说话语调节奏 ---
  static float voiceEnergy = 0.0f;
  static float targetEnergy = 0.0f;
  static unsigned long lastTime = 0;
  static float phase = 0.0f;

  unsigned long now = millis();

  static int duration = 80;
  if (now - lastTime > duration) {
    lastTime = now;
    duration = random(50, 130);

    if (random(0, 100) < 20) {
      targetEnergy = 0.08f;
    } else {
      targetEnergy = (float)random(45, 100) / 100.0f;
    }
  }

  // 滤波平滑
  voiceEnergy += (targetEnergy - voiceEnergy) * 0.4f;
  phase += 0.22f;

  // 1. 刷新状态栏（传入格式化后的时间）
  drawStatusBar(4, timeStr, 90);

  // 2. 绘制离散语音波形条
  renderDiscreteBars(voiceEnergy, phase);

  display.display();
  delay(12);
}