#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// Movecall Moji（0.96 寸 OLED 版）板级配置
// 硬件与 moji-esp32s3 基本一致，仅显示部分由 GC9A01 圆形 LCD（SPI）
// 替换为 0.96 寸 SSD1306 OLED（I2C），故音频/按键等配置保持不变

#include <driver/gpio.h>

// ---------------- 音频参数 ----------------
// 采样率：输入（麦克风）与输出（扬声器）均为 24kHz
#define AUDIO_INPUT_SAMPLE_RATE 24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// ---------------- 音频 I2S 引脚 ----------------
// 连接 ES8311 的 I2S 总线：主时钟、帧同步（WS/LRCK）、位时钟、数据输入/输出
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_6
#define AUDIO_I2S_GPIO_WS GPIO_NUM_12
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_13
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_11

// ---------------- 音频编解码控制引脚 ----------------
// PA：功放使能引脚；I2C：ES8311 寄存器控制总线（I2C0）；ADDR：默认从机地址
#define AUDIO_CODEC_PA_PIN GPIO_NUM_9
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_5
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_4
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR

// ---------------- 板载 LED 与按键 ----------------
#define BUILTIN_LED_GPIO GPIO_NUM_21
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// ---------------- 机器狗舵机（4 路，LEDC 驱动）----------------
// 接线：SERVO1=IO15、SERVO2=IO41、SERVO3=IO07、SERVO4=IO42
// 本机装配对应：SERVO1=右前腿、SERVO2=右后腿、SERVO3=左前腿、SERVO4=左后腿
#define LEFT_FRONT_LEG_PIN GPIO_NUM_7    // 左前腿 -> SERVO3
#define LEFT_REAR_LEG_PIN GPIO_NUM_42    // 左后腿 -> SERVO4
#define RIGHT_FRONT_LEG_PIN GPIO_NUM_15  // 右前腿 -> SERVO1
#define RIGHT_REAR_LEG_PIN GPIO_NUM_41   // 右后腿 -> SERVO2

// ---------------- 0.96 寸 OLED 显示（I2C）----------------
// 使用独立 I2C 总线（I2C1），避免与 ES8311 的 I2C0 冲突
#define DISPLAY_SDA_PIN GPIO_NUM_1
#define DISPLAY_SCL_PIN GPIO_NUM_2

#define DISPLAY_WIDTH 128

// 分辨率由 Kconfig 的 DISPLAY_OLED_TYPE 选择（默认随构建变体设为 128x64）
#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT 32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT 64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT 64
#define SH1106
#else
#error "OLED display type is not selected"
#endif

// ---------------- 电池电量检测（ADC 分压采样）----------------
// 2S 锂电池组（标称 7.4V / 满电 8.4V）：
// VBAT -- R11(750kΩ) --+-- R12(100kΩ) -- GND
//                      |
//                   IO18_ADC（GPIO18 = ADC2_CHANNEL_7）
// 分压比 = 100k / (750k + 100k)，满电时引脚约 0.99V，在 12dB 衰减量程内
#define BATTERY_ADC_GPIO GPIO_NUM_18
#define BATTERY_ADC_UNIT ADC_UNIT_2
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_7
#define BATTERY_UPPER_RESISTOR_OHM 750000.0f
#define BATTERY_LOWER_RESISTOR_OHM 100000.0f

// OLED 整屏 180° 反转（设备倒装场景）：
// 相对原始可读方向（X/Y 同时镜像）把 X、Y 两个方向同时取反，
// 效果等于把整块屏幕物理翻转过来，上下与左右都颠倒
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false

#endif  // _BOARD_CONFIG_H_