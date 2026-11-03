#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// Movecall Moji 板级配置
// 对应主控：ESP32-S3；屏幕：GC9A01 圆形 LCD（240x240，SPI 接口）
// 音频编解码：ES8311（I2C 控制 + I2S 数据）
// 本文件只声明引脚映射与硬件参数，不含任何初始化逻辑

#include <driver/gpio.h>

// ---------------- 音频参数 ----------------
// 采样率：输入（麦克风）与输出（扬声器）均为 24kHz
#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// ---------------- 音频 I2S 引脚 ----------------
// 连接 ES8311 的 I2S 总线：主时钟、帧同步（WS/LRCK）、位时钟、数据输入/输出
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_6
#define AUDIO_I2S_GPIO_WS GPIO_NUM_12
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_13
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_11

// ---------------- 音频编解码控制引脚 ----------------
// PA：功放使能引脚；I2C：ES8311 寄存器控制总线；ADDR：ES8311 默认 I2C 从机地址
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_9
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_5
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_4
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR

// ---------------- 板载 LED 与按键 ----------------
// 板载单色 LED 与 BOOT 按键（GPIO0，低电平有效）
#define BUILTIN_LED_GPIO        GPIO_NUM_21
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

// ---------------- 显示参数 ----------------
// GC9A01 圆形屏分辨率 240x240；MIRROR_X 需镜像以匹配实际像素排布
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false

// 显示偏移（该屏无偏移，保持 0）
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

// 背光：PWM 引脚；OUTPUT_INVERT 为 false 表示高电平点亮
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_3
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// ---------------- 显示 SPI 引脚 ----------------
// GC9A01 使用 SPI（SCLK/MOSI/CS/DC/RESET）
#define DISPLAY_SPI_SCLK_PIN    GPIO_NUM_16
#define DISPLAY_SPI_MOSI_PIN    GPIO_NUM_17
#define DISPLAY_SPI_CS_PIN      GPIO_NUM_15
#define DISPLAY_SPI_DC_PIN      GPIO_NUM_7
#define DISPLAY_SPI_RESET_PIN   GPIO_NUM_18

// 显示 SPI 时钟频率：40MHz
#define DISPLAY_SPI_SCLK_HZ     (40 * 1000 * 1000)

#endif // _BOARD_CONFIG_H_
