// Movecall Moji（ESP32-S3 + GC9A01 圆屏）板级实现
// 职责：初始化 I2C（音频编解码控制）、SPI（显示）、GC9A01 屏幕、按键，
// 并向框架提供 LED / Display / Backlight / AudioCodec 等能力

#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"


// 日志标签，用于串口调试定位该板卡输出
#define TAG "MovecallMojiESP32S3"

// 自定义 LCD 显示类：继承 SpiLcdDisplay，仅对圆屏 UI 做小幅适配
class CustomLcdDisplay : public SpiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy) 
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        // 注意：UI 定制应放在 SetupUI() 中，而不是构造函数里，
        // 以保证访问 lvgl 对象时它们已被创建
    }

    virtual void SetupUI() override {
        // 先调用父类 SetupUI()，创建状态栏、表情等全部 lvgl 对象
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        // 屏幕为圆形，状态栏左右两侧需加大内边距，避免文字被圆形边缘裁切
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.33, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.33, 0);
    }
};

// 板卡主类：WifiBoard 派生，负责全部硬件初始化与能力注册
class MovecallMojiESP32S3 : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;  // ES8311 编解码器的 I2C 控制总线
    Button boot_button_;                     // BOOT 按键（GPIO0）
    Display* display_;                       // 全局显示对象

    // 初始化 I2C 主机总线，供 ES8311 编解码器读写寄存器
    void InitializeCodecI2c() {
        // I2C 主控配置：I2C0、SDA/SCL 引脚来自 config.h、启用内部上拉
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    // 初始化显示用 SPI 总线（SPI3）
    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        // 传输缓冲按“整屏一帧”配置：宽 × 高 × 2 字节
        spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(DISPLAY_SPI_SCLK_PIN, DISPLAY_SPI_MOSI_PIN, 
                                    DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // 初始化 GC9A01 圆屏：安装 panel IO、驱动，并创建显示对象
    void InitializeGc9a01Display() {
        ESP_LOGI(TAG, "Init GC9A01 display");

        // 安装 LCD panel IO（SPI 控制通道）：CS/DC 引脚来自 config.h
        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(DISPLAY_SPI_CS_PIN, DISPLAY_SPI_DC_PIN, NULL, NULL);
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;  // 40MHz
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));
    
        // 安装 GC9A01 面板驱动
        ESP_LOGI(TAG, "Install GC9A01 panel driver");
        esp_lcd_panel_handle_t panel_handle = NULL;
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;    // 不使用复位引脚时设为 -1
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR; // 该屏像素为 BGR 顺序
        panel_config.bits_per_pixel = 16;                       // 由 LCD 命令 `3Ah` 实现（16/18bit）

        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));      // 复位
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));       // 初始化（下发厂商初始化序列）
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));  // 反色，修正色彩显示
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false)); // X 轴镜像
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));   // 打开显示

        // 创建 SPI LCD 显示对象，交由框架统一驱动 UI
        display_ = new SpiLcdDisplay(io_handle, panel_handle,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    // 初始化按键：BOOT 键单击行为
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // 启动阶段（尚未连接）按下 BOOT：进入配网模式
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            // 其他状态下切换聊天（开始/停止对话）
            app.ToggleChatState();
        });
    }

public:
    // 构造函数：按顺序初始化各硬件，并恢复背光亮度
    MovecallMojiESP32S3() : boot_button_(BOOT_BUTTON_GPIO) {  
        InitializeCodecI2c();
        InitializeSpi();
        InitializeGc9a01Display();
        InitializeButtons();
        GetBacklight()->RestoreBrightness();
    }

    // 提供板载单色 LED
    virtual Led* GetLed() override {
        static SingleLed led_strip(BUILTIN_LED_GPIO);
        return &led_strip;
    }

    // 提供显示对象
    virtual Display* GetDisplay() override {
        return display_;
    }
    
    // 提供 PWM 背光（引脚与极性来自 config.h）
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    // 提供 ES8311 音频编解码器（I2S 引脚、PA 使能、I2C 地址来自 config.h）
    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }
};

// 向构建系统注册唯一的板卡工厂
DECLARE_BOARD(MovecallMojiESP32S3);
