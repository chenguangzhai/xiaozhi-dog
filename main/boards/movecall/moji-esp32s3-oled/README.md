# Movecall Moji（0.96 寸 OLED 版）

在 Moji（ESP32-S3）硬件上，将原 GC9A01 圆形 LCD（SPI）替换为与面包板同款的
0.96 寸 OLED（I2C，SSD1306 128x64）。

## 硬件说明

- 显示：SSD1306 128x64，I2C 接口
  - SDA：GPIO1
  - SCL：GPIO2
  - 使用独立 I2C 总线（I2C1），与 ES8311 编解码器的 I2C0 分离
- 音频：ES8311（I2S + I2C 控制），引脚与原 Moji 一致
- 板载 LED：GPIO21；BOOT 按键：GPIO0
- 机器狗舵机：4 路 PWM（LEDC 驱动，50Hz，正弦步态 Oscillator）
  - SERVO1 → GPIO15 → 右前腿
  - SERVO2 → GPIO41 → 右后腿
  - SERVO3 → GPIO7  → 左前腿
  - SERVO4 → GPIO42 → 左后腿
  - 由 `eda_dog_movements.*` 提供步态，`eda_dog_controller.cc` 注册 MCP 工具

> 该板为独立板子变体，不修改原 `movecall/moji-esp32s3`（板子身份影响 OTA 兼容性）。
> 若使用 SH1106 屏，可在 menuconfig 中将 OLED Type 切换为 SH1106 128x64。

## 编译配置命令

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子（Target Board）：**

```
Xiaozhi Assistant -> Target Board -> Movecall Moji (0.96 OLED)
```

**选择 OLED 型号（OLED Type）：**

```
Xiaozhi Assistant -> OLED Type -> SSD1306 128*64
```

> 注意：该板 `OLED Type` 默认为 `SSD1306 128*64`（自动匹配 0.96 寸屏）；
> 若接 1.3 寸 SH1106 屏，需手动切换为 `SH1106 128*64`。
> 若用 `scripts/build.py` 构建，则由 config.json 默认指定 `SSD1306 128*64`，
> 可用 `--build-options-json '{"display_model": "OLED_SH1106_128X64"}'` 切换。

**编译：**

```bash
idf.py build
```

也可使用构建脚本：

```bash
python3 scripts/build.py movecall/moji-esp32s3-oled --name movecall-moji-esp32s3-oled
```