// Movecall Moji（ESP32-S3 + 0.96 寸 SSD1306 OLED）板级实现
// 与 moji-esp32s3 的区别：显示由 GC9A01 圆屏（SPI）改为 0.96 寸 OLED（I2C），
// 音频仍使用 ES8311，按键/ LED 保持一致

#include "application.h"
#include "assets/lang_config.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display/oled_display.h"
#include "led/single_led.h"
#include "wifi_board.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>

// SH1106 驱动需单独包含（config.h 中根据 Kconfig 选择定义 SH1106）
#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

// 日志标签，用于串口调试定位该板卡输出
#define TAG "MovecallMojiESP32S3Oled"

// 机器狗运动控制器初始化与 MCP 工具注册（定义在 eda_dog_controller.cc）
extern void InitializeEDARobotDogController();

// ============================ 启动音频自检 ============================
// 目的：喇叭没声音 / 喊唤醒词没反应时，把「输出链路」和「输入链路」分开验证。
//   1) 输出：播 1kHz 正弦音 600ms。听到"嘀"说明 I2S → ES8311 → 功放 → 喇叭 整条通；
//   2) 输入：读 300ms 麦克风数据算 RMS。数值接近 0 说明麦克风没信号。
//
// 两条链路都失败时，优先怀疑 MCLK（GPIO6）没到芯片或 I2S 接线错 —— 那是它们共用的。
// 只有输出失败 -> 查功放使能（PA_PIN）极性和喇叭接线。
// 只有输入失败 -> 查麦克风接线、MICBIAS、ES8311 的 ADC 输入配置。
//
// 调试完把 kEnableAudioSelfTest 改成 false 即可关闭，不影响正常功能。
constexpr bool kEnableAudioSelfTest = false;

static void RunAudioSelfTest(AudioCodec* codec) {
    if (codec == nullptr) {
        ESP_LOGW(TAG, "Audio self-test skipped: codec is null");
        return;
    }
    const int sample_rate = codec->output_sample_rate();
    if (sample_rate <= 0) {
        ESP_LOGW(TAG, "Audio self-test skipped: invalid sample rate %d", sample_rate);
        return;
    }

    constexpr int kToneHz = 1000;
    constexpr int kToneMs = 600;
    constexpr int kAmplitude = 8000;  // 约 -12dBFS，够响但不至于破音
    constexpr int kChunk = 240;       // 10ms @24kHz，与 DMA 帧数对齐
    constexpr float kTwoPi = 6.28318530718f;

    // -------------------- 输出自检 --------------------
    codec->EnableOutput(true);
    if (!codec->output_enabled()) {
        // EnableOutput 在 codec_if_ 为空时会直接返回，output_enabled_ 保持 false
        ESP_LOGE(TAG, "Audio self-test: output NOT enabled, codec init failed");
        return;
    }

    const int total = sample_rate * kToneMs / 1000;
    std::vector<int16_t> buffer(kChunk);
    ESP_LOGI(TAG, "Audio self-test: playing %dHz for %dms", kToneHz, kToneMs);
    for (int done = 0; done < total; done += kChunk) {
        int n = std::min(kChunk, total - done);
        buffer.resize(n);
        for (int i = 0; i < n; i++) {
            float t = static_cast<float>(done + i) / static_cast<float>(sample_rate);
            buffer[i] = static_cast<int16_t>(kAmplitude * sinf(kTwoPi * kToneHz * t));
        }
        codec->OutputData(buffer);
    }

    // -------------------- 输入自检 --------------------
    codec->EnableInput(true);
    if (codec->input_enabled()) {
        constexpr int kInputMs = 300;
        const int need = sample_rate * kInputMs / 1000;
        std::vector<int16_t> in(kChunk);
        int64_t sum_sq = 0;
        int peak = 0;
        int count = 0;
        for (int done = 0; done < need; done += kChunk) {
            int n = std::min(kChunk, need - done);
            in.assign(n, 0);
            codec->InputData(in);
            for (int i = 0; i < n; i++) {
                int v = in[i];
                sum_sq += static_cast<int64_t>(v) * v;
                if (v < 0) {
                    v = -v;
                }
                peak = std::max(peak, v);
            }
            count += n;
        }
        int rms = count > 0 ? static_cast<int>(sqrtf(static_cast<float>(sum_sq) / count)) : 0;
        ESP_LOGI(TAG, "Audio self-test: mic rms=%d peak=%d (%d samples)", rms, peak, count);
        if (peak < 100) {
            ESP_LOGE(TAG, "Audio self-test: mic has no signal, check MCLK/mic wiring/MICBIAS");
        }
    } else {
        ESP_LOGE(TAG, "Audio self-test: input NOT enabled, codec init failed");
    }
    codec->EnableInput(false);

    codec->EnableOutput(false);
    ESP_LOGI(TAG, "Audio self-test done");
}

// ============================ 功放链路探针 ============================
// 目的：没有示波器时，用耳朵把「ES8311 模拟输出 → NS4150B → 喇叭」这三段切开。
//
// 原理：ES8311 的 DAC 每次上电/掉电，AOUT 的直流偏置会在 0V 与 VMID(≈1.5V) 之间跳变，
//       这个阶跃被 NS4150B 放大后，就是喇叭里一声「噗」。
//       因此：
//         听得到 pop  -> AVDD、VMID、NS4150B、喇叭 这一整段是好的，
//                        嫌疑立刻收窄到 MCLK 或 I2S 数字通路；
//         听不到 pop  -> 上游模拟域（AVDD / VMID）或功放 / 喇叭 有问题。
//       注意这是**单向判据**：听到能证明这一段好；听不到不能单独定论
//       （DAC 本身没输出时同样不会有 pop）。
//
// 之后紧跟一段 400Hz 满幅单音，比自检音更响更长，用来排除「音量太小根本没听见」。
//
// 代价：开机多约 4 秒。调试完把 kEnableAmpProbe 改成 false。
constexpr bool kEnableAmpProbe = false;

// 播放一段单音：hz 频率、ms 时长、amplitude 峰值（32767 为满幅）
static void PlayTone(AudioCodec* codec, int hz, int ms, int amplitude) {
    const int sample_rate = codec->output_sample_rate();
    if (sample_rate <= 0) {
        return;
    }
    constexpr int kChunk = 240;  // 10ms @24kHz，与 DMA 帧数对齐
    constexpr float kTwoPi = 6.28318530718f;

    const int total = sample_rate * ms / 1000;
    ESP_LOGW(TAG, "AUDIOTEST tone: %d Hz, %d ms, amp=%d, rate=%d, total=%d", hz, ms, amplitude,
             sample_rate, total);
    std::vector<int16_t> buffer(kChunk);
    for (int done = 0; done < total; done += kChunk) {
        int n = std::min(kChunk, total - done);
        buffer.resize(n);
        for (int i = 0; i < n; i++) {
            float t = static_cast<float>(done + i) / static_cast<float>(sample_rate);
            buffer[i] = static_cast<int16_t>(amplitude * sinf(kTwoPi * hz * t));
        }
        codec->OutputData(buffer);
    }
}

// ============================ MCLK 驱动强度 × 出声测试 ============================
// 背景（2026-10-22 17:0x）：日志已证明提示音的 PCM 数据是正常的
//   （peak 峰值 24063，约 73% 满幅），解码成功、输出已使能、全部写进了 codec，
//   但喇叭没有任何声音。也就是说数据到了 DAC 门口就断了。
// 剩下的嫌疑集中在「ES8311 收到的 MCLK 是否足够」：
//   I2C 与 MCLK 是两条独立的通路，I2C 通不代表 MCLK 能让内部 PLL 锁定；
//   而为了压住串扰，我们把 MCLK 降到了最弱驱动（CAP0），边沿被拉得很缓。
// 这个测试用「听到几声」来区分条件，不需要示波器：
//   2 声 = CAP0（当前实际运行档位）
//   3 声 = CAP3（最强驱动，边沿最陡）
// 判读：
//   只听到 3 声      -> MCLK 驱动太弱导致 PLL 锁不住，必须靠串阻在"够陡"和"够缓"之间取平衡
//   2 声和 3 声都有   -> MCLK 没问题，问题在 I2S 数据通路（DOUT/DIN 接反？格式不匹配？）
//   都没听到         -> 模拟通路或功放有问题，回到 AVDD/VMID/NS4150B
// 2026-10-28 关闭：该测试会在开机 10 秒后播放「2 声 800Hz + 两次提示音」，
// 并把输出音量强制拉到 100 且不回滚，正常使用时表现为一段很长的开机音。
// 排查 MCLK 驱动时再临时改回 true。
constexpr bool kEnableMclkDriveAudioTest = false;

static void RunMclkDriveAudioTest(AudioCodec* codec) {
    if (codec == nullptr) {
        return;
    }
    // 2026-10-22 18:3x 回退到「用户确认都能听到」的那一版顺序，作为可用基线。
    // 顺序：2 声参考音 -> 提示音@当前音量 -> 提示音@100
    // 全程 CAP0，不碰 CAP3（CAP3 会把 codec 打坏）。
    ESP_LOGW(TAG, "AUDIOTEST START: volume=%d（基线版：先参考音再提示音）", codec->output_volume());
    codec->EnableOutput(true);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGW(TAG, "AUDIOTEST 第一组: 请听 2 声参考音 800Hz");
    for (int i = 0; i < 2; i++) {
        PlayTone(codec, 800, 300, 28000);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGW(TAG, "AUDIOTEST 第二组: 请听提示音（音量 %d）", codec->output_volume());
    Application::GetInstance().PlaySound(Lang::Sounds::OGG_SUCCESS);
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGW(TAG, "AUDIOTEST 第三组: 请听提示音（音量 100）");
    codec->SetOutputVolume(100);
    Application::GetInstance().PlaySound(Lang::Sounds::OGG_SUCCESS);
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGW(TAG, "AUDIOTEST END: 2声参考音 + 提示音两次");
}

static void RunAmpProbe(AudioCodec* codec) {
    if (codec == nullptr) {
        ESP_LOGW(TAG, "Amp probe skipped: codec is null");
        return;
    }

    // 先探一次能否打开：codec_if_ 为空时 EnableOutput 会直接返回且不置位
    codec->EnableOutput(true);
    if (!codec->output_enabled()) {
        ESP_LOGE(TAG, "Amp probe skipped: codec not initialized");
        return;
    }
    codec->EnableOutput(false);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGW(TAG, "===== AMP PROBE START: listen for %d pops =====", 4);
    for (int i = 1; i <= 4; i++) {
        codec->EnableOutput(true);   // DAC 上电 -> AOUT 跳到 VMID -> 一声「噗」
        vTaskDelay(pdMS_TO_TICKS(300));
        codec->EnableOutput(false);  // DAC 掉电 -> AOUT 回 0V -> 又一声「噗」
        vTaskDelay(pdMS_TO_TICKS(300));
        ESP_LOGW(TAG, "pop %d/4 emitted", i);
    }

    ESP_LOGW(TAG, "Amp probe: now a 1.5s 400Hz full-scale tone");
    codec->EnableOutput(true);
    PlayTone(codec, 400, 1500, 28000);
    codec->EnableOutput(false);
    ESP_LOGW(TAG, "===== AMP PROBE END: pop+tone heard? =====");
    ESP_LOGW(TAG, "  heard -> AVDD/VMID/NS4150B/speaker OK, suspect MCLK or I2S");
    ESP_LOGW(TAG, "  silent -> check AVDD(pin17) / VMID / NS4150B CTRL / speaker");
}

// ============================ 麦克风底噪扫描 ============================
// 目的：把"沙沙"声量化，并判断它到底来自 PGA **之前**还是**之后**。
//
// 原理：ES8311 的 PGA 每 +6dB，前级（麦克风/偏置/漏电）的噪声就翻一倍。
//       所以把增益从 0dB 扫到 30dB，看 RMS 怎么变：
//         RMS 近似 ×31（每 6dB 翻倍）-> 噪声在 PGA 之前，降增益只是治标，
//                                      真正要处理的是洗板 / 补焊 MIC1P/MIC1N；
//         RMS 几乎不变              -> 噪声在 PGA 之后（ADC / 时钟 / 数字），降增益直接有效；
//         介于两者之间              -> 两级都有贡献。
//
// 另一个用途：如果 30dB 下安静房间的 RMS 只有几十，说明麦克风本身是干净的，
//             听到的"沙沙"其实是喇叭 / DAC 的静态底噪，方向要换。
//
// 注意：`AudioCodec::SetInputGain()` 只改成员变量，必须关闭再打开 codec
//       才会把新增益写进芯片（见 Es8311AudioCodec::UpdateDeviceState）。
//
// 调试完把 kEnableMicNoiseSweep 改成 false。
constexpr bool kEnableMicNoiseSweep = false;

// 采集 ms 毫秒，返回 RMS；peak 通过出参带回
static int MeasureMicRms(AudioCodec* codec, int ms, int* peak_out) {
    const int sample_rate = codec->input_sample_rate();
    if (sample_rate <= 0) {
        return -1;
    }
    constexpr int kChunk = 240;  // 10ms @24kHz

    const int need = sample_rate * ms / 1000;
    std::vector<int16_t> in(kChunk);
    int64_t sum_sq = 0;
    int peak = 0;
    int count = 0;
    for (int done = 0; done < need; done += kChunk) {
        int n = std::min(kChunk, need - done);
        in.assign(n, 0);
        codec->InputData(in);
        for (int i = 0; i < n; i++) {
            int v = in[i];
            sum_sq += static_cast<int64_t>(v) * v;
            if (v < 0) {
                v = -v;
            }
            peak = std::max(peak, v);
        }
        count += n;
    }
    if (peak_out != nullptr) {
        *peak_out = peak;
    }
    return count > 0 ? static_cast<int>(sqrtf(static_cast<float>(sum_sq) / count)) : 0;
}

static void RunMicNoiseSweep(AudioCodec* codec) {
    if (codec == nullptr) {
        return;
    }
    constexpr float kGains[] = {0.0f, 12.0f, 24.0f, 30.0f};
    constexpr int kGainCount = sizeof(kGains) / sizeof(kGains[0]);
    constexpr int kMeasureMs = 400;
    constexpr float kDefaultGain = 30.0f;

    ESP_LOGW(TAG, "===== MIC NOISE SWEEP: keep quiet for ~2s =====");
    int rms_at[kGainCount] = {0};
    for (int i = 0; i < kGainCount; i++) {
        codec->SetInputGain(kGains[i]);
        codec->EnableInput(false);  // 关掉，让下次 open 时把新增益写进芯片
        vTaskDelay(pdMS_TO_TICKS(50));
        codec->EnableInput(true);
        if (!codec->input_enabled()) {
            ESP_LOGE(TAG, "Mic noise sweep aborted: input not enabled");
            codec->SetInputGain(kDefaultGain);
            return;
        }
        int peak = 0;
        int rms = MeasureMicRms(codec, kMeasureMs, &peak);
        rms_at[i] = rms;
        ESP_LOGW(TAG, "gain %4.1f dB -> rms=%5d peak=%5d", kGains[i], rms, peak);
    }

    if (rms_at[0] > 0 && rms_at[kGainCount - 1] > 0) {
        float ratio = static_cast<float>(rms_at[kGainCount - 1]) / static_cast<float>(rms_at[0]);
        ESP_LOGW(TAG, "noise ratio 30dB/0dB = %.1fx  (pure pre-PGA noise would be ~31x)", ratio);
        if (ratio > 15.0f) {
            ESP_LOGW(TAG, "  => noise scales with gain: BEFORE PGA (mic / bias / flux leakage)");
        } else if (ratio < 4.0f) {
            ESP_LOGW(TAG, "  => noise flat vs gain: AFTER PGA (ADC / clock / digital)");
        } else {
            ESP_LOGW(TAG, "  => mixed: both stages contribute");
        }
    }
    ESP_LOGW(TAG, "  quiet-room rms@30dB <100 means the mic is clean (hiss is from DAC/speaker)");
    ESP_LOGW(TAG, "===== MIC NOISE SWEEP END =====");

    // 恢复默认增益，并把输入关掉（后续 AudioService 会按自己的节奏重新打开）
    codec->SetInputGain(kDefaultGain);
    codec->EnableInput(false);
}

// ============================ I2S / I2C 引脚互短自检 ============================
// 实测现象：I2S 一启动，ES8311 立刻从 99/100 ACK 变成 0/100 NACK（全 NACK、零超时）。
// 「时钟一跑就把控制线掐死」最常见的物理原因，就是 I2S 引脚与 I2C 引脚搭锡。
// 这里在两条 I2C 总线建立之前（引脚还自由）做一次互短检测：
//   把一个 I2S 脚驱动低，再看 I2C 脚（上拉输入）是否被拉低 -> 拉低即短路。
// 不依赖万用表和示波器，几毫秒出结果。
constexpr bool kEnablePinShortCheck = true;

static void RunPinShortCheck() {
    struct PinRef {
        gpio_num_t pin;
        const char* name;
    };
    const PinRef i2s_pins[] = {
        {AUDIO_I2S_GPIO_MCLK, "MCLK"},
        {AUDIO_I2S_GPIO_BCLK, "BCLK"},
        {AUDIO_I2S_GPIO_WS, "WS"},
        {AUDIO_I2S_GPIO_DOUT, "DOUT"},
        {AUDIO_I2S_GPIO_DIN, "DIN"},
    };
    const PinRef i2c_pins[] = {
        {AUDIO_CODEC_I2C_SDA_PIN, "SDA"},
        {AUDIO_CODEC_I2C_SCL_PIN, "SCL"},
    };

    // 基线：不驱动任何脚，只靠 2.2k 外部上拉，两条 I2C 线应当读到高
    for (const auto& q : i2c_pins) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << q.pin;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&cfg);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    for (const auto& q : i2c_pins) {
        if (gpio_get_level(q.pin) == 0) {
            ESP_LOGE(TAG, "PINCHECK: %s(GPIO%d) 无驱动也读低 -> 被拉到地", q.name, q.pin);
        }
    }

    int shorts = 0;
    for (const auto& p : i2s_pins) {
        gpio_config_t out_cfg = {};
        out_cfg.pin_bit_mask = 1ULL << p.pin;
        out_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&out_cfg);
        gpio_set_level(p.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (const auto& q : i2c_pins) {
            if (gpio_get_level(q.pin) == 0) {
                ESP_LOGE(TAG, "PINCHECK: %s(GPIO%d) <-> %s(GPIO%d) 短路", p.name, p.pin, q.name,
                         q.pin);
                shorts++;
            }
        }
        gpio_reset_pin(p.pin);
    }

    // 自回读：**边驱动边读**（GPIO_MODE_INPUT_OUTPUT），这样能分清两种完全不同的情况：
    //   驱动高仍读到低 -> 驱动拉不动，硬短路（焊锡桥/直接短地）
    //   驱动高读到高   -> 只是有个弱下拉电阻（设计如此，不是故障）
    // 注意不能用「改成输入再读」的办法：那样只要网上有个 10k 下拉，1ms 后就必然是低，
    // 会把正常的下拉电阻误判成短路。
    for (const auto& p : i2s_pins) {
        gpio_config_t io_cfg = {};
        io_cfg.pin_bit_mask = 1ULL << p.pin;
        io_cfg.mode = GPIO_MODE_INPUT_OUTPUT;
        gpio_config(&io_cfg);

        gpio_set_level(p.pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        int hi = gpio_get_level(p.pin);

        gpio_set_level(p.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        int lo = gpio_get_level(p.pin);

        if (hi == 0) {
            ESP_LOGE(TAG, "PINCHECK: %s(GPIO%d) 驱动高仍读低 -> 硬短路到地", p.name, p.pin);
            shorts++;
        }
        if (lo == 1) {
            ESP_LOGE(TAG, "PINCHECK: %s(GPIO%d) 驱动低仍读高 -> 硬短路到 3V3", p.name, p.pin);
            shorts++;
        }
        gpio_reset_pin(p.pin);
    }

    if (shorts == 0) {
        ESP_LOGW(TAG, "PINCHECK: I2S 与 I2C 引脚之间没测到短路");
    } else {
        ESP_LOGE(TAG, "PINCHECK: 共测到 %d 处短路，先处理焊接再谈软件", shorts);
    }

    // 归还引脚（后续 I2C/I2S 驱动会重新配置）
    for (const auto& q : i2c_pins) {
        gpio_reset_pin(q.pin);
    }
}

// ============================ MCLK / CE 桥接判定 ============================
// 已确认的事实：GPIO6(MCLK) 一旦输出 6.144MHz，ES8311 立刻 100% NACK；摘掉 MCLK 就 100/100。
// 剩下两个候选解释，用「慢速翻转 + 双地址探测」就能分开：
//   1) MCLK 与 U4 的 CE（地址选择脚）搭锡 -> 地址跟着 MCLK 电平在 0x18/0x19 之间跳；
//   2) 芯片 MCLK 输入脚损伤（或供电一加时钟就塌）-> 只要被驱动就死，与电平/频率无关。
//
// 做法：把 MCLK 当普通 GPIO，每 5ms 翻一次，翻转稳定后分别探 0x18 和 0x19：
//   低电平只认 0x18、高电平只认 0x19   -> CE 桥在 MCLK 上（确凿，去清那个焊点）
//   两个电平下 0x18 都应答               -> 慢速没问题，问题与频率相关（耦合/供电）
//   两个电平下都 0                       -> 芯片 MCLK 脚已损伤，只能换片
static void RunMclkCeBridgeCheck(i2c_master_bus_handle_t bus) {
    if (bus == nullptr) {
        return;
    }
    const uint8_t addr_low = static_cast<uint8_t>(AUDIO_CODEC_ES8311_ADDR >> 1);  // 0x18
    const uint8_t addr_high = static_cast<uint8_t>(addr_low + 1);                 // 0x19

    gpio_config_t out_cfg = {};
    out_cfg.pin_bit_mask = 1ULL << AUDIO_I2S_GPIO_MCLK;
    out_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out_cfg);

    constexpr int kRounds = 20;
    int a_low = 0, b_low = 0, a_high = 0, b_high = 0;
    for (int i = 0; i < kRounds; i++) {
        gpio_set_level(AUDIO_I2S_GPIO_MCLK, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        if (i2c_master_probe(bus, addr_low, 20) == ESP_OK) a_low++;
        if (i2c_master_probe(bus, addr_high, 20) == ESP_OK) b_low++;

        gpio_set_level(AUDIO_I2S_GPIO_MCLK, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
        if (i2c_master_probe(bus, addr_low, 20) == ESP_OK) a_high++;
        if (i2c_master_probe(bus, addr_high, 20) == ESP_OK) b_high++;
    }
    gpio_reset_pin(AUDIO_I2S_GPIO_MCLK);

    ESP_LOGW(TAG, "MCLKSLOW: MCLK=0 -> 0x%02X %d/%d, 0x%02X %d/%d", addr_low, a_low, kRounds,
             addr_high, b_low, kRounds);
    ESP_LOGW(TAG, "MCLKSLOW: MCLK=1 -> 0x%02X %d/%d, 0x%02X %d/%d", addr_low, a_high, kRounds,
             addr_high, b_high, kRounds);

    if (a_low > 0 && b_high > 0) {
        ESP_LOGE(TAG, "MCLKSLOW: 地址跟着 MCLK 电平翻转 -> MCLK 与 CE 搭锡（确凿）");
    } else if (a_low == 0 && a_high == 0) {
        ESP_LOGE(TAG, "MCLKSLOW: 一驱动 MCLK 就全 NACK -> 芯片 MCLK 脚损伤或供电塌陷");
    } else {
        ESP_LOGW(TAG, "MCLKSLOW: 慢速翻转无影响 -> 故障与频率相关（耦合/供电/接触）");
    }
}

// ============================ MCLK 频率扫描 ============================
// 已知：慢速翻转（100Hz 量级）完全不影响 I2C，6.144MHz 一开就 100% NACK。
// 那「从哪个频率开始坏」就决定了病因：
//   低频就坏（100kHz 也不应答）-> 芯片 MCLK 输入级损伤，一有时钟就吃大电流，只能换片；
//   低频正常、高频才坏          -> 动态电流把供电/地拉塌，问题在 U4 的电源脚与 GND（虚焊）。
// 用 LEDC 硬件产生方波，频率精确且不占 CPU，方波运行期间直接探 I2C。
// 用 CHANNEL_0 / TIMER_2：舵机走 TIMER_1 且跳过 CHANNEL_0，不会打架。
static const uint32_t kSweepFreqsFull[] = {100000, 300000, 1000000, 3000000, 6144000};
static const uint32_t kSweepFreqsTop[]  = {6144000};

// cap：这一轮用哪种 GPIO 驱动能力。0=最弱(约5mA) 3=最强(约40mA)。
// 2026-10-21 19:20 发现的关键缺陷：原版扫频跑在「默认驱动」下，而 ApplyWeakMclkDrive()
// 在它之后才执行，所以扫频测的是「强驱动」、真实播放用的是「最弱驱动」——两者不是同一个
// 条件。之前那个 6.144MHz=0/20 的可怕结果，根本不能代表实际运行状态。
// 现在把驱动强度变成显式参数，在同一个频率上做 弱/强 对照，才能真正回答
// 「到底是不是边沿速率驱动的容性串扰」。
static void RunMclkFreqSweep(i2c_master_bus_handle_t bus, gpio_drive_cap_t cap, const char* label,
                             const uint32_t* freqs, size_t freq_count, int samples) {
    if (bus == nullptr || samples <= 0) {
        return;
    }
    const uint8_t addr = static_cast<uint8_t>(AUDIO_CODEC_ES8311_ADDR >> 1);  // 0x18

    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = LEDC_TIMER_1_BIT;
    timer_cfg.timer_num = LEDC_TIMER_2;
    timer_cfg.freq_hz = freqs[0];
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "MCLKSWEEP[%s] skipped: ledc timer config failed", label);
        return;
    }

    ledc_channel_config_t ch_cfg = {};
    ch_cfg.gpio_num = AUDIO_I2S_GPIO_MCLK;
    ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_cfg.channel = LEDC_CHANNEL_0;
    ch_cfg.timer_sel = LEDC_TIMER_2;
    ch_cfg.duty = 1;  // 1bit 分辨率下 50% 占空比
    ch_cfg.hpoint = 0;
    if (ledc_channel_config(&ch_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "MCLKSWEEP[%s] skipped: ledc channel config failed", label);
        return;
    }

    // 必须在 ledc_channel_config 之后设置：它内部会重新配一次引脚方向，
    // 驱动能力放在前面有被覆盖的风险。
    gpio_set_drive_capability(AUDIO_I2S_GPIO_MCLK, cap);
    gpio_drive_cap_t actual = GPIO_DRIVE_CAP_DEFAULT;
    gpio_get_drive_capability(AUDIO_I2S_GPIO_MCLK, &actual);
    ESP_LOGW(TAG, "MCLKSWEEP[%s] 驱动档位请求=%d 实读=%d (0最弱 3最强)", label, (int)cap, (int)actual);

    for (size_t i = 0; i < freq_count; i++) {
        uint32_t hz = freqs[i];
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_2, hz);
        vTaskDelay(pdMS_TO_TICKS(10));  // 让方波先跑稳

        int ok = 0;
        for (int n = 0; n < samples; n++) {
            if (i2c_master_probe(bus, addr, 10) == ESP_OK) {
                ok++;
            }
        }
        ESP_LOGW(TAG, "MCLKSWEEP[%s]: %u Hz -> 0x%02X %d/%d (%.0f%%)", label, (unsigned)hz, addr, ok,
                 samples, 100.0f * ok / samples);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    gpio_reset_pin(AUDIO_I2S_GPIO_MCLK);
}

// ============================ 「无 MCLK」对照开关 ============================
// 2026-10-21 实测：I2S 启动之前 probe 0x18 = 99/100 通；I2S 一启动立刻 0/100 全 NACK。
// 也就是说「芯片在、但被 I2S 时钟掐死」。要区分是 MCLK 这条线/这个信号的问题，
// 还是芯片供电本身撑不住，做一次单变量对照：
//
//   把下面这行改成 GPIO_NUM_NC 后重新烧录 —— I2S 不再驱动 MCLK（GPIO6），
//   同时 use_mclk 传 false，告诉 ES8311 驱动不要等主时钟。
//     codec 能初始化（日志出现 Es8311AudioCodec initialized）-> 故障由 MCLK 引入
//                                                            （搭锡/短路/串扰），查 GPIO6 这条线
//     仍然全 NACK                                            -> 与 MCLK 无关，回去查 VDD/AVDD 与芯片本身
// 注意：这个状态下喇叭和麦克风都不会有声音，只用于看 I2C 是否恢复。做完记得改回来。
//
// 2026-10-21 18:0x 第二次实验：PINCHECK 已排除「I2S 与 I2C 引脚搭锡」，
// 故直接把 MCLK 摘掉做单变量对照（BCLK/WS/DOUT/DIN 仍在跑）：
//   I2C 恢复 -> 元凶就是 MCLK 这个信号（供电塌陷 / 芯片 MCLK 脚损伤 / 地弹）
//   I2C 仍全 NACK -> 与 MCLK 无关，回到 I2S 其他信号或芯片本身
//
// 18:50 第三次实验结论：MCLK 摘掉后 100/100，装回去就 0/100，且 LEDC 扫频得到
// 100k=19/20、300k=20/20、1M=19/20、3M=17/20、6.144M=3/20 的剂量-反应曲线。
// 对照原理图：排针 7=MCLK 紧贴 8=SCL，整块音频板只有 pin1 一个地且远在另一端
// -> 判定为串扰/地回流，不是供电塌陷。现在装回 MCLK，改用「降低驱动能力」验证。
constexpr gpio_num_t kAudioMclkPin = AUDIO_I2S_GPIO_MCLK;

// ============================ MCLK 驱动能力试验 ============================
// 原理：GPIO 驱动能力越弱，边沿越缓，高次谐波越少，对相邻 SCL 的串扰越小。
// 若把 MCLK 降到最低档（约 5mA）后 codec 能正常初始化，就坐实了「串扰」这个判断，
// 同时也是一个不改硬件的临时可用状态。
// 注意：必须在 I2S 接管该引脚之前设置；之后回读一次确认没被驱动重置。
constexpr bool kEnableMclkWeakDrive = true;

static void ApplyWeakMclkDrive() {
    if (!kEnableMclkWeakDrive) {
        return;
    }
    esp_err_t ret = gpio_set_drive_capability(AUDIO_I2S_GPIO_MCLK, GPIO_DRIVE_CAP_0);
    gpio_drive_cap_t cap = GPIO_DRIVE_CAP_DEFAULT;
    gpio_get_drive_capability(AUDIO_I2S_GPIO_MCLK, &cap);
    ESP_LOGW(TAG, "MCLKDRV: 设置最弱驱动 ret=0x%x, 回读=%d (0=最弱 3=最强)", ret, (int)cap);
}

// ============================ ES8311 总线诊断梯 ============================
// 起因（2026-10-21 日志）：开机时 i2c_master_probe 在 0x18 拿到 97/100 应答，
// 但随后 ES8311 的每一笔寄存器读写全部失败（I2C_If: Fail to write to dev 30）。
//
// probe 与写寄存器在协议层的差别只有「事务长度」：
//   probe     = 1 个地址字节，9 个时钟
//   write_reg = 地址 + 寄存器号 + 数据，27 个时钟，3 次 ACK
// 所以按长度逐级加码，看失败从哪一级开始，就能把「芯片/供电」和「连线/信号完整性」分开：
//   地址级就挂            -> 芯片不在、地址错、供电死
//   地址 OK 但 1 字节挂   -> 从机能收地址、收不了数据（芯片内部状态或供电塌陷）
//   2 字节才挂            -> 传输途中掉线（接触电阻 / 容性 / 上拉）
//   100kHz 全挂、10kHz 通 -> 纯信号完整性问题，芯片本身是好的
//
// 两个刻意的设计：
//   1) 本函数在 I2S（MCLK/BCLK/WS）已经跑起来之后执行，与开机那次 probe 构成
//      「前后对照」，可判断故障是否与时间/供电/时钟相关；
//   2) 阶梯里的写操作只用不存在的寄存器 0xFF，只为验证「总线能不能扛住 3 字节事务」，
//      不去碰 0x00（那是软复位），避免在 codec 已初始化成功时把它打回原形。
//
// 调试完把 kEnableCodecBusDiag 改成 false 即可关闭。
constexpr bool kEnableCodecBusDiag = true;

static void LogI2cStep(const char* step, esp_err_t ret) {
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "  DIAG %-20s OK", step);
    } else {
        // 这里必须打出真实 esp_err_t：esp_codec_dev 只把一切失败折叠成 0x10d，
        // 分不清「从机 NACK」和「总线超时」，而这两种原因的排查方向完全不同。
        ESP_LOGE(TAG, "  DIAG %-20s FAIL err=0x%x (%s)", step, ret, esp_err_to_name(ret));
    }
}

// 在指定时钟下跑一遍「事务长度阶梯」
static void RunI2cLadder(i2c_master_bus_handle_t bus, uint8_t addr7, uint32_t hz) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr7;
    dev_cfg.scl_speed_hz = hz;

    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  DIAG add_device @%uHz failed: 0x%x", (unsigned)hz, ret);
        return;
    }

    ESP_LOGW(TAG, "--- DIAG ladder @ %u Hz ---", (unsigned)hz);

    // A：只发地址（等价 probe，但走真正的 device handle）
    LogI2cStep("A addr only", i2c_master_probe(bus, addr7, 20));
    // B：地址 + 寄存器号（1 字节数据）
    uint8_t reg = 0xFF;
    LogI2cStep("B addr+reg", i2c_master_transmit(dev, &reg, 1, 100));
    // C：地址 + 寄存器号 + 数据（与 ResetCodec 同长度的 3 字节事务）
    uint8_t wr[2] = {0xFF, 0x00};
    LogI2cStep("C addr+reg+data", i2c_master_transmit(dev, wr, 2, 100));
    // D：写指针后回读 1 字节（读方向也要验，读走的是另一条 SDA 时序）
    uint8_t reg0 = 0x00;
    uint8_t rd = 0;
    ret = i2c_master_transmit_receive(dev, &reg0, 1, &rd, 1, 100);
    LogI2cStep("D read reg0x00", ret);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "  DIAG reg0x00 = 0x%02X", rd);
    }

    i2c_master_bus_rm_device(dev);
}

static void RunCodecBusDiagnostics(i2c_master_bus_handle_t bus) {
    if (bus == nullptr) {
        ESP_LOGW(TAG, "DIAG skipped: codec I2C bus is null");
        return;
    }
    const uint8_t addr7 = static_cast<uint8_t>(AUDIO_CODEC_ES8311_ADDR >> 1);  // 0x30 -> 0x18

    ESP_LOGW(TAG, "===== CODEC I2C DIAG START (after I2S started) =====");

    // 后置 probe：与开机时那次（I2S 起来之前）的成功率对比
    int ok = 0;
    int timeout = 0;
    int nack = 0;
    for (int i = 0; i < 100; i++) {
        esp_err_t ret = i2c_master_probe(bus, addr7, 10);
        if (ret == ESP_OK) {
            ok++;
        } else if (ret == ESP_ERR_TIMEOUT) {
            timeout++;
        } else {
            nack++;
        }
    }
    ESP_LOGW(TAG, "DIAG probe(after I2S) 0x%02X: %d/100 ok, %d timeout, %d nack", addr7, ok,
             timeout, nack);
    if (ok == 0) {
        ESP_LOGE(TAG, "DIAG: chip fell off the bus after I2S start -> power / MCLK coupling");
    } else if (ok < 100) {
        ESP_LOGW(TAG, "DIAG: still flaky (%d%%) -> contact / signal integrity", ok);
    }

    RunI2cLadder(bus, addr7, 100 * 1000);
    RunI2cLadder(bus, addr7, 10 * 1000);

    // 排除 CE 引脚接错：CE 拉高时 7bit 地址为 0x19
    const uint8_t alt_addr = static_cast<uint8_t>(addr7 + 1);
    if (i2c_master_probe(bus, alt_addr, 20) == ESP_OK) {
        ESP_LOGW(TAG, "DIAG 0x%02X also ACKs -> check ES8311 CE strap", alt_addr);
    }

    ESP_LOGW(TAG, "===== CODEC I2C DIAG END =====");
    ESP_LOGW(TAG, "  A挂=芯片/供电  B挂=芯片内部  C挂但10kHz通=连线/容性");
}

// 板卡主类：WifiBoard 派生，负责全部硬件初始化与能力注册
class MovecallMojiESP32S3Oled : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;         // ES8311 编解码器的 I2C 控制总线（I2C0）
    i2c_master_bus_handle_t display_i2c_bus_;       // OLED 的 I2C 总线（I2C1，独立于编解码器）
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;  // OLED panel IO
    esp_lcd_panel_handle_t panel_ = nullptr;        // OLED panel 驱动句柄
    Display* display_ = nullptr;                    // 全局显示对象
    Button boot_button_;                            // BOOT 按键（GPIO0）

    // 电池电量检测（GPIO18/ADC2_CH7，经 750k/100k 分压采样）
    adc_oneshot_unit_handle_t battery_adc_handle_ = nullptr;
    adc_cali_handle_t battery_adc_cali_handle_ = nullptr;
    bool battery_cali_valid_ = false;
    float battery_voltage_ = 0.0f;       // 最近一次有效读数（电池端电压，单位 V）
    bool battery_voltage_valid_ = false;
    int64_t battery_last_read_us_ = 0;

    // 初始化 ES8311 编解码器所用的 I2C 主机总线（I2C0）
    void InitializeCodecI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
        ScanCodecI2c();
    }

    // 诊断：先统计 ES8311 的应答成功率，再扫描整条总线。
    // 顺序很重要：单次扫描撞上 1~2% 的抖动就会误报"没有设备"，
    // 用 100 次重复探测的成功率作为主判据更可靠。
    // ES8311 的 7bit 地址由 CE 引脚决定：CE 接地 = 0x18（默认），CE 拉高 = 0x19。
    void ScanCodecI2c() {
        const uint8_t probe_addr = (AUDIO_CODEC_ES8311_ADDR >> 1);
        const int kProbeTimes = 100;
        int ok = 0;
        int timeout = 0;
        for (int i = 0; i < kProbeTimes; i++) {
            esp_err_t ret = i2c_master_probe(codec_i2c_bus_, probe_addr, 10);
            if (ret == ESP_OK) {
                ok++;
            } else if (ret == ESP_ERR_TIMEOUT) {
                timeout++;
            }
        }
        // 判据：
        //   100/100 -> 连接稳定；部分成功 -> 间歇性电气故障（虚焊/接触/上拉/供电）；
        //   0/100   -> 完全不通（未焊、接错、无供电、地址不对）。
        // 注意即便成功率很高也不能当作"已修复"：每次启动要做几十笔寄存器写，
        // 单笔失败率 p 会放大成 1-(1-p)^N 的整体失败概率。
        ESP_LOGI(TAG, "I2C0 probe 0x%02X: %d/%d ok, %d timeout", probe_addr, ok, kProbeTimes,
                 timeout);
        if (ok == 0) {
            ESP_LOGE(TAG, "I2C0: ES8311 not responding (SDA=GPIO%d, SCL=GPIO%d)",
                     AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        }

        int found = 0;
        int abnormal = 0;
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            if (addr == probe_addr) {
                continue;
            }
            esp_err_t ret = i2c_master_probe(codec_i2c_bus_, addr, 10);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "I2C0 scan: found device at 0x%02X", addr);
                found++;
            } else if (ret != ESP_ERR_NOT_FOUND) {
                // 空地址应返回 NOT_FOUND（从机 NACK）。返回超时说明总线被拉住或
                // 上升沿过慢导致采样失败，属于电气异常，记下来便于定位。
                ESP_LOGW(TAG, "I2C0 scan: addr 0x%02X unexpected err 0x%x", addr, ret);
                abnormal++;
            }
        }
        if (found > 0) {
            ESP_LOGI(TAG, "I2C0 scan: %d other device(s) on the bus", found);
        }
        if (abnormal > 0) {
            ESP_LOGW(TAG, "I2C0 scan: %d address(es) timed out, bus is glitchy", abnormal);
        }
    }

    // 初始化 OLED 所用的 I2C 主机总线（I2C1，SDA=GPIO1 / SCL=GPIO2）
    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    // 初始化 SSD1306/SH1106 OLED 显示
    void InitializeSsd1306Display() {
        // OLED 面板 IO 配置：I2C 地址 0x3C，400kHz 时钟，控制字节 1B
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags =
                {
                    .dc_low_on_data = 0,
                    .disable_control_phase = 0,
                },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        // 面板通用配置：OLED 无复位引脚，单色 1bit/pixel
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

        // 根据 config.h 中的型号选择 SH1106 或 SSD1306 驱动
#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // 复位并初始化面板；失败时降级为无屏显示，避免整机启动失败
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // 打开显示
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                   DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#ifdef SH1106
        // 第三方 SH1106 驱动的 mirror_x 实现有误（发 0xA6/0xA7 颜色反转指令，
        // 而非 0xA0/0xA1 segment remap），LVGL 挂载时的 mirror(false, false)
        // 因此无法撤销 init 阶段写入的 0xA1，导致画面左右镜像。这里在挂载后
        // 补发 0xA0（segment remap off）校正 X 方向；之后的 mirror 调用只会
        // 发送 0xA6/0xC0，不会破坏该设置。
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, 0xA0, NULL, 0));
#endif
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

    // 初始化电池电压采样：ADC2_CH7（GPIO18）+ 板载 750k/100k 分压
    void InitializeBatteryMonitor() {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = BATTERY_ADC_UNIT,
        };
        if (adc_oneshot_new_unit(&init_cfg, &battery_adc_handle_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create battery ADC unit");
            battery_adc_handle_ = nullptr;
            return;
        }
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_oneshot_config_channel(battery_adc_handle_, BATTERY_ADC_CHANNEL, &chan_cfg) !=
            ESP_OK) {
            ESP_LOGE(TAG, "Failed to config battery ADC channel");
            adc_oneshot_del_unit(battery_adc_handle_);
            battery_adc_handle_ = nullptr;
            return;
        }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = BATTERY_ADC_UNIT,
            .chan = BATTERY_ADC_CHANNEL,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        battery_cali_valid_ =
            adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali_handle_) == ESP_OK;
#endif
        ESP_LOGI(TAG, "Battery monitor initialized on GPIO%d (ADC2_CH7)", BATTERY_ADC_GPIO);
    }

    // 读取电池端电压（分压还原，单位 V）。
    // 注意：ESP32-S3 上 ADC2 与 Wi-Fi 共用，联网时采样可能失败，
    // 失败时沿用上次有效读数，避免显示跳变。
    bool ReadBatteryVoltage(float& voltage) {
        if (battery_adc_handle_ == nullptr) {
            return false;
        }
        int64_t now = esp_timer_get_time();
        // 500ms 内复用上次结果，避免高频采样
        if (battery_voltage_valid_ && now - battery_last_read_us_ < 500 * 1000) {
            voltage = battery_voltage_;
            return true;
        }

        int64_t mv_sum = 0;
        int samples = 0;
        for (int i = 0; i < 8; i++) {
            int raw = 0;
            if (adc_oneshot_read(battery_adc_handle_, BATTERY_ADC_CHANNEL, &raw) != ESP_OK) {
                continue;
            }
            int mv = 0;
            if (battery_cali_valid_) {
                if (adc_cali_raw_to_voltage(battery_adc_cali_handle_, raw, &mv) != ESP_OK) {
                    continue;
                }
            } else {
                mv = raw * 3100 / 4095;  // 无校准时按 12dB 量程满量程约 3.1V 估算
            }
            mv_sum += mv;
            samples++;
        }
        battery_last_read_us_ = now;
        if (samples == 0) {
            // 采样全部失败（如 Wi-Fi 占用 ADC2）：沿用上次有效值
            voltage = battery_voltage_;
            return battery_voltage_valid_;
        }
        battery_voltage_ = (mv_sum / samples / 1000.0f) *
                           ((BATTERY_UPPER_RESISTOR_OHM + BATTERY_LOWER_RESISTOR_OHM) /
                            BATTERY_LOWER_RESISTOR_OHM);
        battery_voltage_valid_ = true;
        voltage = battery_voltage_;
        return true;
    }

    // 2S 锂电池组电压-电量换算（单节 OCV-SOC 曲线 x2，分段线性插值）
    static int VoltageToLevel(float voltage) {
        static const struct {
            float voltage;
            int level;
        } kPoints[] = {
            {8.40f, 100}, {8.12f, 90}, {7.96f, 80}, {7.84f, 70}, {7.74f, 60}, {7.64f, 50},
            {7.56f, 40},  {7.48f, 30}, {7.36f, 20}, {6.90f, 5},  {6.00f, 0},
        };
        if (voltage >= kPoints[0].voltage) {
            return kPoints[0].level;
        }
        for (size_t i = 0; i < sizeof(kPoints) / sizeof(kPoints[0]) - 1; i++) {
            if (voltage >= kPoints[i + 1].voltage) {
                float span = kPoints[i].voltage - kPoints[i + 1].voltage;
                float offset = voltage - kPoints[i + 1].voltage;
                return kPoints[i + 1].level +
                       (int)((kPoints[i].level - kPoints[i + 1].level) * offset / span);
            }
        }
        return 0;
    }

public:
    // 构造函数：先初始化两路 I2C，再初始化 OLED、按键与机器狗舵机
    MovecallMojiESP32S3Oled() : boot_button_(BOOT_BUTTON_GPIO) {
        // 必须在建立任何总线之前跑：此时引脚还是自由的，互短检测才有意义
        if (kEnablePinShortCheck) {
            RunPinShortCheck();
        }
        InitializeCodecI2c();
        // 必须在 I2S 启动之前跑：这时 GPIO6 还没被 I2S 占用，可以当普通 GPIO 慢速翻转
        if (kEnablePinShortCheck) {
            RunMclkCeBridgeCheck(codec_i2c_bus_);
            // 驱动强度对照实验：同一频率下 最弱 vs 最强。
            // 若 弱驱动明显好于 强驱动 -> 坐实「边沿速率驱动的容性串扰」，
            //   同时说明 ApplyWeakMclkDrive() 这个软件规避是有效的；
            // 若 两者一样差 -> 与边沿无关，回到供电/地/接触电阻（虚焊）。
            //
            // 2026-10-22 16:0x 修：20 次采样对「部分改善」完全分辨不出来。
            // 实测同一块板连跑 5 次，strong 档 6.144MHz 在 3~8/20 之间乱跳
            // （15%~40%，二项分布标准差约 ±9 个百分点）。要做前后对比，必须加大样本。
            // 现在：形状扫描每频点 50 次；四个驱动档位各 300 次（标准差压到约 ±2.3%）。
            //
            // 为什么要把四档都测一遍，而不是只测最弱/最强：
            //   只看 0 和 3 只能知道「有差别」，不知道「余量有多大」。
            //   四档连成一条剂量-反应曲线，才能回答「当前这个软件规避稳不稳」：
            //     CAP0=99% 且 CAP1 也高  -> 余量充足，可以放心用
            //     CAP0=99% 但 CAP1 已崩 -> 余量为零，任何扰动都会复发，必须改硬件
            RunMclkFreqSweep(codec_i2c_bus_, GPIO_DRIVE_CAP_0, "weak", kSweepFreqsFull,
                             sizeof(kSweepFreqsFull) / sizeof(kSweepFreqsFull[0]), 50);
            RunMclkFreqSweep(codec_i2c_bus_, GPIO_DRIVE_CAP_0, "CAP0", kSweepFreqsTop,
                             sizeof(kSweepFreqsTop) / sizeof(kSweepFreqsTop[0]), 300);
            RunMclkFreqSweep(codec_i2c_bus_, GPIO_DRIVE_CAP_1, "CAP1", kSweepFreqsTop,
                             sizeof(kSweepFreqsTop) / sizeof(kSweepFreqsTop[0]), 300);
            RunMclkFreqSweep(codec_i2c_bus_, GPIO_DRIVE_CAP_2, "CAP2", kSweepFreqsTop,
                             sizeof(kSweepFreqsTop) / sizeof(kSweepFreqsTop[0]), 300);
            RunMclkFreqSweep(codec_i2c_bus_, GPIO_DRIVE_CAP_3, "CAP3", kSweepFreqsTop,
                             sizeof(kSweepFreqsTop) / sizeof(kSweepFreqsTop[0]), 300);
            ESP_LOGW(TAG, "MCLKSWEEP 判读: CAP0 到 CAP3 的曲线就是余量；"
                          "CAP1 已明显掉 = 余量为零，必须改硬件");
        }
        // 必须在 I2S 接管 GPIO6 之前设置驱动能力。
        // 注意：上面扫频结尾的 gpio_reset_pin 会把驱动档位打回默认，所以这里必须再设一次。
        ApplyWeakMclkDrive();
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        // 当前样机没有接电池，先关闭电量检测：不创建 ADC 单元，也就不采样。
        // 恢复时取消下面这行注释，并同时恢复 GetBatteryLevel / GetBatteryVoltage 两个 override。
        // InitializeBatteryMonitor();
        InitializeEDARobotDogController();
    }

    // 提供板载单色 LED
    virtual Led* GetLed() override {
        static SingleLed led_strip(BUILTIN_LED_GPIO);
        return &led_strip;
    }

    // 提供显示对象
    virtual Display* GetDisplay() override { return display_; }

    // 【临时关闭】电量显示与低电量提醒
    // 当前样机没有接电池，所以不覆写这两个接口，直接沿用 Board 的默认实现（返回 false）。
    // 返回 false 后，显示层（lvgl_display.cc:257）整段电池逻辑都不会进入，效果是：
    //   - 顶栏不显示电池图标和 "xx% x.xV" 文本
    //   - 不会弹出低电量提示，也不会播放低电量提示音（OGG_LOW_BATTERY）
    // 接上电池后取消下面注释即可恢复。
    //
    // // 提供电量百分比（2S 电池曲线换算）。硬件无充电检测引脚，充电状态未知
    // virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
    //     float voltage = 0.0f;
    //     if (!ReadBatteryVoltage(voltage)) {
    //         return false;
    //     }
    //     level = VoltageToLevel(voltage);
    //     charging = false;
    //     discharging = true;
    //     return true;
    // }
    //
    // // 提供电池端电压（V），供显示层展示
    // virtual bool GetBatteryVoltage(float& voltage) override {
    //     return ReadBatteryVoltage(voltage);
    // }

    // 提供 ES8311 音频编解码器（I2S 引脚、PA 使能、I2C 地址来自 config.h）
    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            kAudioMclkPin, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR,
            kAudioMclkPin != GPIO_NUM_NC);
        // 总线诊断只跑一次：此时 I2S 已经启动，可与开机时的 probe 前后对照
        static bool codec_diag_done = false;
        if (kEnableCodecBusDiag && !codec_diag_done) {
            codec_diag_done = true;
            RunCodecBusDiagnostics(codec_i2c_bus_);
        }
        // 自检只跑一次：此时 AudioService 还没启动，不会和播放/录音任务抢 I2S
        static bool self_test_done = false;
        if (kEnableAudioSelfTest && !self_test_done) {
            self_test_done = true;
            RunAudioSelfTest(&audio_codec);
        }
        // 功放链路探针（临时调试用，见 kEnableAmpProbe）
        static bool amp_probe_done = false;
        if (kEnableAmpProbe && !amp_probe_done) {
            amp_probe_done = true;
            RunAmpProbe(&audio_codec);
        }
        // 麦克风底噪扫描（临时调试用，见 kEnableMicNoiseSweep）
        static bool noise_sweep_done = false;
        if (kEnableMicNoiseSweep && !noise_sweep_done) {
            noise_sweep_done = true;
            RunMicNoiseSweep(&audio_codec);
        }
        // MCLK 驱动强度 × 出声测试：必须等系统进 idle、AudioService 不再抢占 I2S 之后
        // 才能做，所以开一个延迟任务，10 秒后执行。codec 指针在这里直接捕获，
        // 避免在任务里二次调用 GetAudioCodec()（静态局部变量不是线程安全的）。
        static bool drive_audio_test_started = false;
        if (kEnableMclkDriveAudioTest && !drive_audio_test_started) {
            drive_audio_test_started = true;
            xTaskCreate(
                [](void* arg) {
                    auto* codec = static_cast<AudioCodec*>(arg);
                    vTaskDelay(pdMS_TO_TICKS(10000));
                    RunMclkDriveAudioTest(codec);
                    vTaskDelete(NULL);
                },
                "mclk_audio_test", 4096, &audio_codec, 3, nullptr);
        }
        return &audio_codec;
    }
};

// 向构建系统注册唯一的板卡工厂
DECLARE_BOARD(MovecallMojiESP32S3Oled);