//--------------------------------------------------------------
//-- Oscillator.pde
//-- Generate sinusoidal oscillations in the servos
//--------------------------------------------------------------
//-- (c) Juan Gonzalez-Gomez (Obijuan), Dec 2011
//-- (c) txp666 for esp32, 202503
//-- GPL license
//--------------------------------------------------------------
//------------------------------------------------------------------------------
// Oscillator 实现：把“正弦振荡”变成实际写入舵机的 PWM 信号。
//
// 数据流：
//   Refresh()  ->  NextSample() 判断是否到点
//              ->  pos = amplitude*sin(phase+phase0)+offset (+90 归一到 0~180)
//              ->  Write(pos) 限速 -> 加 trim 钳位 -> 换算 LEDC 占空比 -> 输出
//              ->  phase += inc_（推进相位，形成连续振荡）
//------------------------------------------------------------------------------
#include "oscillator.h"

#include <driver/ledc.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

static const char* TAG = "Oscillator";

extern unsigned long IRAM_ATTR millis();  // 毫秒级时间源，由 eda_dog_movements.cc 提供

// 全局通道分配计数器：每 Attach 一个舵机就自增，依次占用 LEDC_CHANNEL_1..
// 这样多个舵机（4 条腿）能自动拿到互不相同的通道，无需调用方手动指定。
static ledc_channel_t next_free_channel = LEDC_CHANNEL_0;

Oscillator::Oscillator(int trim) {
    trim_ = trim;
    diff_limit_ = 0;  // 默认不限速
    is_attached_ = false;

    sampling_period_ = 30;                         // 采样周期 30ms
    period_ = 2000;                                // 默认振荡周期 2000ms
    number_samples_ = period_ / sampling_period_;  // 一个周期内的采样点数 ≈ 66
    inc_ = 2 * M_PI / number_samples_;  // 每采样一次相位前进 2π/N，绕一圈正好一个完整正弦周期

    amplitude_ = 45;  // 默认振幅 45 度
    phase_ = 0;
    phase0_ = 0;
    offset_ = 0;
    stop_ = false;
    rev_ = false;

    pos_ = 90;  // 舵机默认落在中位 90°
    previous_millis_ = 0;
}

Oscillator::~Oscillator() { Detach(); }

// 把“角度”线性映射为“脉宽(us)”，返回的其实就是 us 值：
//   angle 从 -90 → +90 时，脉宽从 500us → 2500us
//   pulse = (angle - (-90)) * (2500 - 500) / (90 - (-90)) + 500
//         = (angle + 90) * 2000 / 180 + 500
uint32_t Oscillator::AngleToCompare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
               (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
           SERVO_MIN_PULSEWIDTH_US;
}

// 采样节拍判断：距离上次采样超过 sampling_period_（30ms）才返回 true。
// 用“时间差”而非固定次数计时，可让振荡速度与实际时间一致。
bool Oscillator::NextSample() {
    current_millis_ = millis();

    if (current_millis_ - previous_millis_ > sampling_period_) {
        previous_millis_ = current_millis_;
        return true;
    }

    return false;
}

// 绑定舵机：配置 50Hz/13bit 的 LEDC 定时器，并为该舵机分配一路通道。
// pin：GPIO；rev：安装方向反转标志。
void Oscillator::Attach(int pin, bool rev) {
    if (is_attached_) {
        Detach();  // 重复绑定时先解绑，避免通道/引脚冲突
    }

    pin_ = pin;
    rev_ = rev;

    // 50Hz、13bit 分辨率的定时器（13bit：占空比满量程 8191）。
    // 所有舵机共用 LEDC_TIMER_1，因此频率/分辨率一致，只是通道不同。
    ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                      .duty_resolution = LEDC_TIMER_13_BIT,
                                      .timer_num = LEDC_TIMER_1,
                                      .freq_hz = 50,
                                      .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 通道轮转分配：last_channel 初值 0，(0+1)%7+1 = 2，之后依次 3,4,5,6,7,1,2...
    // 即跳过 CHANNEL_0，在 CHANNEL_1..7 之间循环取用，保证 4 个舵机通道不重复。
    static int last_channel = 0;
    last_channel = (last_channel + 1) % 7 + 1;
    ledc_channel_ = (ledc_channel_t)last_channel;

    // 把该 GPIO 绑定到指定通道，初始占空比 0（舵机无信号）。
    ledc_channel_config_t ledc_channel = {.gpio_num = pin_,
                                          .speed_mode = LEDC_LOW_SPEED_MODE,
                                          .channel = ledc_channel_,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .timer_sel = LEDC_TIMER_1,
                                          .duty = 0,
                                          .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_speed_mode_ = LEDC_LOW_SPEED_MODE;

    // pos_ = 90;
    // Write(pos_);
    previous_servo_command_millis_ = millis();  // 记录基准时间，供限速计算使用

    is_attached_ = true;
}

// 解绑：停止该通道 PWM 输出并标记未绑定。
void Oscillator::Detach() {
    if (!is_attached_)
        return;

    ESP_ERROR_CHECK(ledc_stop(ledc_speed_mode_, ledc_channel_, 0));

    is_attached_ = false;
}

// 设置振荡周期 T（毫秒），并据此重算采样点数与每步相位增量。
// 周期越大 -> 每周期采样点越多 -> inc_ 越小 -> 摆动越慢越平滑。
void Oscillator::SetT(unsigned int T) {
    period_ = T;

    number_samples_ = period_ / sampling_period_;
    inc_ = 2 * M_PI / number_samples_;
}

// 直接设置舵机位置（会走 Write 的限速与钳位流程）。
void Oscillator::SetPosition(int position) { Write(position); }

// 推进一个采样周期：
//   非停止状态下，按 pos = amplitude*sin(phase+phase0)+offset 求值，
//   反转则取反，再 +90 偏移到 0~180 区间后写入。
//   无论停止与否都会推进相位 phase_，保证节奏连续。
void Oscillator::Refresh() {
    if (NextSample()) {
        if (!stop_) {
            int pos = std::round(amplitude_ * std::sin(phase_ + phase0_) + offset_);
            if (rev_)
                pos = -pos;
            Write(pos + 90);  // 正弦值域为 ±amplitude，+90 后映射到舵机 0~180 区间
        }

        phase_ = phase_ + inc_;
    }
}

// 写入舵机位置。处理顺序：限速 -> 校准偏移 -> 角度钳位 -> 占空比换算 -> 输出 PWM。
// position：目标位置（0~180 度，90 为中位）。
void Oscillator::Write(int position) {
    if (!is_attached_)
        return;

    // 限速（斜坡限制）：diff_limit_ 单位为“度/秒”。
    // 允许的最大变化量 = 距上次命令的时间(秒) * diff_limit_，
    // 若本次目标跨度超过它，则只朝目标方向移动 limit 度，避免舵机猛跳。
    long currentMillis = millis();
    if (diff_limit_ > 0) {
        int limit = std::max(
            1, (((int)(currentMillis - previous_servo_command_millis_)) * diff_limit_) / 1000);
        if (abs(position - pos_) > limit) {
            pos_ += position < pos_ ? -limit : limit;
        } else {
            pos_ = position;
        }
    } else {
        pos_ = position;  // 未启用限速，直接到位
    }
    previous_servo_command_millis_ = currentMillis;

    int angle = pos_ + trim_;  // 叠加机械安装误差的校准偏移

    angle = std::min(std::max(angle, 0), 180);  // 钳位到合法范围，防止越界导致舵机堵转

    // 占空比换算（13bit，满量程 8191）：
    //   50Hz 周期 = 20ms；13bit 满量程对应 20ms，故占空比 = (脉宽ms / 20ms) * 8191。
    //   舵机脉宽与角度关系：脉宽(ms) = 0.5 + (angle/180)*2，即 0°→0.5ms、180°→2.5ms。
    //   合并得 duty = ((angle/180)*2 + 0.5) * 8191 / 20。
    uint32_t duty = (uint32_t)(((angle / 180.0) * 2.0 + 0.5) * 8191 / 20.0);

    ESP_ERROR_CHECK(ledc_set_duty(ledc_speed_mode_, ledc_channel_, duty));
    ESP_ERROR_CHECK(ledc_update_duty(ledc_speed_mode_, ledc_channel_));
}