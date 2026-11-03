//--------------------------------------------------------------
//-- Oscillator.pde
//-- Generate sinusoidal oscillations in the servos
//--------------------------------------------------------------
//-- (c) Juan Gonzalez-Gomez (Obijuan), Dec 2011
//-- (c) txp666 for esp32, 202503
//-- GPL license
//--------------------------------------------------------------
//------------------------------------------------------------------------------
// Oscillator：单个舵机的“正弦振荡器”抽象。
//
// 核心思想（移植自 Obijuan 的 Oscillator.pde）：
//   让舵机位置按  pos(t) = amplitude * sin(phase + phase0) + offset
//   的规律往复摆动，而不是直接写死角度。
//   多个舵机各自的 amplitude / offset / phase0（相位差）不同，就能叠加出
//   平滑连贯的步态（走路、转身、摇摆等）。其中 phase0（相位差）是最关键的
//   参数：它决定同一时刻各条腿分别处于正弦波形的哪个位置，从而决定
//   “哪条腿抬起、哪条腿落地”。
//
// 一个 Oscillator 对象管理一个舵机（一路 LEDC/PWM 通道）。
// 典型用法：SetA/SetO/SetT/SetPh 配置参数 -> 循环调用 Refresh()，
// 每个采样周期自动算出新位置并写入 PWM。
//------------------------------------------------------------------------------
#ifndef __OSCILLATOR_H__
#define __OSCILLATOR_H__

#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define M_PI 3.14159265358979323846  // 圆周率，用于角度<->弧度换算

#ifndef DEG2RAD
#define DEG2RAD(g) ((g) * M_PI) / 180  // 角度转弧度：相位差用弧度表示，调用处传入度数更方便
#endif

//-- 舵机脉宽/角度标定：标准 50Hz 舵机，脉宽 500us 对应 -90°，2500us 对应 +90°
#define SERVO_MIN_PULSEWIDTH_US 500           // 最小脉宽（微秒），对应 -90°
#define SERVO_MAX_PULSEWIDTH_US 2500          // 最大脉宽（微秒），对应 +90°
#define SERVO_MIN_DEGREE -90                  // 最小角度（度）
#define SERVO_MAX_DEGREE 90                   // 最大角度（度）
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick —— 定时器 1 个 tick = 1us
#define SERVO_TIMEBASE_PERIOD 20000           // 20000 ticks, 20ms —— 50Hz 的周期

//-- Oscillator 类：管理单个舵机的正弦往复运动（详见文件顶部说明）
class Oscillator {
public:
    Oscillator(int trim = 0);  // trim：机械安装误差的初始校准偏移（度），默认 0
    ~Oscillator();             // 析构时自动 Detach，释放 PWM 通道
    void Attach(int pin,
                bool rev = false);  // 绑定 GPIO 并配置 LEDC；rev=true 表示该舵机安装方向相反
    void Detach();                  // 停止并解绑 PWM 输出

    void SetA(unsigned int amplitude) { amplitude_ = amplitude; };  // 设置振幅（度），即摆动幅度
    void SetO(int offset) { offset_ = offset; };                    // 设置偏置（度），即摆动中心
    void SetPh(double Ph) { phase0_ = Ph; };   // 设置相位差（弧度），步态关键参数
    void SetT(unsigned int period);            // 设置振荡周期（毫秒）
    void SetTrim(int trim) { trim_ = trim; };  // 设置校准偏移（度）
    void SetLimiter(int diff_limit) { diff_limit_ = diff_limit; };  // 启用限速，单位：度/秒
    void DisableLimiter() { diff_limit_ = 0; };                     // 关闭限速（0 表示不限制）
    int GetTrim() { return trim_; };                                // 读取当前校准偏移
    void SetPosition(int position);     // 直接写入一个位置（0~180），会经过限速
    void Stop() { stop_ = true; };      // 暂停摆动（仍会推进相位，但不写 PWM）
    void Play() { stop_ = false; };     // 恢复摆动
    void Reset() { phase_ = 0; };       // 相位归零，重启振荡
    void Refresh();                     // 推进一次采样：计算新位置并写入（步态循环里反复调用）
    int GetPosition() { return pos_; }  // 读取当前舵机位置（0~180）

private:
    bool NextSample();                   // 判断是否到达下一个采样时刻（基于 millis 的 30ms 节拍）
    void Write(int position);            // 把位置写进 PWM（内部完成限速、trim、占空比换算）
    uint32_t AngleToCompare(int angle);  // 角度(-90~+90) -> LEDC 比较值/脉宽(us)

private:
    bool is_attached_;  // 是否已绑定 PWM 输出

    //-- Oscillators parameters
    unsigned int amplitude_;  //-- Amplitude (degrees) —— 振幅（度）
    int offset_;              //-- Offset (degrees) —— 偏置（度），摆动中心
    unsigned int period_;     //-- Period (miliseconds) —— 周期（毫秒）
    double phase0_;           //-- Phase (radians) —— 初始相位差（弧度），决定步态

    //-- Internal variables
    int pos_;                       //-- Current servo pos —— 当前舵机位置（0~180）
    int pin_;                       //-- Pin where the servo is connected —— 舵机所在 GPIO
    int trim_;                      //-- Calibration offset —— 校准偏移（度）
    double phase_;                  //-- Current phase —— 当前相位（弧度），每采样周期递增
    double inc_;                    //-- Increment of phase —— 每采样周期的相位增量
    double number_samples_;         //-- Number of samples —— 一个周期内的采样点数
    unsigned int sampling_period_;  //-- sampling period (ms) —— 采样周期（毫秒）

    long previous_millis_;  // 上一次采样的时间戳（ms）
    long current_millis_;   // 本次采样的时间戳（ms）

    //-- Oscillation mode. If true, the servo is stopped
    bool stop_;  // 为 true 时暂停摆动（只推进相位，不输出）

    //-- Reverse mode
    bool rev_;  // 为 true 时反转舵机方向（安装方向相反时使用）

    int diff_limit_;                      // 限速斜率，单位：度/秒；0 表示不限速
    long previous_servo_command_millis_;  // 上一次写 PWM 的时间戳，用于按时间差计算限速

    ledc_channel_t ledc_channel_;  // 本舵机占用的 LEDC 通道
    ledc_mode_t ledc_speed_mode_;  // LEDC 速度模式（低速模式）
};

#endif  // __OSCILLATOR_H__