//------------------------------------------------------------------------------
// EDARobotDog：四足机器狗的运动协调层。
//
// 职责：把 4 个 Oscillator（每条腿一个舵机）组织起来，实现成套动作：
//   - 基础运动：MoveServos（线性插值移动）、MoveSingle、Home
//   - 步态动作：Walk（走路）、Turn（转身）、Sit/Stand（坐/站）、
//               Stretch（伸懒腰）、Shake（摇摆）、Sleep（睡觉）
//   - 单腿抬起：LiftLeftFrontLeg 等 4 个
//
// 运动来源：算法移植自 Otto 机器人 / Obijuan 的 Oscillator.pde。
// 步态由“正弦振荡 + 相位差”产生：给四条腿设置不同的 amplitude(振幅)、
// offset(偏置)、phase_diff(相位差)、period(周期) 数组，就能让它们在正确的
// 时机抬起/落下，组合出对角步态（trot）等。
//
// 硬件：4 个舵机经 LEDC（50Hz、500~2500us 对应 -90°~+90°）驱动，
// 腿的索引固定为 左前/左后/右前/右后。
//------------------------------------------------------------------------------
#ifndef __EDA_DOG_MOVEMENTS_H__
#define __EDA_DOG_MOVEMENTS_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oscillator.h"

//-- Constants
//-- 方向/幅度符号常量：让调用处语义更直观（例如 Walk(steps, period, FORWARD)）
#define FORWARD 1    // 前进方向
#define BACKWARD -1  // 后退方向
#define LEFT 1       // 左转方向
#define RIGHT -1     // 右转方向
#define SMALL 5      // 小步幅/幅度
#define MEDIUM 15    // 中等步幅/幅度
#define BIG 30       // 大步幅/幅度

// -- Servo delta limit default. degree / sec
//-- 舵机默认限速斜率：240 度/秒，即每毫秒最多转 0.24°
#define SERVO_LIMIT_DEFAULT 240

// -- Servo indexes for easy access
//-- 四条腿在 servo_[] 数组中的下标：左前、左后、右前、右后
#define LEFT_FRONT_LEG 0
#define LEFT_REAR_LEG 1
#define RIGHT_FRONT_LEG 2
#define RIGHT_REAR_LEG 3
#define SERVO_COUNT 4  // 舵机总数

//-- 四足机器狗类：管理 4 个舵机的协调运动
class EDARobotDog {
public:
    EDARobotDog();
    ~EDARobotDog();

    //-- EDA Dog initialization
    // 初始化：记录四条腿各自连接的 GPIO（未接的传 -1），并挂载 PWM
    void Init(int left_front_leg, int left_rear_leg, int right_front_leg, int right_rear_leg);

    //-- Attach & detach functions
    void AttachServos();  // 挂载所有已配置舵机（绑定 LEDC 通道）
    void DetachServos();  // 卸载所有舵机（停止 PWM）

    //-- Oscillator Trims
    // 设置四路舵机的校准偏移（度），用于修正机械安装误差
    void SetTrims(int left_front_leg, int left_rear_leg, int right_front_leg, int right_rear_leg);

    //-- Predetermined Motion Functions
    // 在 time 毫秒内让所有舵机线性插值平滑到达 servo_target[] 指定的角度
    void MoveServos(int time, int servo_target[]);
    // 让单个舵机直接到达 position（超范围时归中 90）
    void MoveSingle(int position, int servo_number);
    // 核心步态驱动：按四组振荡参数做 cycle 个正弦振荡周期
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                         double phase_diff[SERVO_COUNT], float cycle);

    //-- HOME = Dog at rest position
    void Home();                    // 回到站立静止姿态（所有腿 90°）
    bool GetRestState();            // 查询是否处于静止姿态
    void SetRestState(bool state);  // 设置静止姿态标志

    //-- Basic leg movements
    void LiftLeftFrontLeg(int period = 1000, int height = 45);   // 抬起左前腿
    void LiftLeftRearLeg(int period = 1000, int height = 45);    // 抬起左后腿
    void LiftRightFrontLeg(int period = 1000, int height = 45);  // 抬起右前腿
    void LiftRightRearLeg(int period = 1000, int height = 45);   // 抬起右后腿

    //-- Dog gait movements
    void GetCurrentPositions(int pos[SERVO_COUNT]);                    // 读取当前四腿位置到 pos[]
    void Walk(float steps = 4, int period = 1000, int dir = FORWARD);  // 行走（对角 trot 步态）
    void Turn(float steps = 4, int period = 2000, int dir = LEFT);     // 原地转身
    void Sit(int period = 1500);                                       // 坐下
    void Stand(int period = 1500);                                     // 站立
    void Stretch(int period = 2000);                                   // 伸懒腰
    void Shake(int period = 1000);                                     // 摇摆/抖身
    void Sleep();                                                      // 睡觉动作

    // -- Servo limiter
    // 启用所有舵机限速，参数为速度上限（度/秒），默认 240
    void EnableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void DisableServoLimit();  // 关闭限速（舵机可全速跳变）

private:
    Oscillator servo_[SERVO_COUNT];  // 四条腿对应的四个正弦振荡器

    int servo_pins_[SERVO_COUNT];  // 四条腿的 GPIO，-1 表示未连接
    int servo_trim_[SERVO_COUNT];  // 四条腿的校准偏移（度）

    unsigned long final_time_;      // 本轮 MoveServos 的结束时间（ms）
    unsigned long partial_time_;    // MoveServos 中单步的预计时间点（ms）
    float increment_[SERVO_COUNT];  // MoveServos 中每 10ms 每腿的角度增量

    bool is_dog_resting_;  // 是否处于静止姿态（用于避免重复回中）

    // 内部实现：把 steps 拆成整数个完整周期 + 一个不完整周期执行
    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                 double phase_diff[SERVO_COUNT], float steps);
};

#endif  // __EDA_DOG_MOVEMENTS_H__