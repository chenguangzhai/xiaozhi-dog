//------------------------------------------------------------------------------
// EDARobotDog 实现（四足机器狗运动协调）。
//
// 两条实现路径：
//   1) 线性插值路径：MoveServos 把每个舵机每 10ms 递增 increment_，在给定时间内
//      平滑到达目标角度，用于定点姿态动作（Home/Sit/Stretch/Sleep 等）。
//   2) 正弦步态路径：OscillateServos/Execute 反复调用各 Oscillator 的 Refresh，
//      由 amplitude/offset/phase_diff/period 组合出连续的行走等步态。
//
// 关于相位差 phase_diff（步态的灵魂）：
//   每条腿的位置 = amplitude*sin(phase + phase_diff) + offset。
//   phase_diff 决定同一时刻各腿处于正弦波的哪个阶段：
//     phase_diff=0    与参考腿同相（同时抬起、同时落下）
//     phase_diff=180° 与参考腿反相（一抬一落）
//   四足行走常用“对角步态(trot)”：对角的两条腿同相，另一对对角腿反相，
//   于是同一时刻总有一对对角腿着地支撑、另一对抬起前摆，形成稳定前进。
//------------------------------------------------------------------------------
#include "eda_dog_movements.h"

#include <algorithm>
#include <cmath>

#include "oscillator.h"

static const char* TAG = "EDARobotDogMovements";

#define LEG_HOME_POSITION 90  // 舵机中位角度，即站立静止位置

// 构造函数：把四条腿标记为未连接(-1)、校准偏移清零。
EDARobotDog::EDARobotDog() {
    is_dog_resting_ = false;
    // 初始化所有舵机管脚为-1（未连接）
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

EDARobotDog::~EDARobotDog() { DetachServos(); }

// 系统运行毫秒数（单调递增，来自高精度 esp_timer）。
// IRAM_ATTR 使其常驻指令 RAM，减少 flash 访问抖动。
unsigned long IRAM_ATTR millis() { return (unsigned long)(esp_timer_get_time() / 1000ULL); }

// 初始化：记录四条腿的 GPIO，并立即挂载 PWM。
// 参数依次为：左前、左后、右前、右后腿的 GPIO；未接的舵机可传 -1。
void EDARobotDog::Init(int left_front_leg, int left_rear_leg, int right_front_leg,
                       int right_rear_leg) {
    servo_pins_[LEFT_FRONT_LEG] = left_front_leg;
    servo_pins_[LEFT_REAR_LEG] = left_rear_leg;
    servo_pins_[RIGHT_FRONT_LEG] = right_front_leg;
    servo_pins_[RIGHT_REAR_LEG] = right_rear_leg;

    AttachServos();
    is_dog_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
// 挂载所有“已配置(GPIO != -1)”的舵机，为其申请 LEDC 通道并开始输出 PWM。
void EDARobotDog::AttachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Attach(servo_pins_[i]);
        }
    }
}

// 卸载所有已配置舵机：停止各自的 PWM 输出。
void EDARobotDog::DetachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Detach();
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
// 设置四条腿的校准偏移（度）：补偿舵机机械安装角度误差，使站立时各腿真正水平。
// 参数依次为：左前、左后、右前、右后腿的 trim。
void EDARobotDog::SetTrims(int left_front_leg, int left_rear_leg, int right_front_leg,
                           int right_rear_leg) {
    servo_trim_[LEFT_FRONT_LEG] = left_front_leg;
    servo_trim_[LEFT_REAR_LEG] = left_rear_leg;
    servo_trim_[RIGHT_FRONT_LEG] = right_front_leg;
    servo_trim_[RIGHT_REAR_LEG] = right_rear_leg;

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetTrim(servo_trim_[i]);
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
// 在 time 毫秒内让所有舵机平滑移动到 servo_target[] 目标角度（线性插值）。
// 原理：把总位移按 (time/10) 等分为若干步，每 10ms 给每个舵机增加固定增量
//       increment_[i] = (目标 - 当前) / (time/10)，形成匀速直线运动。
// 特殊处理：
//   - time <= 10ms 时不做插值，直接赋值并延时到位。
//   - 循环结束后再做一次“最终微调”（最多 10 轮），把因浮点/整除误差残留的
//     偏差精确补齐，确保最终停在目标角度。
void EDARobotDog::MoveServos(int time, int servo_target[]) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if (time > 10) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
            }
        }

        for (int iteration = 1; millis() < final_time_; iteration++) {
            partial_time_ = millis() + 10;
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // final adjustment to the target.
    // 最终微调：反复把未到位的舵机直接设到目标值，直到全部到位或超过 10 轮上限。
    bool f = true;
    int adjustment_count = 0;
    while (f && adjustment_count < 10) {
        f = false;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition()) {
                f = true;
                break;
            }
        }
        if (f) {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_target[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            adjustment_count++;
        }
    };
}

// 让单个舵机移动到 position。
// position 超出 0~180 时归中到 90（保护舵机）；仅当舵机已连接且编号合法才执行。
void EDARobotDog::MoveSingle(int position, int servo_number) {
    if (position > 180)
        position = 90;
    if (position < 0)
        position = 90;

    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1) {
        servo_[servo_number].SetPosition(position);
    }
}

// 核心步态驱动：设置 4 个振荡器的参数，然后做 cycle 个正弦周期。
// amplitude[]：各腿摆动幅度（度）
// offset[]   ：各腿摆动中心（度）
// period     ：一个完整正弦周期的时长（ms）
// phase_diff[]：各腿相位差（弧度），决定抬腿/落地的先后，是成步态的关键
// cycle      ：要执行的周期数（可为小数，执行到某个相位就返回）
void EDARobotDog::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                                  double phase_diff[SERVO_COUNT], float cycle = 1) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    // 以毫秒为时间基准，循环调用 Refresh，直到走完 cycle 个周期。
    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);  // 5ms 轮询一次；实际写 PWM 由各自 30ms 采样节拍控制
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

// 执行 steps 个振荡：拆成整数个完整周期 + 一个不完整周期（小数部分）。
// 例如 steps=3.5 -> 3 个完整周期 + 0.5 个周期。
// 这样调用方可以精确控制停止在半周期位置，衔接下一个动作更自然。
void EDARobotDog::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                          double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    //-- Execute the final not complete cycle
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Dog at rest position --------------------------------//
///////////////////////////////////////////////////////////////////
// 让机器狗回到静止站立姿态（四条腿都回到 90° 中位）。
// 仅当当前不处于静止态时才动作，避免重复回中；完成后置静止标志并短暂延时稳定。
void EDARobotDog::Home() {
    if (is_dog_resting_ == false) {  // Go to rest position only if necessary
        int homes[SERVO_COUNT] = {LEG_HOME_POSITION, LEG_HOME_POSITION, LEG_HOME_POSITION,
                                  LEG_HOME_POSITION};
        MoveServos(500, homes);
        is_dog_resting_ = true;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

bool EDARobotDog::GetRestState() { return is_dog_resting_; }

void EDARobotDog::SetRestState(bool state) { is_dog_resting_ = state; }

///////////////////////////////////////////////////////////////////
//-- BASIC LEG MOVEMENTS ----------------------------------------//
///////////////////////////////////////////////////////////////////
// 抬起左前腿：先读取当前各腿位置，只动左前腿，做 3 次“摆到一端再回”的往复，
// 最后回到 90°。period/height 参数在当前实现中未使用（保留接口）。
void EDARobotDog::LiftLeftFrontLeg(int period, int height) {
    // 获取当前位置
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    // 重复3次摇摆动作
    for (int num = 0; num < 3; num++) {
        // servo1.write(180); delay(100);
        current_pos[LEFT_FRONT_LEG] = 0;  // servo1 —— 左前腿摆到 0°（该腿的抬起方向）
        MoveServos(100, current_pos);

        // servo1.write(150); delay(100);
        current_pos[LEFT_FRONT_LEG] = 30;  // servo1
        MoveServos(100, current_pos);
    }

    // servo1.write(90);
    current_pos[LEFT_FRONT_LEG] = 90;  // servo1  // 回中
    MoveServos(100, current_pos);
}

// 抬起左后腿：逻辑同上，左后腿“抬起方向”为增大到 180°。
void EDARobotDog::LiftLeftRearLeg(int period, int height) {
    // 获取当前位置
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    // 重复3次摇摆动作
    for (int num = 0; num < 3; num++) {
        // servo1.write(180); delay(100);
        current_pos[LEFT_REAR_LEG] = 180;  // servo1
        MoveServos(100, current_pos);

        // servo1.write(150); delay(100);
        current_pos[LEFT_REAR_LEG] = 150;  // servo1
        MoveServos(100, current_pos);
    }

    // servo1.write(90);
    current_pos[LEFT_REAR_LEG] = 90;  // servo1
    MoveServos(100, current_pos);
}

// 抬起右前腿：右前腿“抬起方向”为增大到 180°。
void EDARobotDog::LiftRightFrontLeg(int period, int height) {
    // 获取当前位置
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    // 重复3次摇摆动作
    for (int num = 0; num < 3; num++) {
        // servo1.write(180); delay(100);
        current_pos[RIGHT_FRONT_LEG] = 180;  // servo1
        MoveServos(100, current_pos);

        // servo1.write(150); delay(100);
        current_pos[RIGHT_FRONT_LEG] = 150;  // servo1
        MoveServos(100, current_pos);
    }

    // servo1.write(90);
    current_pos[RIGHT_FRONT_LEG] = 90;  // servo1
    MoveServos(100, current_pos);
}

// 抬起右后腿：右后腿“抬起方向”为减小到 0°。
// 注意：函数末尾回中的是 RIGHT_FRONT_LEG（保持与原实现一致）。
void EDARobotDog::LiftRightRearLeg(int period, int height) {
    // 获取当前位置
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    // 重复3次摇摆动作
    for (int num = 0; num < 3; num++) {
        // servo1.write(180); delay(100);
        current_pos[RIGHT_REAR_LEG] = 0;  // servo1
        MoveServos(100, current_pos);

        // servo1.write(150); delay(100);
        current_pos[RIGHT_REAR_LEG] = 30;  // servo1
        MoveServos(100, current_pos);
    }

    // servo1.write(90);
    current_pos[RIGHT_FRONT_LEG] = 90;  // servo1
    MoveServos(100, current_pos);
}

///////////////////////////////////////////////////////////////////
//-- DOG GAIT MOVEMENTS -----------------------------------------//
///////////////////////////////////////////////////////////////////

// 舵机方向分析（从 Stretch/Sleep 推断物理前后方向）：
//
//   Stretch: LF=0, LR=180, RF=180, RR=0
//   → 前腿向前伸，后腿向后伸
//
//   LEFT_FRONT_LEG  (0): 减小=向前, 增大=向后
//   LEFT_REAR_LEG   (1): 减小=向前, 增大=向后
//   RIGHT_FRONT_LEG (2): 增大=向前, 减小=向后
//   RIGHT_REAR_LEG  (3): 增大=向前, 减小=向后
//
//   即：左侧两腿同向（减=前），右侧两腿同向（增=前）
//   对角线A: 左前 + 右后 → 前进时左前减小, 右后减小（都是各自的"向前"）
//   对角线B: 左后 + 右前 → 前进时左后减小, 右前增大（都是各自的"向前"）
//
//   抬腿方向（从 Lift 函数）：
//   LEFT_FRONT_LEG:  0°   (减小=抬)
//   LEFT_REAR_LEG:   180° (增大=抬)
//   RIGHT_FRONT_LEG: 180° (增大=抬)
//   RIGHT_REAR_LEG:  0°   (减小=抬)

// 辅助：获取当前所有舵机位置
// 输出到 pos[4]（依次为左前/左后/右前/右后）；未连接的腿按中位 90 填充。
void EDARobotDog::GetCurrentPositions(int pos[SERVO_COUNT]) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            pos[i] = servo_[i].GetPosition();
        } else {
            pos[i] = LEG_HOME_POSITION;
        }
    }
}

// 原地转身：dir=LEFT 左转、dir=RIGHT 右转；steps 为步数，period 为单步节拍基准。
// 实现为 6 拍的“对角步态”循环——每步都由一对对角腿抬起摆动、另一对在对角
// 腿着地时反向蹬地，从而让身体绕竖轴旋转（左转=左侧后退+右侧前进）。
// 每拍用 MoveServos 做定点移动（不使用正弦振荡），因此动作是分段线性的。
void EDARobotDog::Turn(float steps, int period, int dir) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    // 转弯 = 左侧后退 + 右侧前进（左转），或反过来（右转）
    // 从 Walk 的已验证方向推导：
    //   前进: LF=90-s, LR=90-s, RF=90+s, RR=90+s
    //   后退: LF=90+s, LR=90+s, RF=90-s, RR=90-s
    //
    // 左转(d=1): 左侧后退+右侧前进
    //   LF=90+s, LR=90+s, RF=90+s, RR=90+s → 全部 90+s
    //   地面反推: 全部 90-s
    //
    // 右转(d=-1): 左侧前进+右侧后退
    //   LF=90-s, LR=90-s, RF=90-s, RR=90-s → 全部 90-s
    //   地面反推: 全部 90+s

    const int swing = 60;  // 空中/地面的摆动/蹬地幅度（度）
    const int lift = 25;   // 抬腿的抬升幅度（度）
    int t = period / 6;    // 每一步分 6 拍，故单拍时长 = period/6
    if (t < 50)
        t = 50;  // 单拍下限 50ms，避免过快导致舵机跟不上、动作发抖

    int d = (dir == LEFT) ? -1 : 1;  // 转动方向因子：左转为 -1，右转为 +1

    for (int step = 0; step < (int)steps; step++) {
        int pos[SERVO_COUNT];
        GetCurrentPositions(pos);

        // 拍1: 抬A（LF+RR）
        pos[LEFT_FRONT_LEG] = 90 - lift;
        pos[RIGHT_REAR_LEG] = 90 - lift;
        MoveServos(t, pos);

        // 拍2: A空中摆 + B地面反推
        pos[LEFT_FRONT_LEG] = 90 - lift + d * swing;  // LF空中
        pos[RIGHT_REAR_LEG] = 90 - lift + d * swing;  // RR空中
        pos[LEFT_REAR_LEG] = 90 - d * swing;          // LR地面反推
        pos[RIGHT_FRONT_LEG] = 90 - d * swing;        // RF地面反推
        MoveServos(t, pos);

        // 拍3: 放下A
        pos[LEFT_FRONT_LEG] = 90 + d * swing;
        pos[RIGHT_REAR_LEG] = 90 + d * swing;
        MoveServos(t / 2, pos);

        // 拍4: 抬B（LR+RF）
        pos[LEFT_REAR_LEG] = 90 + lift;
        pos[RIGHT_FRONT_LEG] = 90 + lift;
        MoveServos(t, pos);

        // 拍5: B空中摆 + A地面反推
        pos[LEFT_REAR_LEG] = 90 + lift + d * swing;    // LR空中
        pos[RIGHT_FRONT_LEG] = 90 + lift + d * swing;  // RF空中
        pos[LEFT_FRONT_LEG] = 90 - d * swing;          // LF地面反推
        pos[RIGHT_REAR_LEG] = 90 - d * swing;          // RR地面反推
        MoveServos(t, pos);

        // 拍6: 放下B
        pos[LEFT_REAR_LEG] = 90 + d * swing;
        pos[RIGHT_FRONT_LEG] = 90 + d * swing;
        MoveServos(t / 2, pos);
    }

    // 结束回中
    int home[SERVO_COUNT] = {90, 90, 90, 90};
    MoveServos(150, home);
}

// 行走：dir=FORWARD 前进、dir=BACKWARD 后退；steps 为步数，period 为单步节拍基准。
// 采用“对角步态(Trot)”：对角线的两条腿（LF+RR 与 LR+RF）交替抬起、摆动并落地，
// 抬起的一对在空中前摆，同时落地的一对贴地后推，从而推动身体前进。
// 同样用 6 拍 + MoveServos 定点移动实现；结束时回到中位 90。
void EDARobotDog::Walk(float steps, int period, int dir) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    // Trot 对角步态
    // 方向: LF减=前, LR减=前, RF增=前, RR增=前
    // 抬腿: LF减=抬, LR增=抬, RF增=抬, RR减=抬

    const int lift = 25;   // 抬腿幅度（度）
    const int swing = 30;  // 前后摆腿/蹬地幅度（度）
    int t = period / 6;    // 单拍时长 = period/6
    if (t < 50)
        t = 50;  // 单拍下限 50ms，保证舵机跟得上

    int fwd = (dir == FORWARD) ? 1 : -1;  // 前进因子：前进 +1，后退 -1

    for (int step = 0; step < (int)steps; step++) {
        int pos[SERVO_COUNT];
        GetCurrentPositions(pos);

        // 拍1: 抬起对角线A（LF+RR）
        pos[LEFT_FRONT_LEG] = 90 - lift;
        pos[RIGHT_REAR_LEG] = 90 - lift;
        MoveServos(t, pos);

        // 拍2: A空中前摆 + B地面后推（同时）
        pos[LEFT_FRONT_LEG] = 90 - lift - fwd * swing;  // LF: 保持抬 + 前摆
        pos[RIGHT_REAR_LEG] = 90 - lift + fwd * swing;  // RR: 保持抬 + 前摆(增=前)
        pos[LEFT_REAR_LEG] = 90 + fwd * swing;          // LR: 地面后推(增=后)
        pos[RIGHT_FRONT_LEG] = 90 - fwd * swing;        // RF: 地面后推(减=后)
        MoveServos(t, pos);

        // 拍3: 放下A（着地到前摆位置）
        pos[LEFT_FRONT_LEG] = 90 - fwd * swing;
        pos[RIGHT_REAR_LEG] = 90 + fwd * swing;
        MoveServos(t / 2, pos);

        // 拍4: 抬起对角线B（LR+RF）
        pos[LEFT_REAR_LEG] = 90 + lift;
        pos[RIGHT_FRONT_LEG] = 90 + lift;
        MoveServos(t, pos);

        // 拍5: B空中前摆 + A地面后推（同时）
        pos[LEFT_REAR_LEG] = 90 + lift - fwd * swing;    // LR: 保持抬 + 前摆(减=前)
        pos[RIGHT_FRONT_LEG] = 90 + lift + fwd * swing;  // RF: 保持抬 + 前摆(增=前)
        pos[LEFT_FRONT_LEG] = 90 + fwd * swing;          // LF: 地面后推(增=后)
        pos[RIGHT_REAR_LEG] = 90 - fwd * swing;          // RR: 地面后推(减=后)
        MoveServos(t, pos);

        // 拍6: 放下B（不回中，直接衔接下一步）
        pos[LEFT_REAR_LEG] = 90 - fwd * swing;
        pos[RIGHT_FRONT_LEG] = 90 + fwd * swing;
        MoveServos(t / 2, pos);
    }

    // 结束回中
    int home[SERVO_COUNT] = {90, 90, 90, 90};
    MoveServos(150, home);
}

// 坐下：保持前腿不动，把两条后腿折到极端位置（左后 0°、右后 180°），
// 使身体后部下沉呈坐姿。period 参数在当前实现中未使用。
void EDARobotDog::Sit(int period) {
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    current_pos[LEFT_REAR_LEG] = 0;     // servo2 左后腿折起
    current_pos[RIGHT_REAR_LEG] = 180;  // servo4 右后腿折起（两侧方向相反）
    MoveServos(100, current_pos);
}

// 站立：所有腿回到中立位置 90°（直接复用 Home，走回中流程）。
void EDARobotDog::Stand(int period) {
    // 站立：所有腿回到中立位置
    Home();
}

// 伸懒腰：前腿朝前、后腿朝后同时伸展（左前/右后到 0°，左后/右前到 180°），
// 形成“趴下前伸”的拉伸姿态。period 参数在当前实现中未使用。
void EDARobotDog::Stretch(int period) {
    // 获取当前位置
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    current_pos[LEFT_FRONT_LEG] = 0;     // servo1 左前腿前伸
    current_pos[RIGHT_REAR_LEG] = 0;     // servo3 右后腿后伸
    current_pos[LEFT_REAR_LEG] = 180;    // servo2 左后腿后伸
    current_pos[RIGHT_FRONT_LEG] = 180;  // servo4 右前腿前伸
    MoveServos(100, current_pos);
}

// 摇摆（抖身）：这是唯一使用正弦振荡的固定动作，用 Execute 跑 3 个周期。
// 参数数组含义：
//   A(amplitude) = {20,0,20,0}   仅两条前腿摆幅 20°，后腿 0（不动）
//   O(offset)    = {0,90,0,90}   两条前腿以 0° 为中心摆动，后腿停在 90° 中位
//   phase_diff   = {180°,0,0°,0} 左前腿与右前腿相位差 180°（完全反相），
//                                于是一条前腿抬起时另一条下压，形成左右摇摆/抖身效果；
//                                后腿相位置 0 不参与摆动。
void EDARobotDog::Shake(int period) {
    // 摇摆：左右摇摆身体，左前腿和右后腿运动方向相反
    int A[SERVO_COUNT] = {20, 0, 20, 0};  // 只有前腿摇摆
    int O[SERVO_COUNT] = {0, LEG_HOME_POSITION, 0, LEG_HOME_POSITION};
    // 左前腿和右前腿反相摇摆
    double phase_diff[SERVO_COUNT] = {DEG2RAD(180), 0, DEG2RAD(0), 0};

    Execute(A, O, period, phase_diff, 3);
}

// 启用所有已连接舵机的限速，speed_limit 单位“度/秒”（默认 240）。
// 限速可抑制舵机瞬时大角度跳变，动作更平顺、也更保护齿轮。
void EDARobotDog::EnableServoLimit(int diff_limit) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetLimiter(diff_limit);
        }
    }
}

// 关闭所有已连接舵机的限速（恢复全速响应）。
void EDARobotDog::DisableServoLimit() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].DisableLimiter();
        }
    }
}

// 睡觉：四条腿全部摊平贴地（左前 0°、右后 180°、左后 180°、右前 0°），
// 与 Stretch 姿态相同，呈现“瘫软趴下”的睡姿。period 参数未使用。
void EDARobotDog::Sleep() {
    int current_pos[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_pos[i] = servo_[i].GetPosition();
        } else {
            current_pos[i] = LEG_HOME_POSITION;
        }
    }

    // servo1.write(0); servo3.write(180); servo2.write(180); servo4.write(0);
    current_pos[LEFT_FRONT_LEG] = 0;    // servo1
    current_pos[RIGHT_REAR_LEG] = 180;  // servo3
    current_pos[LEFT_REAR_LEG] = 180;   // servo2
    current_pos[RIGHT_FRONT_LEG] = 0;   // servo4
    MoveServos(100, current_pos);
}