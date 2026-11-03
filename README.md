# xiaozhi-dog 小智机器狗

一台放在桌面的四足 AI 陪伴机器人：**用语音对话，用表情和动作回应**。

目标不是做一个会说话的盒子，而是把「会聊天的大模型」变成看得见、会动的实体——唤醒、对话、动作全链路跑通，硬件与固件全部开源，单台成本可控。

本项目固件基于 [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 二次开发，新增 `movecall/moji-esp32s3-oled` 板级支持与四足运动控制层。

> **当前版本：1.0（V1 样机）** — 已完成三块板设计，跑通语音对话与四足行走。

---

## 目录

- [它能做什么](#它能做什么)
- [硬件方案](#硬件方案)
- [引脚定义](#引脚定义)
- [软件架构](#软件架构)
- [AI 可调用的动作（MCP 工具）](#ai-可调用的动作mcp-工具)
- [一次典型交互](#一次典型交互)
- [编译与烧录](#编译与烧录)
- [仓库结构](#仓库结构)
- [已知问题（V1）](#已知问题v1)
- [路线图](#路线图)
- [开源协议与致谢](#开源协议与致谢)

---

## 它能做什么

| 能力 | 实现方式 |
|---|---|
| **语音对话** | 本地唤醒词常驻检测 → 云端 ASR → 大模型对话 → TTS 回传播放，支持打断与连续对话 |
| **回声消除（边听边说）** | ES8311 单编解码器，麦克风与喇叭共用同一 MCLK/BCLK 时钟域，配合软件 AEC |
| **四足行走** | 4 路舵机正弦振荡步态（trot），支持前进、后退、原地转身 |
| **姿态动作** | 坐下、站立、伸展、摇摆、单腿抬起 |
| **情绪显示** | 0.96 寸 OLED 显示眼睛、表情、联网状态与电量 |
| **整机管理** | 2S 18650 供电，ADC 采样电量并低电提示，Type-C 一线烧录调试，支持 OTA |
| **可扩展** | 预留 I2C / UART / 9P 扩展排针，可外接触摸、ToF、IMU |

**唤醒词默认为「小白」**（`xiao bai`）。如需修改，见 [编译与烧录](#编译与烧录) 一节中的唤醒词配置。

---

## 硬件方案

整机采用「主控底板 + 音频板 + 电池板」三块 PCB 分体设计，数字、电声与功率分离，便于独立迭代调试。

### 一、主控底板（3.0）

- **主控**：ESP32-S3-WROOM-1-N16R8，双核 LX7 @240MHz，16MB Flash + 8MB PSRAM
  - 选 16/8 组合是因为端侧需同时驻留唤醒词模型、Opus 缓冲与图形资源，4MB 版本不够
- **下载调试**：CH340K USB-TTL + 自动下载电路，单根 Type-C 线完成烧录与日志查看；另留 4P 备用下载排针
- **显示**：0.96 寸 SSD1306 OLED（128×64），走独立 I2C1，与音频 I2C0 隔离
- **舵机**：4 路 50Hz PWM 由 LEDC 直接输出，每腿独占 GPIO，便于单腿标定
- **保护**：USB 端 RCLAMP0521T ESD 阵列 + MT9700 限流开关

### 二、音频板（2.0）

- **编解码**：ES8311（I2S 从机 + I2C 配置），收发音均 24kHz
- **拾音**：ZTS6216 模拟 MEMS 麦接 ES8311 的 MIC1P/N，由片内 PGA 与 ADC 采样
- **功放**：NS4150B D 类 3W，差分输出推 8Ω 喇叭；CTRL 脚由主控 GPIO 软件控制，可主动压制开关机 POP 声
- **板间接口**：9P 排针引出 3V3 / GND / SDA / SCL / MCLK / BCLK / LRCK / DIN / DOUT

> **关键取舍**：麦克风与喇叭共用同一 MCLK/BCLK 时钟域，是端侧 AEC 成立的硬件前提。若采用两颗独立编解码芯片，收发时钟不同源，回声消除将无法收敛。

### 三、电池板（2.0）

- **电芯**：2S 18650（标称 7.4V / 满电 8.4V），机械开关通断
- **一级降压**：MP2315SGJ-Z 同步 Buck + 4.7µH 电感 → 5V 主轨，供舵机与功放；底板再由 AMS1117-3.3 降到 3.3V
- **保护与采样**：SMAJ12A TVS 吸收浪涌；750k/100k 分压后送 ADC 采样

---

## 引脚定义

以下数值均取自固件 `main/boards/movecall/moji-esp32s3-oled/config.h`。

### 音频（ES8311，I2S + I2C0）

| 信号 | GPIO | 说明 |
|---|---|---|
| MCLK | 6 | 主时钟 |
| WS / LRCK | 12 | 帧同步 |
| BCLK | 14 | 位时钟 |
| DIN | 13 | 数据输入（麦克风方向） |
| DOUT | 11 | 数据输出（喇叭方向） |
| PA | 9 | 功放使能 |
| SDA | 5 | ES8311 控制总线（I2C0） |
| SCL | 4 | 同上 |

采样率：输入 / 输出均为 **24000 Hz**。

### 显示（SSD1306 OLED，I2C1）

| 信号 | GPIO | 说明 |
|---|---|---|
| SDA | 1 | 独立 I2C1，避免与 ES8311 的 I2C0 冲突 |
| SCL | 2 | 同上 |

分辨率 128×64；若换 1.3 寸 SH1106 屏，需在 menuconfig 中把 `OLED Type` 切为 `SH1106 128*64`。

### 舵机（4 路，LEDC 50Hz）

| 腿 | GPIO | 原理图丝印 |
|---|---|---|
| 左前腿 | 7 | SERVO3 |
| 左后腿 | 42 | SERVO4 |
| 右前腿 | 15 | SERVO1 |
| 右后腿 | 41 | SERVO2 |

### 其它

| 信号 | GPIO |
|---|---|
| 板载 LED | 21 |
| BOOT 按键 | 0 |
| 电池采样 | 18（ADC2_CH7，750k/100k 分压） |

> ⚠️ **电池采样引脚存在图纸与固件不一致**，详见 [已知问题](#已知问题v1)。

---

## 软件架构

四足运动控制分为三层，职责清晰、互不耦合：

```
语音 / 大模型
     │  MCP 工具调用（self.dog.*）
     ▼
eda_dog_controller.cc      控制器层：MCP 工具注册、动作队列、NVS 校准值持久化
     │  独立 FreeRTOS 任务顺序执行
     ▼
eda_dog_movements.cc       协调层：把 4 个振荡器组织成成套动作（walk/sit/turn…）
     │  正弦振荡 + 相位差
     ▼
oscillator.cc              执行层：单个舵机的 LEDC PWM 输出与限速
```

**几个设计要点：**

1. **动作必须异步执行。** 动作函数内部有大量 `vTaskDelay`，直接放在 MCP 回调里会阻塞协议栈。因此所有动作经队列投递给独立的 `dog_action` 任务（3KB 栈、优先级 `MAX-1`），MCP 回调入队后立即返回。
2. **步态靠相位差，不靠死写角度。** 给四条腿设置不同的 `amplitude`（振幅）、`offset`（偏置）、`phase_diff`（相位差）、`period`（周期），正弦振荡自然组合出对角步态（trot）。算法移植自 Otto 机器人 / Obijuan 的 `Oscillator.pde`。
3. **机械误差用 trim 修正。** 四条腿的安装角度不可能完全一致，`set_trim` 可对单腿做 ±50° 校准，写入 NVS（`dog_trims` 键空间）后断电不丢。
4. **舵机限速 240°/s。** 防止大角度跳变带来的电流冲击与机械抖动。

---

## AI 可调用的动作（MCP 工具）

大模型通过 MCP 工具调用直接驱动动作，**无需预先写死按键逻辑**，做到「说到就动」。

| 工具名 | 作用 | 参数 |
|---|---|---|
| `self.dog.walk` | 行走 | `steps` 1-100、`speed` 500-2000（越小越快）、`direction` -1 后退 / 1 前进 |
| `self.dog.turn` | 原地转身 | `steps`、`speed`、`direction` 1 左转 / -1 右转 |
| `self.dog.sit` | 坐下 | `speed` |
| `self.dog.stand` | 站立 | `speed` |
| `self.dog.stretch` | 伸展（伸懒腰） | `speed` |
| `self.dog.shake` | 摇摆 / 抖身 | `speed` |
| `self.dog.lift_left_front_leg` | 抬左前腿 | `speed`、`height` 10-90° |
| `self.dog.lift_left_rear_leg` | 抬左后腿 | `speed`、`height` |
| `self.dog.lift_right_front_leg` | 抬右前腿 | `speed`、`height` |
| `self.dog.lift_right_rear_leg` | 抬右后腿 | `speed`、`height` |
| `self.dog.stop` | 立即停止并回中位 | — |
| `self.dog.set_trim` | 单腿校准并永久保存 | `servo_type`、`trim_value` -50~50 |
| `self.dog.get_trims` | 读取四腿校准值 | — 返回 JSON |
| `self.dog.get_status` | 查询状态 | — 返回 `moving` / `idle` |
| `self.dog.bark` | 叫一声 | — 返回「汪汪汪」文本，由大模型念出 |

> 除「坐下」和「回中位」外，动作结束后会自动 `Home()` 回到静止姿态，避免狗僵在奇怪姿势。

---

## 一次典型交互

> 用户：「小白，往前走两步，然后摇个头。」

1. **本地**唤醒词检出，建立上行链路
2. **云端**识别出文字，大模型理解意图
3. **大模型**调用 `self.dog.walk(steps=2, direction=1)` 与 `self.dog.shake()`
4. **端侧**舵机执行动作，OLED 同步切换表情
5. **云端** TTS 回传，喇叭播报回应

唤醒在本地完成、对话在云端完成、动作在端侧闭环——兼顾响应速度与对话能力。

---

## 编译与烧录

### 开发环境

- **ESP-IDF ≥ v6.0.1**（推荐 v6.1，不支持 5.x）
- VSCode + ESP-IDF 插件，或命令行
- 本项目使用 Google C++ 代码风格

### 方式一：构建脚本（推荐）

```bash
idf.py set-target esp32s3
python3 scripts/build.py movecall/moji-esp32s3-oled --name movecall-moji-esp32s3-oled
```

### 方式二：menuconfig 手动配置

```bash
idf.py set-target esp32s3
idf.py menuconfig
```

依次选择：

```
Xiaozhi Assistant -> Target Board -> Movecall Moji (0.96 OLED)
Xiaozhi Assistant -> OLED Type   -> SSD1306 128*64
```

然后编译烧录：

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash monitor      # Windows 下换成 COMx
```

### 首次克隆后必须先生成配置

本仓库不提交 `sdkconfig`、`dependencies.lock` 与 `managed_components/`（体积大且与本机环境绑定），首次编译前请执行：

```bash
idf.py reconfigure     # 拉取依赖组件并生成 sdkconfig
```

### 修改唤醒词

板级默认唤醒词配置在 `main/boards/movecall/moji-esp32s3-oled/config.json`：

```json
"CONFIG_CUSTOM_WAKE_WORD": "\"xiao bai\"",
"CONFIG_CUSTOM_WAKE_WORD_DISPLAY": "\"小白\""
```

也可在 `idf.py menuconfig` 的 `Xiaozhi Assistant -> Wake Word` 中修改。

### 接入大模型

设备默认接入 [xiaozhi.me](https://xiaozhi.me) 官方服务器，注册账号即可免费使用实时语音模型。登录控制台可配置角色音色、绑定 MCP 工具。

---

## 仓库结构

```
├── main/
│   ├── boards/movecall/moji-esp32s3-oled/   ★ 本项目新增板卡
│   │   ├── config.h                         板级引脚定义
│   │   ├── config.json                      构建变体与唤醒词默认值
│   │   ├── movecall_moji_esp32s3_oled.cc    板级初始化（显示 / 音频 / 按键）
│   │   ├── eda_dog_controller.cc            ★ 控制器层：MCP 工具 + 动作队列
│   │   ├── eda_dog_movements.cc/.h          ★ 协调层：四足成套动作
│   │   └── oscillator.cc/.h                 ★ 执行层：舵机 PWM 与限速
│   ├── audio/                               音频服务（AEC / 唤醒 / 编解码）
│   ├── display/                             显示抽象（OLED / LCD / LVGL）
│   ├── protocols/                           WebSocket / MQTT 通信
│   ├── mcp_server.cc                        MCP 协议实现
│   └── assets/                              多语言提示音与表情资源
├── partitions/                              分区表
├── scripts/                                 构建、资源打包、音频转换工具
└── docs/                                    协议与开发文档
```

★ 标记为本项目相对上游的主要改动。

---

## 已知问题（V1）

这些是 V1 样机自检发现、**V2 待修正**的问题。公开记录在此，一是避免复现者踩同一个坑，二是留作硬件迭代的检查表。

| # | 问题 | 现象 | 影响 | V2 对策 |
|---|---|---|---|---|
| 1 | **电源裕量不足** | MP2315 额定 3A，四路舵机同时堵转峰值约 3.6A | 极限动作下可能触发降压保护 | 换 5A 级同步 Buck |
| 2 | **电池采样引脚不一致** | 原理图画的是 IO9（ADC1_CH8），固件 `config.h` 仍是 GPIO18（ADC2_CH7） | **ADC2 在 Wi-Fi 工作时不可用**，联网状态下电量读数失效 | 统一到 ADC1 引脚，改图纸或改固件 |
| 3 | **功放使能未真正受控** | NS4150B 的 CTRL 脚接了 GPIO9，但缺少上电时序控制 | 开关机 POP 声无软件压制手段 | 加 RC 延时 + 软件时序接管 |
| 4 | **2S 电池缺少保护** | 只有放电通路，无均衡充电与过充过放保护 | 不具备产品级安全性 | 加保护板或换带保护的电池包 |

> 第 2 条是最容易踩的坑：ESP32-S3 的 ADC2 与 Wi-Fi 共用硬件，联网后读数不可靠。选采样引脚时优先 ADC1。

---

## 路线图

- [x] V1：三块板设计完成，语音对话 + 四足行走跑通
- [ ] V2：修正上述 4 项硬件问题——电源加裕量、采样迁到 ADC1、功放受控、加电池保护
- [ ] V2：结构件优化，提升装配精度与公差
- [ ] 补充：原理图 / PCB / BOM 开源发布，装配说明与踩坑记录
- [ ] 目标指标：唤醒响应 ≤ 500ms、端到端对话 ≤ 1.5s、连续对话续航 ≥ 2h

---

## 应用场景

1. **桌面陪伴** — 办公桌与家庭场景的语音助手兼情绪陪伴。相比纯语音音箱，动作与表情能显著提升「在场感」。
2. **AI 硬件教学** — 三块板分体、全开源，覆盖 ESP32-S3、I2S 音频链路、AEC、PWM 舵机、Buck 电源与分板设计的完整样机。学生可只改一块板验证单一知识点，也可整机复现。
3. **具身智能入门平台** — 打通「大模型 → 动作」的最小闭环，成本远低于机械臂或轮式平台。
4. **展示与潮玩** — 展会、门店互动展品，具备小批量复制条件。

---

## 开源协议与致谢

- 本项目固件基于 [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 二次开发，遵循 **MIT License**
- 四足步态算法移植自 [Otto Robot](https://github.com/OttoDIY) / Obijuan 的 `Oscillator.pde`
- 语音对话与 MCP 协议部分由上游项目实现，感谢原作者与社区

开源协议详见 [LICENSE](LICENSE)。
