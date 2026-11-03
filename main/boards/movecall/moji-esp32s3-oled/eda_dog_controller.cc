/*
    EDA机器狗控制器 - MCP协议版本

    作用：把 EDARobotDog（四足运动库）封装成一组可被 AI / 语音调用的 MCP 工具，
    例如 self.dog.walk、self.dog.sit、self.dog.set_trim 等。

    设计要点：
      - 所有动作都通过一个独立的 FreeRTOS 任务(ActionTask)顺序执行，避免在
        MCP 回调里长时间阻塞（动作函数内部都有大量 vTaskDelay）。
      - 外部只调用 QueueAction() 把动作参数压入队列即可立即返回。
      - 每条腿的机械校准值(trim)用 NVS(键空间 dog_trims)持久化，重启后仍然有效。

    MCP 工具名/参数/含义（均为 AI 可调用）：
      self.dog.walk              行走：steps(1-100) speed(500-2000) direction(-1后退/1前进)
      self.dog.turn              转身：steps(1-100) speed(500-2000) direction(-1右转/1左转)
      self.dog.sit               坐下：speed(500-2000)
      self.dog.stand             站立：speed(500-2000)
      self.dog.stretch           伸展：speed(500-2000)
      self.dog.shake             摇摆：speed(500-2000)
      self.dog.lift_left_front_leg   抬左前腿：speed(500-2000) height(10-90)
      self.dog.lift_left_rear_leg    抬左后腿：speed(500-2000) height(10-90)
      self.dog.lift_right_front_leg  抬右前腿：speed(500-2000) height(10-90)
      self.dog.lift_right_rear_leg   抬右后腿：speed(500-2000) height(10-90)
      self.dog.stop              立即停止（删除任务、清空队列、回 HOME）
      self.dog.set_trim          设置单腿校准值并永久保存：servo_type, trim_value(-50~50)
      self.dog.get_trims         读取当前四腿校准值(JSON)
      self.dog.get_status        读取状态：moving / idle
      self.dog.bark              叫一声：返回“汪汪汪”文本，由大模型念出来
*/

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "application.h"
#include "board.h"
#include "config.h"
#include "eda_dog_movements.h"
#include "mcp_server.h"
#include "sdkconfig.h"
#include "settings.h"

#define TAG "EDARobotDogController"

// 机器狗控制器：持有 EDARobotDog 实例，负责队列调度、动作分派和 MCP 工具注册。
class EDARobotDogController {
private:
    EDARobotDog dog_;                            // 底层四足运动对象（4 个舵机振荡器）
    TaskHandle_t action_task_handle_ = nullptr;  // 动作任务句柄，nullptr 表示未创建
    QueueHandle_t action_queue_;                 // 动作参数队列，用于向任务投递请求
    bool is_action_in_progress_ = false;         // 是否正在执行动作（供 get_status 查询）

    // 一次动作请求的参数；各项含义视 action_type 而定，未用到的填 0。
    struct DogActionParams {
        int action_type;  // 动作类型，取 ActionType 枚举值
        int steps;        // 步数（walk/turn 用，其余为 1）
        int speed;        // 周期/速度参数（毫秒，数值越小越快）
        int direction;    // 方向（-1/1，walk/turn 用）
        int height;       // 抬腿高度（lift_* 用，其余为 0）
    };

    // 支持的动作类型，与 ActionTask 的 switch 分支一一对应。
    enum ActionType {
        ACTION_WALK = 1,              // 行走
        ACTION_TURN = 2,              // 转身
        ACTION_SIT = 3,               // 坐下
        ACTION_STAND = 4,             // 站立
        ACTION_STRETCH = 5,           // 伸展
        ACTION_SHAKE = 6,             // 摇摆
        ACTION_LIFT_LEFT_FRONT = 7,   // 抬左前腿
        ACTION_LIFT_LEFT_REAR = 8,    // 抬左后腿
        ACTION_LIFT_RIGHT_FRONT = 9,  // 抬右前腿
        ACTION_LIFT_RIGHT_REAR = 10,  // 抬右后腿
        ACTION_HOME = 11              // 回中位（静止站立姿态）
    };

    // 动作任务主体：常驻后台，循环从队列取动作并同步执行。
    // 之所以独立成任务，是因为动作函数内部大量 vTaskDelay 会阻塞，不能放在 MCP 回调里。
    static void ActionTask(void* arg) {
        EDARobotDogController* controller = static_cast<EDARobotDogController*>(arg);
        DogActionParams params;
        controller->dog_.AttachServos();  // 任务启动时确保舵机已挂载

        while (true) {
            // 阻塞最多 1s 等待新动作，超时则继续等待
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                controller->is_action_in_progress_ = true;

                // 按类型分派到 EDARobotDog 的对应动作函数
                switch (params.action_type) {
                    case ACTION_WALK:
                        controller->dog_.Walk(params.steps, params.speed, params.direction);
                        break;
                    case ACTION_TURN:
                        controller->dog_.Turn(params.steps, params.speed, params.direction);
                        break;
                    case ACTION_SIT:
                        controller->dog_.Sit(params.speed);
                        break;
                    case ACTION_STAND:
                        controller->dog_.Stand(params.speed);
                        break;
                    case ACTION_STRETCH:
                        controller->dog_.Stretch(params.speed);
                        break;
                    case ACTION_SHAKE:
                        controller->dog_.Shake(params.speed);
                        break;
                    case ACTION_LIFT_LEFT_FRONT:
                        controller->dog_.LiftLeftFrontLeg(params.speed, params.height);
                        break;
                    case ACTION_LIFT_LEFT_REAR:
                        controller->dog_.LiftLeftRearLeg(params.speed, params.height);
                        break;
                    case ACTION_LIFT_RIGHT_FRONT:
                        controller->dog_.LiftRightFrontLeg(params.speed, params.height);
                        break;
                    case ACTION_LIFT_RIGHT_REAR:
                        controller->dog_.LiftRightRearLeg(params.speed, params.height);
                        break;
                    case ACTION_HOME:
                        controller->dog_.Home();
                        break;
                }

                // 除 HOME / SIT 外，动作结束后自动回到静止姿态，避免狗僵在奇怪姿势
                if (params.action_type != ACTION_HOME && params.action_type != ACTION_SIT) {
                    controller->dog_.Home();
                }
                controller->is_action_in_progress_ = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    // 惰性创建动作任务：仅在尚未创建时创建，避免重复创建多个任务。
    // 任务栈 3KB，优先级为最高级-1（高于普通任务，保证动作流畅）。
    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "dog_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    // 把一次动作请求压入队列，并确保动作任务已启动。
    // 参数含义见 DogActionParams；本函数会阻塞直到队列有空位（portMAX_DELAY）。
    void QueueAction(int action_type, int steps, int speed, int direction, int height) {
        ESP_LOGI(TAG, "动作控制: 类型=%d, 步数=%d, 速度=%d, 方向=%d, 高度=%d", action_type, steps,
                 speed, direction, height);

        DogActionParams params = {action_type, steps, speed, direction, height};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    // 从 NVS 读取四条腿的校准值并应用（键空间 dog_trims，默认 0）。
    // 只读模式(false)；在校准值被 set_trim 修改后，需要重新加载才会生效（set_trim 里直接调用
    // SetTrims）。
    void LoadTrimsFromNVS() {
        Settings settings("dog_trims", false);

        int left_front_leg = settings.GetInt("left_front_leg", 0);
        int left_rear_leg = settings.GetInt("left_rear_leg", 0);
        int right_front_leg = settings.GetInt("right_front_leg", 0);
        int right_rear_leg = settings.GetInt("right_rear_leg", 0);

        ESP_LOGI(TAG, "从NVS加载微调设置: 左前腿=%d, 左后腿=%d, 右前腿=%d, 右后腿=%d",
                 left_front_leg, left_rear_leg, right_front_leg, right_rear_leg);

        dog_.SetTrims(left_front_leg, left_rear_leg, right_front_leg, right_rear_leg);
    }

public:
    // 构造：初始化四条腿 GPIO（来自 config.h 的 *_LEG_PIN），加载校准值，
    // 建立动作队列，先排一个 HOME 动作让狗摆到站立姿态，最后注册所有 MCP 工具。
    EDARobotDogController() {
        dog_.Init(LEFT_FRONT_LEG_PIN, LEFT_REAR_LEG_PIN, RIGHT_FRONT_LEG_PIN, RIGHT_REAR_LEG_PIN);

        ESP_LOGI(TAG, "EDA机器狗初始化完成");

        LoadTrimsFromNVS();

        action_queue_ = xQueueCreate(10, sizeof(DogActionParams));  // 队列深度 10

        QueueAction(ACTION_HOME, 1, 1000, 0, 0);

        RegisterMcpTools();
    }

    // 把所有动作注册为 MCP 工具（AI/语音可调用）。每个工具的：
    //   name        - 工具名（self.dog.*）
    //   description - 自然语言说明，供大模型理解参数含义
    //   PropertyList- 参数定义（类型、默认值、范围）
    //   回调 lambda - 参数校验后调用 QueueAction 入队，立即返回 true
    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        ESP_LOGI(TAG, "开始注册MCP工具...");

        // 基础移动动作
        // 行走：把 steps/speed/direction 入队为 ACTION_WALK
        mcp_server.AddTool("self.dog.walk",
                           "行走。steps: 行走步数(1-100); speed: "
                           "行走速度(500-2000，数值越小越快); "
                           "direction: 行走方向(-1=后退, 1=前进)",
                           PropertyList({Property("steps", kPropertyTypeInteger, 4, 1, 100),
                                         Property("speed", kPropertyTypeInteger, 1000, 500, 2000),
                                         Property("direction", kPropertyTypeInteger, 1, -1, 1)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int direction = properties["direction"].value<int>();
                               QueueAction(ACTION_WALK, steps, speed, direction, 0);
                               return true;
                           });

        // 转身：direction 1=左转 / -1=右转
        mcp_server.AddTool("self.dog.turn",
                           "转身。steps: 转身步数(1-100); speed: "
                           "转身速度(500-2000，数值越小越快); "
                           "direction: 转身方向(1=左转, -1=右转)",
                           PropertyList({Property("steps", kPropertyTypeInteger, 4, 1, 100),
                                         Property("speed", kPropertyTypeInteger, 2000, 500, 2000),
                                         Property("direction", kPropertyTypeInteger, 1, -1, 1)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int direction = properties["direction"].value<int>();
                               QueueAction(ACTION_TURN, steps, speed, direction, 0);
                               return true;
                           });

        // 姿态动作
        // 坐下：注意 ActionTask 对 SIT 不自动回中，坐下后保持坐姿
        mcp_server.AddTool("self.dog.sit", "坐下。speed: 坐下速度(500-2000，数值越小越快)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1500, 500, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               QueueAction(ACTION_SIT, 1, speed, 0, 0);
                               return true;
                           });

        // 站立：等价于回中位
        mcp_server.AddTool("self.dog.stand", "站立。speed: 站立速度(500-2000，数值越小越快)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1500, 500, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               QueueAction(ACTION_STAND, 1, speed, 0, 0);
                               return true;
                           });

        // 伸展（伸懒腰）
        mcp_server.AddTool("self.dog.stretch", "伸展。speed: 伸展速度(500-2000，数值越小越快)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 2000, 500, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               QueueAction(ACTION_STRETCH, 1, speed, 0, 0);
                               return true;
                           });

        // 摇摆/抖身
        mcp_server.AddTool("self.dog.shake", "摇摆。speed: 摇摆速度(500-2000，数值越小越快)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1000, 500, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               QueueAction(ACTION_SHAKE, 1, speed, 0, 0);
                               return true;
                           });

        // 单腿抬起动作
        // 抬左前腿：height 为抬腿高度(10-90 度)
        mcp_server.AddTool("self.dog.lift_left_front_leg",
                           "抬起左前腿。speed: 动作速度(500-2000，数值越小越快); height: "
                           "抬起高度(10-90度)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1000, 500, 2000),
                                         Property("height", kPropertyTypeInteger, 45, 10, 90)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int height = properties["height"].value<int>();
                               QueueAction(ACTION_LIFT_LEFT_FRONT, 1, speed, 0, height);
                               return true;
                           });

        // 抬左后腿
        mcp_server.AddTool("self.dog.lift_left_rear_leg",
                           "抬起左后腿。speed: 动作速度(500-2000，数值越小越快); height: "
                           "抬起高度(10-90度)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1000, 500, 2000),
                                         Property("height", kPropertyTypeInteger, 45, 10, 90)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int height = properties["height"].value<int>();
                               QueueAction(ACTION_LIFT_LEFT_REAR, 1, speed, 0, height);
                               return true;
                           });

        // 抬右前腿
        mcp_server.AddTool("self.dog.lift_right_front_leg",
                           "抬起右前腿。speed: 动作速度(500-2000，数值越小越快); height: "
                           "抬起高度(10-90度)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1000, 500, 2000),
                                         Property("height", kPropertyTypeInteger, 45, 10, 90)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int height = properties["height"].value<int>();
                               QueueAction(ACTION_LIFT_RIGHT_FRONT, 1, speed, 0, height);
                               return true;
                           });

        // 抬右后腿
        mcp_server.AddTool("self.dog.lift_right_rear_leg",
                           "抬起右后腿。speed: 动作速度(500-2000，数值越小越快); height: "
                           "抬起高度(10-90度)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 1000, 500, 2000),
                                         Property("height", kPropertyTypeInteger, 45, 10, 90)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int height = properties["height"].value<int>();
                               QueueAction(ACTION_LIFT_RIGHT_REAR, 1, speed, 0, height);
                               return true;
                           });

        // 系统工具
        // 立即停止：强制删除动作任务、清空队列，然后重新排一个 HOME 让狗回到站立。
        mcp_server.AddTool("self.dog.stop", "立即停止", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (action_task_handle_ != nullptr) {
                                   vTaskDelete(action_task_handle_);
                                   action_task_handle_ = nullptr;
                               }
                               is_action_in_progress_ = false;
                               xQueueReset(action_queue_);

                               QueueAction(ACTION_HOME, 1, 1000, 0, 0);
                               return true;
                           });

        // 设置单腿校准：先读出当前四腿 trim，只改指定腿，写回 NVS(可写模式 true) 并立即应用。
        // servo_type 取值：left_front_leg/left_rear_leg/right_front_leg/right_rear_leg
        // trim_value 范围 -50~50 度；非法类型返回错误字符串。
        mcp_server.AddTool(
            "self.dog.set_trim",
            "校准单个舵机位置。设置指定舵机的微调参数以调整机器狗的初始站立姿态，设"
            "置将永久保存。"
            "servo_type: "
            "舵机类型(left_front_leg/left_rear_leg/right_front_leg/"
            "right_rear_leg); "
            "trim_value: 微调值(-50到50度)",
            PropertyList({Property("servo_type", kPropertyTypeString, "left_front_leg"),
                          Property("trim_value", kPropertyTypeInteger, 0, -50, 50)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();

                ESP_LOGI(TAG, "设置舵机微调: %s = %d度", servo_type.c_str(), trim_value);

                // 获取当前所有微调值
                Settings settings("dog_trims", true);
                int left_front_leg = settings.GetInt("left_front_leg", 0);
                int left_rear_leg = settings.GetInt("left_rear_leg", 0);
                int right_front_leg = settings.GetInt("right_front_leg", 0);
                int right_rear_leg = settings.GetInt("right_rear_leg", 0);

                // 更新指定舵机的微调值
                if (servo_type == "left_front_leg") {
                    left_front_leg = trim_value;
                    settings.SetInt("left_front_leg", left_front_leg);
                } else if (servo_type == "left_rear_leg") {
                    left_rear_leg = trim_value;
                    settings.SetInt("left_rear_leg", left_rear_leg);
                } else if (servo_type == "right_front_leg") {
                    right_front_leg = trim_value;
                    settings.SetInt("right_front_leg", right_front_leg);
                } else if (servo_type == "right_rear_leg") {
                    right_rear_leg = trim_value;
                    settings.SetInt("right_rear_leg", right_rear_leg);
                } else {
                    return "错误：无效的舵机类型，请使用: left_front_leg, "
                           "left_rear_leg, right_front_leg, right_rear_leg";
                }

                // 立即把新校准值应用到运行中的舵机
                dog_.SetTrims(left_front_leg, left_rear_leg, right_front_leg, right_rear_leg);

                QueueAction(ACTION_HOME, 1, 500, 0, 0);  // 回中位以便观察校准效果

                return "舵机 " + servo_type + " 微调设置为 " + std::to_string(trim_value) +
                       " 度，已永久保存";
            });

        // 读取当前四腿校准值，返回 JSON 字符串
        mcp_server.AddTool("self.dog.get_trims", "获取当前的舵机微调设置", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("dog_trims", false);

                               int left_front_leg = settings.GetInt("left_front_leg", 0);
                               int left_rear_leg = settings.GetInt("left_rear_leg", 0);
                               int right_front_leg = settings.GetInt("right_front_leg", 0);
                               int right_rear_leg = settings.GetInt("right_rear_leg", 0);

                               std::string result =
                                   "{\"left_front_leg\":" + std::to_string(left_front_leg) +
                                   ",\"left_rear_leg\":" + std::to_string(left_rear_leg) +
                                   ",\"right_front_leg\":" + std::to_string(right_front_leg) +
                                   ",\"right_rear_leg\":" + std::to_string(right_rear_leg) + "}";

                               ESP_LOGI(TAG, "获取微调设置: %s", result.c_str());
                               return result;
                           });

        // 查询状态：正在执行动作为 moving，否则 idle
        mcp_server.AddTool("self.dog.get_status", "获取机器狗状态，返回 moving 或 idle",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        // 叫：不做任何舵机动作，也不播放本地音效，只把“汪汪汪”这段文本作为工具结果
        // 返回给服务器端大模型，由大模型把这几个字念出来（走云端 TTS），
        // 于是听到的就是“汪汪汪”的叫声。
        mcp_server.AddTool("self.dog.bark",
                           "让机器狗叫一声，发出“汪汪汪”的叫声。"
                           "当用户要求小狗叫、吠、汪一下时调用。"
                           "调用后直接用“汪汪汪”回复用户，不要再加任何解释或描述。",
                           PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               ESP_LOGI(TAG, "小狗叫：汪汪汪");
                               return "汪汪汪";
                           });

        ESP_LOGI(TAG, "MCP工具注册完成");
    }

    // 析构：删除动作任务并销毁队列，释放资源。
    ~EDARobotDogController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static EDARobotDogController* g_dog_controller = nullptr;  // 全局单例指针

// 初始化入口：单例式创建控制器（重复调用不会重复创建）。
// 由板级初始化代码调用一次即可。
void InitializeEDARobotDogController() {
    if (g_dog_controller == nullptr) {
        g_dog_controller = new EDARobotDogController();
        ESP_LOGI(TAG, "EDA机器狗控制器已初始化并注册MCP工具");
    }
}