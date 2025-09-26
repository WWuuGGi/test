/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    key_state_machine.c
  * @brief   按键状态机实现文件
  ******************************************************************************
  */
/* USER CODE END Header */

#include "key_state_machine.h"
#include "main.h"
#include "calc.h"
#include "A1_motor_drive.h"
#include "joint.h"


// 按键实例化
static KeyTypeDef key1 = {K1_GPIO_Port, K1_Pin, KEY_STATE_IDLE, 0, 0, 0, 0, KEY_EVENT_NONE,0,0};
static KeyTypeDef key2 = {K2_GPIO_Port, K2_Pin, KEY_STATE_IDLE, 0, 0, 0, 0, KEY_EVENT_NONE,0,0};
static KeyTypeDef key3 = {K3_GPIO_Port, K3_Pin, KEY_STATE_IDLE, 0, 0, 0, 0, KEY_EVENT_NONE,0,0};
static KeyTypeDef key4 = {K4_GPIO_Port, K4_Pin, KEY_STATE_IDLE, 0, 0, 0, 0, KEY_EVENT_NONE,0,0};

// 全局变量
static uint8_t task_running = 0;    // 任务运行标志
static uint8_t current_mode = 0;    // 当前模式

// 函数声明
static uint8_t Key_ReadPin(KeyTypeDef* key);
static void Key_StateMachine(KeyTypeDef* key);
static void Key_HandleEvents(void);

uint16_t step_mode_1 = 0;
uint16_t step_mode_2 = 0;
uint16_t step_mode_3 = 0;
uint8_t zero_init = 1;
uint8_t data_logging = 0;
uint8_t turn = 1;
/**
  * @brief  初始化按键
  * @param  无
  * @retval 无
  */
void Key_Init(void) {
    // 确保GPIO已在MX_GPIO_Init中初始化
    // K0和K1应配置为输入模式，带下拉电阻
}

/**
  * @brief  读取按键引脚状态
  * @param  key: 按键结构体指针
  * @retval 1: 按键按下, 0: 按键释放
  */
static uint8_t Key_ReadPin(KeyTypeDef* key) {
    // 假设按键按下为低电平(上拉输入)
    return (HAL_GPIO_ReadPin(key->gpio_port, key->gpio_pin) == GPIO_PIN_RESET) ? 1 : 0;
}

/**
  * @brief  按键状态机处理
  * @param  key: 按键结构体指针
  * @retval 无
  */
static void Key_StateMachine(KeyTypeDef* key) {
    uint8_t raw_pin_state = Key_ReadPin(key);
    uint32_t current_time = HAL_GetTick();
    
    key->event = KEY_EVENT_NONE;

    // 按键消抖：检测到引脚状态变化后，等待KEY_DEBOUNCE_MS再确认
    if (raw_pin_state != key->stable_pin_state) {
        // 首次检测到变化，记录时间
        if (key->state_change_time == 0) {
            key->state_change_time = current_time;
        }
        // 等待消抖时间后，确认状态变化
        else if (current_time - key->state_change_time >= KEY_DEBOUNCE_MS) {
            key->stable_pin_state = raw_pin_state;  // 更新稳定状态
            key->state_change_time = 0;             // 重置消抖计时
        }
        //return;  // 消抖期间不处理状态切换
    } else {
        // 状态未变化，重置消抖计时
        key->state_change_time = 0;
    }

    // 以下使用消抖后的stable_pin_state进行状态判断
    uint8_t pin_state = key->stable_pin_state;

    switch (key->state) {
        case KEY_STATE_IDLE:
            if (pin_state) {  // 按键按下（消抖后确认）
                key->state = KEY_STATE_PRESSED;
                key->press_time = current_time;
                key->click_count = 0;  // 重置点击计数
                key->long_press_flag = 0;
            }
            break;

        case KEY_STATE_PRESSED:
            // 检测长按（消抖后确认按下状态持续时间）
            if (current_time - key->press_time >= KEY_LONG_PRESS_MS) {
                key->state = KEY_STATE_LONG_PRESS;
                key->long_press_flag = 1;
                key->event = KEY_EVENT_LONG_PRESS;
            }
            // 检测释放（消抖后确认释放）
            else if (!pin_state) {
                key->state = KEY_STATE_RELEASED;
                key->release_time = current_time;
                key->click_count++;  // 计数+1（可能是单击或双击的第一次）
            }
            break;

        case KEY_STATE_RELEASED:
            // 双击窗口超时：确认单击/双击
            if (current_time - key->release_time >= KEY_DOUBLE_CLICK_MS) {
                if (key->click_count == 1) {
                    key->event = KEY_EVENT_CLICK;  // 单击事件（快速响应）
                } else if (key->click_count == 2) {
                    key->event = KEY_EVENT_DOUBLE_CLICK;  // 双击事件
                }
                key->state = KEY_STATE_IDLE;
                key->click_count = 0;
								//添加重置施放时间
								key->release_time = 0; // 重置释放时间，避免残留值干扰
            }
            // 双击窗口内再次按下：计数+1
            else if (pin_state) {
                key->state = KEY_STATE_PRESSED;
                key->press_time = current_time;
            }
            break;

        case KEY_STATE_LONG_PRESS:
            // 长按后释放
            if (!pin_state) {
                key->state = KEY_STATE_IDLE;
                key->click_count = 0;
            }
            break;

        default:
            key->state = KEY_STATE_IDLE;
            break;
    }
}

/**
  * @brief  按键事件处理
  * @param  无
  * @retval 无
  */
static void Key_HandleEvents(void) {
    // 处理K1按键事件 - 控制任务启动/停止
    if (key4.event == KEY_EVENT_CLICK) {
        if (task_running) {
            // 急停任务
            task_running = 0;
						current_mode = 0;
            // 急停相关代码
            step_mode_1 = 0;
						step_mode_2 = 0;
						step_mode_3 = 0;
						zero_init = 1;
						//调成零力矩模式，等待拖拽回中
						//motor_relax();
					
        } else {
            // 启动任务
            task_running = 1;
            current_mode = 0;  // 启动时默认进入模式0
						step_mode_1 = 0;
						step_mode_2 = 0;
						step_mode_3 = 0;
					
						//调成零力矩模式，等待接收指令
						//motor_relax();
            
        }
    }
    
    // 只有任务运行时，才处理K1的模式切换
    if (task_running) {
        // 处理K1按键事件 - 模式切换
        if (key1.event == KEY_EVENT_CLICK) {
            // 单击切换到模式1
            current_mode = 1;
        } else if (key2.event == KEY_EVENT_CLICK) {
            // 双击切换到模式2
            current_mode = 2;
        } else if (key3.event == KEY_EVENT_CLICK) {
            // 长按切换到模式3
            current_mode = 3;
        }
    }
}

/**
  * @brief  任务执行函数
  * @param  无
  * @retval 无
  * @note   根据当前模式执行不同的任务逻辑
  */
void Task_Execute(void) {
	if(task_running)
	{
    switch (current_mode) {
        case 1:
            // 模式1的任务逻辑
            //
					if((step_mode_2 == 0) && (step_mode_3 == 0))
					{
						if(step_mode_1 == 0)
						{
							//初次进入任务时，把其他标志位清零
							//step_mode_2 = 0;
							//step_mode_3 = 0;
							Pose start_pose = {0.0f, 0.0f, 0.135f, 0.0f, 0.0f, 0.0f};
							Pose end_pose = {0.25f, 0.25f, 0.335f, 0.0f, 0.0f, 0.0f};
							switch(turn)
							{
								case 1:
									start_pose.data[0] = 0.0f;
									start_pose.data[1] = 0.0f;
							}
							
							// 初始速度和加速度为零
							Velocity start_vel = {0};
							Velocity end_vel = {0};
							Acceleration start_acc = {0};
							Acceleration end_acc = {0};
							cdpr_init(&start_pose, &start_vel, &start_acc, &end_pose, &end_vel, &end_acc,10.0f);
						}
						
						if(step_mode_1 < STEP_NUM && task_running)
						{
							
							Joint_Full_PW_Control(step_mode_1);
//							Joint_Full_Position_Control(step_mode_1);
//							modify_speed_cmd(&MotorA1_send_group1,1,0.5f);
//							//modify_torque_cmd(&MotorA1_send_group1,1,0.75f);
//							unitreeA1_rxtx(&huart1,1);
							step_mode_1++;

						}
						else
						{
							Joint_Full_PW_Control(step_mode_1 - 1);
						}
					}
					else
					{
						motor_relax();
					}
            break;
        case 2:
            // 模式2的任务逻辑
            //
					if((step_mode_1 == 0) && (step_mode_3 == 0))
					{
							if(step_mode_2 == 0)
							{
								//初次进入任务时，把其他标志位清零
								//step_mode_1 = 0;
								//step_mode_3 = 0;
								
								//初次进入时，计算轨迹路径
							Pose start_pose = {0.0f, 0.0f, 0.135f, 0.0f, 0.0f, 0.0f};
							Pose end_pose = {-0.25f, -0.25f, 0.335f, 0.0f, 0.0f, 0.0f};

								// 初始速度和加速度为零
								Velocity start_vel = {0};
								Velocity end_vel = {0};
								Acceleration start_acc = {0};
								Acceleration end_acc = {0};

								// 或者设置非零的初始和末尾速度
								// Velocity start_vel = {0.1f, 0.1f, 0, 0, 0, 0};  // 初始有小速度
								// Velocity end_vel = {0.1f, 0.1f, 0, 0, 0, 0};    // 末尾有小速度

								// 初始化CDPR系统
								//	cdpr_init(&start, &end);
								cdpr_init(&start_pose, &start_vel, &start_acc, &end_pose, &end_vel, &end_acc,10.0f);
							}
							
							if(step_mode_2 < STEP_NUM && task_running)
							{
								
								Joint_Full_PW_Control(step_mode_2);
								step_mode_2++;

							}
							else
							{
								motor_relax();
							}
					}
					else
					{
						motor_relax();
					}
						break;
        case 3:
            // 模式3的任务逻辑
           //
					if((step_mode_1 == 0) && (step_mode_2 == 0))
					{
						if(step_mode_3 == 0)
						{

							data_logging =1;
							
							//初次进入时，计算轨迹路径
							Pose start_pose = {0.0f, 0.0f, 0.135f, 0.0f, 0.0f, 0.0f};
							Pose end_pose = {-0.25f, -0.25f, 0.635f, 0.0f, 0.0f, 0.0f};

							// 初始速度和加速度为零
							Velocity start_vel = {0};
							Velocity end_vel = {0};
							Acceleration start_acc = {0};
							Acceleration end_acc = {0};

							cdpr_init(&start_pose, &start_vel, &start_acc, &end_pose, &end_vel, &end_acc,10.0f);
						}
						
						if(step_mode_3 < STEP_NUM && task_running)
						{
							
							Joint_Full_PW_Control(step_mode_3);
							//Joint_Full_Position_Control(step_mode_3);
							step_mode_3++;
						}
						else
						{
							motor_relax();
							
						}
					}
					else
					{
						motor_relax();
					}
            break;
        default:
            // 默认模式处理
            current_mode = 0;
            break;
    }
	}
}


/**
  * @brief  按键处理主函数，需周期性调用(建议10ms)
  * @param  无
  * @retval 无
  */
void Key_Process(void) {
    Key_StateMachine(&key1);
    Key_StateMachine(&key2);
		Key_StateMachine(&key3);
		Key_StateMachine(&key4);
	
    Key_HandleEvents();
    
    // 如果任务运行中，执行任务逻辑
}

/**
  * @brief  获取任务运行状态
  * @param  无
  * @retval 1: 任务运行中, 0: 任务已停止
  */
uint8_t Key_GetTaskState(void) {
    return task_running;
}

/**
  * @brief  获取当前模式
  * @param  无
  * @retval 当前模式值
  */
uint8_t Key_GetCurrentMode(void) {
    return current_mode;
}

KeyTypeDef Key1_scope(void) {
		return key1;
}

KeyTypeDef Key2_scope(void) {
		return key2;
}

KeyTypeDef Key3_scope(void) {
		return key3;
}

KeyTypeDef Key4_scope(void) {
		return key4;
}
