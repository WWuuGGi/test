/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    key_state_machine.h
  * @brief   按键状态机头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __KEY_STATE_MACHINE_H
#define __KEY_STATE_MACHINE_H

#include "stm32f4xx_hal.h"

// 按键引脚定义，请根据实际硬件修改
#define K1_GPIO_Port GPIOF
#define K1_Pin GPIO_PIN_1
#define K2_GPIO_Port GPIOF
#define K2_Pin GPIO_PIN_2
#define K3_GPIO_Port GPIOF
#define K3_Pin GPIO_PIN_3
#define K4_GPIO_Port GPIOF
#define K4_Pin GPIO_PIN_4


// 按键参数定义
#define KEY_LONG_PRESS_MS 1500    // 长按判定时间(ms)
#define KEY_DOUBLE_CLICK_MS 300   // 双击判定时间间隔(ms)
#define KEY_DEBOUNCE_MS 20        // 按键消抖时间（过滤机械抖动，约10-20ms）

// 按键状态枚举
typedef enum {
    KEY_STATE_IDLE,         // 空闲状态
    KEY_STATE_PRESSED,      // 按下状态
    KEY_STATE_RELEASED,     // 释放状态
    KEY_STATE_LONG_PRESS    // 长按状态
} KeyStateTypeDef;

// 按键事件枚举
typedef enum {
    KEY_EVENT_NONE,         // 无事件
    KEY_EVENT_CLICK,        // 单击事件
    KEY_EVENT_DOUBLE_CLICK, // 双击事件
    KEY_EVENT_LONG_PRESS    // 长按事件
} KeyEventTypeDef;

// 按键结构体定义
typedef struct {
    GPIO_TypeDef* gpio_port;    // 按键GPIO端口
    uint16_t gpio_pin;          // 按键GPIO引脚
    KeyStateTypeDef state;      // 当前状态
    uint32_t press_time;        // 按下时间戳
    uint32_t release_time;      // 释放时间戳
    uint8_t click_count;        // 点击计数
    uint8_t long_press_flag;    // 长按标志
    KeyEventTypeDef event;      // 按键事件
		uint32_t state_change_time;  // 状态变化的时间戳（用于消抖）
    uint8_t stable_pin_state;    // 经过消抖后的稳定引脚状态
} KeyTypeDef;

extern uint16_t step_mode_1;
extern uint16_t step_mode_2;
extern uint16_t step_mode_3;

// 函数声明
void Key_Init(void);
void Key_Process(void);
void Task_Execute(void);
uint8_t Key_GetTaskState(void);
uint8_t Key_GetCurrentMode(void);
KeyTypeDef Key1_scope(void);
KeyTypeDef Key2_scope(void);
KeyTypeDef Key3_scope(void);
KeyTypeDef Key4_scope(void);


#endif /* __KEY_STATE_MACHINE_H */

