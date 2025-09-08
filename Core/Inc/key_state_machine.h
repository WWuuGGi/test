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
#define K0_GPIO_Port GPIOE
#define K0_Pin GPIO_PIN_4
#define K1_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_3

// 按键参数定义
#define KEY_LONG_PRESS_MS 1000    // 长按判定时间(ms)
#define KEY_DOUBLE_CLICK_MS 300   // 双击判定时间间隔(ms)

// LED定义(用于指示)，请根据实际硬件修改
#define LED_GPIO_Port GPIOF
#define LED_PIN GPIO_PIN_10

// 函数声明
void Key_Init(void);
void Key_Process(void);
void Task_Execute(void);
uint8_t Key_GetTaskState(void);
uint8_t Key_GetCurrentMode(void);

#endif /* __KEY_STATE_MACHINE_H */