/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    key_state_machine.h
  * @brief   ����״̬��ͷ�ļ�
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __KEY_STATE_MACHINE_H
#define __KEY_STATE_MACHINE_H

#include "stm32f4xx_hal.h"

// �������Ŷ��壬�����ʵ��Ӳ���޸�
#define K0_GPIO_Port GPIOE
#define K0_Pin GPIO_PIN_4
#define K1_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_3

// ������������
#define KEY_LONG_PRESS_MS 1000    // �����ж�ʱ��(ms)
#define KEY_DOUBLE_CLICK_MS 300   // ˫���ж�ʱ����(ms)

// LED����(����ָʾ)�������ʵ��Ӳ���޸�
#define LED_GPIO_Port GPIOF
#define LED_PIN GPIO_PIN_10

// ��������
void Key_Init(void);
void Key_Process(void);
void Task_Execute(void);
uint8_t Key_GetTaskState(void);
uint8_t Key_GetCurrentMode(void);

#endif /* __KEY_STATE_MACHINE_H */