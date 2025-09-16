
#ifndef __UNITREEA1_CMD__
#define __UNITREEA1_CMD__

#include "motor_msg.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "gom_protocol.h"

// 4组电机的发送结构体（每组对应一个发送数据体）
extern motor_send_t MotorA1_send_group1;  // 第1组电机发送数据
extern motor_send_t MotorA1_send_group2;  // 第2组电机发送数据
//extern motor_send_t MotorA1_send_group3;  // 第3组电机发送数据
//extern motor_send_t MotorA1_send_group4;  // 第4组电机发送数据

// 4组电机的接收结构体（每组2个ID，共8个电机）
// 第1组
extern motor_recv_t MotorA1_recv_group1_id0;
extern motor_recv_t MotorA1_recv_group1_id1;
extern motor_recv_t MotorA1_recv_group1_id2;
// 第2组
extern motor_recv_t MotorA1_recv_group2_id0;
extern motor_recv_t MotorA1_recv_group2_id1;
extern motor_recv_t MotorA1_recv_group2_id2;
// 第3组
//extern motor_recv_t MotorA1_recv_group3_id0;
//extern motor_recv_t MotorA1_recv_group3_id1;
// 第4组
//extern motor_recv_t MotorA1_recv_group4_id0;
//extern motor_recv_t MotorA1_recv_group4_id1;

// 每组接收数据缓冲区（78字节，与硬件协议匹配）
extern uint8_t Date_group1[78];
extern uint8_t Date_group2[78];
//extern uint8_t Date_group3[78];
//extern uint8_t Date_group4[78];
extern uint8_t Date_go_group3[16]; // go协议接收缓冲区（RIS_MotorData_t为16字节）

// 第四组改为go_protocol的结构体（替换原有group4）
extern MotorCmd_t Motor_go_send_group3;       // 发送结构体（go协议）
extern MotorData_t Motor_go_recv_group3_id0;  // 接收结构体（ID0）
extern MotorData_t Motor_go_recv_group3_id1;  // 接收结构体（ID1）


// 通信状态变量
extern HAL_StatusTypeDef rec_st[3];  // 4组接收状态
extern HAL_StatusTypeDef trans_st[3];// 4组发送状态

#define GROUP_PORT_1 GPIOB
#define GROUP_PORT_2 GPIOA
#define GROUP_PORT_3 GPIOC
//#define GROUP_PORT_4 GPIOC

#define GROUP_PIN_1 GPIO_PIN_10
#define GROUP_PIN_2 GPIO_PIN_8
#define GROUP_PIN_3 GPIO_PIN_8
//#define GROUP_PIN_4 GPIO_PIN_8


/**
 @brief 对应电机参数修改
 @param send 为MotorA1_send_left或MotorA1_send_right，分别控制左右侧腿部
 @param id   发送接收目标电机的id
 @param pos  为电机旋转圈数，1为一圈
 @param KP   电机刚度系数
 @param KW   电机速度系数
*/
void modify_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW);

// 速度模式
void modify_speed_cmd(motor_send_t *send,uint8_t id, float Omega);

// 力矩模式
void modify_torque_cmd(motor_send_t *send,uint8_t id, float torque);

void modify_PW_cmd(motor_send_t *send,uint8_t id, float Pos, float Omega, float KP, float KW);

void modify_changeid_cmd(motor_send_t *send,uint8_t mode);

/// @brief 用来和电机通讯的代码，将获取的数据存入对应结构体中
/// @param huart 需要使用的串口，huart1为左侧，6为右侧

void unitreeA1_rxtx(UART_HandleTypeDef *huart, uint8_t group);

void motor_relax(void);

uint32_t rc32_core_Ver3(uint32_t *ptr, uint32_t len);

#endif
