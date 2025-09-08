#ifndef JOINT_H
#define JOINT_H

#include "main.h"
#include "A1_motor_drive.h"
#include "calc.h"

// �궨��
#define PI 3.1415926535f
#define DGR2RAD PI/180
#define RAD2DGR 180/PI
#define False 0
#define True 1

// �趨�ؽڵ������ʼλ��
extern float zero_left_ID0;
extern float zero_left_ID1;
//extern float zero_right_ID0;
//extern float zero_right_ID1;

// ������ݷ��ͽṹ��
extern motor_send_t MotorA1_send_left;         // ����һ�ŵ��������
extern motor_send_t MotorA1_send_right;        // ����һ�ŵ��������

extern motor_recv_t Date_left;                // ���ȵ������������
extern motor_recv_t MotorA1_recv_left_id00;   // ����00�ŵ������������
extern motor_recv_t MotorA1_recv_left_id01;   // ����01�ŵ������������

extern motor_recv_t Date_right;                // ���ȵ������������
extern motor_recv_t MotorA1_recv_right_id00;   // ����00�ŵ������������
extern motor_recv_t MotorA1_recv_right_id01;   // ����01�ŵ������������

extern uint8_t STOP; // ��ͣ״̬

// ������̲���
// ��¼�����λ
//typedef struct {
//    float zero_l_ID0; // ���ٺ�ĽǶ��� ���
//    float zero_l_ID1;
//    float zero_r_ID0;
//    float zero_r_ID1;

//} Chassis_ME_t;

//// ���̳�ʼ��
//Chassis_ME_t *Chassis_Init();

// �������Լ�
int Joint_Zero_OK(void);

// �������ȡ (���λ�� = �ϵ�λ��)
void Joint_Zero_init_Type1(void);

// �������ȡ (���λ�� = ��λλ��)
void Joint_Zero_init_Type2(void);

// ��������
void Joint_GOTO_zero(void);

// ��ص��λ��������״̬
void Joint_Monitor(void);

// ����λ�ÿ���
void Joint_Position_Control(float Pos_Front[][STEP_NUM],float kp,float kw,uint16_t step);

void Joint_PW_Control(float Pos_Front,float Omega,uint8_t id,float kp,float kw);

// �����ٶȿ���
void Joint_Speed_Control(float Speed_Front);//, float Speed_Back

//void Joint_Full_Position_Control(float Pos_Front_Left, float Pos_Front_Right, float Pos_Back_Left, float Pos_Back_Right);

// ��ؼ�� (����)
// uint8_t Joint_IsOn_Ground();


#endif // !JOINT_H
