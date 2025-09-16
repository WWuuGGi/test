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

//// �趨�ؽڵ������ʼλ��
//extern float zero_left_ID0;
//extern float zero_left_ID1;
////extern float zero_right_ID0;
////extern float zero_right_ID1;

//// ������ݷ��ͽṹ��
//extern motor_send_t MotorA1_send_left;         // ����һ�ŵ��������
//extern motor_send_t MotorA1_send_right;        // ����һ�ŵ��������

//extern motor_recv_t Date_left;                // ���ȵ������������
//extern motor_recv_t MotorA1_recv_left_id00;   // ����00�ŵ������������
//extern motor_recv_t MotorA1_recv_left_id01;   // ����01�ŵ������������

//extern motor_recv_t Date_right;                // ���ȵ������������
//extern motor_recv_t MotorA1_recv_right_id00;   // ����00�ŵ������������
//extern motor_recv_t MotorA1_recv_right_id01;   // ����01�ŵ������������

// 4��������������ÿ��ID0��ID1��
extern float zero_group1_ID0;
extern float zero_group1_ID1;
extern float zero_group1_ID2;
extern float zero_group2_ID0;
extern float zero_group2_ID1;
extern float zero_group2_ID2;
extern float zero_group3_ID0;
extern float zero_group3_ID1;



// �������/���սṹ����������A1_motor_drive.h��Ӧ��
extern motor_send_t MotorA1_send_group1;
extern motor_send_t MotorA1_send_group2;
//extern motor_send_t MotorA1_send_group3;
//extern motor_send_t MotorA1_send_group4;

// �滻Ϊgo_protocol�Ľṹ�壺
extern MotorCmd_t Motor_go_send_group2;
extern MotorData_t Motor_go_recv_group2_id0;
extern MotorData_t Motor_go_recv_group2_id1;

extern motor_recv_t MotorA1_recv_group1_id0;
extern motor_recv_t MotorA1_recv_group1_id1;
extern motor_recv_t MotorA1_recv_group1_id2;
extern motor_recv_t MotorA1_recv_group2_id0;
extern motor_recv_t MotorA1_recv_group2_id1;
extern motor_recv_t MotorA1_recv_group2_id2;
extern motor_recv_t MotorA1_recv_group3_id0;
extern motor_recv_t MotorA1_recv_group3_id1;
//extern motor_recv_t MotorA1_recv_group4_id0;
//extern motor_recv_t MotorA1_recv_group4_id1;


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
//void Joint_Monitor(void);

// ����λ�ÿ���
void Joint_Position_Control(uint8_t group, uint8_t id, float Pos[][STEP_NUM], float kp, float kw, uint16_t step);

void Joint_PW_Control(uint8_t group, uint8_t id,float Pos[][STEP_NUM],float Omega[][STEP_NUM],float kp,float kw,uint16_t step);

// �����ٶȿ���
//void Joint_Speed_Control(float Speed_Front);//, float Speed_Back

void Joint_Full_Position_Control(uint16_t step);

void Joint_Full_PW_Control(uint16_t step);
// ��ؼ�� (����)
// uint8_t Joint_IsOn_Ground();


#endif // !JOINT_H
