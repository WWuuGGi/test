#include <stdlib.h>
#include <stdio.h>
#include "joint.h"

#define LIMIT_RANGE(Pos, Min, Max) ((Pos) = ((Pos) > (Max) ? (Max) : (Pos) < (Min) ? (Min) : (Pos)))

// Ĭ�ϵ����ʼ���
float zero_left_ID0  = 0.0f;
float zero_left_ID1  = 0.0f;
//float zero_right_ID0 = 0.0f;
//float zero_right_ID1 = 0.0f;

// �����Ϣ�ṹ��
//Chassis_ME_t Chassis;

uint8_t STOP = False;

static float home_speed  = 0.4f;  // ���ٺ���ٶ� rad/s
static float home_torque = 1.0f;  // ���ٺ����� Nm
static float UP_LIMIT    = 20.0f; // ���ٺ�Ƕ� ��
static float DOWN_LIMIT  = 80.0f; // ���ٺ�Ƕ� ��
static float TOLERANCE   = -5.0f;  // �ݲ� ��

// �������̽ṹ��
//Chassis_ME_t *Chassis_Init()
//{
//    Chassis.zero_l_ID0 = 0.0f;
//    Chassis.zero_l_ID1 = 0.0f;
//    Chassis.zero_r_ID0 = 0.0f;
//    Chassis.zero_r_ID1 = 0.0f;
//    return &Chassis;
//}

// �������Լ�
int Joint_Zero_OK() {
    // ���������λ�Ƿ���(-180, 180)��Χ��
    // һ����˵ left_ID0 ��λ�����ܵ��� right_ID0 ��λ
    if ((zero_left_ID0  > -180 && zero_left_ID0  < 180) && zero_left_ID0 != 0 )
			{
        return 1;  // ���������λ���ڷ�Χ�ڣ��򷵻� true
			}
    return 0;  // ���򷵻� false
}

// �������ȡ (��ʼλ�� = �ϵ�λ��)
void Joint_Zero_init_Type1()
{
  // �����λ Ĭ��Ϊ1000��Ϊ��ѭ���ж�������ôд 
  // �����λ ������������
  // ʹ��whileѭ��ȷ��0λ��ȷ

  // �Լ첻ͨ�� D2����
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET); //

  while (Joint_Zero_OK() == False) {

      modify_torque_cmd(&MotorA1_send_left, 0, 0);    //modify_torque_cmd(&MotorA1_send_right, 0, 0);
      unitreeA1_rxtx(&huart1);               //unitreeA1_rxtx(&huart6);
      zero_left_ID0  = (float) MotorA1_recv_left_id00.Pos ;
      //zero_right_ID0 = (float) MotorA1_recv_right_id00.Pos ;
      //osDelay(2);

//      modify_torque_cmd(&MotorA1_send_left, 1, 0);    modify_torque_cmd(&MotorA1_send_right, 1, 0);
//      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
//      zero_left_ID1  = (float) MotorA1_recv_left_id01.Pos ;
//      zero_right_ID1 = (float) MotorA1_recv_right_id01.Pos ;
//      osDelay(2);
      
//      osDelay(20);
      }

  // �Լ�ɹ� ���Ϩ��
  modify_speed_cmd(&MotorA1_send_left,  0, 0);    
//  modify_speed_cmd(&MotorA1_send_right, 0, 0);
//  osDelay(1);
//  modify_speed_cmd(&MotorA1_send_left,  1, 0);    
//  modify_speed_cmd(&MotorA1_send_right, 1, 0);
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET); //

}

// �����λ��ȡ (��ʼλ�� = ��λλ��)
//void Joint_Zero_init_Type2()
//{
//// �Լ첻ͨ�� �������
//HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET); //

//while (Joint_Zero_OK() == False) {

//    modify_speed_cmd(&MotorA1_send_left,  0, -home_speed);    
//    modify_speed_cmd(&MotorA1_send_right, 0, +home_speed);
//    unitreeA1_rxtx(&huart1);                           unitreeA1_rxtx(&huart6);
//    if ((MotorA1_recv_left_id00.T)  <= -home_torque) {
//        zero_left_ID0  = (float) MotorA1_recv_left_id00.Pos + UP_LIMIT;} // zero_left_ID0 �Ǽ��ٺ�ĽǶ� (���ǻ���)
//    osDelay(1);
//    if ((MotorA1_recv_right_id00.T) >= +home_torque) {
//        zero_right_ID0 = (float) MotorA1_recv_right_id00.Pos - UP_LIMIT;}
//    osDelay(1);

//    modify_speed_cmd(&MotorA1_send_left,  1, +home_speed);    
//    modify_speed_cmd(&MotorA1_send_right, 1, -home_speed);
//    unitreeA1_rxtx(&huart1);                           unitreeA1_rxtx(&huart6);
//    if ((MotorA1_recv_left_id01.T)  >= +home_torque) {
//        zero_left_ID1  = (float) MotorA1_recv_left_id01.Pos - UP_LIMIT;}
//    osDelay(1);
//    if ((MotorA1_recv_right_id01.T) <= -home_torque) {
//        zero_right_ID1 = (float) MotorA1_recv_right_id01.Pos + UP_LIMIT;}
//    osDelay(1);
//    }
//  modify_speed_cmd(&MotorA1_send_left,  0, 0);    
//  modify_speed_cmd(&MotorA1_send_right, 0, 0);
//  osDelay(1);
//  modify_speed_cmd(&MotorA1_send_left,  1, 0);    
//  modify_speed_cmd(&MotorA1_send_right, 1, 0);
//  osDelay(1);
//  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET); //
//}

// �ع����
void Joint_GOTO_zero()
{
    modify_pos_cmd(&MotorA1_send_left,0,(float) zero_left_ID0, 0.006, 1.0);  // 0.005 0.5  
    unitreeA1_rxtx(&huart1); 

    HAL_Delay(2);
}

// ����Ƿ񳬹�����λ ��Ϊת���������⣬��ʱ��ʹ�øú���
//void Joint_Monitor()
//{   
//    if (((MotorA1_recv_left_id00.Pos  - zero_left_ID0)  <= -(UP_LIMIT+TOLERANCE) || (MotorA1_recv_left_id00.Pos - zero_left_ID0) >= +(DOWN_LIMIT+TOLERANCE)) && zero_left_ID0 != 0)
//        {STOP = True;}
//    if (((MotorA1_recv_left_id01.Pos  - zero_left_ID1)  >= +(UP_LIMIT+TOLERANCE) || (MotorA1_recv_left_id01.Pos - zero_left_ID1) <= -(DOWN_LIMIT+TOLERANCE)) && zero_left_ID1 != 0)
//        {STOP = True;}
//    if (((MotorA1_recv_right_id00.Pos - zero_right_ID0) >= +(UP_LIMIT+TOLERANCE) || (MotorA1_recv_right_id00.Pos - zero_right_ID0) <= -(DOWN_LIMIT+TOLERANCE)) && zero_right_ID0 != 0)
//        {STOP = True;}
//    if (((MotorA1_recv_right_id01.Pos - zero_right_ID1) <= -(UP_LIMIT+TOLERANCE) || (MotorA1_recv_right_id01.Pos - zero_right_ID1) >= +(DOWN_LIMIT+TOLERANCE)) && zero_right_ID1 != 0)
//        {STOP = True;}

//    if(STOP==True)
//      {HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12); 
//       osDelay(300);
//      } // �����˸
//}



// ���̹ؽڵ�� 4��ͬʱ����

/**
  * @brief          ���̹ؽ�λ�ÿ���
  * @param[in]      Pos_Front: ���ٺ�-�Ƕ��� ��ֵǰ�����ϰڶ�
  * @param[in]      Pos_Back: ���ٺ�-�Ƕ��� ��ֵ�������ϰڶ�
  */
void Joint_Position_Control(float Pos_Front,float kp,float kw)//, float Pos_Back
{   
    // �Ƕ� �޷�����
    LIMIT_RANGE(Pos_Front, -400, +400);
    //LIMIT_RANGE(Pos_Back,  -79, +19);
    modify_pos_cmd(&MotorA1_send_left,0, (float) Pos_Front + zero_left_ID0, kp, kw);  // 0.005 0.5     0.006 1.0
    //modify_pos_cmd(&MotorA1_send_right,0,(float) +Pos_Front + zero_right_ID0, 0.006,1.0); 
    unitreeA1_rxtx(&huart1); 
    //unitreeA1_rxtx(&huart6);
    //osDelay(1);
		//HAL_Delay(1);
	
//    modify_pos_cmd(&MotorA1_send_left,1, (float) +Pos_Back + zero_left_ID1, 0.006, 1.0);   
//    modify_pos_cmd(&MotorA1_send_right,1,(float) -Pos_Back + zero_right_ID1, 0.006, 1.0);
//    unitreeA1_rxtx(&huart1);
//    unitreeA1_rxtx(&huart6);
//    osDelay(1);
}


void Joint_PW_Control(float Pos_Front,float Omega,uint8_t id,float kp,float kw)//, float Pos_Back
{   
    // �Ƕ� �޷�����
    LIMIT_RANGE(Pos_Front, -400, +400);
    //LIMIT_RANGE(Pos_Back,  -79, +19);
	if (id == 0)
	{
    modify_PW_cmd(&MotorA1_send_left,id, (float) Pos_Front + zero_left_ID0,Omega, kp, kw);  // 0.005 0.5     0.006 1.0
	}
	else if (id == 1)
	{
		modify_PW_cmd(&MotorA1_send_left,id, (float) Pos_Front + zero_left_ID1,Omega, kp, kw); 
	}
		//modify_pos_cmd(&MotorA1_send_right,0,(float) +Pos_Front + zero_right_ID0, 0.006,1.0); 
    unitreeA1_rxtx(&huart1); 
    //unitreeA1_rxtx(&huart6);
    //osDelay(1);
		//HAL_Delay(1);
	
//    modify_pos_cmd(&MotorA1_send_left,1, (float) +Pos_Back + zero_left_ID1, 0.006, 1.0);   
//    modify_pos_cmd(&MotorA1_send_right,1,(float) -Pos_Back + zero_right_ID1, 0.006, 1.0);
//    unitreeA1_rxtx(&huart1);
//    unitreeA1_rxtx(&huart6);
//    osDelay(1);
}

//void Joint_Full_Position_Control(float Pos_Front_L, float Pos_Front_R, float Pos_Back_L, float Pos_Back_R)
//{
//    LIMIT_RANGE(Pos_Front_L, -79, +19);
//    LIMIT_RANGE(Pos_Front_R,  -79, +19);
//    LIMIT_RANGE(Pos_Back_L, -79, +19);
//    LIMIT_RANGE(Pos_Back_R,  -79, +19);
//    modify_pos_cmd(&MotorA1_send_left,0, (float) -Pos_Front_L + zero_left_ID0, 0.006, 1.0);  // 0.005 0.5  
//    modify_pos_cmd(&MotorA1_send_right,0,(float) +Pos_Front_R + zero_right_ID0, 0.006,1.0); 
//    unitreeA1_rxtx(&huart1); 
//    //unitreeA1_rxtx(&huart6);
//    //osDelay(1);
//    modify_pos_cmd(&MotorA1_send_left,1, (float) +Pos_Back_L + zero_left_ID1, 0.006, 1.0);   
//    modify_pos_cmd(&MotorA1_send_right,1,(float) -Pos_Back_R + zero_right_ID1, 0.006, 1.0);
//    unitreeA1_rxtx(&huart1);
//    //unitreeA1_rxtx(&huart6);
//    //osDelay(1);
//}


/**
  * @brief          ���̹ؽ��ٶȿ���
  * @param[in]      Speed_Front: ���ٺ�-�Ƕ��� ��ֵǰ�����°ڶ�
  * @param[in]      Speed_Back:  ���ٺ�-�Ƕ��� ��ֵ�������°ڶ�
  */
void Joint_Speed_Control(float Speed_Front)//, float Speed_Back
{
    modify_speed_cmd(&MotorA1_send_left,0, (float) +Speed_Front);  
//    modify_speed_cmd(&MotorA1_send_right,0,(float) -Speed_Front); 
    unitreeA1_rxtx(&huart1); 
		HAL_Delay(1);
//    unitreeA1_rxtx(&huart6);
//    osDelay(1);
//    modify_speed_cmd(&MotorA1_send_left,1, (float) -Speed_Back);   
//    modify_speed_cmd(&MotorA1_send_right,1,(float) +Speed_Back);
//    unitreeA1_rxtx(&huart1);
//    unitreeA1_rxtx(&huart6);
//    osDelay(1);
}


// // ��ؼ��
// uint8_t Joint_IsOn_Ground()
// {
//     if (MotorA1_recv_right_id01.T < 0 && MotorA1_recv_right_id00.T > 0 )
//     {
//         return 0;
//     }
//     if (MotorA1_recv_left_id00.T < 0 && MotorA1_recv_left_id01.T > 0 )
//     {
//         return 0;
//     }
//     return 1;
// }
