#include <stdlib.h>
#include <stdio.h>
#include "joint.h"


#define LIMIT_RANGE(Pos, Min, Max) ((Pos) = ((Pos) > (Max) ? (Max) : (Pos) < (Min) ? (Min) : (Pos)))

// Ĭ�ϵ����ʼ���
//float zero_left_ID0  = 0.0f;
//float zero_left_ID1  = 0.0f;
//float zero_right_ID0 = 0.0f;
//float zero_right_ID1 = 0.0f;

// ��ʼ��4����������
float zero_group1_ID0 = 0.0f;
float zero_group1_ID1 = 0.0f;
float zero_group1_ID2 = 0.0f;
float zero_group2_ID0 = 0.0f;
float zero_group2_ID1 = 0.0f;
float zero_group2_ID2 = 0.0f;
float zero_group3_ID0 = 0.0f;
float zero_group3_ID1 = 0.0f;


uint8_t STOP = False;

//static float home_speed  = 0.4f;  // ���ٺ���ٶ� rad/s
//static float home_torque = 1.0f;  // ���ٺ����� Nm
//static float UP_LIMIT    = 20.0f; // ���ٺ�Ƕ� ��
//static float DOWN_LIMIT  = 80.0f; // ���ٺ�Ƕ� ��
//static float TOLERANCE   = -5.0f;  // �ݲ� ��

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
    
    if (fabsf(zero_group1_ID0) <= 1e-6f || fabsf(zero_group1_ID1) <= 1e-6f // fabsf(zero_group1_ID2) <= 1e-6f 
			||	fabsf(zero_group2_ID0) <= 1e-6f || fabsf(zero_group2_ID1) <= 1e-6f || fabsf(zero_group2_ID2) <= 1e-6f
		||	fabsf(zero_group3_ID0) <= 1e-6f || fabsf(zero_group3_ID1) <= 1e-6f )
			{
        return 0;  //����λû�б����ã�����false
			}
    return 1;  // ���򷵻� true
}

// �������ȡ (��ʼλ�� = �ϵ�λ��)
void Joint_Zero_init_Type1()
{
  // �����λ Ĭ��Ϊ1000��Ϊ��ѭ���ж�������ôд 
  // �����λ ������������
  // ʹ��whileѭ��ȷ��0λ��ȷ

  while (!Joint_Zero_OK()) 
		{
		//group1
			// ��ȡID0���
			modify_torque_cmd(&MotorA1_send_group1, 0, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID0 = MotorA1_recv_group1_id0.Pos;
			
			HAL_Delay(5);
			
			// ��ȡID1���
			modify_torque_cmd(&MotorA1_send_group1, 1, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID1 = MotorA1_recv_group1_id1.Pos;
			
			HAL_Delay(5);
			
			modify_torque_cmd(&MotorA1_send_group1, 2, 0.0f);
			unitreeA1_rxtx(&huart1, 1);
			zero_group1_ID2 = MotorA1_recv_group1_id2.Pos;
			
			HAL_Delay(5);
			
			//group2
			// ��ȡID0���
			modify_torque_cmd(&MotorA1_send_group2, 0, 0.0f);
			unitreeA1_rxtx(&huart1, 2);
			zero_group2_ID0 = MotorA1_recv_group2_id0.Pos;
			
			HAL_Delay(5);
			
			// ��ȡID1���
			modify_torque_cmd(&MotorA1_send_group2, 1, 0.0f);
			unitreeA1_rxtx(&huart1, 2);
			zero_group2_ID1 = MotorA1_recv_group2_id1.Pos;
			
			HAL_Delay(5);
			
			modify_torque_cmd(&MotorA1_send_group2, 2, 0.0f);
			unitreeA1_rxtx(&huart1, 2);
			zero_group2_ID2 = MotorA1_recv_group2_id2.Pos;

			HAL_Delay(5);

			//group3
			// ��ȡID0���
			go_torque_cmd(&Motor_go_send_group3,0,0.0f);
			unitreeA1_rxtx(&huart6,3);
			zero_group3_ID0 = Motor_go_recv_group3_id0.Pos;
			
			HAL_Delay(5);

			// ��ȡID1���
			go_torque_cmd(&Motor_go_send_group3,1,0.0f);
			unitreeA1_rxtx(&huart6,3);
			zero_group3_ID1 = Motor_go_recv_group3_id1.Pos;
			
			HAL_Delay(5);
	}
}


//// �ع����
//void Joint_GOTO_zero()
//{
//    modify_pos_cmd(&MotorA1_send_left,0,(float) zero_left_ID0, 0.006, 1.0);  // 0.005 0.5  
//    unitreeA1_rxtx(&huart1); 

//    HAL_Delay(2);
//}

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

/**
  * @brief          ���̹ؽ�λ�ÿ���
  * @param[in]      Pos_Front: ���ٺ�-�Ƕ��� ��ֵǰ�����ϰڶ�
  * @param[in]      Pos_Back: ���ٺ�-�Ƕ��� ��ֵ�������ϰڶ�
  */
void Joint_Position_Control(uint8_t group, uint8_t id, float Pos[][STEP_NUM], float kp, float kw, uint16_t step)//, float Pos_Back
{   
    // �Ƕ� �޷�����
    //LIMIT_RANGE(Pos_Front, -400, +400);
    //LIMIT_RANGE(Pos_Back,  -79, +19);
	
		float target_pos = 0.0f;
		motor_send_t *send_struct = NULL;
		UART_HandleTypeDef *huart = NULL;
		MotorCmd_t *send_struct_go = NULL;
	
		// �����Ӧ�ķ��ͽṹ��ʹ���
    switch(group) {
        case 1: send_struct = &MotorA1_send_group1; huart = &huart1; break;
        case 2: send_struct = &MotorA1_send_group2; huart = &huart1; break;
				case 3: send_struct_go = &Motor_go_send_group3; huart = &huart6; break;
        //case 3: send_struct = &MotorA1_send_group3; huart = &huart3; break;
        //case 4: send_struct = &MotorA1_send_group4; huart = &huart6; break;
        default: return;
    }
		
// ����Ŀ��λ�ã�������㣩
    if (id == 0) {
        switch(group) {
            case 1: target_pos = Pos[7][step] + zero_group1_ID0; 

										break;
            case 2: target_pos = Pos[5][step] + zero_group2_ID0;

										break;
            case 3: target_pos = -1.0f * Pos[3][step] + zero_group3_ID0; 

										break;
//            case 4: target_pos = Pos[6][step] + zero_group4_ID0; 
//										target_spd = Omega[6][step];
//										break;
        }
    } else if (id == 1){
        switch(group) {
            case 1: target_pos = -1.0f * Pos[6][step] + zero_group1_ID1; 

										break;
            case 2: target_pos = -1.0f * Pos[4][step] + zero_group2_ID1; 

										break;
            case 3: target_pos = Pos[0][step] + zero_group3_ID1; 

										break;
				}
//            case 4: target_pos = Pos[7][step] + zero_group4_ID1; 
//										target_spd = Omega[7][step];
//										break;
		} else if (id == 2){
				switch(group) {
						case 1: target_pos = Pos[2][step] + zero_group1_ID2; 

										break;
            case 2: target_pos = -1.0f * Pos[1][step] + zero_group2_ID2; 

										break;
			}
    }
		
		if(group == 3)
		{
				go_pos_cmd(send_struct_go,id,target_pos,kp,kw);
		}
		else {
				modify_pos_cmd(send_struct, id, target_pos, kp, kw);
    }
			unitreeA1_rxtx(huart, group);

}


void Joint_PW_Control(uint8_t group, uint8_t id,float Pos[][STEP_NUM],float Omega[][STEP_NUM],float kp,float kw,uint16_t step)//, float Pos_Back
{   
		float target_pos = 0.0f;
		float target_spd = 0.0f;
		motor_send_t *send_struct = NULL;
		UART_HandleTypeDef *huart = NULL;
		MotorCmd_t *send_struct_go = NULL;
		
	
		// �����Ӧ�ķ��ͽṹ��ʹ���
    switch(group) {
        case 1: send_struct = &MotorA1_send_group1; huart = &huart1; break;
        case 2: send_struct = &MotorA1_send_group2; huart = &huart1; break;
        case 3: send_struct_go = &Motor_go_send_group3; huart = &huart6; break;
        //case 4: send_struct = &MotorA1_send_group4; huart = &huart6; break;
        default: return;
    }
//b1 - G3 ID 1
//b2 - G2 ID 2
//b3 - G1 ID 2
//b4 - G3 ID 0
//b5 - G2 ID 1
//b6 - G2 ID 0 
//b7 - G1 ID 1
//b8 - G1 ID 0 
		// ����Ŀ��λ�ã�������㣩
    if (id == 0) {
        switch(group) {
            case 1: target_pos = +1.0f * Pos[7][step] + zero_group1_ID0; 
										target_spd = Omega[7][step];
										break;
            case 2: target_pos = +1.0f * Pos[5][step] + zero_group2_ID0;
										target_spd = Omega[5][step];
										break;
            case 3: target_pos = -1.0f * Pos[3][step] + zero_group3_ID0; 
										target_spd = -1.0f * Omega[3][step];
										break;
//            case 4: target_pos = Pos[6][step] + zero_group4_ID0; 
//										target_spd = Omega[6][step];
//										break;
        }
    } else if (id == 1){
        switch(group) {
            case 1: target_pos = -1.0f * Pos[6][step] + zero_group1_ID1; 
										target_spd = -1.0f * Omega[6][step];
										break;
            case 2: target_pos = -1.0f * Pos[4][step] + zero_group2_ID1; 
										target_spd = -1.0f * Omega[4][step];
										break;
            case 3: target_pos = +1.0f * Pos[0][step] + zero_group3_ID1; 
										target_spd = Omega[0][step];
										break;
				}
//            case 4: target_pos = Pos[7][step] + zero_group4_ID1; 
//										target_spd = Omega[7][step];
//										break;
		} else if (id == 2){
				switch(group) {
						case 1: target_pos = +1.0f * Pos[2][step] + zero_group1_ID2; 
										target_spd = Omega[2][step];
										break;
            case 2: target_pos = -1.0f * Pos[1][step] + zero_group2_ID2; 
										target_spd = -1.0f *Omega[1][step];
										break;
			}
    }
		
		if(group == 3)
		{
				go_pw_cmd(send_struct_go,id,target_pos,target_spd,kp,kw);
		}
		else {
				modify_PW_cmd(send_struct, id, target_pos,target_spd, kp, kw);
    }
			unitreeA1_rxtx(huart, group);
}

void Joint_Full_Position_Control(uint16_t step)
{
	Joint_Position_Control(1, 0, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(1, 1, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(1, 2, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(2, 0, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(2, 1, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(2, 2, motor_angle, 0.022f, 0.1f, step);
	Joint_Position_Control(3, 0, motor_angle, 0.20f, 0.001f, step);
	Joint_Position_Control(3, 1, motor_angle, 0.20f, 0.001f, step);
	
}

void Joint_Full_PW_Control(uint16_t step)
{
	Joint_PW_Control(1, 0, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(1, 1, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(1, 2, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(2, 0, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(2, 1, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(2, 2, motor_angle, motor_omega, 0.022f, 0.1f, step);
	//	Joint_PW_Control(3, 0, motor_angle, motor_omega, 0.022f, 0.1f, step);
//	Joint_PW_Control(3, 1, motor_angle, motor_omega, 0.022f, 0.1f, step);
	Joint_PW_Control(3, 0, motor_angle, motor_omega, 0.21f, 0.001f, step);
	Joint_PW_Control(3, 1, motor_angle, motor_omega, 0.21f, 0.001f, step);
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
