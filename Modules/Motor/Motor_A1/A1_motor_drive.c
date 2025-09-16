#include "A1_motor_drive.h"

#define PI 3.14159

//motor_send_t MotorA1_send_left;       // 左腿一号电机数据体
//motor_send_t MotorA1_send_right;      // 右腿一号电机数据体

//motor_recv_t Date_left;               // 左腿电机接收数据体
//motor_recv_t MotorA1_recv_left_id00;  // 左腿00号电机接收数据体
//motor_recv_t MotorA1_recv_left_id01;  // 左腿01号电机接收数据体
//motor_recv_t MotorA1_recv_left_id02;  // 左腿02号电机接收数据体

//motor_recv_t Date_right;              // 右腿电机接收数据体
//motor_recv_t MotorA1_recv_right_id00; // 右腿00号电机接收数据体
//motor_recv_t MotorA1_recv_right_id01; // 右腿01号电机接收数据体
//motor_recv_t MotorA1_recv_right_id02; // 右腿02号电机接收数据体
//uint8_t Date[78];

// 4组电机发送结构体初始化
motor_send_t MotorA1_send_group1 = {0};
motor_send_t MotorA1_send_group2 = {0};
//motor_send_t MotorA1_send_group3 = {0};
//motor_send_t MotorA1_send_group4 = {0};

// 第四组替换为go_protocol结构体
MotorCmd_t Motor_go_send_group3 = {0};       // 发送命令结构体
MotorData_t Motor_go_recv_group3_id0 = {0};  // ID0接收数据
MotorData_t Motor_go_recv_group3_id1 = {0};  // ID1接收数据

// 4组电机接收结构体初始化
motor_recv_t MotorA1_recv_group1_id0 = {0};
motor_recv_t MotorA1_recv_group1_id1 = {0};
motor_recv_t MotorA1_recv_group1_id2 = {0};
motor_recv_t MotorA1_recv_group2_id0 = {0};
motor_recv_t MotorA1_recv_group2_id1 = {0};
motor_recv_t MotorA1_recv_group2_id2 = {0};
//motor_recv_t MotorA1_recv_group3_id0 = {0};
//motor_recv_t MotorA1_recv_group3_id1 = {0};
//motor_recv_t MotorA1_recv_group4_id0 = {0};
//motor_recv_t MotorA1_recv_group4_id1 = {0};

// 接收缓冲区
uint8_t Date_group1[78] = {0};
uint8_t Date_group2[78] = {0};
//uint8_t Date_group3[78] = {0};
//uint8_t Date_group4[78] = {0};
uint8_t Date_go_group3[16] = {0};  // go协议接收缓冲区（16字节）

// 通信状态
HAL_StatusTypeDef rec_st[3] = {HAL_OK};
HAL_StatusTypeDef trans_st[3] = {HAL_OK};
uint32_t err_state;
uint32_t received;

// CRC校验位的代码
uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// 电机位置修改
void modify_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	  send->id   = id;

    send->Pos  = 2*PI/360*9.1*Pos;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
	  //输入的pos是经过减速器的，乘以9.1之后是输出轴的
	  //这样计算之后，输入函数的pos就是角度值的了，转换成未减速的弧度
    send->W    = 0;
    send->T    = 0.0;
    send->K_P  = KP;
    send->K_W  = KW;
}

void modify_changeid_cmd(motor_send_t *send,uint8_t mode)
{

    send->hex_len = 34;

    send->mode = mode;
	  send->id   = 0xBB;

    send->Pos  = 0.0;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
	  //输入的pos是经过减速器的，乘以9.1之后是输出轴的
	  //这样计算之后，输入函数的pos就是角度值的了，转换成未减速的弧度
    send->W    = 0.0;
    send->T    = 0.0;
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}


// 电机速度修改
void modify_speed_cmd(motor_send_t *send,uint8_t id, float Omega)
{

    send->hex_len = 34;

    send->mode = 10;
		send->id   = id;

    send->Pos  = 0;
    send->W    = Omega * 9.1f;
    send->T    = 0.0;
    send->K_P  = 0.0;
    send->K_W  = 3.0;
}

// 电机力矩修改
void modify_torque_cmd(motor_send_t *send,uint8_t id, float torque)
{

    send->hex_len = 34;

    send->mode = 10;
	  send->id   = id;

    send->Pos  = 0.0;
    send->W    = 0.0;
    if (torque > 10.0f){torque = 0.0f;} // 限幅
    send->T    = torque / 9.1f;//减速输出端的力矩，转换成电机轴的力矩
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}

void modify_PW_cmd(motor_send_t *send,uint8_t id, float Pos, float Omega, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	  send->id   = id;

    send->Pos  = 2*PI/360*9.1*Pos;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
	  //输入的pos是经过减速器的，乘以9.1之后是输出轴的
	  //这样计算之后，输入函数的pos就是角度值的了，转换成未减速的弧度
    send->W    = Omega;
    send->T    = 0.0;
    send->K_P  = KP;
    send->K_W  = KW;
}

// 电机发送接收函数
void unitreeA1_rxtx(UART_HandleTypeDef *huart, uint8_t group)
{
        //uint8_t A1MotorA1_send_left[34]; // 发送数据
	if (group != 3)
	{
				uint8_t *send_buf = NULL;
				uint8_t *recv_buf = NULL;
				motor_send_t *send_struct = NULL;
				motor_recv_t *recv_id0 = NULL;
				motor_recv_t *recv_id1 = NULL;
				motor_recv_t *recv_id2 = NULL;

					// 根据组别绑定缓冲区和结构体
				switch(group) {
						case 1:
								send_struct = &MotorA1_send_group1;
								send_buf = (uint8_t*)&MotorA1_send_group1.motor_send_data;
								recv_buf = Date_group1;
								recv_id0 = &MotorA1_recv_group1_id0;
								recv_id1 = &MotorA1_recv_group1_id1;
								recv_id2 = &MotorA1_recv_group1_id2;
								break;
						case 2:
								send_struct = &MotorA1_send_group2;
								send_buf = (uint8_t*)&MotorA1_send_group2.motor_send_data;
								recv_buf = Date_group2;
								recv_id0 = &MotorA1_recv_group2_id0;
								recv_id1 = &MotorA1_recv_group2_id1;
								recv_id2 = &MotorA1_recv_group2_id2;
								break;
						default: return;
				}

        // 此处为左腿电机结构体//
        send_struct->motor_send_data.head.start[0] = 0xFE;
        send_struct->motor_send_data.head.start[1] = 0xEE;
        send_struct->motor_send_data.head.motorID  = send_struct->id;
        send_struct->motor_send_data.head.reserved = 0x00;

        send_struct->motor_send_data.Mdata.mode      = send_struct->mode;  // mode = 10
        send_struct->motor_send_data.Mdata.ModifyBit = 0xFF;
        send_struct->motor_send_data.Mdata.ReadBit   = 0x00;
        send_struct->motor_send_data.Mdata.reserved  = 0x00;
        send_struct->motor_send_data.Mdata.Modify.F  = 0;
        send_struct->motor_send_data.Mdata.T         = send_struct->T * 256;
        send_struct->motor_send_data.Mdata.W         = send_struct->W * 128;
        send_struct->motor_send_data.Mdata.Pos       = (int)((send_struct->Pos / 6.2832f) * 16384.0f); // 单位 rad
			//Mdata里存的是期望为减速弧度的16384/2/pi描述
        send_struct->motor_send_data.Mdata.K_P       = send_struct->K_P * 2048;
        send_struct->motor_send_data.Mdata.K_W       = send_struct->K_W * 1024;

        send_struct->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        send_struct->motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        send_struct->motor_send_data.Mdata.Res[0] = send_struct->Res;

        send_struct->motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&send_struct->motor_send_data), 7); // CRC校验

//        memcpy(A1MotorA1_send_left, &send_struct->motor_send_data, 34);
				
				// 发送与接收
				switch(group)	//使能
				{
					case 1:
						HAL_GPIO_WritePin(GROUP_PORT_1, GROUP_PIN_1, GPIO_PIN_SET);  // 使能发送（根据硬件调整引脚）
						HAL_GPIO_WritePin(GROUP_PORT_2, GROUP_PIN_2, GPIO_PIN_RESET);
						break;
					case 2:
						HAL_GPIO_WritePin(GROUP_PORT_2, GROUP_PIN_2, GPIO_PIN_SET);  // 使能发送（根据硬件调整引脚）
						HAL_GPIO_WritePin(GROUP_PORT_1, GROUP_PIN_1, GPIO_PIN_RESET);
						break;
				}
				
				trans_st[group-1] = HAL_UART_Transmit(huart, send_buf, 34, 1);
				
				switch(group)	//使能
				{
					case 1:
						HAL_GPIO_WritePin(GROUP_PORT_1, GROUP_PIN_1, GPIO_PIN_RESET);  // 使能发送（根据硬件调整引脚）
						HAL_GPIO_WritePin(GROUP_PORT_2, GROUP_PIN_2, GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(GROUP_PORT_2, GROUP_PIN_2, GPIO_PIN_RESET);  // 使能发送（根据硬件调整引脚）
						HAL_GPIO_WritePin(GROUP_PORT_1, GROUP_PIN_1, GPIO_PIN_SET);
						break;
				}
				
				rec_st[group-1] = HAL_UART_Receive(huart, recv_buf, 78, 1);
				
				err_state = HAL_UART_GetError(huart);
				received = 78 - huart->RxXferCount;

				// 解析接收数据（根据ID存入对应结构体）
				motor_recv_t temp_recv;
				temp_recv.motor_recv_data.head.motorID = recv_buf[2];
				temp_recv.motor_recv_data.Mdata.mode = recv_buf[4];
				temp_recv.motor_recv_data.Mdata.Temp = recv_buf[6];
				temp_recv.motor_recv_data.Mdata.MError = recv_buf[7];
				temp_recv.motor_recv_data.Mdata.T = recv_buf[13] << 8 | recv_buf[12];
				temp_recv.motor_recv_data.Mdata.W = recv_buf[15] << 8 | recv_buf[14];
				temp_recv.motor_recv_data.Mdata.Acc = recv_buf[27] << 8 | recv_buf[26];
				temp_recv.motor_recv_data.Mdata.Pos = recv_buf[33] << 24 | recv_buf[32] << 16 | recv_buf[31] << 8 | recv_buf[30];
				// 转换物理量（与原有逻辑一致）
				temp_recv.motor_id = temp_recv.motor_recv_data.head.motorID;
				temp_recv.mode = temp_recv.motor_recv_data.Mdata.mode;
				temp_recv.Temp = temp_recv.motor_recv_data.Mdata.Temp;
				temp_recv.MError = temp_recv.motor_recv_data.Mdata.MError;
				temp_recv.T = (float)temp_recv.motor_recv_data.Mdata.T / 256 * 9.1f;  // 减速后扭矩
				temp_recv.W = (float)temp_recv.motor_recv_data.Mdata.W / 128 / 9.1f;  // 减速后角速度
				temp_recv.Pos = (float)temp_recv.motor_recv_data.Mdata.Pos / (16384.0f/2.0f/PI) * (180/PI/9.1f);  // 减速后角度
				temp_recv.Acc = temp_recv.motor_recv_data.Mdata.Acc;

				// 根据ID存入对应组的ID0/ID1结构体
				if (temp_recv.motor_id == 0) {
						*recv_id0 = temp_recv;
				} else if (temp_recv.motor_id == 1) {
						*recv_id1 = temp_recv;
				} else if (temp_recv.motor_id == 2) {
					  *recv_id2 = temp_recv;
				}
	}
	else if (group == 3)
	{
			MotorCmd_t *go_send = &Motor_go_send_group3;
			MotorData_t *recv_id0 = &Motor_go_recv_group3_id0;
			MotorData_t *recv_id1 = &Motor_go_recv_group3_id1;
			MotorData_t temp;
			uint8_t *send_buf = (uint8_t*)&go_send->motor_send_data;  // go协议发送缓冲区
			uint8_t *recv_buf = Date_go_group3;                      // go协议接收缓冲区
			
			// 1. 处理发送数据（使用go_protocol的modify_data）
			modify_data(go_send);  // 自动填充包头、转换物理量、计算CRC

			// 2. 硬件使能控制（与原有group4一致）
			HAL_GPIO_WritePin(GROUP_PORT_3, GROUP_PIN_3, GPIO_PIN_SET);
			// 发送go协议数据包（RIS_ControlData_t为17字节）
			trans_st[2] = HAL_UART_Transmit(huart, send_buf, sizeof(RIS_ControlData_t), 1);
			HAL_GPIO_WritePin(GROUP_PORT_3, GROUP_PIN_3, GPIO_PIN_RESET);

			// 3. 接收go协议数据（RIS_MotorData_t为16字节）
			rec_st[2] = HAL_UART_Receive(huart, (uint8_t *)&temp.motor_recv_data, sizeof(temp.motor_recv_data), 1);

			// 4. 解析接收数据（使用go_protocol的extract_data）
			// 先将接收缓冲区数据拷贝到MotorData_t的接收结构体
		
			extract_data(&temp);  // 自动校验包头、CRC，解析物理量

			// 若接收的是ID1的数据，同步更新到recv_id1
			if (temp.motor_id == 1) {
					*recv_id1 = temp;
		  }
			else
			{
					*recv_id0 = temp;
			}
	}
	
				
}


void motor_relax(void)
{
	modify_torque_cmd(&MotorA1_send_group1,0, 0.0f);
	unitreeA1_rxtx(&huart1,1);
	modify_torque_cmd(&MotorA1_send_group1,1, 0.0f);
	unitreeA1_rxtx(&huart1,1);
	modify_torque_cmd(&MotorA1_send_group1,2, 0.0f);
	unitreeA1_rxtx(&huart1,1);
	modify_torque_cmd(&MotorA1_send_group2,0, 0.0f);
	unitreeA1_rxtx(&huart1,2);
	modify_torque_cmd(&MotorA1_send_group2,1, 0.0f);
	unitreeA1_rxtx(&huart1,2);
	modify_torque_cmd(&MotorA1_send_group2,2, 0.0f);
	unitreeA1_rxtx(&huart1,2);
	go_torque_cmd(&Motor_go_send_group3,0,0.0f);
	unitreeA1_rxtx(&huart6,3);
	go_torque_cmd(&Motor_go_send_group3,1,0.0f);
	unitreeA1_rxtx(&huart6,3);

}


