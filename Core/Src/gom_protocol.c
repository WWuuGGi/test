#include "gom_protocol.h"
#include <string.h>

#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}
	
void go_pos_cmd(MotorCmd_t *cmd,uint8_t id,float pos ,float KP, float KW)
{
	cmd->id = id;
	cmd->mode = 1;
	cmd->K_P = KP;
	cmd->K_W = KW;
	cmd->Pos = pos*6.33f;//������
	cmd->W = 0;
	cmd->T = 0;
}
void go_spd_cmd(MotorCmd_t *cmd,uint8_t id,float spd)
{
	cmd->id = id;
	cmd->mode = 1;
	cmd->K_P = 0.0f;
	cmd->K_W = 0.03f;
	cmd->Pos = 0.0f;
	cmd->W = spd*6.33f;//������
	cmd->T = 0;
}
void go_torque_cmd(MotorCmd_t *cmd,uint8_t id,float torque)
{
	cmd->id = id;
	cmd->mode = 1;
	cmd->K_P = 0.0f;
	cmd->K_W = 0.0f;
	cmd->Pos = 0.0f;
	cmd->W = 0.0f;
	cmd->T = torque;
}
	

/// @brief �����͸�����ĸ������ת��Ϊ�������Ͳ���
/// @param motor_s Ҫת���ĵ��ָ��ṹ��
void modify_data(MotorCmd_t *motor_s)
{
	motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	SATURATE(motor_s->id, 0, 15);
	SATURATE(motor_s->mode, 0, 7);
	SATURATE(motor_s->K_P, 0.0f, 25.599f);
	SATURATE(motor_s->K_W, 0.0f, 25.599f);
	SATURATE(motor_s->T, -127.99f, 127.99f);
	SATURATE(motor_s->W, -804.00f, 804.00f);
	SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

	motor_s->motor_send_data.mode.id = motor_s->id;
	motor_s->motor_send_data.mode.status = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.28318f * 32768.0f;
	motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.28318f * 256.0f;
	motor_s->motor_send_data.comd.tor_des = motor_s->T * 256.0f;
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, sizeof(RIS_ControlData_t) - sizeof(motor_s->motor_send_data.CRC16));
}

/// @brief �����յ��Ķ�������ԭʼ����ת��Ϊ�����������
/// @param motor_r Ҫת���ĵ�������ṹ��
void extract_data(MotorData_t *motor_r)
{
	if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}
	motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, sizeof(RIS_MotorData_t) - sizeof(motor_r->motor_recv_data.CRC16));
	if (motor_r->motor_recv_data.CRC16 != motor_r->calc_crc)
	{
		memset(&motor_r->motor_recv_data, 0, sizeof(RIS_MotorData_t));
		motor_r->correct = 0;
		motor_r->bad_msg++;
		return;
	}
	else
	{
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256.0f) * 6.28318f;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256.0f;
		motor_r->Pos = 6.28318f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		return;
	}
}
