#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

#include "main.h"
//#include <stdbool.h>

// ����ʽPID�ṹ������
typedef struct {
    // PID����
    float kp;         // ����ϵ��
    float ki;         // ����ϵ�����ѳ��Բ���ʱ��Ts��
    float kd;         // ΢��ϵ�����ѳ��Բ���ʱ��Ts��
    
    // ƫ�����
    float e0;         // ��ǰƫ�� e(k)
    float e1;         // ��һʱ��ƫ�� e(k-1)
    float e2;         // ����ʱ��ƫ�� e(k-2)
    
    // �������
    float output_max; // ����������
    float output_min; // ��С�������
    
    // ���ַ�����ֵ����ѡ���ܣ�
    float integral_threshold; 
    int  integral_enable;    // ����ʹ�ܱ�־
    
    // ���������
    float output;     // ��ǰ��� u(k)
} IncrementalPID;

/**
 * @brief ��ʼ������ʽPID����
 * @param pid: PID�ṹ��ָ��
 * @param kp: ����ϵ��
 * @param ki: ����ϵ������Ԥ�ȳ��Բ���ʱ��Ts��
 * @param kd: ΢��ϵ������Ԥ�ȳ��Բ���ʱ��Ts��
 * @param output_max: ����������
 * @param output_min: ��С�������
 * @param integral_threshold: ���ַ�����ֵ��ƫ�����ֵʱ�رջ��֣�
 */
void incremental_pid_init(IncrementalPID *pid, float kp, float ki, float kd,
                          float output_max, float output_min, float integral_threshold);

/**
 * @brief ����PID�������������ʷƫ��������
 * @param pid: PID�ṹ��ָ��
 */
void incremental_pid_reset(IncrementalPID *pid);

/**
 * @brief ��������ʽPID���
 * @param pid: PID�ṹ��ָ��
 * @param target: Ŀ��ֵ���趨ֵ��
 * @param feedback: ����ֵ��ʵ�ʲ���ֵ��
 * @return ��ǰ�������
 */
float incremental_pid_calc(IncrementalPID *pid, float target, float feedback);

#endif // INCREMENTAL_PID_H
