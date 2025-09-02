#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

#include "main.h"
//#include <stdbool.h>

// 增量式PID结构体声明
typedef struct {
    // PID参数
    float kp;         // 比例系数
    float ki;         // 积分系数（已乘以采样时间Ts）
    float kd;         // 微分系数（已除以采样时间Ts）
    
    // 偏差变量
    float e0;         // 当前偏差 e(k)
    float e1;         // 上一时刻偏差 e(k-1)
    float e2;         // 上两时刻偏差 e(k-2)
    
    // 输出限制
    float output_max; // 最大输出限制
    float output_min; // 最小输出限制
    
    // 积分分离阈值（可选功能）
    float integral_threshold; 
    int  integral_enable;    // 积分使能标志
    
    // 控制器输出
    float output;     // 当前输出 u(k)
} IncrementalPID;

/**
 * @brief 初始化增量式PID参数
 * @param pid: PID结构体指针
 * @param kp: 比例系数
 * @param ki: 积分系数（需预先乘以采样时间Ts）
 * @param kd: 微分系数（需预先除以采样时间Ts）
 * @param output_max: 最大输出限制
 * @param output_min: 最小输出限制
 * @param integral_threshold: 积分分离阈值（偏差超过此值时关闭积分）
 */
void incremental_pid_init(IncrementalPID *pid, float kp, float ki, float kd,
                          float output_max, float output_min, float integral_threshold);

/**
 * @brief 重置PID控制器（清空历史偏差和输出）
 * @param pid: PID结构体指针
 */
void incremental_pid_reset(IncrementalPID *pid);

/**
 * @brief 计算增量式PID输出
 * @param pid: PID结构体指针
 * @param target: 目标值（设定值）
 * @param feedback: 反馈值（实际测量值）
 * @return 当前控制输出
 */
float incremental_pid_calc(IncrementalPID *pid, float target, float feedback);

#endif // INCREMENTAL_PID_H
