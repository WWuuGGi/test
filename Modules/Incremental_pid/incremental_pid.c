#include "incremental_pid.h"

// 增量式PID结构体定义
//typedef struct {
//    // PID参数
//    float kp;         // 比例系数
//    float ki;         // 积分系数（已乘以采样时间Ts）
//    float kd;         // 微分系数（已除以采样时间Ts）
//    
//    // 偏差变量
//    float e0;         // 当前偏差 e(k)
//    float e1;         // 上一时刻偏差 e(k-1)
//    float e2;         // 上两时刻偏差 e(k-2)
//    
//    // 输出限制
//    float output_max; // 最大输出限制
//    float output_min; // 最小输出限制
//    
//    // 积分分离阈值（可选功能）
//    float integral_threshold; 
//    bool  integral_enable;    // 积分使能标志
//    
//    // 控制器输出
//    float output;     // 当前输出 u(k)
//} IncrementalPID;

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
                          float output_max, float output_min, float integral_threshold) {
    if (pid == NULL) return;
    
    // 初始化参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // 初始化偏差
    pid->e0 = 0.0f;
    pid->e1 = 0.0f;
    pid->e2 = 0.0f;
    
    // 初始化输出限制
    pid->output_max = output_max;
    pid->output_min = output_min;
    
    // 初始化积分分离
    pid->integral_threshold = integral_threshold;
    pid->integral_enable = 1;
    
    // 初始化输出
    pid->output = 0.0f;
}

/**
 * @brief 重置PID控制器（清空历史偏差和输出）
 * @param pid: PID结构体指针
 */
void incremental_pid_reset(IncrementalPID *pid) {
    if (pid == NULL) return;
    
    pid->e0 = 0.0f;
    pid->e1 = 0.0f;
    pid->e2 = 0.0f;
    pid->output = 0.0f;
    pid->integral_enable = 1;
}

/**
 * @brief 计算增量式PID输出
 * @param pid: PID结构体指针
 * @param target: 目标值（设定值）
 * @param feedback: 反馈值（实际测量值）
 * @return 当前控制输出
 */
float incremental_pid_calc(IncrementalPID *pid, float target, float feedback) {
    if (pid == NULL) return 0.0f;
    
    // 1. 计算当前偏差 e(k) = 目标值 - 反馈值
    pid->e0 = target - feedback;
    
    // 2. 积分分离逻辑（偏差过小时才启用积分）
    if (pid->integral_threshold > 0) {
        pid->integral_enable = (pid->e0 < pid->integral_threshold) && 
                              (pid->e0 > -pid->integral_threshold);
    }
    
    // 3. 计算增量式PID的控制增量 Δu(k)
    float delta_u = 0.0f;
    delta_u += pid->kp * (pid->e0 - pid->e1);               // 比例部分
    delta_u += (pid->integral_enable ? pid->ki * pid->e0 : 0); // 积分部分（带使能）
    delta_u += pid->kd * (pid->e0 - 2 * pid->e1 + pid->e2);  // 微分部分
    
    // 4. 计算当前输出 u(k) = u(k-1) + Δu(k)
    pid->output += delta_u;
    
    // 5. 输出限幅（防止执行器超限）
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // 6. 更新偏差历史（为下次计算做准备）
    pid->e2 = pid->e1;
    pid->e1 = pid->e0;
    
    return pid->output;
}

// 示例使用
#ifdef PID_EXAMPLE
int main(void) {
    // 1. 定义并初始化PID控制器
    IncrementalPID pid;
    // 采样时间Ts=0.01s（10ms），则：
    // ki实际值 = 设定值 * Ts，kd实际值 = 设定值 / Ts
    incremental_pid_init(&pid, 
                       2.0f,   // kp
                       0.5f,   // ki（已乘以Ts=0.01）
                       0.1f,   // kd（已除以Ts=0.01）
                       100.0f, // 最大输出
                       -100.0f,// 最小输出
                       5.0f);  // 积分分离阈值
    
    // 2. 模拟控制循环（实际应在定时器中断中执行）
    float target = 50.0f;    // 目标值
    float feedback = 0.0f;   // 反馈值（实际应从传感器读取）
    float output = 0.0f;     // 控制输出
    
    while (1) {
        // 模拟传感器反馈（实际应替换为真实采样）
        feedback += output * 0.01f;  // 简化模型：输出影响反馈
        
        // 计算PID输出
        output = incremental_pid_calc(&pid, target, feedback);
        
        // 此处添加输出到执行器的代码
        // ...
        
        // 延时模拟采样周期（实际应使用定时器）
        delay_ms(10);  // 10ms采样周期
    }
    
    return 0;
}
#endif
