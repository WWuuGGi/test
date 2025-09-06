#ifndef __CALC_H
#define __CALC_H

#include "stm32f4xx_hal.h"
#include <math.h>

// 数据类型定义
typedef float float32_t;

/**
 * @brief 四维点结构体（含齐次坐标）
 * 用于表示基座锚点和末端执行器附着点坐标
 */
typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
    float32_t w; // 齐次坐标，通常为1.0
} Point4f;

/**
 * @brief 4x4矩阵结构体
 * 用于坐标变换和旋转计算
 */
typedef struct {
    float32_t data[4][4];
} Matrix4f;

/**
 * @brief 位姿结构体（位置+姿态）
 * data[0-2]: x,y,z位置坐标
 * data[3-5]: alpha,beta,gamma姿态角（旋转角度）
 */
typedef struct {
    float32_t data[6]; // x,y,z,alpha,beta,gamma
} Pose;

/**
 * @brief 速度结构体（线速度+角速度）
 * data[0-2]: vx,vy,vz线速度
 * data[3-5]: va,vb,vg角速度
 */
typedef struct {
    float32_t data[6]; // vx,vy,vz,va,vb,vg
} Velocity;

// 系统参数宏定义
#define CABLE_NUM 8       // 绳索数量
#define STEP_NUM 501      // 轨迹总步数（0.01s步长，5s共501个点）

// 外部全局变量声明
extern float32_t t_vec[STEP_NUM];                  // 时间向量数组
extern Pose pose_trj[STEP_NUM];                    // 位姿轨迹数组（位置+姿态）
extern Velocity v_trj[STEP_NUM];                   // 速度轨迹数组（线速度+角速度）
extern float32_t cable_length[CABLE_NUM][STEP_NUM]; // 绳索长度数组（每个时刻的长度）
extern float32_t cable_velocity[CABLE_NUM][STEP_NUM]; // 绳索速度数组（每个时刻的速度）

// 末端执行器物理参数（外部可见）
extern const float32_t mass_ee;    // 末端执行器质量
extern const float32_t Ixx_ee;     // x轴转动惯量
extern const float32_t Iyy_ee;     // y轴转动惯量
extern const float32_t Izz_ee;     // z轴转动惯量

// 基座锚点坐标（全局坐标系，外部可见）
extern const Point4f base_g[CABLE_NUM];

// 末端执行器附着点坐标（局部坐标系，外部可见）
extern const Point4f attach_e[CABLE_NUM];

// 函数声明

/**
 * @brief 4x4矩阵乘法
 * @param result 输出参数，存储乘法结果
 * @param a 第一个乘数矩阵
 * @param b 第二个乘数矩阵
 */
void matrix_multiply(Matrix4f *result, const Matrix4f *a, const Matrix4f *b);

/**
 * @brief 点与矩阵相乘（坐标变换）
 * @param p 输入点（齐次坐标）
 * @param m 变换矩阵
 * @return 变换后的点
 */
Point4f point_mult_matrix(const Point4f *p, const Matrix4f *m);

/**
 * @brief 生成旋转矩阵（Z-Y-X顺序）
 * @param rot 输出参数，存储生成的旋转矩阵
 * @param ax X轴旋转角（弧度）
 * @param ay Y轴旋转角（弧度）
 * @param az Z轴旋转角（弧度）
 */
void get_rotation_matrix(Matrix4f *rot, float32_t ax, float32_t ay, float32_t az);

/**
 * @brief 生成平移矩阵
 * @param trans 输出参数，存储生成的平移矩阵
 * @param x x方向平移量
 * @param y y方向平移量
 * @param z z方向平移量
 */
void get_translation_matrix(Matrix4f *trans, float32_t x, float32_t y, float32_t z);

/**
 * @brief 向量叉乘
 * @param res 输出参数，存储叉乘结果
 * @param a 第一个输入向量（3元素）
 * @param b 第二个输入向量（3元素）
 */
void cross_product(float32_t *res, const float32_t *a, const float32_t *b);

/**
 * @brief 计算向量的模长
 * @param v 输入向量（3元素）
 * @return 向量的模长
 */
float32_t vector_norm(const float32_t *v);

/**
 * @brief 生成五次多项式轨迹（包含位置和姿态）
 * @param t_start 轨迹起始时间（s）
 * @param t_end 轨迹结束时间（s）
 * @param t_step 时间步长（s）
 */
void generate_trajectory(float32_t t_start, float32_t t_end, float32_t t_step,
                        const Pose *start_pose, const Pose *end_pose);

/**
 * @brief 计算动力学参数（绳索长度和速度）
 * 根据位姿轨迹计算每个时刻的绳索长度和速度
 */
void calculate_dynamics(void);

/**
 * @brief 初始化CDPR系统
 * 生成轨迹并计算动力学参数，系统启动时调用
 */
void cdpr_init(const Pose *start_pose, const Pose *end_pose);

/**
 * @brief 获取当前时刻的绳索状态
 * @param time_idx 时间索引（0到STEP_NUM-1）
 * @param length 输出参数，存储当前所有绳索的长度
 * @param velocity 输出参数，存储当前所有绳索的速度
 */
void cdpr_get_current_state(uint16_t time_idx, 
                           float32_t length[CABLE_NUM], 
                           float32_t velocity[CABLE_NUM]);

#endif

