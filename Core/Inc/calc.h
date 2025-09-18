#ifndef __CALC_H
#define __CALC_H

#include "stm32f4xx_hal.h"
#include <math.h>

// �������Ͷ���
typedef float float32_t;

/**
 * @brief ��ά��ṹ�壨��������꣩
 * ���ڱ�ʾ����ê���ĩ��ִ�������ŵ�����
 */
typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
    float32_t w; // ������꣬ͨ��Ϊ1.0
} Point4f;

/**
 * @brief 4x4����ṹ��
 * ��������任����ת����
 */
typedef struct {
    float32_t data[4][4];
} Matrix4f;

/**
 * @brief λ�˽ṹ�壨λ��+��̬��
 * data[0-2]: x,y,zλ������
 * data[3-5]: alpha,beta,gamma��̬�ǣ���ת�Ƕȣ�
 */
typedef struct {
    float32_t data[6]; // x,y,z,alpha,beta,gamma
} Pose;

/**
 * @brief �ٶȽṹ�壨���ٶ�+���ٶȣ�
 * data[0-2]: vx,vy,vz���ٶ�
 * data[3-5]: va,vb,vg���ٶ�
 */
typedef struct {
    float32_t data[6]; // vx,vy,vz,va,vb,vg
} Velocity;

typedef struct {
    float32_t data[6]; // ax,ay,az,aa,ab,ag
} Acceleration;

// ��ζ���ʽϵ���ṹ��
typedef struct {
    float32_t a0; // ������
    float32_t a1; // һ����
    float32_t a2; // ������
    float32_t a3; // ������
    float32_t a4; // �Ĵ���
    float32_t a5; // �����
} Poly5Coeff;

// ������ζ���ʽϵ��
static void calculate_poly5_coeff(Poly5Coeff *coeff, 
                                 float32_t s0, float32_t v0, float32_t a0,  // ���߽�����
                                 float32_t s1, float32_t v1, float32_t a1,  // �յ�߽�����
                                 float32_t t_total);                          // ��ʱ��


// ϵͳ�����궨��
#define CABLE_NUM 8       // ��������
#define STEP_NUM 1001      // �켣�ܲ�����0.01s������5s��501���㣩
#define MOTOR_PULLEY_RADIUS 0.0475f  // ��� pulley �뾶����λ���� (47.5mm)
#define GO_PULLEY_RADIUS 0.0510f  // ��� pulley �뾶����λ���� (47.5mm)

// �ⲿȫ�ֱ�������
//extern float32_t t_vec[STEP_NUM];                  // ʱ����������
//extern Pose pose_trj[STEP_NUM];                    // λ�˹켣���飨λ��+��̬��
//extern Velocity v_trj[STEP_NUM];                   // �ٶȹ켣���飨���ٶ�+���ٶȣ�
//extern float32_t cable_length[CABLE_NUM][STEP_NUM]; // �����������飨ÿ��ʱ�̵ĳ��ȣ�
//extern float32_t cable_velocity[CABLE_NUM][STEP_NUM]; // �����ٶ����飨ÿ��ʱ�̵��ٶȣ�
extern float32_t cable_initial_length[CABLE_NUM];  // ÿ�������ĳ�ʼ����(���ο�)
extern float32_t motor_angle[CABLE_NUM][STEP_NUM]; // ����Ƕȹ켣(�Ƕ�)
extern float32_t motor_omega[CABLE_NUM][STEP_NUM]; // ������ٶ�(�Ƕ�)

// ĩ��ִ��������������ⲿ�ɼ���
extern const float32_t mass_ee;    // ĩ��ִ��������
extern const float32_t Ixx_ee;     // x��ת������
extern const float32_t Iyy_ee;     // y��ת������
extern const float32_t Izz_ee;     // z��ת������

// ����ê�����꣨ȫ������ϵ���ⲿ�ɼ���
extern const Point4f base_g[CABLE_NUM];

// ĩ��ִ�������ŵ����꣨�ֲ�����ϵ���ⲿ�ɼ���
extern const Point4f attach_e[CABLE_NUM];

// ��������

/**
 * @brief 4x4����˷�
 * @param result ����������洢�˷����
 * @param a ��һ����������
 * @param b �ڶ�����������
 */
void matrix_multiply(Matrix4f *result, const Matrix4f *a, const Matrix4f *b);

/**
 * @brief ���������ˣ�����任��
 * @param p ����㣨������꣩
 * @param m �任����
 * @return �任��ĵ�
 */
Point4f point_mult_matrix(const Point4f *p, const Matrix4f *m);

/**
 * @brief ������ת����Z-Y-X˳��
 * @param rot ����������洢���ɵ���ת����
 * @param ax X����ת�ǣ����ȣ�
 * @param ay Y����ת�ǣ����ȣ�
 * @param az Z����ת�ǣ����ȣ�
 */
void get_rotation_matrix(Matrix4f *rot, float32_t ax, float32_t ay, float32_t az);

/**
 * @brief ����ƽ�ƾ���
 * @param trans ����������洢���ɵ�ƽ�ƾ���
 * @param x x����ƽ����
 * @param y y����ƽ����
 * @param z z����ƽ����
 */
void get_translation_matrix(Matrix4f *trans, float32_t x, float32_t y, float32_t z);

/**
 * @brief �������
 * @param res ����������洢��˽��
 * @param a ��һ������������3Ԫ�أ�
 * @param b �ڶ�������������3Ԫ�أ�
 */
void cross_product(float32_t *res, const float32_t *a, const float32_t *b);

/**
 * @brief ����������ģ��
 * @param v ����������3Ԫ�أ�
 * @return ������ģ��
 */
float32_t vector_norm(const float32_t *v);

/**
 * @brief ������ζ���ʽ�켣������λ�ú���̬��
 * @param t_start �켣��ʼʱ�䣨s��
 * @param t_end �켣����ʱ�䣨s��
 * @param t_step ʱ�䲽����s��
 */
void generate_trajectory_and_angles(float32_t t_start, float32_t t_end, float32_t t_step,
                                         const Pose *start_pose, const Velocity *start_vel, const Acceleration *start_acc,
                                         const Pose *end_pose, const Velocity *end_vel, const Acceleration *end_acc);



/**
 * @brief ��ʼ��CDPRϵͳ
 * ���ɹ켣�����㶯��ѧ������ϵͳ����ʱ����
 */
void cdpr_init(const Pose *start_pose, const Velocity *start_vel, const Acceleration *start_acc,
              const Pose *end_pose, const Velocity *end_vel, const Acceleration *end_acc,float time);


/**
 * @brief ��ȡ��ǰʱ�̵�����״̬
 * @param time_idx ʱ��������0��STEP_NUM-1��
 * @param length ����������洢��ǰ���������ĳ���
 * @param velocity ����������洢��ǰ�����������ٶ�
 */
void cdpr_get_current_motor_angles(uint16_t time_idx, float32_t angles[CABLE_NUM]);


#endif

