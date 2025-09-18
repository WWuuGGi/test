#include "calc.h"

const float32_t mass_ee = 2.0f;
const float32_t Ixx_ee = 0.1016f;
const float32_t Iyy_ee = 0.165f;
const float32_t Izz_ee = 0.165f;

// ȫ�ֱ���(�洢������)
//float32_t t_vec[STEP_NUM];
//Pose pose_trj[STEP_NUM];
//Velocity v_trj[STEP_NUM];
//Acceleration a_trj[STEP_NUM];
//float32_t cable_length[CABLE_NUM][STEP_NUM];
//float32_t cable_velocity[CABLE_NUM][STEP_NUM];
float32_t cable_initial_length[CABLE_NUM];  // ÿ�������ĳ�ʼ����(���ο�)
float32_t motor_angle[CABLE_NUM][STEP_NUM]; // ����Ƕȹ켣(�Ƕ�)
float32_t motor_omega[CABLE_NUM][STEP_NUM];
Poly5Coeff coeffs[6];                       // ����ʽϵ��(���ã����洢�����켣)

// ������ζ���ʽϵ��
static void calculate_poly5_coeff(Poly5Coeff *coeff, 
                                 float32_t s0, float32_t v0, float32_t a0,  // ���߽�����
                                 float32_t s1, float32_t v1, float32_t a1,  // �յ�߽�����
                                 float32_t t_total)                          // ��ʱ��
{
    float32_t t2 = t_total * t_total;
    float32_t t3 = t2 * t_total;
    float32_t t4 = t3 * t_total;
    float32_t t5 = t4 * t_total;

    // ��������ϵ��
    coeff->a0 = s0;  // λ�ó�ʼֵ
    coeff->a1 = v0;  // �ٶȳ�ʼֵ
    coeff->a2 = a0 / 2.0f;  // ���ٶȳ�ʼֵ
    
    coeff->a3 = (10.0f*s1 - 10.0f*s0 - 4.0f*v1*t_total - 6.0f*v0*t_total 
               - 0.5f*a0*t2 + 0.5f*a1*t2) / t3;
               
    coeff->a4 = (-15.0f*s1 + 15.0f*s0 + 7.0f*v1*t_total + 8.0f*v0*t_total 
               + 0.5f*a0*t2 - a1*t2) / t4;
               
    coeff->a5 = (6.0f*s1 - 6.0f*s0 - 3.0f*v1*t_total - 3.0f*v0*t_total 
               - 0.5f*a0*t2 + 0.5f*a1*t2) / t5;
}



// ����ê������(ȫ������ϵ)
const Point4f base_g[CABLE_NUM] = {
    {-0.397f,  0.324f, 0.820f, 1.0f}, // b1
    {-0.397f, -0.324f, 0.820f, 1.0f}, // b2
    {0.397f, -0.324f, 0.820f, 1.0f},  // b3
    {0.397f,  0.324f, 0.820f, 1.0f},  // b4
    {-0.397f,  0.324f, -0.004f, 1.0f}, // b5
    {-0.397f, -0.324f, -0.004f, 1.0f}, // b6
    {0.397f, -0.324f,  -0.004f, 1.0f},  // b7
    {0.397f,  0.324f,  -0.004f, 1.0f}   // b8
};

// ĩ��ִ�������ŵ�����(�ֲ�����ϵ)
const Point4f attach_e[CABLE_NUM] = {
    {-0.05f,  0.05f, -0.05f, 1.0f}, // a1
    {-0.05f, -0.05f, -0.05f, 1.0f}, // a2
    {0.05f, -0.05f, -0.05f, 1.0f},  // a3
    {0.05f,  0.05f, -0.05f, 1.0f},  // a4
    {-0.05f,  0.05f,  0.05f, 1.0f}, // a5
    {-0.05f, -0.05f,  0.05f, 1.0f}, // a6
    {0.05f, -0.05f,  0.05f, 1.0f},  // a7
    {0.05f,  0.05f,  0.05f, 1.0f}   // a8
};



// ����˷�(4x4)
void matrix_multiply(Matrix4f *result, const Matrix4f *a, const Matrix4f *b) {
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            result->data[i][j] = 0.0f;
            for (uint8_t k = 0; k < 4; k++) {
                result->data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
}

// ��˾���
Point4f point_mult_matrix(const Point4f *p, const Matrix4f *m) {
    Point4f res;
    res.x = p->x * m->data[0][0] + p->y * m->data[0][1] + p->z * m->data[0][2] + p->w * m->data[0][3];
    res.y = p->x * m->data[1][0] + p->y * m->data[1][1] + p->z * m->data[1][2] + p->w * m->data[1][3];
    res.z = p->x * m->data[2][0] + p->y * m->data[2][1] + p->z * m->data[2][2] + p->w * m->data[2][3];
    res.w = p->x * m->data[3][0] + p->y * m->data[3][1] + p->z * m->data[3][2] + p->w * m->data[3][3];
    return res;
}

// ������ת����(Z-Y-X˳��)
void get_rotation_matrix(Matrix4f *rot, float32_t ax, float32_t ay, float32_t az) {
    float32_t cx = cosf(ax);
    float32_t sx = sinf(ax);
    float32_t cy = cosf(ay);
    float32_t sy = sinf(ay);
    float32_t cz = cosf(az);
    float32_t sz = sinf(az);

    // Z��ת
    Matrix4f Rz = {
			.data = {
        {cz, -sz, 0, 0},
        {sz,  cz, 0, 0},
        {0,   0,  1, 0},
        {0,   0,  0, 1}
			}
		};

    // Y��ת
    Matrix4f Ry = {
			.data = {
        {cy, 0, sy, 0},
        {0,  1,  0, 0},
        {-sy,0, cy, 0},
        {0,  0,  0, 1}
			}
		};

    // X��ת
    Matrix4f Rx = {
			.data = {
        {1, 0,   0, 0},
        {0, cx, -sx, 0},
        {0, sx,  cx, 0},
        {0, 0,   0, 1}
			}
		};

    // �����ת: Rz * Ry * Rx
    Matrix4f temp;
    matrix_multiply(&temp, &Rz, &Ry);
    matrix_multiply(rot, &temp, &Rx);
}

// ����ƽ�ƾ���
void get_translation_matrix(Matrix4f *trans, float32_t x, float32_t y, float32_t z) {
    trans->data[0][0] = 1; trans->data[0][1] = 0; trans->data[0][2] = 0; trans->data[0][3] = x;
    trans->data[1][0] = 0; trans->data[1][1] = 1; trans->data[1][2] = 0; trans->data[1][3] = y;
    trans->data[2][0] = 0; trans->data[2][1] = 0; trans->data[2][2] = 1; trans->data[2][3] = z;
    trans->data[3][0] = 0; trans->data[3][1] = 0; trans->data[3][2] = 0; trans->data[3][3] = 1;
}

// �������
void cross_product(float32_t *res, const float32_t *a, const float32_t *b) {
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}

// ��������
float32_t vector_norm(const float32_t *v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// ������ζ���ʽ�켣
// ֧��ָ�������յ��λ�á��ٶȺͼ��ٶ�
// ������ζ���ʽ�켣��ֱ�Ӽ������Ƕ�(���洢�м�켣)
static void generate_trajectory_and_angles(float32_t t_start, float32_t t_end, float32_t t_step,
                                         const Pose *start_pose, const Velocity *start_vel, const Acceleration *start_acc,
                                         const Pose *end_pose, const Velocity *end_vel, const Acceleration *end_acc) {
    float32_t t_total = t_end - t_start;
    
    // �������ʽϵ��(���洢ϵ�������洢�����켣)
    for (uint8_t i = 0; i < 6; i++) {
        calculate_poly5_coeff(&coeffs[i],
                             start_pose->data[i], start_vel->data[i], start_acc->data[i],
                             end_pose->data[i], end_vel->data[i], end_acc->data[i],
                             t_total);
    }

    // ���ʱ�䲽���㣬ֱ�����ɵ���Ƕ�
    for (uint16_t i = 0; i < STEP_NUM; i++) {
        float32_t t = t_start + i * t_step;
        float32_t t2 = t * t;
        float32_t t3 = t2 * t;
        float32_t t4 = t3 * t;
        float32_t t5 = t4 * t;

        // ���㵱ǰʱ�̵���̬(�ֲ����������洢)
        Pose current_pose;
        Velocity current_vel;
        for (uint8_t j = 0; j < 6; j++) {
            current_pose.data[j] = coeffs[j].a0 + coeffs[j].a1*t + coeffs[j].a2*t2 +
                                 coeffs[j].a3*t3 + coeffs[j].a4*t4 + coeffs[j].a5*t5;
            current_vel.data[j] = coeffs[j].a1 + 2*coeffs[j].a2*t + 3*coeffs[j].a3*t2 +
                                4*coeffs[j].a4*t3 + 5*coeffs[j].a5*t4;
        }

        // ������α任����
        Matrix4f trans, rot, Tr;
        get_translation_matrix(&trans, current_pose.data[0], current_pose.data[1], current_pose.data[2]);
        get_rotation_matrix(&rot, current_pose.data[3], current_pose.data[4], current_pose.data[5]);
        matrix_multiply(&Tr, &trans, &rot);

        // ����ÿ�������ĳ��Ȳ�ת��Ϊ����Ƕ�
        for (uint8_t c = 0; c < CABLE_NUM; c++) {
            // ���ŵ�ȫ������
            Point4f a_g = point_mult_matrix(&attach_e[c], &Tr);
            
            // ���������볤��
            float32_t vec_cable[3] = {
                base_g[c].x - a_g.x,
                base_g[c].y - a_g.y,
                base_g[c].z - a_g.z
            };
            float32_t current_length = vector_norm(vec_cable);

            // �洢��ʼ����(��0��)
            if (i == 0) {
                cable_initial_length[c] = current_length;
            }

            // �������ǶȲ��洢(�������˽��)
            float32_t length_change = current_length - cable_initial_length[c];
            motor_angle[c][i] = length_change / MOTOR_PULLEY_RADIUS / 3.1415926f * 180.0f;
						
						//        // ����ÿ�������ĳ��Ⱥ��ٶ�
						float32_t jaco[CABLE_NUM][6]; // �ſ˱Ⱦ��� (8x6)
						
						// ��λ����
            float32_t unit_vec[3] = {
                vec_cable[0] / current_length,
                vec_cable[1] / current_length,
                vec_cable[2] / current_length
            };

            // ĩ��ִ�����ϵ����� (���ĵ����ŵ�)
            float32_t vec_ee[3] = {
                a_g.x - current_pose.data[0],
                a_g.y - current_pose.data[1],
                a_g.z - current_pose.data[2]
            };

            // ��˽��
            float32_t cross_res[3];
            cross_product(cross_res, vec_ee, unit_vec);

            // ����ſ˱Ⱦ���
            jaco[c][0] = unit_vec[0];
            jaco[c][1] = unit_vec[1];
            jaco[c][2] = unit_vec[2];
            jaco[c][3] = cross_res[0];
            jaco[c][4] = cross_res[1];
            jaco[c][5] = cross_res[2];

            // ���������ٶ�
            motor_omega[c][i] = 0.0f;
            for (uint8_t k = 0; k < 6; k++) {
							if(c != 2 && c != 3)
							{
                motor_omega[c][i] += -1.0f * jaco[c][k] * current_vel.data[k] / MOTOR_PULLEY_RADIUS;
							}
							else
							{
								motor_omega[c][i] += -1.0f * jaco[c][k] * current_vel.data[k] / GO_PULLEY_RADIUS;
							}
							
							
						}
        }
    }
}

//// ���㶯��ѧ����
//void calculate_dynamics() {
//    for (uint16_t i = 0; i < STEP_NUM; i++) {
//        Pose pose = pose_trj[i];
//        Velocity vel = v_trj[i];

//        // ������α任����
//        Matrix4f trans, rot, Tr;
//        get_translation_matrix(&trans, pose.data[0], pose.data[1], pose.data[2]);
//        get_rotation_matrix(&rot, pose.data[3], pose.data[4], pose.data[5]);
//        matrix_multiply(&Tr, &trans, &rot);

//        // ����ÿ�������ĳ��Ⱥ��ٶ�
//        float32_t jaco[CABLE_NUM][6]; // �ſ˱Ⱦ��� (8x6)

//        for (uint8_t c = 0; c < CABLE_NUM; c++) {
//            // ���ŵ�ȫ������
//            Point4f a_g = point_mult_matrix(&attach_e[c], &Tr);
//            
//            // �������� (���������ŵ�)
//            float32_t vec_cable[3] = {
//                base_g[c].x - a_g.x,
//                base_g[c].y - a_g.y,
//                base_g[c].z - a_g.z
//            };

//            // ��������
//            float32_t len = vector_norm(vec_cable);
//            cable_length[c][i] = len;

//            // ��λ����
//            float32_t unit_vec[3] = {
//                vec_cable[0] / len,
//                vec_cable[1] / len,
//                vec_cable[2] / len
//            };

//            // ĩ��ִ�����ϵ����� (���ĵ����ŵ�)
//            float32_t vec_ee[3] = {
//                a_g.x - pose.data[0],
//                a_g.y - pose.data[1],
//                a_g.z - pose.data[2]
//            };

//            // ��˽��
//            float32_t cross_res[3];
//            cross_product(cross_res, vec_ee, unit_vec);

//            // ����ſ˱Ⱦ���
//            jaco[c][0] = unit_vec[0];
//            jaco[c][1] = unit_vec[1];
//            jaco[c][2] = unit_vec[2];
//            jaco[c][3] = cross_res[0];
//            jaco[c][4] = cross_res[1];
//            jaco[c][5] = cross_res[2];

//            // ���������ٶ�
//            cable_velocity[c][i] = 0.0f;
//            for (uint8_t k = 0; k < 6; k++) {
//                cable_velocity[c][i] += jaco[c][k] * vel.data[k];
//            }
//        }
//    }
//		// �������е���Ƕȹ켣
//    calculate_all_motor_angles();
//}

// ��ʼ����������Ҫ���������ı߽�������
void cdpr_init(const Pose *start_pose, const Velocity *start_vel, const Acceleration *start_acc,
              const Pose *end_pose, const Velocity *end_vel, const Acceleration *end_acc) {
    generate_trajectory_and_angles(0.0f, 5.0f, 0.01f,
                       start_pose, start_vel, start_acc,
                       end_pose, end_vel, end_acc);
}


// ��ȡ��ǰʱ�̵ĵ���Ƕ�
void cdpr_get_current_motor_angles(uint16_t time_idx, float32_t angles[CABLE_NUM]) {
    if (time_idx < STEP_NUM && angles != NULL) {
        for (uint8_t i = 0; i < CABLE_NUM; i++) {
            angles[i] = motor_angle[i][time_idx];
        }
    }
}
		


