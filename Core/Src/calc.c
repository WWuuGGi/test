#include "calc.h"

const float32_t mass_ee = 2.0f;
const float32_t Ixx_ee = 0.1016f;
const float32_t Iyy_ee = 0.165f;
const float32_t Izz_ee = 0.165f;

// 全局变量(存储计算结果)
float32_t t_vec[STEP_NUM];
Pose pose_trj[STEP_NUM];
Velocity v_trj[STEP_NUM];
float32_t cable_length[CABLE_NUM][STEP_NUM];
float32_t cable_velocity[CABLE_NUM][STEP_NUM];

// 基座锚点坐标(全局坐标系)
const Point4f base_g[CABLE_NUM] = {
    {-0.397f,  0.324f, 0.820f, 1.0f}, // b1
    {-0.397f, -0.324f, 0.820f, 1.0f}, // b2
    {0.397f, -0.324f, 0.820f, 1.0f},  // b3
    {0.397f,  0.324f, 0.820f, 1.0f},  // b4
    {-0.397f,  0.324f, 0.169f, 1.0f}, // b5
    {-0.397f, -0.324f, 0.169f, 1.0f}, // b6
    {0.397f, -0.324f, 0.169f, 1.0f},  // b7
    {0.397f,  0.324f, 0.169f, 1.0f}   // b8
};

// 末端执行器附着点坐标(局部坐标系)
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

// 矩阵乘法(4x4)
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

// 点乘矩阵
Point4f point_mult_matrix(const Point4f *p, const Matrix4f *m) {
    Point4f res;
    res.x = p->x * m->data[0][0] + p->y * m->data[0][1] + p->z * m->data[0][2] + p->w * m->data[0][3];
    res.y = p->x * m->data[1][0] + p->y * m->data[1][1] + p->z * m->data[1][2] + p->w * m->data[1][3];
    res.z = p->x * m->data[2][0] + p->y * m->data[2][1] + p->z * m->data[2][2] + p->w * m->data[2][3];
    res.w = p->x * m->data[3][0] + p->y * m->data[3][1] + p->z * m->data[3][2] + p->w * m->data[3][3];
    return res;
}

// 生成旋转矩阵(Z-Y-X顺序)
void get_rotation_matrix(Matrix4f *rot, float32_t ax, float32_t ay, float32_t az) {
    float32_t cx = cosf(ax);
    float32_t sx = sinf(ax);
    float32_t cy = cosf(ay);
    float32_t sy = sinf(ay);
    float32_t cz = cosf(az);
    float32_t sz = sinf(az);

    // Z旋转
    Matrix4f Rz = {
			.data = {
        {cz, -sz, 0, 0},
        {sz,  cz, 0, 0},
        {0,   0,  1, 0},
        {0,   0,  0, 1}
			}
		};

    // Y旋转
    Matrix4f Ry = {
			.data = {
        {cy, 0, sy, 0},
        {0,  1,  0, 0},
        {-sy,0, cy, 0},
        {0,  0,  0, 1}
			}
		};

    // X旋转
    Matrix4f Rx = {
			.data = {
        {1, 0,   0, 0},
        {0, cx, -sx, 0},
        {0, sx,  cx, 0},
        {0, 0,   0, 1}
			}
		};

    // 组合旋转: Rz * Ry * Rx
    Matrix4f temp;
    matrix_multiply(&temp, &Rz, &Ry);
    matrix_multiply(rot, &temp, &Rx);
}

// 生成平移矩阵
void get_translation_matrix(Matrix4f *trans, float32_t x, float32_t y, float32_t z) {
    trans->data[0][0] = 1; trans->data[0][1] = 0; trans->data[0][2] = 0; trans->data[0][3] = x;
    trans->data[1][0] = 0; trans->data[1][1] = 1; trans->data[1][2] = 0; trans->data[1][3] = y;
    trans->data[2][0] = 0; trans->data[2][1] = 0; trans->data[2][2] = 1; trans->data[2][3] = z;
    trans->data[3][0] = 0; trans->data[3][1] = 0; trans->data[3][2] = 0; trans->data[3][3] = 1;
}

// 向量叉乘
void cross_product(float32_t *res, const float32_t *a, const float32_t *b) {
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}

// 向量范数
float32_t vector_norm(const float32_t *v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// 生成五次多项式轨迹
// 修改：添加起点和终点姿态作为函数参数
void generate_trajectory(float32_t t_start, float32_t t_end, float32_t t_step,
                        const Pose *start_pose, const Pose *end_pose) {
    // 使用传入的起点和终点姿态
    const Pose p_start = *start_pose;
    const Pose p_end = *end_pose;
    float32_t t_total = t_end - t_start;

    // 计算多项式系数（位置和姿态六个维度）
    float32_t dx = p_end.data[0] - p_start.data[0];
    float32_t dy = p_end.data[1] - p_start.data[1];
    float32_t dz = p_end.data[2] - p_start.data[2];
    float32_t da = p_end.data[3] - p_start.data[3];
    float32_t db = p_end.data[4] - p_start.data[4];
    float32_t dr = p_end.data[5] - p_start.data[5];

    // 系数计算 (五次多项式)
    float32_t a3_x = 10.0f * dx / powf(t_total, 3);
    float32_t a4_x = -15.0f * dx / powf(t_total, 4);
    float32_t a5_x = 6.0f * dx / powf(t_total, 5);

    float32_t a3_y = 10.0f * dy / powf(t_total, 3);
    float32_t a4_y = -15.0f * dy / powf(t_total, 4);
    float32_t a5_y = 6.0f * dy / powf(t_total, 5);

    float32_t a3_z = 10.0f * dz / powf(t_total, 3);
    float32_t a4_z = -15.0f * dz / powf(t_total, 4);
    float32_t a5_z = 6.0f * dz / powf(t_total, 5);

    float32_t a3_a = 10.0f * da / powf(t_total, 3);
    float32_t a4_a = -15.0f * da / powf(t_total, 4);
    float32_t a5_a = 6.0f * da / powf(t_total, 5);

    float32_t a3_b = 10.0f * db / powf(t_total, 3);
    float32_t a4_b = -15.0f * db / powf(t_total, 4);
    float32_t a5_b = 6.0f * db / powf(t_total, 5);

    float32_t a3_r = 10.0f * dr / powf(t_total, 3);
    float32_t a4_r = -15.0f * dr / powf(t_total, 4);
    float32_t a5_r = 6.0f * dr / powf(t_total, 5);

    // 计算每个时间步的轨迹
    for (uint16_t i = 0; i < STEP_NUM; i++) {
        t_vec[i] = t_start + i * t_step;
        float32_t t = t_vec[i];
        float32_t t2 = t * t;
        float32_t t3 = t2 * t;
        float32_t t4 = t3 * t;
        float32_t t5 = t4 * t;

        // 位置和姿态计算
        pose_trj[i].data[0] = p_start.data[0] + a3_x * t3 + a4_x * t4 + a5_x * t5;
        pose_trj[i].data[1] = p_start.data[1] + a3_y * t3 + a4_y * t4 + a5_y * t5;
        pose_trj[i].data[2] = p_start.data[2] + a3_z * t3 + a4_z * t4 + a5_z * t5;
        pose_trj[i].data[3] = p_start.data[3] + a3_a * t3 + a4_a * t4 + a5_a * t5;
        pose_trj[i].data[4] = p_start.data[4] + a3_b * t3 + a4_b * t4 + a5_b * t5;
        pose_trj[i].data[5] = p_start.data[5] + a3_r * t3 + a4_r * t4 + a5_r * t5;

        // 速度计算
        v_trj[i].data[0] = 3 * a3_x * t2 + 4 * a4_x * t3 + 5 * a5_x * t4;
        v_trj[i].data[1] = 3 * a3_y * t2 + 4 * a4_y * t3 + 5 * a5_y * t4;
        v_trj[i].data[2] = 3 * a3_z * t2 + 4 * a4_z * t3 + 5 * a5_z * t4;
        v_trj[i].data[3] = 3 * a3_a * t2 + 4 * a4_a * t3 + 5 * a5_a * t4;
        v_trj[i].data[4] = 3 * a3_b * t2 + 4 * a4_b * t3 + 5 * a5_b * t4;
        v_trj[i].data[5] = 3 * a3_r * t2 + 4 * a4_r * t3 + 5 * a5_r * t4;
    }
}

// 计算动力学参数
void calculate_dynamics() {
    for (uint16_t i = 0; i < STEP_NUM; i++) {
        Pose pose = pose_trj[i];
        Velocity vel = v_trj[i];

        // 构建齐次变换矩阵
        Matrix4f trans, rot, Tr;
        get_translation_matrix(&trans, pose.data[0], pose.data[1], pose.data[2]);
        get_rotation_matrix(&rot, pose.data[3], pose.data[4], pose.data[5]);
        matrix_multiply(&Tr, &trans, &rot);

        // 计算每个绳索的长度和速度
        float32_t jaco[CABLE_NUM][6]; // 雅克比矩阵 (8x6)

        for (uint8_t c = 0; c < CABLE_NUM; c++) {
            // 附着点全局坐标
            Point4f a_g = point_mult_matrix(&attach_e[c], &Tr);
            
            // 绳索向量 (基座到附着点)
            float32_t vec_cable[3] = {
                base_g[c].x - a_g.x,
                base_g[c].y - a_g.y,
                base_g[c].z - a_g.z
            };

            // 绳索长度
            float32_t len = vector_norm(vec_cable);
            cable_length[c][i] = len;

            // 单位向量
            float32_t unit_vec[3] = {
                vec_cable[0] / len,
                vec_cable[1] / len,
                vec_cable[2] / len
            };

            // 末端执行器上的向量 (质心到附着点)
            float32_t vec_ee[3] = {
                a_g.x - pose.data[0],
                a_g.y - pose.data[1],
                a_g.z - pose.data[2]
            };

            // 叉乘结果
            float32_t cross_res[3];
            cross_product(cross_res, vec_ee, unit_vec);

            // 填充雅克比矩阵
            jaco[c][0] = unit_vec[0];
            jaco[c][1] = unit_vec[1];
            jaco[c][2] = unit_vec[2];
            jaco[c][3] = cross_res[0];
            jaco[c][4] = cross_res[1];
            jaco[c][5] = cross_res[2];

            // 计算绳索速度
            cable_velocity[c][i] = 0.0f;
            for (uint8_t k = 0; k < 6; k++) {
                cable_velocity[c][i] += jaco[c][k] * vel.data[k];
            }
        }
    }
}

// 初始化函数
void cdpr_init(const Pose *start_pose, const Pose *end_pose) {
    generate_trajectory(0.0f, 5.0f, 0.01f, start_pose, end_pose);
    calculate_dynamics();
}

// 主循环中调用，获取当前时刻的绳索长度和速度
void cdpr_get_current_state(uint16_t time_idx, 
                           float32_t length[CABLE_NUM], 
                           float32_t velocity[CABLE_NUM]) {
    if (time_idx < STEP_NUM) {
        for (uint8_t i = 0; i < CABLE_NUM; i++) {
            length[i] = cable_length[i][time_idx];
            velocity[i] = cable_velocity[i][time_idx];
        }
    }
}






