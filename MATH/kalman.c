// 作者：咸
// 电话:18878881386
// 邮箱:3078510877@qq.com
// 日期:2026.3.6
// 版本:1.0
//  卡尔曼滤波算法模块（一维/二维/三维实现）
 
#include <math.h>
#include "ALL_DEFINE.h"

/*********************************************************************************************************************
** 卡尔曼滤波核心原理
** @brief:  标准卡尔曼滤波五步法实现
** @param[in]  InputData 滤波前的原始数据（QR参数）
** @param[out] None
** @return 滤波后的输出值
** @remark: 通过调整过程噪声Q和测量噪声R的值控制滤波效果
** 
** 卡尔曼滤波基础公式：
** X(k) = A*X(k-1) + B*U(k) + W(k)  【状态方程：系统状态转移】
** Z(k) = H*X(k) + V(k)            【观测方程：测量值与状态的关系】
** 
** 其中：
** AB = 系统参数（本文简化为1）
** X = K时刻的系统状态
** H = 观测系统参数（本文简化为1）
** Z = K时刻的测量值
** W/V = 过程噪声/测量噪声（高斯白噪声）
** 
** 卡尔曼滤波5个核心步骤：
** 1. 状态预测：X(k|k-1) = X(k-1|k-1) （预测当前状态）
** 2. 协方差预测：P(k|k-1) = P(k-1|k-1) + Q （预测协方差）
** 3. 计算卡尔曼增益：Kg(k) = P(k|k-1) / (P(k|k-1) + R)
** 4. 状态更新：X(k|k) = X(k|k-1) + Kg(k)*(Z(k)-X(k|k-1)) （融合预测值与测量值）
** 5. 协方差更新：P(k|k) = (1-Kg(k))*P(k|k-1) （更新协方差）
** 
** 符号说明：
** (k-1|k-1) = 上一状态最优估计值
** (k|k-1)   = 根据上一状态预测的当前状态值
** (k|k)     = 当前状态最优估计值
** Q = 系统过程噪声协方差（越大响应越快，滤波越弱）
** R = 测量噪声协方差（越大滤波越强，响应越慢）
** 卡尔曼增益 Kg = Q/(Q+R)
** P = 协方差矩阵
** 
** 注意事项：
** 1. 本文实现为简化模型，假设H/I=1、U=0，A/B初始值为1
** 2. X(0|0) 初始化为系统初始状态（本文为0）
** 3. P(0|0) 一般不取0（取0会导致Kg=0，系统仅使用初始值，无更新）
 **********************************************************************************************************************/
 		//		const float Q = 0.018;//0.01;  // 过程噪声（可调）
		//		const float R = 0.543;//0.9;    // 测量噪声（可调）			
//float kalman_1(float InputData,float Q,float R)  // 一维卡尔曼滤波（旧版）
//{
//	struct Kalman{
//		float K_1_K_1;//(k-1|k-1) 上一状态最优值
//		float K_K_1;  //(k|k-1)   当前预测值
//		float K_K;    //(k|k)     当前最优值
//	};
//	static struct Kalman P={0};  // 协方差矩阵
//	static struct Kalman X={0};  // 状态值
//	float Kg;                    // 卡尔曼增益
//	
//	X.K_K_1 = X.K_1_K_1;                 // 1. 状态预测
//	P.K_K_1 = P.K_1_K_1 + Q;             // 2. 协方差预测
//	Kg = P.K_K_1 / (P.K_K_1 + R);        // 3. 计算卡尔曼增益
//	X.K_K = X.K_K_1 + Kg * (InputData - X.K_K_1); // 4. 状态更新
//	P.K_K = (1-Kg) * P.K_K_1 ;          // 5. 协方差更新
//	
//	X.K_1_K_1 = X.K_K;                  // 保存当前最优值为下一状态初始值
//	P.K_1_K_1 = P.K_K;                  // 保存当前协方差为下一状态初始值
//	return X.K_K;
//}

/**************************************************************
 * @brief  一维卡尔曼滤波（结构体版）
 * @param  ekf: 指向一阶卡尔曼滤波器结构体的指针
 * @param  input: 输入的原始测量数据
 * @retval 无（滤波结果存储在ekf->out）
 * @note   1. 简化版实现，所有参数封装在结构体中
 *         2. 适用于单一传感器数据滤波（如加速度计、陀螺仪）
 ***************************************************************/
void kalman_1(struct _1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;               // 1. 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);    // 2. 计算卡尔曼增益
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out); // 3. 状态更新：融合预测值与测量值
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;        // 4. 协方差更新
}

/**************************************************************
 * @brief  二维卡尔曼滤波（角度+角速度融合）
 * @param  InputAngle: 加速度计解算的角度（测量值）
 * @param  InputGyro: 陀螺仪角速度（rad/s）
 * @param  dt: 采样时间间隔（秒）
 * @retval float: 融合后的最优角度值
 * @note   1. 融合加速度计角度（静态准）和陀螺仪积分角度（动态准）
 *         2. 状态量为角度和陀螺仪偏差，消除陀螺仪零漂
 ***************************************************************/
float kalman_2_Update(float InputAngle, float InputGyro, float dt)
{
    // 卡尔曼状态结构体
    struct Kalman {
        float k_1_k_1; // (k-1|k-1) 上一状态最优值
        float k_k_1;   // (k|k-1)   当前预测值
        float k_k;     // (k|k)     当前最优值
    };
    static struct Kalman X = {0};  // 状态值（角度）
		
    // 滤波参数配置
    const float R_angle = 0.5;     // 角度测量噪声协方差		
    const float h_0 = 1;           // 二维卡尔曼观测参数
    const float Q_angle = 0.001;   // 角度过程噪声
    const float Q_gyro = 0.001;    // 陀螺仪过程噪声	
		
    static float P[2][2];          // 2x2协方差矩阵
    float k_0, k_1;                // 卡尔曼增益（角度/陀螺仪偏差）
    float t0, t1;                  // 临时变量
    float PHt_0, PHt_1;            // P*H^T 计算中间值
    float Pdot[4];                 // 协方差微分
    float E;                       // 分母项

    // 1. 状态预测：X(k|k-1) = X(k-1|k-1) + (陀螺仪角速度 - 偏差)*dt
    static float Q_bias;           // 陀螺仪偏差估计值
    X.k_k_1 = X.k_1_k_1 + (InputGyro - Q_bias) * dt;

    // 2. 协方差预测：P(k|k-1) = A*P(k-1|k-1)*A' + Q
    Pdot[0] = Q_angle - P[0][1] - P[1][0]; 
    Pdot[1] = - P[1][1];
    Pdot[2] = - P[1][1];
    Pdot[3] = Q_gyro;

    // 积分更新协方差
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    // 3. 计算卡尔曼增益
    PHt_0 = h_0 * P[0][0];
    PHt_1 = h_0 * P[1][0];
    E = R_angle + h_0 * PHt_0;     // 分母项
    k_0 = PHt_0 / E;               // 角度增益
    k_1 = PHt_1 / E;               // 陀螺仪偏差增益

    // 4. 更新协方差矩阵
    t0 = PHt_0;
    t1 = h_0 * P[0][1];
    P[0][0] -= k_0 * t0;
    P[0][1] -= k_0 * t1;
    P[1][0] -= k_1 * t0;
    P[1][1] -= k_1 * t1;

    // 5. 状态更新：融合加速度计角度与陀螺仪积分角度
    X.k_k = X.k_k_1 + k_0 * (InputAngle - X.k_k_1);
    // 更新陀螺仪偏差
    Q_bias = Q_bias + k_1 * (InputAngle - X.k_k_1);

    // 保存当前最优值为下一状态初始值
    X.k_1_k_1 = X.k_k;
	
    return X.k_k;
}

/**************************************************************
 * 卡尔曼滤波结构体定义（通用版）
 **************************************************************/
// 一维卡尔曼滤波状态结构体
typedef struct {
    float x;      // 状态值
    float p;      // 协方差
    float A;      // 状态转移矩阵
    float H;      // 观测矩阵
    float q;      // 过程噪声
    float r;      // 测量噪声
    float gain;   // 卡尔曼增益
} kalman1_state;

// 二维卡尔曼滤波状态结构体
typedef struct {
    float x[2];       // 状态向量（2个状态）
    float p[2][2];    // 2x2协方差矩阵
    float A[2][2];    // 2x2状态转移矩阵
    float H[2];       // 1x2观测矩阵
    float q[2];       // 过程噪声向量
    float r;          // 测量噪声
    float gain[2];    // 卡尔曼增益向量
} kalman2_state;

/**************************************************************
 * @brief  一维卡尔曼滤波初始化
 * @param  state: 滤波器状态结构体指针
 * @param  init_x: 初始状态值
 * @param  init_p: 初始协方差
 * @retval 无
 ***************************************************************/
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;          // 状态转移矩阵
    state->H = 1;          // 观测矩阵
    state->q = 2e2;        // 过程噪声
    state->r = 5e2;        // 测量噪声
}

/**************************************************************
 * @brief  一维卡尔曼滤波（通用版）
 * @param  state: 滤波器状态结构体指针
 * @param  z_measure: 测量值
 * @retval float: 滤波后的状态值
 ***************************************************************/
float kalman1_filter(kalman1_state *state, float z_measure)
{
    // 1. 状态预测
    state->x = state->A * state->x;
    // 2. 协方差预测
    state->p = state->A * state->A * state->p + state->q;  

    // 3. 计算卡尔曼增益
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    // 4. 状态更新
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    // 5. 协方差更新
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/**************************************************************
 * @brief  二维卡尔曼滤波初始化
 * @param  state: 滤波器状态结构体指针
 * @param  init_x: 初始状态向量
 * @param  init_p: 初始协方差矩阵
 * @retval 无
 **************************************************************/
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    // 初始化状态值
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    // 初始化协方差矩阵
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];

    // 状态转移矩阵
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;

    // 观测矩阵
    state->H[0]    = 1;
    state->H[1]    = 0;

    // 噪声配置
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7; 
}

/**************************************************************
 * @brief  二维卡尔曼滤波（通用版）
 * @param  state: 滤波器状态结构体指针
 * @param  z_measure: 测量值
 * @retval float: 滤波后的主状态值（x[0]）
 **************************************************************/
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0;
    float temp1;
    float temp;

    /* Step1: 状态预测 */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    
    /* Step1: 协方差预测 p(n|n-1)=A*p(n-1|n-1)*A' + q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: 计算卡尔曼增益 */
    /* gain = p * H^T * [r + H * p * H^T]^(-1) */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;

    /* Step3: 状态更新 */
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Step4: 协方差更新 p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}

/**************************************************************
 * 三维卡尔曼滤波（姿态融合专用）
 **************************************************************/
// 全局参数配置
float dtTimer   = 0.008;          // 采样时间（8ms）
float xk[9] = {0,0,0,0,0,0,0,0,0}; // 3x3状态矩阵（角度）
float pk[9] = {1,0,0,0,1,0,0,0,1}; // 3x3协方差矩阵
float I[9]  = {1,0,0,0,1,0,0,0,1}; // 3x3单位矩阵
float R[9]  = {0.5,0,0,0,0.5,0,0,0,0.01}; // 测量噪声矩阵
float Q[9] = {0.005,0,0,0,0.005,0,0,0,0.001}; // 过程噪声矩阵

/**************************************************************
 * @brief  3x3矩阵加法
 * @param  mata: 矩阵A
 * @param  matb: 矩阵B
 * @param  matc: 结果矩阵C = A+B
 * @retval 无
 **************************************************************/
void matrix_add(float* mata, float* matb, float* matc) {
    uint8_t i,j;
    for (i=0; i<3; i++) {
       for (j=0; j<3; j++) {
          matc[i*3+j] = mata[i*3+j] + matb[i*3+j];
       }
    }
}

/**************************************************************
 * @brief  3x3矩阵减法
 * @param  mata: 矩阵A
 * @param  matb: 矩阵B
 * @param  matc: 结果矩阵C = A-B
 * @retval 无
 **************************************************************/
void matrix_sub(float* mata, float* matb, float* matc) {
    uint8_t i,j;
    for (i=0; i<3; i++) {
       for (j=0; j<3; j++) {
          matc[i*3+j] = mata[i*3+j] - matb[i*3+j];
       }
    }
}

/**************************************************************
 * @brief  3x3矩阵乘法
 * @param  mata: 矩阵A
 * @param  matb: 矩阵B
 * @param  matc: 结果矩阵C = A*B
 * @retval 无
 **************************************************************/
void matrix_multi(float* mata, float* matb, float* matc) {
    uint8_t i,j,m;
    for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            matc[i*3+j] = 0.0;
            for (m=0; m<3; m++) {
                matc[i*3+j] += mata[i*3+m] * matb[m*3+j];
            }
        }
    }
}

/**************************************************************
 * @brief  三维卡尔曼滤波（姿态融合）
 * @param  am_angle_mat: 加速度计解算的角度矩阵（3x3）
 * @param  gyro_angle_mat: 陀螺仪积分的角度矩阵（3x3）
 * @retval 无（结果存储在xk全局数组）
 * @note   1. 融合加速度计角度（静态）和陀螺仪角度（动态）
 *         2. 实现完整的矩阵运算，适用于三维姿态解算
 **************************************************************/
void KalmanFilter(float* am_angle_mat, float* gyro_angle_mat)
{
    uint8_t i,j;
    float yk[9];         // 残差矩阵
    float pk_new[9];     // 新协方差矩阵
    float K[9];          // 卡尔曼增益矩阵
    float KxYk[9];       // K*yk
    float I_K[9];        // I-K
    float S[9];          // 中间矩阵
    float S_invert[9];   // S的逆矩阵
    float sdet;          // S的行列式
		 
    // 1. 状态预测：xk = xk + uk（陀螺仪积分角度）
    matrix_add(xk, gyro_angle_mat, xk);
    
    // 2. 协方差预测：pk = pk + Q
    matrix_add(pk, Q, pk);
    
    // 3. 计算残差：yk = 加速度计角度 - 预测角度
    matrix_sub(am_angle_mat, xk, yk);
    
    // 4. 计算S = Pk + R
    matrix_add(pk, R, S);
    
    // 5. 计算S的逆矩阵（伴随矩阵法）
    // 先计算行列式
    sdet = S[0] * S[4] * S[8]
          + S[1] * S[5] * S[6]
          + S[2] * S[3] * S[7]
          - S[2] * S[4] * S[6]
          - S[5] * S[7] * S[0]
          - S[8] * S[1] * S[3];
		 
    // 计算逆矩阵（伴随矩阵 / 行列式）
    S_invert[0] = (S[4] * S[8] - S[5] * S[7])/sdet;
    S_invert[1] = (S[2] * S[7] - S[1] * S[8])/sdet;
    S_invert[2] = (S[1] * S[7] - S[4] * S[6])/sdet;
		 
    S_invert[3] = (S[5] * S[6] - S[3] * S[8])/sdet;
    S_invert[4] = (S[0] * S[8] - S[2] * S[6])/sdet;
    S_invert[5] = (S[2] * S[3] - S[0] * S[5])/sdet;
		 
    S_invert[6] = (S[3] * S[7] - S[4] * S[6])/sdet;
    S_invert[7] = (S[1] * S[6] - S[0] * S[7])/sdet;
    S_invert[8] = (S[0] * S[4] - S[1] * S[3])/sdet;
    
    // 6. 计算卡尔曼增益：K = Pk * S_invert
    matrix_multi(pk, S_invert, K);
    
    // 7. 计算K*yk
    matrix_multi(K, yk, KxYk);
    
    // 8. 状态更新：xk = xk + K * yk
    matrix_add(xk, KxYk, xk);
    
    // 9. 协方差更新：pk = (I - K) * pk
    matrix_sub(I, K, I_K);
    matrix_multi(I_K, pk, pk_new);
    
    // 10. 更新协方差矩阵
    for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            pk[i*3+j] = pk_new[i*3+j];
        }
    }
}


