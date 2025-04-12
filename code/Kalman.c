/*
* Kalman.c
*
*  Created on: 2025年2月5日
*      Author: 20483
*/

#include "zf_common_headfile.h"



volatile float q0, q1, q2, q3, w1, w2, w3;  // 全局四元数
float angle[3] = {0};
float Ki_Ah = 0.2;
float Kp_Ah = 0.8;

#if AHRS_MAG_ENABLE
float A[49], B[49], E[42], FF[36], X[49], Z[49], Ht[42], Ft[49], K[42], O[49], T[6], F[49], Y[7], P1[49], U1[36], U1t[36], DD[36], X1[36], X2[36];

float P[49] = {0.0001, 0, 0, 0, 0, 0, 0,
               0, 0.0001, 0, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0, 0.0002, 0, 0,
               0, 0, 0, 0, 0, 0.0002, 0,
               0, 0, 0, 0, 0, 0, 0.0002};

float Q[49] = {0.0001, 0, 0, 0, 0, 0, 0,
               0, 0.0001, 0, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0, 0.0005, 0, 0,
               0, 0, 0, 0, 0, 0.0005, 0,
               0, 0, 0, 0, 0, 0, 0.0005};

float R[36] = {0.003, 0, 0, 0, 0, 0,
               0, 0.003, 0, 0, 0, 0,
               0, 0, 0.003, 0, 0, 0,
               0, 0, 0, 0.005, 0, 0,
               0, 0, 0, 0, 0.005, 0,
               0, 0, 0, 0, 0, 0.005};

float H[42] = {0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0};

float I[49] = {1, 0, 0, 0, 0, 0, 0,
               0, 1, 0, 0, 0, 0, 0,
               0, 0, 1, 0, 0, 0, 0,
               0, 0, 0, 1, 0, 0, 0,
               0, 0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 0, 1};
#endif

/****************************************************************************************************
//  @brief      快速计算1/Sqrt(x)
//  @param      x                      变量
//  @return     y                      1/Sqrt(x)
//  @since
//  Sample usage:
****************************************************************************************************/
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/****************************************************************************************************
//  @brief      IMU相关初始化
//  @param      void                      
//  @return     void                   
//  @since
//  Sample usage:
****************************************************************************************************/
void AHRS_init()
{
    // 陀螺仪偏差
    // w1 = Gyro_Offset.Xdata;
    // w2 = Gyro_Offset.Ydata;
    // w3 = Gyro_Offset.Zdata;
    w1 = 0;
    w2 = 0;
    w3 = 0;

    q0 = 1.0;
    q1 = 0;
    q2 = 0;
    q3 = 0;
}

#if AHRS_MAG_ENABLE
void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float g = 9.78784;
    float Ha1, Ha2, Ha3, Ha4, Hb1, Hb2, Hb3, Hb4;
    float e1, e2, e3, e4, e5, e6;
    float halfT;

    // 先把这些用得到的值算好
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 地区磁场
    bx = 0.5500;
    bz = 0.8351;
    halfT = 0.0025;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm * g;
    ay = ay * norm * g;
    az = az * norm * g;

    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    gx = gx - w1;
    gy = gy - w2;
    gz = gz - w3;
/*****************************************************************************************************************************************
	黄小平 《卡尔曼滤波原理与应用》4.2.1节 P80页【EKF滤波流程】
	第5步：观测转移矩阵赋值
	H = dH(X)/dX
	=============================================================================
	=============================重力加速度======================================
	=============================================================================
	根据四元数表示的姿态角计算重力加速度在固联坐标系下的预测值
	q0q0 + q1q1 + q2q2 + q3q3 = 1
	Reb表示RotationEarthFromBody 从固联坐标系转为地球坐标系(右手坐标系)
	Rbe表示RotationBodyFromEarth 从地球坐标系转为固联坐标系(右手坐标系)
				|q0q0+q1q1-q2q2-q3q3  	2(q1q2+q0q3)  				2(q1q3-q0q2)| 
	Rbe = 		|2(q1q2-q0q3)   		q0q0-q1q1+q2q2-q3q3  		2(q2q3+q0q1)| 【这是右乘矩阵： b' = R(rigth)*a'】
				|2(q1q3+q0q2)  			2(q2q3-q0q1)  		 q0q0-q1q1-q2q2+q3q3|	惯性导航P295页所列的为Reb的右乘矩阵（固联坐标系到地球坐标系）
	
	[Gx Gy Gz]'=Rbe*[0 0 g]'
	
	因此：（其中vx vy vz为重力加速度在固联坐标系的投影）
	vx = 2*(q1q3 - q0q2)*g;
	vy = 2*(q0q1 + q2q3)*g;
	vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
	=============================================================================
	===============================磁场强度======================================
	=============================================================================
	同理：根据四元数表示的姿态角计算磁场强度在固联坐标系下的预测值
	Reb表示RotationEarthFromBody 从固联坐标系转为地球坐标系(右手坐标系)
	Rbe表示RotationBodyFromEarth 从地球坐标系转为固联坐标系(右手坐标系)
				|q0q0+q1q1-q2q2-q3q3  	2(q1q2+q0q3)  					2(q1q3-q0q2)|
	Rbe = 		|2(q1q2-q0q3)   		q0q0-q1q1+q2q2-q3q3  			2(q2q3+q0q1)|【这是右乘矩阵： b' = R(rigth)*a'】
				|2(q1q3+q0q2)  			2(q2q3-q0q1)  			 q0q0-q1q1-q2q2+q3q3| 惯性导航P295页所列的为Reb的右乘矩阵（固联坐标系到地球坐标系）
	[Mx My Mz]'=Rbe*[bx 0 bz]'
	
	因此：(其中 wx wy wz 为磁场强度在固联坐标系的投影)
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	=============================================================================
	=============================================================================
	=============================================================================
	则有：
			|@ax/@q0		@ax/@q1			@ax/@q2  		@ax/@q3			@ax/@w1			@ax/@w2  		@ax/@w3	|
			|@ay/@q0		@ay/@q1  		@ay/@q2  		@ay/@q3			@ay/@w1			@ay/@w2  		@ay/@w3	|
	 H =  	|@az/@q0 		@az/@q1			@az/@q2 		@az/@q3			@az/@w1			@az/@w2  		@az/@w3	|
			|@mx/@q0		@mx/@q1  		@mx/@q2  		@mx/@q3			@mx/@w1			@mx/@w2  		@mx/@w3	|
			|@my/@q0		@my/@q1  		@my/@q2  		@my/@q3			@my/@w1			@my/@w2  		@my/@w3	|
			|@mz/@q0		@mz/@q1  		@mz/@q2  		@mz/@q3			@mz/@w1			@mz/@w2  		@mz/@w3	|

			|	-q2*g			q3*g			-q0*g  			q1*g		0	0  	0	|这里第四个多了个负号，实际代码正确的
			|	q1*g			q0*g  			q3*g  			q2*g		0	0  	0	|
		= 2*|	q0*g 	 		-q1*g			-q2*g 			q3*g		0	0  	0	|
			|bx*q0-bz*q2	bx*q1+bz*q3  	-bx*q2-bz*q0  -bx*q3+bz*q1		0	0  	0	|
			|-bx*q3+bz*q1	bx*q2+bz*q0  	bx*q1+bz*q3 	-bx*q0+bz*q2	0	0  	0	|
			|bx*q2+bz*q0	bx*q3-bz*q1  	bx*q0-bz*q2  	bx*q1+bz*q3		0	0  	0	|
	=============================================================================
	=============================================================================
	=============================================================================
	*/

    Ha1 = 2 * (-q2) * g;
    Ha2 = 2 * ( q3) * g;
    Ha3 = 2 * (-q0) * g;
    Ha4 = 2 * ( q1) * g;
    Hb1 = 2 * ( bx * q0 - bz * q2);
    Hb2 = 2 * ( bx * q1 + bz * q3);
    Hb3 = 2 * (-bx * q2 - bz * q0);
    Hb4 = 2 * (-bx * q3 + bz * q1);

    H[0]  =  Ha1; H[1]  =  Ha2; H[2]  = Ha3; H[3]  =  Ha4;  // H[4]  = 0; H[5]  = 0; H[6]  = 0;
    H[7]  =  Ha4; H[8]  = -Ha3; H[9]  = Ha2; H[10] = -Ha1;  // H[11] = 0; H[12] = 0; H[13] = 0;
    H[14] = -Ha3; H[15] = -Ha4; H[16] = Ha1; H[17] =  Ha2;  // H[18] = 0; H[19] = 0; H[20] = 0;
    H[21] =  Hb1; H[22] =  Hb2; H[23] = Hb3; H[24] =  Hb4;  // H[25] = 0; H[26] = 0; H[27] = 0;
    H[28] =  Hb4; H[29] = -Hb3; H[30] = Hb2; H[31] = -Hb1;  // H[32] = 0; H[33] = 0; H[34] = 0;
    H[35] = -Hb3; H[36] = -Hb4; H[37] = Hb1; H[38] =  Hb2;  // H[39] = 0; H[40] = 0; H[41] = 0;

    // Ha1 = -q2 * g;
    // Ha2 =  q3 * g;
    // Ha3 = -q0 * g;
    // Ha4 =  q1 * g;
    // Hb1 =  bx * q0 - bz * q2;
    // Hb2 =  bx * q1 + bz * q3;
    // Hb3 = -bx * q2 - bz * q0;
    // Hb4 = -bx * q3 + bz * q1;

    // H[0]  = Ha1;
    // H[1]  = Ha2;
    // H[2]  = Ha3;
    // H[3]  = Ha4;
    // H[7]  = Ha4;
    // H[8]  = -Ha3;
    // H[9]  = Ha2;
    // H[10] = -Ha1;
    // H[14] = -Ha3;
    // H[15] = -Ha4;
    // H[16] = Ha1;
    // H[17] = Ha2;
    // H[21] = Hb1;
    // H[22] = Hb2;
    // H[23] = Hb3;
    // H[24] = Hb4;
    // H[28] = Hb4;
    // H[29] = -Hb3;
    // H[30] = Hb2;
    // H[31] = -Hb1;
    // H[35] = -Hb3;
    // H[36] = -Hb4;
    // H[37] = Hb1;
    // H[38] = Hb2;

    // 状态更新
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;
    // 四元数归一
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    // F阵赋值
    F[ 0] =   1;         F[ 1] = -gx * halfT;  F[ 2] = -gz * halfT;  F[ 3] = -gz * halfT;  F[ 4] = 0;   F[ 5] = 0;   F[ 6] = 0;
    F[ 7] = gx * halfT;  F[ 8] =    1;         F[ 9] =  gz * halfT;  F[10] = -gy * halfT;  F[11] = 0;   F[12] = 0;   F[13] = 0;
    F[14] = gy * halfT;  F[15] = -gz * halfT;  F[16] =    1;         F[17] =  gx * halfT;  F[18] = 0;   F[19] = 0;   F[20] = 0;
    F[21] = gz * halfT;  F[22] =  gy * halfT;  F[23] = -gx * halfT;  F[24] =    1;         F[25] = 0;   F[26] = 0;   F[27] = 0;
    F[28] =   0;         F[29] =    0;         F[30] =    0;         F[31] =    0;         F[32] = 1;   F[33] = 0;   F[34] = 0;
    F[35] =   0;         F[36] =    0;         F[37] =    0;         F[38] =    0;         F[39] = 0;   F[40] = 1;   F[41] = 0;
    F[42] =   0;         F[43] =    0;         F[44] =    0;         F[45] =    0;         F[46] = 0;   F[47] = 0;   F[48] = 1;

    // 卡尔曼滤波
    MatrixMultiply(F, 7, 7, P, 7, 7, A);        // A=F*P
    MatrixTranspose(F, 7, 7, Ft);               // F转置F'
    MatrixMultiply(A, 7, 7, Ft, 7, 7, B);       // B=A*F'=F*P*F'
    MatrixAdd(B, Q, P1, 7, 7);
    MatrixTranspose(H, 6, 7, Ht);               // H转置H'
    MatrixMultiply(P1, 7, 7, Ht, 7, 6, E);      // E=P*H'
    MatrixMultiply(H, 6, 7, E, 7, 6, FF);       // FF=H*P*H' 6*6
    MatrixAdd(FF, R, X, 6, 6);                  // X=FF+R 6*6
    UD(X, 6, U1, DD);                           // X的UD分解
    MatrixTranspose(U1, 6, 6, U1t);             // U1的转置
    MatrixMultiply(U1, 6, 6, DD, 6, 6, X1);     // X1=U1*DD
    MatrixMultiply(X1, 6, 6, U1t, 6, 6, X2);    // X2=U1*DD*U1t
    MatrixInverse(X2, 6, 0);                    // X逆
    MatrixMultiply(E, 7, 6, X2, 6, 6, K);       // 增益K   7*6

    vx = 2 * (q1q3 - q0q2) * g;
    vy = 2 * (q0q1 + q2q3) * g;
    vz = (q0q0 - q1q1 - q2q2 + q3q3) * g;

    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    e1 = ax - vx;
    e2 = ay - vy;
    e3 = az - vz;
    e4 = mx - wx;
    e5 = my - wy;
    e6 = mz - wz;
    T[0] = e1;
    T[1] = e2;
    T[2] = e3;
    T[3] = e4;
    T[4] = e5;
    T[5] = e6;
    MatrixMultiply(K, 7, 6, T, 6, 1, Y); // Y=K*(Z-Y) 7*1
    q0 = q0 + Y[0];
    q1 = q1 + Y[1];
    q2 = q2 + Y[2];
    q3 = q3 + Y[3];
    w1 = w1 + Y[4];
    w2 = w2 + Y[5];
    w3 = w3 + Y[6];

    MatrixMultiply(K, 7, 6, H, 6, 7, Z); // Z= K*H 7*7
    MatrixSub(I, Z, O, 7, 7); // O=I-K*H
    MatrixMultiply(O, 7, 7, P1, 7, 7, P);

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
}
#else
void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    float exInt = 0;
    float eyInt = 0;
    float ezInt = 0;
    float halfT = 0.0025;

    if(ax*ay*az==0)
    {
        return;
    }

    // 第一步：对加速度数据进行归一化
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // 第二步：DCM矩阵旋转
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 第三步：在机体坐标系下做向量叉积得到补偿数据
    ex = ay * vz - az * vy ;
    ey = az * vx - ax * vz ;
    ez = ax * vy - ay * vx ;

    // 第四步：对误差进行PI计算，补偿角速度
    exInt = exInt + ex * Ki_Ah;
    eyInt = eyInt + ey * Ki_Ah;
    ezInt = ezInt + ez * Ki_Ah;

    gx = gx + Kp_Ah * ex + exInt;
    gy = gy + Kp_Ah * ey + eyInt;
    gz = gz + Kp_Ah * ez + ezInt;

    // 第五步：按照四元数微分公式进行四元数更新
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
}
#endif

/****************************************************************************************************
//  @brief      获取四元数
//  @param      q                     四元数数组
//  @since
//  Sample usage:
****************************************************************************************************/
void AHRS_getQ(float * q)
{
    AHRS_AHRSupdate(ANGLE_TO_RAD(imu963ra_gyro_transition(imu963ra_gyro_x - Gyro_Offset.Xdata)),
                    ANGLE_TO_RAD(imu963ra_gyro_transition(imu963ra_gyro_y - Gyro_Offset.Ydata)),
                    ANGLE_TO_RAD(imu963ra_gyro_transition(imu963ra_gyro_z - Gyro_Offset.Zdata)),
                    imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,
                    imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z);
    // 将陀螺仪的测量值转成弧度每秒
    // 加速度和磁力计保持ADC值不需要转换

    q[0] = q0; // 返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}



/****************************************************************************************************
//  @brief      获取姿态角
//  @param      angles                姿态角数组
//  @since
//  Sample usage:
****************************************************************************************************/
void AHRS_getYawPitchRoll(float * angles)
{
    imu963ra_get_acc();     // 获取加速度计数据
    imu963ra_get_gyro();    // 获取陀螺仪数据
    imu963ra_get_mag();     // 获取磁力计数据

    float q[4];             // 四元数
    AHRS_getQ(q);           // 更新全局四元数
    angles[0] =  atan2(2 * q[0] * q[1] + 2 * q[2] * q[3], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180 / PI;// 横滚角roll
    angles[1] =  asin( 2 * q[0] * q[2] - 2 * q[1] * q[3]) * 180 / PI;                                        // 俯仰角pitch
    angles[2] = -atan2(2 * q[0] * q[3] + 2 * q[1] * q[2], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / PI;// 偏航角yaw

    // angle[2] -= ((int16)System_Time / 10) * 0.122;
    if(angle[2] < -180)
    {
        angle[2] += 360;
    }
    if(angle[2] > 180)
    {
        angle[2] -= 360;
    }
}


#if AHRS_MAG_ENABLE
/****************************************************************************************************
//  @brief      矩阵加法
//  @param      fMatrixA              第一个矩阵
//  @param      fMatrixB              第二个矩阵
//  @param      Result                结果矩阵
//  @param      m                     矩阵的行数
//  @param      n                     矩阵的列数
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixAdd(float* fMatrixA, float* fMatrixB, float* Result, unsigned int m, unsigned int n)
{
    unsigned int index_i = 0;
    unsigned int index_j = 0;
    unsigned int itemp = 0;

    for (index_i = 0; index_i < m; index_i++)
        for (index_j = 0; index_j < n; index_j++)
        {
            itemp = index_i * n + index_j;
            *(Result + itemp) = *(fMatrixA + itemp) + *(fMatrixB + itemp);
        }
} 


/****************************************************************************************************
//  @brief      矩阵减法
//  @param      fMatrixA              第一个矩阵
//  @param      fMatrixB              第二个矩阵
//  @param      Result                结果矩阵
//  @param      m                     矩阵的行数
//  @param      n                     矩阵的列数
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixSub(float* fMatrixA, float* fMatrixB, float* Result, unsigned int m, unsigned int n)
{
    unsigned int index_i = 0;
    unsigned int index_j = 0;
    unsigned int itemp = 0;

    for (index_i = 0; index_i < m; index_i++)
    {
        for (index_j = 0; index_j < n; index_j++)
        {
            itemp = index_i * n + index_j;
            *(Result + itemp) = *(fMatrixA + itemp) - *(fMatrixB + itemp);
        }
    }
}


/****************************************************************************************************
//  @brief      矩阵乘法
//  @param      fMatrixA              第一个矩阵
//  @param      uRowA                 第一个矩阵的行数
//  @param      uColA                 第一个矩阵的列数
//  @param      fMatrixB              第二个矩阵
//  @param      uRowB                 第二个矩阵的行数
//  @param      uColB                 第二个矩阵的列数
//  @param      MatrixResult          结果矩阵
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixMultiply(float* fMatrixA, unsigned int uRowA, unsigned int uColA,
                    float* fMatrixB, unsigned int uRowB, unsigned int uColB,
                    float* MatrixResult)
{
    unsigned int index_i = 0;
    unsigned int index_j = 0;
    unsigned int index_l = 0;
    unsigned int index_u = 0;
    unsigned int index_k = 0;
    unsigned int index_v = 0;

    for (index_i = 0; index_i < uRowA; index_i++)
    {
        for (index_j = 0; index_j < uColB; index_j++)
        {
            index_u = index_i * uColB + index_j;
            MatrixResult[index_u] = 0.0;

            for (index_l = 0; index_l < uColA; index_l++)
            {
                index_k = index_i * uColA + index_l;
                index_v = index_l * uColB + index_j;

                *(MatrixResult + index_u) += (*(fMatrixA + index_k)) * (*(fMatrixB + index_v));
            }
        }
    }
}


/****************************************************************************************************
//  @brief      矩阵转置
//  @param      fMatrixA              输入矩阵
//  @param      m                     输入矩阵的行数
//  @param      n                     输入矩阵的列数
//  @param      fMatrixB              输出矩阵
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixTranspose(float* fMatrixA, unsigned int m, unsigned int n, float* fMatrixB)
{
    unsigned int index_i = 0;
    unsigned int index_j = 0;
    unsigned int index_u = 0;
    unsigned int index_v = 0;

    for (index_i = 0; index_i < m; index_i++)
    {
        for (index_j = 0; index_j < n; index_j++)
        {
            index_u = index_j * m + index_i;
            index_v = index_i * n + index_j;
            *(fMatrixB + index_u) = *(fMatrixA + index_v);
        }
    }
}


/****************************************************************************************************
//  @brief      矩阵乘法
//  @param      A                     第一个矩阵
//  @param      m                     第一个矩阵的行数
//  @param      n                     第一个矩阵的列数
//  @param      B                     第二个矩阵
//  @param      k                     第二个矩阵的列数
//  @param      C                     结果矩阵
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixProduct(float* A, int m, int n, float* B, int k, float* C)
{
    int tem = 0;
    int i = 0;
    int j = 0;
    int l = 0;
    int met = 0;

    for (tem = 0; tem < m; tem++)
    {
        for (met = 0; met < k; met++)
        {
            C[tem * k + met] = 0.;
        }
    }

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < k; j++)
        {
            for (l = 0; l < n; l++)
            {
                C[i * k + j] = C[i * k + j] + A[i * n + l] * B[l * k + j];
            }
        }
    }
}


/****************************************************************************************************
//  @brief      生成单位矩阵
//  @param      fMatrixA              输入矩阵
//  @param      n                     矩阵的维度
//  @since
//  Sample usage:
****************************************************************************************************/
void MatrixE(float* fMatrixA, unsigned int n)
{
    unsigned int index_i = 0;
    unsigned int index_j = 0;

    for (; index_i < n; index_i++)
    {
        for (index_j = 0; index_j < n; index_j++)
        {
            if (index_i == index_j)
                *(fMatrixA + index_i * n + index_j) = 1;
            else
                *(fMatrixA + index_i * n + index_j) = 0;
        }
    }
}


/****************************************************************************************************
//  @brief      计算二阶矩阵的行列式
//  @param      fMatrixA              输入矩阵
//  @since
//  Sample usage:
****************************************************************************************************/
double MatrixDet2(float* fMatrixA)
{
    return (*fMatrixA) * (*(fMatrixA + 3)) - (*(fMatrixA + 1)) * (*(fMatrixA + 2));
}


/****************************************************************************************************
//  @brief      计算二阶矩阵的逆矩阵
//  @param      fMatrixA              输入矩阵
//  @param      fMatrixB              输出矩阵
//  @return     0：成功，1：失败
//  @since
//  Sample usage:
****************************************************************************************************/
int MatrixInverse2(float* fMatrixA, float* fMatrixB)
{
    float fmatrix_vlaue = 0.0;

    fmatrix_vlaue = MatrixDet2(fMatrixA);

    if (fmatrix_vlaue == 0.0)
    {
        return 1;
    }

    *fMatrixB = *(fMatrixA + 3) / fmatrix_vlaue;
    *(fMatrixB + 1) = -(*(fMatrixA + 2) / fmatrix_vlaue);
    *(fMatrixB + 2) = -(*(fMatrixA + 1) / fmatrix_vlaue);
    *(fMatrixB + 3) = -(*(fMatrixA) / fmatrix_vlaue);
    return 0;
}


/****************************************************************************************************
//  @brief      计算矩阵的逆矩阵
//  @param      fMatrixA              输入矩阵
//  @param      n                     矩阵的维度
//  @param      ep                    精度
//  @return     0：成功，1：失败
//  @since
//  Sample usage:
****************************************************************************************************/
int MatrixInverse(float* fMatrixA, int n, float ep)
{
    int i = 0;
    int j = 0;
    int k = 0;
    int l = 0;
    int u = 0;
    int v = 0;
    int temp_row[10], temp_col[10];
    int *prow = temp_row, urow = 0;    //找到的最大值的行数
    int *pcol = temp_col, ucol = 0;    //找到的最大值的列数
    float ftemp_max = 0.0;
    float ftemp = 0.0;

    // ep = ep;                    //防止ep没有使用的警告

    for (i = 0; i < 10; i++)
    {
        temp_row[i] = 0;
        temp_col[i] = 0;
    }

    for (k = 0; k < n; k++)
    {
        ftemp_max = 0.0;    //找到剩余矩阵项中的最大值
        for (i = k; i < n; i++)
            for (j = k; j < n; j++)
            {
                l = i * n + j;
                ftemp = fabs(*(fMatrixA + l));
                if (ftemp_max < ftemp)
                {
                    ftemp_max = ftemp;
                    *(prow + k) = urow = i;
                    *(pcol + k) = ucol = j;
                }
            }

        if (ftemp_max < ep)
        {
            return 0;
        }

        if (urow != k)    //如果当前行不包含最大值换行
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = urow * n + j;
                ftemp = *(fMatrixA + u);
                *(fMatrixA + u) = *(fMatrixA + v);
                *(fMatrixA + v) = ftemp;
            }
        }

        if (ucol != k)    //如果当前列不包含最大值换列
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + ucol;
                ftemp = *(fMatrixA + u);
                *(fMatrixA + u) = *(fMatrixA + v);
                *(fMatrixA + v) = ftemp;
            }
        }

        l = k * n + k;
        ftemp = *(fMatrixA + l) = 1.0 / (*(fMatrixA + l));

        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                *(fMatrixA + u) *= *(fMatrixA + l);
            }
        }

        for (i = 0; i < n; i++)    //还不知道为什么
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        *(fMatrixA + u) -= (*(fMatrixA + i * n + k)) * (*(fMatrixA + k * n + j));
                    }
                }
            }
        }

        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                *(fMatrixA + u) = -(*(fMatrixA + u)) * (*(fMatrixA + l));
            }
        }
    }

    for (k = n - 1; k >= 0; k--)
    {
        if ((*(pcol + k)) != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = (*(pcol + k)) * n + j;
                ftemp = *(fMatrixA + u);
                *(fMatrixA + u) = *(fMatrixA + v);
                *(fMatrixA + v) = ftemp;
            }
        }

        if ((*(prow + k)) != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + (*(prow + k));
                ftemp = *(fMatrixA + u);
                *(fMatrixA + u) = *(fMatrixA + v);
                *(fMatrixA + v) = ftemp;
            }
        }
    }
    return 1;
}


/****************************************************************************************************
//  @brief      矩阵UD分解
//  @param      A                     输入矩阵
//  @param      n                     矩阵的维度
//  @param      U                     U矩阵
//  @param      D                     D矩阵
//  @since
//  Sample usage:
****************************************************************************************************/
void UD(float* A, int n, float* U, float* D)
{
    int i = 0;
    int j = 0;
    int k = 0;
    float sum1 = 0.0;
    float sum2 = 0.0;
    int m = n - 1;

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                U[i * n + j] = 1;
                D[i * n + j] = 0;
            }
            else
            {
                D[i * n + j] = 0;
                U[i * n + j] = 0;
            }
        }
    }

    D[m * n + m] = A[m * n + m];

    for (i = 0; i < n; i++)
    {
        U[i * n + m] = A[i * n + m] / D[m * n + m];
    }

    for (j = n - 2; j >= 0; j--)
    {
        for (k = j + 1; k < n; k++)
        {
            sum2 = sum2 + D[k * n + k] * U[j * n + k] * U[j * n + k];
        }

        D[j * n + j] = A[j * n + j] - sum2;
        sum2 = 0;

        for (i = 0; i < j; i++)
        {
            for (k = j + 1; k < n; k++)
            {
                sum1 = sum1 + D[k * n + k] * U[i * n + k] * U[j * n + k];
            }

            U[i * n + j] = (A[i * n + j] - sum1) / D[j * n + j];
            sum1 = 0;
        }
    }
}


/****************************************************************************************************
//  @brief      求矩阵范数
//  @param      fMatrixA              输入矩阵
//  @param      iRow                  矩阵的行数
//  @param      iCol                  矩阵的列数
//  @return     矩阵的范数
//  @since
//  Sample usage:
****************************************************************************************************/
float Norm(float* fMatrixA, int iRow, int iCol)
{
    int i = 0;
    int j = 0;
    int index = 0;
    float local_result = 0.0;

    for (i = 0; i < iRow; i++)
    {
        for (j = 0; j < iCol; j++)
        {
            index = i * iCol + j;
            local_result += (*(fMatrixA + index)) * (*(fMatrixA + index));
        }
    }
    local_result = sqrt(local_result);

    return local_result;
}
#endif