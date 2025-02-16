/*
 * Kalman.h
 *
 *  Created on: 2025年2月5日
 *      Author: 20483
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_

//===================================================类型定义BEG===================================================
typedef struct                                                 // 卡尔曼零偏参数结构体
{
    float Xdata;                                               // 零偏参数X
    float Ydata;                                               // 零偏参数Y
    float Zdata;                                               // 零偏参数Z
} kalman_param_t;
//===================================================类型定义END===================================================


//===================================================全局变量BEG===================================================
extern float angle[3];
extern kalman_param_t Kalman_Offset;
extern float kalman_Offset_flag;
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
float  invSqrt(float x);                                        // 快速计算1/Sqrt(x)
void   AHRS_init(void);                                         // 初始化IMU相关
void   Kalman_Offset_Init(void);                                // 卡尔曼零偏初始化
void   AHRS_getYawPitchRoll(float *angle);                      // 更新AHRS 更新四元数
void   MatrixAdd(float* fMatrixA, float* fMatrixB, float* Result, unsigned int m, unsigned int n);
void   MatrixSub(float* fMatrixA, float* fMatrixB, float* Result, unsigned int m, unsigned int n);
void   MatrixMultiply(float* fMatrixA, unsigned int uRowA, unsigned int uColA, float* fMatrixB, unsigned int uRowB, unsigned int uColB, float* MatrixResult);
void   MatrixTranspose(float* fMatrixA, unsigned int m, unsigned int n, float* fMatrixB);
void   MatrixProduct(float* A, int m, int n, float* B, int k, float* C);
void   MatrixE(float* fMatrixA, unsigned int n);
double MatrixDet2(float* fMatrixA);
int    MatrixInverse2(float* fMatrixA, float* fMatrixB);
int    MatrixInverse(float* fMatrixA, int n, float ep);
void   UD(float* A, int n, float* U, float* D);
float  Norm(float* fMatrixA, int iRow, int iCol);
//===================================================函数声明END===================================================


#endif /* CODE_KALMAN_H_ */
