/*
 * Kalman.h
 *
 *  Created on: 2025年2月5日
 *      Author: 20483
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_

//===================================================宏定义BEG===================================================
#define AHRS_MAG_ENABLE                 (0)                  // 磁力计是否启用，0不启用，1启用
//===================================================宏定义END===================================================


//===================================================全局变量BEG===================================================
extern float angle[3];
extern float Ki_Ah;
extern float Kp_Ah;
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
float  invSqrt(float x);                                        // 快速计算1/Sqrt(x)
void   AHRS_init(void);                                         // 初始化IMU相关
void   AHRS_getYawPitchRoll(float *angle);                      // 更新AHRS 更新四元数
#if AHRS_MAG_ENABLE == 1
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
#endif
//===================================================函数声明END===================================================


#endif /* CODE_KALMAN_H_ */
