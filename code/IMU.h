/*
 * IMU.h
 *
 *  Created on: 2025年1月7日
 *      Author: 20483
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

//===================================================类型定义BEG===================================================
typedef struct                                                 // 陀螺仪零偏参数结构体
{
    float Xdata;                                               // 零偏参数X
    float Ydata;                                               // 零偏参数Y
    float Zdata;                                               // 零偏参数Z
} gyro_param_t;


typedef struct                                                 // IMU参数结构体
{
    float acc_x;                                               // x轴加速度
    float acc_y;                                               // y轴加速度
    float acc_z;                                               // z轴加速度

    float gyro_x;                                              // x轴角速度
    float gyro_y;                                              // y轴角速度
    float gyro_z;                                              // z轴角速度
} IMU_param_t;
//===================================================类型定义END===================================================


//===================================================全局变量BEG===================================================
extern IMU_param_t IMU_Data;                                   // IMU数据
extern gyro_param_t Gyro_Offset;                               // 零飘参数
extern float gyro_Offset_flag;                                 // 标志位，用于指示陀螺仪零偏校准是否完成
extern float MAX_IMU_Data_gyro_z;                              // Z轴抖动最大值
extern float Z_360;                                            // 用于存储Z轴的360度积分值
//===================================================全局变量END===================================================



//===================================================函数声明BEG===================================================
float IMU_gyro_Offset_Init(void);                              // 陀螺仪零偏校准初始化
void IMU_YAW_integral(void);                                   // 计算Z轴的360度积分
void IMU_Init(void);                                           // IMU初始化
//===================================================函数声明END===================================================

#endif /* CODE_IMU_H_ */
