/*
 * IMU.c
 *
 *  Created on: 2025年1月7日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

gyro_param_t Gyro_Offset;       // 零飘参数
IMU_param_t  IMU_Data;          // 加速度 角速度
float MAX_IMU_Data_gyro_z;      // Z轴抖动最大值
float gyro_Offset_flag = 0;     // 标志位
float Z_360 = 0;                // Z轴规划

// 零飘初始化
float IMU_gyro_Offset_Init()
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;
    // 循环500次，以大量样本为基础计算平均值，提高精度
    for(uint16_t i = 0; i < 500; i++)
    {
        imu963ra_get_gyro();
        Gyro_Offset.Xdata += imu963ra_gyro_x;
        Gyro_Offset.Ydata += imu963ra_gyro_y;
        Gyro_Offset.Zdata += imu963ra_gyro_z;
        system_delay_ms(5);
    }
    Gyro_Offset.Xdata /= 500;
    Gyro_Offset.Ydata /= 500;
    Gyro_Offset.Zdata /= 500;

    return gyro_Offset_flag = 1;
}

// 积分出角度值
void IMU_YAW_integral()
{
    // 陀螺仪角速度转换为弧度制角速度 : deg/s --> rad/s
    // 将采集到的数值转化为实际物理值，并对陀螺仪进行去零飘处理
    // 调试用
    if(IMU_Data.gyro_z > MAX_IMU_Data_gyro_z)
    {
        MAX_IMU_Data_gyro_z = IMU_Data.gyro_z;
    }

    IMU_Data.gyro_z = imu963ra_gyro_transition(imu963ra_gyro_z - Gyro_Offset.Zdata);   // 需要修改，应该不需要 * PI / 180
    if(fabs(IMU_Data.gyro_z) >= 0.25)
    {
        Z_360 -= IMU_Data.gyro_z * 0.005;
    }

    while(Z_360 < 0)
    {
        Z_360 += 360;
    }
    while(Z_360 > 360)
    {
        Z_360 -= 360;
    }

}

// IMU初始化
void IMU_Init()
{
    imu963ra_init();
    IMU_gyro_Offset_Init();         // 零飘初始化
}







