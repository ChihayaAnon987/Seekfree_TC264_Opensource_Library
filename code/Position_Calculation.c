/*
 * Position_Calculation.c
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#include "zf_common_headfile.h"


int Track_Points_NUM   =   0;       // 当前追踪第几个点
double Angle_Error     =   0;       // 方向角与航向角之差
float  Fusion_angle    =   0;       // GPS和IMU互补滤波后的角度
float  Fusion_alpha    = 0.9;       // GPS和IMU互补滤波的权重
int16  Target_Encoder  =   0;       // 转速
int16  Fly_Slope_Alpha = 200;       // 飞坡系数
float  K_Straight      = 1.2;       // 走直线系数


/****************************************************************************************************
//  @brief      将积分的Z_360和逐飞GPS的direction进行互补融合
//  @param      void
//  @return     void
//  @since
//  Sample usage:
****************************************************************************************************/
void GPS_IMU_Complementary_Filtering()
{
    if(Z_360 > 180)
    {
        Z_360 -= 360;
    }
    if(gnss.direction > 180)
    {
        gnss.direction -= 360;
    }
    Fusion_angle = Fusion_alpha * Z_360 + (1 - Fusion_alpha) * gnss.direction;
    if(Fusion_angle > 180)
    {
        Fusion_angle -= 360;
    }
    if(Fusion_angle < -180)
    {
        Fusion_angle += 360;
    }
}

/****************************************************************************************************
//  @brief      核心循迹逻辑
//  @param      void
//  @return     void
//  @since
//  Sample usage:
****************************************************************************************************/
void Track_Follow()
{
    // 计算从第一个点到第二个点的方位角(单位：°)
    // 计算从第一个点到第二个点的距离(单位：m)
    // Distance 作为切换点位的依据

    // 调试用
    // Angle = Test_Angle;

    if((Angle - angle[2]) > 180)
    {
        Angle_Error = Angle - angle[2] - 360;
    }
    else if((Angle - angle[2]) < -180)
    {
        Angle_Error = Angle - angle[2] + 360;
    }
    else
    {
        Angle_Error = Angle - angle[2];
    }
    // 改进点
    // 1.Angle是GPS的方向角，通过对GPS的滤波，可以得到更加准确的方向角
    // 2.Z_360是IMU的航向角，通过对IMU的滤波，可以得到更加准确的航向角（卡尔曼滤波和四元数，上面这两点是数据处理）
    // 3.加入舵机PD控制
    // 4.加入电机PID控制
    // 5.MPC控制和曲率前馈
    // 1234均已实现的差不多，等待实际测试

    if(Track_Points_NUM == Task1_Start_Point || Track_Points_NUM == Task2_Start_Point || Track_Points_NUM == Task3_Start_Point)
    {
        Angle_Error = -K_Straight * angle[2];
    }
    Target_Encoder = GpsTgtEncod[Track_Points_NUM];

    if(fabs(angle[0]) > 20)
    {
        Target_Encoder -= fabs(fabs(angle[0]) - 20) * Fly_Slope_Alpha;
        if(Target_Encoder <= 1500)
        {
            Target_Encoder = 1500;
        }
    }
}

// 切换点位
void Point_Switch()
{
    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, GPS_GET_LAT[Track_Points_NUM], GPS_GET_LOT[Track_Points_NUM]);
    if(Track_Points_NUM != Task1_Start_Point && Track_Points_NUM != Task2_Start_Point && Track_Points_NUM != Task3_Start_Point)
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM ++;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }
    else
    {
        if (Distance > GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM ++;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
            Delta_Angle = get_two_points_azimuth(Start_Lat, Start_Lon, gnss.latitude, gnss.longitude);
        }
    }

}











