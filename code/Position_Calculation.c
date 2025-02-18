/*
 * Position_Calculation.c
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#include "zf_common_headfile.h"


int Track_Points_NUM  =   0;       // 当前追踪第几个点
double Angle_Error    =   0;       // 方向角与航向角之差
float  Fusion_angle   =   0;       // GPS和IMU互补滤波后的角度
float  Fusion_alpha   = 0.9;       // GPS和IMU互补滤波的权重
int16  Target_Encoder =   0;       // 转速

void Stright_Some_Distance()
{
    Angle_Error = -angle[2];
    if (Distance > 6)
    {
        Delta_Angle = get_two_points_azimuth(Start_Lat, Start_Lon, gnss.latitude, gnss.longitude);
        Track_Points_NUM++;
    }
}

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

    // 关于科目一二三的切换，可以借助菜单实现，把Servo_Test换成Task_Select或者新增一个一级菜单都是可以的
    // KEY1、KEY2、KEY3分别对应科目一、科目二、科目三（预设充足的点位给某个科目，按下按键跳转到某科目对应的点位）
    // KEY4发车（设置一个标志位，KEY4按下置为高电平，退出在主程序的一个卡死的while循环，进入循迹循环）
    // 思路大概如此，实现也很简单，但是先算了，把科目一跑完再实现这个逻辑

    switch(Track_Points_NUM)
    {
        case 0:
            Stright_Some_Distance();
            Target_Encoder = 3000;
            break;
        case 1:
            Target_Encoder = 3000;
            break;
        case 2:
            Target_Encoder = 1500;
            break;
        case 3:
            Target_Encoder = 3000;
            break;
        case 4:
        case 5:
            Target_Encoder = 0;
            break;



        case 10:
            Stright_Some_Distance();
            Target_Encoder = 1500;
            break;
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
            Target_Encoder = 1500;
            break;
        
        case 23:
            Target_Encoder = 0;
            break;
        case 50:
            break;

        default:
            break;
    }



}

// 切换点位
void Point_Switch()
{
    Distance = get_two_points_distance(gnss.latitude, gnss.longitude, GPS_GET_LAT[Track_Points_NUM] - Delta_Lat, GPS_GET_LOT[Track_Points_NUM] - Delta_Lon);
    switch(Track_Points_NUM)
    {
        // 暂时假定科目一采5个点，其中下标0是发车点
        case 0:
            break;
        case 1:
        case 2:
        case 3:
            if(Distance < Parameter_set0.Distance)
            {
                Track_Points_NUM ++;
                LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
            }
            break;

        case 4:
        case 5:
            break;

        case 10:
            break;
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
            if(Distance < 1)
            {
                Track_Points_NUM ++;
                LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
            }
            break;
        
        case 23:
            break;
        default:
            break;

        // 科目二设置为GPS循迹，纯惯导虽然稳定，但是速度慢，实现难，暂时不考虑

        // 科目三桥洞标准方案是通过GPS导航到桥洞附近，再通过摄像头识别桥洞中间白色PCV材料。
        //
    }

}











