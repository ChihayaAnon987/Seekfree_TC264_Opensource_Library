/*
 * Position_Calculation.c
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#include "zf_common_headfile.h"


int16 Track_Points_NUM =   0;       // 当前追踪第几个点
double Angle_Error     =   0;       // 方向角与航向角之差
float  Fusion_angle    =   0;       // GPS和IMU互补滤波后的角度
float  Fusion_alpha    = 0.9;       // GPS和IMU互补滤波的权重
int16  Target_Encoder  =   0;       // 转速
int16  Fly_Slope_Alpha = 200;       // 飞坡系数
float  K_Straight      = 1.7;       // 走直线系数
int16  Delay_Time1     = 500;       // 拐弯时间
int16  Delay_Time2     = 500;       // 拐弯时间

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
    else
    {
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
    
#if UART_RECEIVER_ENABLE
    if(uart_receiver.state == 0)
    {
        Target_Encoder = 0;             // 遥控器失控保护
        LED_Buzzer_Flag_Ctrl(LED3);
    }
#endif
}

// 切换点位
void Point_Switch()
{
    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, GPS_GET_LAT[Track_Points_NUM], GPS_GET_LOT[Track_Points_NUM]);
    if(Track_Points_NUM == Task1_Start_Point || Track_Points_NUM == Task2_Start_Point || Track_Points_NUM == Task3_Start_Point)
    {
        if (Distance > GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM ++;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
            Delta_Angle = get_two_points_azimuth(Start_Lat, Start_Lon, gnss.latitude, gnss.longitude);
            if(Delta_Angle > 355 || Delta_Angle < 5)
            {
                Delta_Angle = 0;
            }
            if(fabs(Delta_Angle - 180) < 5)
            {
                Delta_Angle = 180;
            }
        }
    }
    else if(Track_Points_NUM == Task1_Start_Point + 1) // 科目一拐弯
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            if(GPS_GET_LAT[Task1_Start_Point] < GPS_GET_LAT[Task1_Start_Point + 1])  // 向北发车
            {
                if(GPS_GET_LOT[Task1_Start_Point + 1] < GPS_GET_LOT[Task1_Start_Point + 2]) // 右拐弯
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    DRV8701_MOTOR_DRIVER(GpsTgtEncod[Track_Points_NUM + 1]);
                    while (TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
                else  // 左拐弯
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    DRV8701_MOTOR_DRIVER(GpsTgtEncod[Track_Points_NUM + 1]);
                    while (TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
            }
            else //向南发车
            {
                if(GPS_GET_LOT[Task1_Start_Point + 1] < GPS_GET_LOT[Task1_Start_Point + 2]) // 左拐弯
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    DRV8701_MOTOR_DRIVER(GpsTgtEncod[Track_Points_NUM + 1]);
                    while (TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
                else  // 右拐弯
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    DRV8701_MOTOR_DRIVER(GpsTgtEncod[Track_Points_NUM + 1]);
                    while (TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
            }
            Track_Points_NUM = Task1_Start_Point + 3;
        }
    }
    else if(Track_Points_NUM == Task2_Start_Point + Task2_Bucket + 1)  // 科目二拐弯
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            if(GPS_GET_LAT[Task2_Road_Genera + Task2_Bucket + 1] > GPS_GET_LAT[Task2_Road_Genera]) // 向北发车
            {
                if(GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 1] > GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 3])  // 左拐弯
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    while(TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
                else  // 右拐弯
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    while(TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
            }
            else  // 向南发车
            {
                if(GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 1] > GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 3])  // 右拐弯
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    while(TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
                else  // 左拐弯
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    while(TRUE)
                    {
                        if(fabs(angle[2] - 180) < 5)
                        {
                            break;
                        }
                    }
                }
            }
            Track_Points_NUM = Task2_Start_Point + Task2_Bucket + 4;
        }
    }
    else
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM ++;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }

}











