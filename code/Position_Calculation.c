/*
 * Position_Calculation.c
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#include "zf_common_headfile.h"


int16 Track_Points_NUM =   0;       // 当前追踪第几个点
double Angle_Error     =   0;       // 方向角与航向角之差
int16  Target_Encoder  =   0;       // 转速
int16  Fly_Slope_Alpha = 200;       // 飞坡系数
float  K_Straight      = 1.7;       // 走直线系数
int8   Hole_Point      =  52;       // 标记桥洞点位
int8   Ramp_Point      =  52;       // 标记坡道点位
int8   Turn_Point      =  55;       // 标记掉头点位
double Turn_Angle      =   0;       // 掉头方向
int8   Action_Flag[ACTION_COUNT]  = {0};    // 科目四动作标志位
int8   Task_Four_Turn_Flag        = 0;      // 科目四转圈标志位
float  Snack_Advance              = 3;      // 蛇形前进偏移
float  Snack_Back                 = 3;      // 蛇形后退偏移
float  Task4_Start_Direc          = 0;      // 科目四发车角度

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

    if(Star_Time == 0)
    {
        Star_Time = System_Time;
    }

    if(Track_Points_NUM == Task1_Start_Point || Track_Points_NUM == Task2_Start_Point || Track_Points_NUM == Task3_Start_Point)
    {
        Angle_Error = -K_Straight * angle[2];
    }
    else if(Track_Points_NUM == Task1_Start_Point + Task1_Points + 1)
    {
        Angle_Error = 0;
    }
    else if(Track_Points_NUM == Task2_Start_Point + Task2_Points + 1)
    {
        Angle_Error = 0;
    }
    else if(Track_Points_NUM == Task3_Start_Point + Task3_Points + 1)
    {
        Angle_Error = 0;
    }
    else
    {
        Angle_Error = LimitFabs180(Angle - angle[2]);
    }
    Target_Encoder = GpsTgtEncod[Track_Points_NUM];


}

void Task4_Finish()
{
    for(int i = 0; i < ACTION_COUNT; i++)
    {
        switch(Action_Flag[i])
        {
            case FLASHING_LIGHT:
            {
                
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("123");           // 双闪灯123
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case LEFTTURN_LIGHT:
            {
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("456");           // 左转灯456
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case RIGHTURN_LIGHT:
            {
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("789");           // 右转灯789
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case LOWBEAN_HLIGHT:
            {
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("!%*");           // 近光灯!%*
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case HIGBEAN_HLIGHT:
            {
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("!%&");           // 远光灯!%&
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case FOG_LIGHT:
            {
                dot_matrix_screen_set_brightness(5000);         // 设置点阵亮度
                dot_matrix_screen_show_string("!%+");           // 雾灯!%+
                system_delay_ms(2900);
                dot_matrix_screen_show_string("   ");
                system_delay_ms(100);
                Action_Flag[i] = 0;
                break;
            }
            case HEAD_STRAIGHT:
            {
                // 向前直行十米
                Track_Points_NUM = Task4_Start_Point;
                float Straight_Angle = 0;
                if(Straight_Angle == 0)
                {
                    Straight_Angle = angle[2];
                }
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance > GpsDistance[Track_Points_NUM])
                    {
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }

                    Angle_Error = -K_Straight * (angle[2] - Straight_Angle);
                    PDLocServoCtrl();
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case BACK_STRAIGHT:
            {
                // 后退直行十米
                Track_Points_NUM = Task4_Start_Point;
                float Straight_Angle = 0;
                if(Straight_Angle == 0)
                {
                    Straight_Angle = angle[2];
                }
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance > GpsDistance[Track_Points_NUM])
                    {
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }

                    Angle_Error =  K_Straight * (angle[2] - Straight_Angle);
                    PDLocServoCtrl();
                    Target_Encoder = -GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case SNAKE_ADVANCE:
            {
                // 蛇形前进十米
                Track_Points_NUM = Task4_Start_Point;
                double angle = 0;
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance > GpsDistance[Track_Points_NUM])
                    {
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }

                    Servo_Set(SERVO_MOTOR_MID - 15 * sin(angle) + Snack_Advance);
                    angle += 0.1;
                    system_delay_ms(50);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case SNAKE_BACK:
            {
                // 蛇形后退十米
                Track_Points_NUM = Task4_Start_Point;
                double angle = 0;
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance > GpsDistance[Track_Points_NUM])
                    {
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }

                    Servo_Set(SERVO_MOTOR_MID - 15 * sin(angle) + Snack_Back);
                    angle += 0.1;
                    system_delay_ms(50);
                    Target_Encoder = -GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case ROTATE_ANTICLOCK:
            {
                // 逆时针转一圈
                Task_Four_Turn_Flag = 1;
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(Z_360) > 350)
                    {
                        Task_Four_Turn_Flag = 0;
                        break;
                    }
                }
                Action_Flag[i] = 0;
                break;
            }
            case ROTATE_CLOCKWISE:
            {
                // 顺时针转一圈
                Task_Four_Turn_Flag = 1;
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(Z_360) > 350)
                    {
                        Task_Four_Turn_Flag = 0;
                        break;
                    }
                }
                Action_Flag[i] = 0;
                break;
            }
            case PARK_AREA_ONE:
            {
                // 停进停车区一
                Track_Points_NUM = Task4_Start_Point + 1;
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance < GpsDistance[Track_Points_NUM])
                    {
                        Track_Points_NUM = Task4_Start_Point + 4;
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }
                    Angle = get_two_points_azimuth(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    Angle -= Task4_Start_Direc;
                    Angle = LimitFabs180(Angle);

                    Angle_Error = LimitFabs180(Angle - angle[2]);
                    PDLocServoCtrl();
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case PARK_AREA_TWO:
            {
                // 停进停车区二
                Track_Points_NUM = Task4_Start_Point + 2;
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance < GpsDistance[Track_Points_NUM])
                    {
                        Track_Points_NUM = Task4_Start_Point + 4;
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }
                    Angle = get_two_points_azimuth(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    Angle -= Task4_Start_Direc;
                    Angle = LimitFabs180(Angle);

                    Angle_Error = LimitFabs180(Angle - angle[2]);
                    PDLocServoCtrl();
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            case PARK_AREA_THREE:
            {
                // 停进停车区三
                Track_Points_NUM = Task4_Start_Point + 3;
                while(TRUE)
                {
                    Get_Gps();
                    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    if(Distance < GpsDistance[Track_Points_NUM])
                    {
                        Track_Points_NUM = Task4_Start_Point + 4;
                        LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
                        break;
                    }
                    Angle = get_two_points_azimuth(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
                    Angle -= Task4_Start_Direc;
                    Angle = LimitFabs180(Angle);

                    Angle_Error = LimitFabs180(Angle - angle[2]);
                    PDLocServoCtrl();
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                }
                Action_Flag[i] = 0;
                break;
            }
            default:
            {
                Servo_Set(SERVO_MOTOR_MID);
                Target_Encoder = 0;
                #if MOTOR_LOOP_ENABLE == 0
                    MOTOR_Ctrl(Target_Encoder);
                #endif
                break;
            }
        }
    }
}

// 切换点位
void Point_Switch()
{
    Distance = get_two_points_distance(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
    if(Track_Points_NUM == Task1_Start_Point || Track_Points_NUM == Task2_Start_Point || Track_Points_NUM == Task3_Start_Point)
    {
        if (Distance > GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM ++;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
            Delta_Angle = get_two_points_azimuth(Start_Lat, Start_Lon, gnss.latitude, gnss.longitude);
            if(Delta_Angle > 359 || Delta_Angle < 1)
            {
                Delta_Angle = 0;
            }
            if(fabs(Delta_Angle - 180) < 1)
            {
                Delta_Angle = 180;
            }
        }
    }
    else if(Track_Points_NUM == Task4_Start_Point)
    {
        if(Distance > GpsDistance[Track_Points_NUM])
        {
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }
    else if(Track_Points_NUM == Task1_Start_Point + 1) // 科目一拐弯
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            if((Turn_Angle > 0 && Turn_Angle < 90) || (Turn_Angle > 180 && Turn_Angle < 270))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            else if((Turn_Angle > 90 && Turn_Angle < 180) || (Turn_Angle > 270 && Turn_Angle < 360))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            Track_Points_NUM = Task1_Start_Point + 3;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }
    else if(Track_Points_NUM == Task2_Start_Point + Task2_Bucket + 2)  // 科目二拐弯
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            if((Turn_Angle > 0 && Turn_Angle < 90) || (Turn_Angle > 180 && Turn_Angle < 270))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            else if((Turn_Angle > 90 && Turn_Angle < 180) || (Turn_Angle > 270 && Turn_Angle < 360))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            Track_Points_NUM = Task2_Start_Point + Task2_Bucket + 4;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }
    else if(Track_Points_NUM == Turn_Point) // 科目三拐弯
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            if((Turn_Angle > 0 && Turn_Angle < 90) || (Turn_Angle > 180 && Turn_Angle < 270))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_RMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            else if((Turn_Angle > 90 && Turn_Angle < 180) || (Turn_Angle > 270 && Turn_Angle < 360))
            {
                while(TRUE)
                {
                    Servo_Set(SERVO_MOTOR_LMAX);
                    Target_Encoder = GpsTgtEncod[Track_Points_NUM + 1];
                    #if MOTOR_LOOP_ENABLE == 0
                        MOTOR_Ctrl(Target_Encoder);
                    #endif
                    if(fabs(angle[2] - 180) < 5)
                    {
                        break;
                    }
                }
            }
            Track_Points_NUM = Turn_Point + 2;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
        }
    }
    else if(Track_Points_NUM == Task4_Start_Point + 1 || Track_Points_NUM == Task4_Start_Point + 2 || Track_Points_NUM == Task4_Start_Point + 3)
    {
        if(Distance < GpsDistance[Track_Points_NUM])
        {
            Track_Points_NUM = Task4_Start_Point + 4;
            LED_Buzzer_Flag_Ctrl(BUZZER_PIN);
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

    Angle = get_two_points_azimuth(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, Point[Track_Points_NUM].latitude, Point[Track_Points_NUM].lonitude);
    Angle -= Delta_Angle;
    Angle = LimitFabs180(Angle);
    
    if(Stop_Time == 0)
    {
        if(Track_Points_NUM == Task1_Start_Point + Task1_Points)
        {
            Stop_Time = System_Time;
        }
        if(Track_Points_NUM == Task2_Start_Point + Task2_Points)
        {
            Stop_Time = System_Time;
        }
        if(Track_Points_NUM == Task3_Start_Point + Task3_Points)
        {
            Stop_Time = System_Time;
        }
    }
}

float LimitFabs180(float angle)
{
    if(angle > 180)
    {
        angle -= 360;
    }
    else if(angle < -180)
    {
        angle += 360;
    }
    return angle;
}

float LimitFabs360(float angle)
{
    if(angle >= 360)
    {
        angle -= 360;
    }
    else if(angle < 0)
    {
        angle += 360;
    }
    return angle;
}








