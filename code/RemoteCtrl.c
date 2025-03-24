/*
 * RemoteCtrl.c
 *
 *  Created on: 2025年1月12日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

int   Control_Flag               = 0;  // 遥控器控制标志位，每按下一次翻转一次，当它为1时是遥控模式，为0时是循迹模式，默认循迹（为0）
int   Center_Flag                = 0;
int   Channal_3_Press_Flag       = 0;  // 通道3按键是否按下
int   Channal_5_Press_Flag       = 0;  // 通道5按键是否按下
int   Channal_6_Press_Flag       = 0;  // 通道6按键是否按下
int   last_Channal_3             = 0;  // 通道3上一个状态
int   last_Channal_5             = 0;  // 通道5上一个状态
int   last_Channal_6             = 0;  // 通道6上一个状态
int16 RemoteCtrl_Speed;                // 遥控器速度控制量
int16 RemoteCtrl_Direction;            // 遥控器方向控制量

void RemoteCtrl_Program()
{
    if(uart_receiver.state == 0 || uart_receiver.channel[0] == 0)
    {
        #if BLDC_ENABLE
            BLDC_Ctrl(0);
        #else
            DRV8701_MOTOR_DRIVER(0);
        #endif
    }
    else
    {
        Is_Channal_3_Press();
        Is_Channal_5_Press();
        Is_Channal_6_Press();
        RemoteCtrl_Direction_Speed();
        CtrlMode_Switch();
    }
}

void RemoteCtrl_Direction_Speed()
{
    if(Control_Flag == 1)
    {
        #if MOTOR_LOOP_ENABLE
            RemoteCtrl_Speed = (int16)((uart_receiver.channel[1] - 1056) * 500 / 800);
            PIDIncMotorCtrl(RemoteCtrl_Speed);
        #else
            if(uart_receiver.channel[1] - 1056 > 100)
            {
                #if BLDC_ENABLE
                    BLDC_Ctrl(GpsTgtEncod[9]);
                #else
                    DRV8701_MOTOR_DRIVER(GpsTgtEncod[9]);
                #endif
            }
            else if(uart_receiver.channel[1] - 1056 < -100)
            {
                #if BLDC_ENABLE
                    BLDC_Ctrl(-GpsTgtEncod[9]);
                #else
                    DRV8701_MOTOR_DRIVER(-GpsTgtEncod[9]);
                #endif
            }
            else
            {
                #if BLDC_ENABLE
                    BLDC_Ctrl(0);
                #else
                    DRV8701_MOTOR_DRIVER(0);
                #endif
            }
        #endif
        
        // 自动归位
        if(Channal_5_Press_Flag == 1)
        {
            Start_Flag = 1;
            if(Task_Flag == 1)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task1_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task1_Start_Point];
            }
            else if(Task_Flag == 2)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task2_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task2_Start_Point];
            }
            else if(Task_Flag == 3)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task3_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task3_Start_Point];
            }
            else
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task1_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task1_Start_Point];
            }
            Track_Points_NUM = 8;
        }
    }
    if(Control_Flag == 2)
    {
        RemoteCtrl_Direction = (int16)((856 - uart_receiver.channel[0]) * 24 / 800);    // 把其值映射到SERVO_MOTOR_RMAX 到 SERVO_MOTOR_LMAX
        static float CenterAngle;
        if(fabs(856 - uart_receiver.channel[0]) < 20)
        {
            if(Center_Flag == 0)
            {
                CenterAngle = angle[2];
                Center_Flag = 1;
            }
            if(Center_Flag == 1)
            {
                if(RemoteCtrl_Speed != 0)
                {
                    if(RemoteCtrl_Speed > 0)
                    {
                        Angle_Error = -K_Straight * (angle[2] - CenterAngle);
                    }
                    else
                    {
                        Angle_Error =  K_Straight * (angle[2] - CenterAngle);
                    }
                }
                else
                {
                    Angle_Error = 0;
                }

            }
        }
        else
        {
            Center_Flag = 0;
            Servo_Set(SERVO_MOTOR_MID - RemoteCtrl_Direction);                          // 舵机角度
        }

        #if MOTOR_LOOP_ENABLE
            RemoteCtrl_Speed = (int16)((uart_receiver.channel[1] - 1056) * 500 / 800);
            PIDIncMotorCtrl(RemoteCtrl_Speed);
        #else
            RemoteCtrl_Speed = (int16)((uart_receiver.channel[1] - 1056) * 5000 / 800);
            #if BLDC_ENABLE
                BLDC_Ctrl(RemoteCtrl_Speed);
            #else
                DRV8701_MOTOR_DRIVER(RemoteCtrl_Speed);
            #endif
        #endif

        // 自动归位
        if(Channal_5_Press_Flag == 1)
        {
            Start_Flag = 1;
            if(Task_Flag == 1)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task1_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task1_Start_Point];
            }
            else if(Task_Flag == 2)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task2_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task2_Start_Point];
            }
            else if(Task_Flag == 3)
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task3_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task3_Start_Point];
            }
            else
            {
                GPS_GET_LAT[8] = GPS_GET_LAT[Task1_Start_Point];
                GPS_GET_LOT[8] = GPS_GET_LOT[Task1_Start_Point];
            }
            Track_Points_NUM = 8;
        }
    }
}



void CtrlMode_Switch()
{
    if(uart_receiver.channel[3] == CHANNAL_LOW_LEVEL)
    {
        Control_Flag = 0;
    }
    if(uart_receiver.channel[3] == (CHANNAL_LOW_LEVEL + CHANNAL_HIGH_LEVEL) / 2)
    {
        Control_Flag = 1;
    }
    if(uart_receiver.channel[3] == CHANNAL_HIGH_LEVEL)
    {
        Control_Flag = 2;
    }
}


void Is_Channal_3_Press()
{
    if(Channal_3_Press_Flag == 1)
    {
        Channal_3_Press_Flag = 0;
    }
    if(uart_receiver.channel[2] == CHANNAL_HIGH_LEVEL && last_Channal_3 == CHANNAL_LOW_LEVEL)
    {
        Channal_3_Press_Flag = 1;
    }
    if(uart_receiver.channel[2] == CHANNAL_LOW_LEVEL && last_Channal_3 == CHANNAL_HIGH_LEVEL)
    {
        Channal_3_Press_Flag = 1;
    }
    last_Channal_3 = uart_receiver.channel[2];
}

void Is_Channal_5_Press()
{
    if(Channal_5_Press_Flag == 1)
    {
        Channal_5_Press_Flag = 0;
    }
    if(uart_receiver.channel[4] == CHANNAL_HIGH_LEVEL && last_Channal_5 == CHANNAL_LOW_LEVEL)
    {
        Channal_5_Press_Flag = 1;
    }
    if(uart_receiver.channel[4] == CHANNAL_LOW_LEVEL && last_Channal_5 == CHANNAL_HIGH_LEVEL)
    {
        Channal_5_Press_Flag = 1;
    }
    last_Channal_5 = uart_receiver.channel[4];
}

void Is_Channal_6_Press()
{
    if(Channal_6_Press_Flag == 1)
    {
        Channal_6_Press_Flag = 0;
    }
    if(uart_receiver.channel[5] == CHANNAL_HIGH_LEVEL && last_Channal_6 == CHANNAL_LOW_LEVEL)
    {
        Channal_6_Press_Flag = 1;
    }
    if(uart_receiver.channel[5] == CHANNAL_LOW_LEVEL && last_Channal_6 == CHANNAL_HIGH_LEVEL)
    {
        Channal_6_Press_Flag = 1;
    }
    last_Channal_6 = uart_receiver.channel[5];
}
