/*
 * RemoteCtrl.c
 *
 *  Created on: 2025年1月12日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

int   Control_Flag               = 0;  // 遥控器控制标志位
int   Center_Flag                = 0;
int   Fall_Flag                  = 0;  // uart接收机脱落标志位
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
    if(uart_receiver.state == 0 || uart_receiver.channel[0] == 0 || Fall_Flag == 1)
    {
        MOTOR_Ctrl(0);
    }
    else
    {
        UART_RECEIVER_FALL();
        CtrlMode_Switch();
        Is_Channal_3_Press();
        Is_Channal_5_Press();
        Is_Channal_6_Press();
        RemoteCtrl_Direction_Speed();
    }
}

void RemoteCtrl_Direction_Speed()
{
    if(Control_Flag == 1)
    {
        if(uart_receiver.channel[1] - CHANNAL2_MIDDLE_LEVEL > 100)
        {
            MOTOR_Ctrl(GpsTgtEncod[9]);
        }
        else if(uart_receiver.channel[1] - CHANNAL2_MIDDLE_LEVEL < -100)
        {
            MOTOR_Ctrl(-GpsTgtEncod[9]);
        }
        else
        {
            MOTOR_Ctrl(0);
        }
    }

    if(Control_Flag == 2)
    {
        RemoteCtrl_Speed = (int16)((uart_receiver.channel[1] - CHANNAL2_MIDDLE_LEVEL) * 5000 / 800);
        MOTOR_Ctrl(RemoteCtrl_Speed);

        RemoteCtrl_Direction = (int16)((CHANNAL1_MIDDLE_LEVEL - uart_receiver.channel[0]) * 90 / 800);
        static float CenterAngle;
        if(fabs(CHANNAL1_MIDDLE_LEVEL - uart_receiver.channel[0]) < 40)
        {
            if(Center_Flag == 1)
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
                CenterAngle = angle[2];
                Center_Flag = 1;
            }
        }
        else
        {
            Center_Flag = 0;
            Angle_Error = RemoteCtrl_Direction;
        }
    }

    if(Control_Flag == 1 || Control_Flag == 2)
    {
        // 自动归位
        if(Channal_5_Press_Flag == 1)
        {
            Start_Flag = 1;
            if(Task_Flag == 1)
            {
                Point[8].latitude = Point[Task1_Start_Point].latitude;
                Point[8].lonitude = Point[Task1_Start_Point].lonitude;
            }
            else if(Task_Flag == 2)
            {
                Point[8].latitude = Point[Task2_Start_Point].latitude;
                Point[8].lonitude = Point[Task2_Start_Point].lonitude;
            }
            else if(Task_Flag == 3)
            {
                Point[8].latitude = Point[Task3_Start_Point].latitude;
                Point[8].lonitude = Point[Task3_Start_Point].lonitude;
            }
            else if(Task_Flag == 4)
            {
                Point[8].latitude = Point[Task4_Start_Point].latitude;
                Point[8].lonitude = Point[Task4_Start_Point].lonitude;
            }
            else
            {
                Point[8].latitude = Point[Task1_Start_Point].latitude;
                Point[8].lonitude = Point[Task1_Start_Point].lonitude;
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

void UART_RECEIVER_FALL()
{
    static float time = 0;
    if(uart_receiver.finsh_flag == 1)
    {
        time = System_Time;
        Fall_Flag = 0;
    }
    else
    {
        if(System_Time - time > 1.0f)
        {
            Fall_Flag = 1;
        }
    }
}
