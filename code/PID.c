/*
 * PID.c
 *
 *  Created on: 2025年1月12日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

/*
    舵机和电机的PID算法参考：[链接](https://mp.weixin.qq.com/s/zJiQGuqU5JXgHxL_FXBWrQ)
*/

PIDController PID_SERVO;  // 舵机 PID控制器
PIDController PID_MOTOR;  // 电机 PID控制器
ServoPD From_0000_To_2000_ServoPD = {1.75, 0.75};   // 用20%占空比作为参考，含左且含右
ServoPD From_2000_To_4000_ServoPD = {1.10, 0.55};   // 用30%占空比作为参考，不含左和右
ServoPD From_4000_To_5000_ServoPD = {0.85, 0.40};   // 用45%占空比作为参考，含左不含右
ServoPD From_5000_To_6000_ServoPD = {0.63, 0.47};   // 用55%占空比作为参考，含左不含右
ServoPD From_6000_To_7000_ServoPD = {0.47, 0.53};   // 用65%占空比作为参考，含左不含右
ServoPD From_7000_To_8000_ServoPD = {0.28, 0.68};   // 用75%占空比作为参考，含左不含右
ServoPD From_8000_To_9000_ServoPD = {0.40, 0.75};   // 用85%占空比作为参考，含左不含右
ServoPD From_9000_To_9900_ServoPD = {0.40, 0.40};   // 用95%占空比作为参考，含左且含右
ServoPD Encoder0100_ServoPD       = {1.75, 0.75};   // [0   , 150 )
ServoPD Encoder0200_ServoPD       = {1.10, 0.55};   // [150 , 250 )
ServoPD Encoder0300_ServoPD       = {0.85, 0.40};   // [250 , 350 )
ServoPD Encoder0400_ServoPD       = {0.85, 0.40};   // [350 , 450 )
ServoPD Encoder0500_ServoPD       = {0.86, 0.41};   // [450 , 550 )
ServoPD Encoder0600_ServoPD       = {0.86, 0.41};   // [550 , 650 )
ServoPD Encoder0700_ServoPD       = {0.86, 0.41};   // [650 , 750 )
ServoPD Encoder0800_ServoPD       = {0.86, 0.41};   // [750 , 850 )
ServoPD Encoder0900_ServoPD       = {0.86, 0.41};   // [850 , 950 )
ServoPD Encoder1000_ServoPD       = {0.86, 0.41};   // [950 , 1050)
ServoPD Encoder1100_ServoPD       = {0.86, 0.41};   // [1050, 1150)
ServoPD Encoder1200_ServoPD       = {0.86, 0.41};   // [1150, 1250)

/****************************************************************************************************
//  @brief      限制范围
//  @param      x              int型变量
//  @param      low            下限
//  @param      up             上限0
//  @return     int            上下限范围内变量
//  @since
//  Sample usage:
****************************************************************************************************/
int IntClip(int x, int low, int up)
{
    return x > up ? up : x < low ? low : x;
}

/****************************************************************************************************
//  @brief      限制范围
//  @param      x              float型变量
//  @param      low            下限
//  @param      up             上限
//  @return     float          上下限范围内变量
//  @since
//  Sample usage:
****************************************************************************************************/
float FloatClip(float x, float low, float up)
{
    return x > up ? up : x < low ? low : x;
}

/****************************************************************************************************
//  @brief      位置式PID控制
//  @param      *pid                   PID结构体
//  @param      error                  误差
//  @return     output                 PID输出
//  @since
//  Sample usage:
****************************************************************************************************/
float PidLocCtrl(PIDController * pid, float error)
{
    pid->integrator += error;
    pid->integrator = FloatClip(pid->integrator, -pid->imax, pid->imax);

    pid->out_p = pid->Kp * error;
    pid->out_i = pid->Ki * pid->integrator;
    pid->out_d = pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    pid->output = pid->out_p + pid->out_i + pid->out_d;

    return pid->output;
}

/****************************************************************************************************
//  @brief      增量式PID算法
//  @param      *pid                   PID结构体
//  @param      error                  误差
//  @return     output                 PID输出
//  @since
//  Sample usage:
****************************************************************************************************/
float PidIncCtrl(PIDController * pid, float error)
{
    pid->out_p = pid->Kp * (error - pid->last_error);
    pid->out_i = pid->Ki * error;
    pid->out_d = pid->Kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output += pid->out_p + pid->out_i + pid->out_d;
    return pid->output;
}

/****************************************************************************************************
//  @brief      舵机 PD位置式控制器
//  @param      void
//  @return     void
//  @since
//  Sample usage:PiLocServoCtrl();
****************************************************************************************************/
void PDLocServoCtrl()
{
    PID_SERVO.last_error    = PID_SERVO.current_error;
    PID_SERVO.current_error = Angle_Error;
    PID_SERVO.derivative    = PID_SERVO.current_error - PID_SERVO.last_error;
    #if MOTOR_LOOP_ENABLE
        if(Target_Encoder >= 0 && Target_Encoder < 150)
        {
            PID_SERVO.output = Encoder0100_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0100_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 150 && Target_Encoder < 250)
        {
            PID_SERVO.output = Encoder0200_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0200_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 250 && Target_Encoder < 350)
        {
            PID_SERVO.output = Encoder0300_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0300_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 350 && Target_Encoder < 450)
        {
            PID_SERVO.output = Encoder0400_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0400_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 450 && Target_Encoder < 550)
        {
            PID_SERVO.output = Encoder0500_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0500_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 550 && Target_Encoder < 650)
        {
            PID_SERVO.output = Encoder0600_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0600_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 650 && Target_Encoder < 750)
        {
            PID_SERVO.output = Encoder0700_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0700_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 750 && Target_Encoder < 850)
        {
            PID_SERVO.output = Encoder0800_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0800_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 850 && Target_Encoder < 950)
        {
            PID_SERVO.output = Encoder0900_ServoPD.Kp * PID_SERVO.current_error +
            Encoder0900_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 950 && Target_Encoder < 1050)
        {
            PID_SERVO.output = Encoder1000_ServoPD.Kp * PID_SERVO.current_error +
            Encoder1000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 1050 && Target_Encoder < 1150)
        {
            PID_SERVO.output = Encoder1100_ServoPD.Kp * PID_SERVO.current_error +
            Encoder1100_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 1150 && Target_Encoder < 1250)
        {
            PID_SERVO.output = Encoder1200_ServoPD.Kp * PID_SERVO.current_error +
            Encoder1200_ServoPD.Kd * PID_SERVO.derivative;
        }
        else
        {
            PID_SERVO.output = Encoder1200_ServoPD.Kp * PID_SERVO.current_error +
            Encoder1200_ServoPD.Kd * PID_SERVO.derivative;
        }
    #else
        if(Target_Encoder >= 0 && Target_Encoder <= 2000)
        {
            PID_SERVO.output = From_0000_To_2000_ServoPD.Kp * PID_SERVO.current_error +
            From_0000_To_2000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder > 2000 && Target_Encoder < 4000)
        {
            PID_SERVO.output = From_2000_To_4000_ServoPD.Kp * PID_SERVO.current_error +
            From_2000_To_4000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 4000 && Target_Encoder < 5000)
        {
            PID_SERVO.output = From_4000_To_5000_ServoPD.Kp * PID_SERVO.current_error +
            From_4000_To_5000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 5000 && Target_Encoder < 6000)
        {
            PID_SERVO.output = From_5000_To_6000_ServoPD.Kp * PID_SERVO.current_error +
            From_5000_To_6000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 6000 && Target_Encoder < 7000)
        {
            PID_SERVO.output = From_6000_To_7000_ServoPD.Kp * PID_SERVO.current_error +
            From_6000_To_7000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 7000 && Target_Encoder < 8000)
        {
            PID_SERVO.output = From_7000_To_8000_ServoPD.Kp * PID_SERVO.current_error +
            From_7000_To_8000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 8000 && Target_Encoder < 9000)
        {
            PID_SERVO.output = From_8000_To_9000_ServoPD.Kp * PID_SERVO.current_error +
            From_8000_To_9000_ServoPD.Kd * PID_SERVO.derivative;
        }
        else if(Target_Encoder >= 9000 && Target_Encoder <= 10000)
        {
            PID_SERVO.output = From_9000_To_9900_ServoPD.Kp * PID_SERVO.current_error +
            From_9000_To_9900_ServoPD.Kd * PID_SERVO.derivative;
        }
        else
        {
            PID_SERVO.output = Parameter_set0.ServePID[0] * PID_SERVO.current_error +
            Parameter_set0.ServePID[2] * PID_SERVO.derivative;
        }
    #endif

    Servo_Angle = Parameter_set0.Serve_Mid - PID_SERVO.output;
    if(Servo_Angle > SERVO_MOTOR_LMAX) {Servo_Angle = SERVO_MOTOR_LMAX;}
    if(Servo_Angle < SERVO_MOTOR_RMAX) {Servo_Angle = SERVO_MOTOR_RMAX;}
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(Servo_Angle));
}

/****************************************************************************************************
//  @brief      电机 PID增量式控制器
//  @param      TARGET_MOTOR_ENCODER      目标电机转速
//  @return     void
//  @since
//  Sample usage:PIDIncMotorCtrl(3000);
****************************************************************************************************/
void PIDIncMotorCtrl(int16 TARGET_MOTOR_ENCODER)
{
    PID_MOTOR.lastlast_error = PID_MOTOR.last_error;
    PID_MOTOR.last_error     = PID_MOTOR.current_error;
    PID_MOTOR.current_error  = TARGET_MOTOR_ENCODER - Encoder;

    PID_MOTOR.output += Parameter_set0.SpeedPID[0] * (PID_MOTOR.current_error - PID_MOTOR.last_error) +
                        Parameter_set0.SpeedPID[1] * PID_MOTOR.current_error +
                        Parameter_set0.SpeedPID[2] * (PID_MOTOR.current_error - 2 * PID_MOTOR.last_error + PID_MOTOR.lastlast_error);
    PID_MOTOR.output = FloatClip(PID_MOTOR.output, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    int16 MOTOR_DUTY = (int16)PID_MOTOR.output;
#if BLDC_ENABLE
    if(MOTOR_DUTY >= 0)
    {
        gpio_set_level(DIR_CH1, 1);
        pwm_set_duty  (PWM_CH1, MOTOR_DUTY);
    }
    else
    {
        gpio_set_level(DIR_CH1, 0);
        pwm_set_duty  (PWM_CH1, -MOTOR_DUTY);
    }
#else
    if(MOTOR_DUTY >= 0)
    {
        gpio_set_level(DIR_CH1, 0);
        pwm_set_duty  (PWM_CH1, MOTOR_DUTY);
    }
    else
    {
        gpio_set_level(DIR_CH1, 1);
        pwm_set_duty  (PWM_CH1, -MOTOR_DUTY);
    }
#endif
}