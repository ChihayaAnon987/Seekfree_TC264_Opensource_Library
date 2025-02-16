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

PIDController PID_Init;   // 用于 PID 初始化
PIDController PID_GPS;    // 用于 GPS 控制
PIDController PID_IMU;    // 用于 IMU 控制
PIDController PID_SERVO;  // 舵机 PID控制器
PIDController PID_MOTOR;  // 电机 PID控制器


/****************************************************************************************************
//  @brief      限制范围
//  @param      x              int型变量
//  @param      low            下限
//  @param      up             上限
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

void PdGpsCtrl()
{
    int32 pwm = 0;
    PID_GPS.last_error = PID_GPS.current_error;
    PID_GPS.current_error = Yaw - Angle;
    if(PID_GPS.current_error > 180)
    {
        PID_GPS.current_error -= 360;
    }
    else if(PID_GPS.current_error < -180)
    {
        PID_GPS.current_error += 360;
    }
    pwm = (int32)(PID_GPS.current_error * PID_GPS.Kp + (PID_GPS.current_error - PID_GPS.last_error) * PID_GPS.Kd);
    if(pwm > 1300)
    {
        pwm = 1300;
    }
    else if(pwm < -1300)
    {
        pwm = -1300;
    }
    //pwm_set_duty(SERVO_MOTOR_PWM, (uint32)(SERVO_MOTOR_MID_DUTY - pwm));
    //DRV8701_MOTOR_DRIVER(3000);
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
    PID_SERVO.output = Parameter_set0.ServePID[0] * PID_SERVO.current_error +
                       Parameter_set0.ServePID[2] * PID_SERVO.derivative;

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
    // PID_MOTOR.current_error  = TARGET_MOTOR_ENCODER - Encoder;
    PID_MOTOR.current_error  = TARGET_MOTOR_ENCODER - 0;

    PID_MOTOR.output += Parameter_set0.SpeedPID[0] * (PID_MOTOR.current_error - PID_MOTOR.last_error) +
                        Parameter_set0.SpeedPID[1] * PID_MOTOR.current_error +
                        Parameter_set0.SpeedPID[2] * (PID_MOTOR.current_error - 2 * PID_MOTOR.last_error + PID_MOTOR.lastlast_error);
    PID_MOTOR.output = FloatClip(PID_MOTOR.output, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    int32 MOTOR_DUTY = (int32)PID_MOTOR.output;
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
}


// /****************************************************************************************************
// //  @brief      位置式PID控制
// //  @param      *pid                   PID结构体
// //  @param      error                  误差
// //  @return     output                 PID输出
// //  @since
// //  Sample usage:
// ****************************************************************************************************/
// void PidLocCtrl(PIDController *pid, float measuredVal)
// {
//     pid->preError = pid->currentError;
//     pid->currentError = pid->targetVal- measuredVal;
//     pid->derivative = pid->currentError- pid->preError;
//     pid->integral += pid->currentError;
//     // 积分饱和处理
//     FloatClip(pid->integral, -pid->imax, pid->imax);

//     pid->output =
//     pid->Kp * pid->currentError +
//     pid->Ki * pid->integral +
//     pid->Kd * pid->derivative;
// }


// /****************************************************************************************************
// //  @brief      增量式PID算法
// //  @param      *pid                   PID结构体
// //  @param      measuredVal            测量值
// //  @return     void
// //  @since
// //  Sample usage:
// ****************************************************************************************************/
// void PidIncCtrl(PIDController *pid, float measuredVal)
// {
//     pid->prepreError = pid->preError;
//     pid->preError = pid->currentError;
//     pid->currentError = pid->targetVal- measuredVal;

//     pid->output +=
//     pid->Kp * (pid->currentError- pid->preError) +
//     pid->Ki * pid->currentError +
//     pid->Kd * (pid->currentError- 2 * pid->preError + pid->prepreError);
//  }


void GPS_IMU_COM_filtering()
{
    float K = 0.9;          // 互补系数
    float Fusion_angle;     // 融合后的航向角
    // 将积分的YAW和逐飞GPS的direction进行互补融合
    Fusion_angle = K * Z_360 + ( 1 - K ) * gnss.direction;

}

/****************************************************************************************************
//  @brief      滑动平均滤波初始化
//  @param      move_filter            结构体传参
//  @return     void
//  @since
//  Sample usage:move_filter_init(&speed_filter);
****************************************************************************************************/
void move_filter_init(move_filter_struct *move_filter)
{
    move_filter->data_average   = 0;
    move_filter->data_sum       = 0;
    move_filter->index          = 0;
    //设置缓冲区大小
    move_filter->buffer_size    = MOVE_AVERAGE_SIZE;// 定义缓冲区大小

    uint8 i;
    for(i = 0; i < move_filter->buffer_size; i ++)
    {
        move_filter->data_buffer[i] = 0;
    }
}

/****************************************************************************************************
//  @brief      滑动平均滤波计算
//  @param      move_filter            结构体传参
//  @param      new_data               新输入的数据
//  @return     void
//  @since
//  Sample usage:move_filter_calc(&speed_filter, 400);
****************************************************************************************************/
void move_filter_calc(move_filter_struct *move_filter, int32 new_data)//结构体传参，新输入的数据
{
    // 加上新的数值 减去最末尾的数值 求得最新的和
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // 重新求平均值
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // 将数据写入缓冲区
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index ++;
    if(move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}
