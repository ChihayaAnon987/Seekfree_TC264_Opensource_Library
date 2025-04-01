/*
 * PID.h
 *
 *  Created on: 2025年1月12日
 *      Author: 20483
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

//===================================================类型定义BEG===================================================
typedef struct
{
    float Kp;                                                  // 比例系数P
    float Ki;                                                  // 积分系数I
    float Kd;                                                  // 微分系数D
    float imax;                                                // 积分限幅

    float out_p;                                               // KP输出
    float out_i;                                               // KI输出
    float out_d;                                               // KD输出
    float output;                                              // PID输出

    float integrator;                                          // 积分值
    float current_error;                                       // 误差
    float last_error;                                          // 上次误差
    float lastlast_error;                                      // 上上次误差
    float derivative;                                          // 误差微分
    float last_derivative;                                     // 上次误差与上上次误差之差
} PIDController;

typedef struct 
{
    float Kp;
    float Kd;
} ServoPD;
//===================================================类型定义END===================================================


//===================================================全局变量BEG===================================================
extern PIDController PID_SERVO;                                // 舵机 PID控制器
extern PIDController PID_MOTOR;                                // 电机 PID控制器
extern ServoPD From_0000_To_2000_ServoPD;                      // 舵机PD
extern ServoPD From_2000_To_4000_ServoPD;                      // 舵机PD
extern ServoPD From_4000_To_5000_ServoPD;                      // 舵机PD
extern ServoPD From_5000_To_6000_ServoPD;                      // 舵机PD
extern ServoPD From_6000_To_7000_ServoPD;                      // 舵机PD
extern ServoPD From_7000_To_8000_ServoPD;                      // 舵机PD
extern ServoPD From_8000_To_9000_ServoPD;                      // 舵机PD
extern ServoPD From_9000_To_9900_ServoPD;                      // 舵机PD
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
int   IntClip(int x, int low, int up);                                  // 限幅函数
float FloatClip(float x, float low, float up);                          // 限幅函数
float PidLocCtrl(PIDController *pid, float error);                      // PID位置式控制器输出
float PidIncCtrl(PIDController *pid, float error);                      // PID增量式控制器输出
void  PDLocServoCtrl(void);                                             // 舵机 PD位置式控制器
void  PIDIncMotorCtrl(int16 TARGET_MOTOR_ENCODER);                      // 电机 PID增量式控制器
//===================================================函数声明END===================================================
#endif /* CODE_PID_H_ */
