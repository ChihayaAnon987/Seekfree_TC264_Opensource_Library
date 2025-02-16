/*
 * PID.h
 *
 *  Created on: 2025年1月12日
 *      Author: 20483
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

//===================================================宏定义BEG===================================================
#define MOVE_AVERAGE_SIZE           (32)                     // 定义缓冲区大小
//===================================================宏定义END===================================================


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
    // unsigned long last_t;                                      // 上次时间
} PIDController;

// typedef struct
// {
//     float Kp;                                                  // 比例系数P
//     float Ki;                                                  // 积分系数I
//     float Kd;                                                  // 微分系数D
//     float imax;                                                // 积分限幅

//     float targetVal;                                           // 目标值
//     float currentError;                                        // 当前时刻的误差
//     float preError;                                            // 前一时刻的误差
//     float prepreError;                                         // 前两次采样时刻的误差
//     float derivative;                                          // 误差微分
//     float integral;                                            // 误差积分
//     float output;                                              // 输出
// } PIDController;

typedef struct                                                  // 滑动平均滤波结构体
{
    uint8_t index;                                              // 下标
    uint8_t buffer_size;                                        // 缓冲区大小
    int32_t data_buffer[MOVE_AVERAGE_SIZE];                     // 缓冲区
    int32_t data_sum;                                           // 数据和
    int32_t data_average;                                       // 数据平均值
} move_filter_struct;
//===================================================类型定义END===================================================


//===================================================全局变量BEG===================================================
extern PIDController PID_Init;         // PID初始化控制器
extern PIDController PID_GPS;          // GPS PID控制器
extern PIDController PID_IMU;          // IMU PID控制器
extern PIDController PID_SERVO;        // 舵机 PID控制器
extern PIDController PID_MOTOR;        // 电机 PID控制器
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
int   IntClip(int x, int low, int up);                                  // 限幅函数
float FloatClip(float x, float low, float up);                          // 限幅函数
float PidLocCtrl(PIDController *pid, float error);                      // PID位置式控制器输出
float PidIncCtrl(PIDController *pid, float error);                      // PID增量式控制器输出
void  PdGpsCtrl(void);                                                  // GPS PD控制器
void  PDLocServoCtrl(void);                                             // 舵机 PD位置式控制器
void  PIDIncMotorCtrl(int16 TARGET_MOTOR_ENCODER);                      // 电机 PID增量式控制器
// void PidLocCtrl(PIDConntroller *pid, float measuredVal);                 // PID位置式控制器输出
// void PidIncCtrl(PIDController *pid, float measuredVal);                 // PID增量式控制器输出
void move_filter_init(move_filter_struct *move_filter);                 // 滑动平均滤波初始化
void move_filter_calc(move_filter_struct *move_filter, int32_t new_data); // 滑动平均滤波计算
//===================================================函数声明END===================================================
#endif /* CODE_PID_H_ */
