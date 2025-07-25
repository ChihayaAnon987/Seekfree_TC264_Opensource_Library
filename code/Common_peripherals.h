/*
 * Common_peripherals.h
 * 通用外设，包含电机、舵机、编码器、蜂鸣器、按键、LED、开关
 *  Created on: 2025年1月7日
 *      Author: 20483
 */

#ifndef CODE_COMMON_PERIPHERALS_H_
#define CODE_COMMON_PERIPHERALS_H_

//===================================================宏定义BEG===================================================
#define VERSION_INFO             ("V6.6.1")                  // 版本号
#define UART_RECEIVER_ENABLE     (1)                         // 0: 不启用遥控器    1: 启用遥控器
#define WIRELESS_UART_ENABLE     (0)                         // 0: 不启用无线串口  1: 启用无线串口
#define WIFI_UART_ENABLE         (0)                         // 0: 不启用WIFI串口  1: 启用WIFI串口  遥控器、无线串口、WIFI串口只能启用一个
#define CAR_NUMBER               (1)                         // 0: 0号车           1: 1号车
#define MT9V03X_ENABLE           (0)                         // 0: 不启用摄像头    1: 启用摄像头
#define BLDC_ENABLE              (1)                         // 0: 不启用无刷      1: 启用无刷
#define MOTOR_LOOP_ENABLE        (0)                         // 0: 开环控制        1: 闭环PID控制

#define IPS200_TYPE     (IPS200_TYPE_SPI)
// 双排排针 MCU 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
// 单排排针 SPI 两寸屏     这里宏定义填写 IPS200_TYPE_SPI

#define BUZZER_PIN              (P33_10)                     // 蜂鸣器控制引脚

#define LED1                    (P20_9)                      // LED1 控制引脚
#define LED2                    (P20_8)                      // LED2 控制引脚
#define LED3                    (P21_5)                      // LED3 控制引脚
#define LED4                    (P21_4)                      // LED4 控制引脚

#define KEY1                    (P20_6)                      // 按键1 控制引脚
#define KEY2                    (P20_7)                      // 按键2 控制引脚
#define KEY3                    (P11_2)                      // 按键3 控制引脚
#define KEY4                    (P11_3)                      // 按键4 控制引脚
#define KEY5                    (P20_2)                      // 按键5 控制引脚

#define SWITCH1                 (P33_11)                     // 开关1 控制引脚
#define SWITCH2                 (P21_2 )                     // 开关2 控制引脚

#if BLDC_ENABLE
#define PWM_CH1             ATOM0_CH7_P02_7
#define DIR_CH1             P02_6
#define ENCODER1_TIM        TIM5_ENCODER
#define ENCODER1_PLUS       TIM5_ENCODER_CH1_P10_3
#define ENCODER1_DIR        TIM5_ENCODER_CH2_P10_1
#else
#define MAX_DUTY            50
#define PWM_CH1             ATOM0_CH5_P02_5
#define DIR_CH1             P02_4
#define ENCODER1_TIM        TIM5_ENCODER
#define ENCODER1_PLUS       TIM5_ENCODER_CH1_P10_3
#define ENCODER1_DIR        TIM5_ENCODER_CH2_P10_1
#endif

#define SERVO_MOTOR_PWM             (ATOM1_CH1_P33_9)        // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (300)                    // 定义主板上舵机频率  请务必注意范围 50-300
#if CAR_NUMBER == 0
#define SERVO_MOTOR_RMAX            (66)
#define SERVO_MOTOR_MID             (92)
#define SERVO_MOTOR_LMAX            (128)
#elif CAR_NUMBER == 1
#define SERVO_MOTOR_RMAX            (73)
#define SERVO_MOTOR_MID             (105)
#define SERVO_MOTOR_LMAX            (137)
#endif

#define SERVO_MOTOR_RMIN_DUTY       (SERVO_MOTOR_DUTY(SERVO_MOTOR_RMAX))     // 右，定义主板上舵机活动范围 占空比
#define SERVO_MOTOR_MID_DUTY        (SERVO_MOTOR_DUTY(SERVO_MOTOR_MID))      // 中值 占空比
#define SERVO_MOTOR_LMIN_DUTY       (SERVO_MOTOR_DUTY(SERVO_MOTOR_LMAX))     // 左，定义主板上舵机活动范围 占空比
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
//===================================================宏定义END===================================================


//===================================================全局变量BEG===================================================
extern int16 Encoder;                                          // 编码器计数值
extern int16 Servo_Angle;                                      // 舵机角度
extern float System_Time;                                      // 系统时间
extern uint8 LED_Buzzer_Flag;                                  // 标志
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
void CPU0_Init(void);                                          // CPU0 初始化
void CPU1_Init(void);                                          // CPU1 初始化
void ips200_Init(void);                                        // 屏幕初始化
void Oscilloscope_Init(uint8 Channel_Num);                     // 无线串口初始化
void MOTOR_Init(void);                                         // 电机初始化
void MOTOR_Ctrl(int16 MOTOR_PWM);                              // 电机驱动
void Encoder_Get(void);                                        // 编码器读取
void SERVO_Init(void);                                         // 舵机初始化
void Servo_SetTest(int16 angle);                               // 舵机驱动，用于调试
void Servo_Set(int16 angle);                                   // 舵机驱动
void Buzzer_Init(void);                                        // 蜂鸣器初始化
void Buzzer_Check(int time);                                   // 蜂鸣器自检函数
void KEY_Init(void);                                           // 按键初始化
void System_Time_Count(void);                                  // 系统时间计时
void LED_Buzzer_Flag_Ctrl(gpio_pin_enum pin);                  // LED、蜂鸣器控制
void LED_Buzzer_Ctrl(void);                                    // LED、蜂鸣器控制
//===================================================函数声明END===================================================




#endif /* CODE_COMMON_PERIPHERALS_H_ */
