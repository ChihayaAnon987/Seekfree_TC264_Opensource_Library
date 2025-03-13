/*
 * Common_peripherals.c
 *
 *  Created on: 2025年1月7日
 *      Author: 20483
 */
#include "zf_common_headfile.h"

int16 Servo_Angle     = SERVO_MOTOR_MID;
int16 Encoder;
float System_Time     = 0;
uint8 LED_Buzzer_Flag = 0;
void CPU0_Init()
{
    AHRS_init();                                                    // AHRS初始化
    IMU_Init();                                                     // IMU初始化
    gnss_init(TAU1201);                                             // GPS初始化
    SERVO_Init();                                                   // 舵机初始化
    DRV8701_Init();                                                 // 电机初始化
    encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR);    // 编码器初始化
#if UART_RECEIVER_ENABLE == 1
    uart_receiver_init();                                           // sbus接收机初始化
#elif UART_RECEIVER_ENABLE == 0
    wireless_uart_init();                                           // 初始化无线串口
#endif
    Oscilloscope_Init(8);                                           // 逐飞示波器初始化
    pit_ms_init(CCU60_CH0, 5);                                      // 中断，IMU数据采集
    pit_ms_init(CCU60_CH1, 5);                                      // 中断 编码器、舵机电机PID
    pit_ms_init(CCU61_CH0, 100);                                    // 中断 GPS数据解析
    pit_ms_init(CCU61_CH1, 5);                                      // 中断 遥控器
}

void CPU1_Init()
{
    Buzzer_Init();                                                  // 蜂鸣器初始化
    KEY_Init();                                                     // 按键初始化
#if MT9V03X_ENABLE == 1
    mt9v03x_init();                                                 // 总钻风初始化
#endif
    key_init(10);                                                   // 按键初始化
    ips200_Init();                                                  // 屏幕初始化
    Buzzer_Check(200);                                              // 自检，表示初始化成功
}

void ips200_Init()
{
    ips200_init(IPS200_TYPE);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);                   // 护眼模式
    ips200_full(RGB565_BLACK);
}
void Oscilloscope_Init(uint8 Channel_Num)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART); // 逐飞助手初始化
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, LeftLine_x, RightLine_x, CenterLine_x, LeftLine_y, RightLine_y, CenterLine_y);
    for(int i = 0; i < Channel_Num; i++)
    {
        oscilloscope_data.data[i] = 0;
    }
    oscilloscope_data.channel_num = Channel_Num;
}

// DRV8701初始化
void DRV8701_Init(void)
{
    gpio_init(DIR_CH1, GPO, 0, GPO_PUSH_PULL);
    pwm_init (PWM_CH1, 17000, 0);

}

void DRV8701_MOTOR_DRIVER(int Motor_PWM)
{
    Motor_PWM = IntClip(Motor_PWM, -PWM_DUTY_MAX, PWM_DUTY_MAX); // 限幅
    if(Motor_PWM >= 0)                      // 电机正转
    {
        gpio_set_level(DIR_CH1, 0);
        pwm_set_duty  (PWM_CH1, Motor_PWM);
    }
    else                                // 电机反转
    {
        gpio_set_level(DIR_CH1, 1);
        pwm_set_duty  (PWM_CH1, -Motor_PWM);
    }
}

void Encoder_Get()
{
    Encoder = -encoder_get_count(ENCODER1_TIM);                         // 采集对应编码器数据
    encoder_clear_count(ENCODER1_TIM);                                  // 清除对应计数

    double speed= Encoder * 45 / 248;                     // 计算速度m/s（放大了1000倍）
    Delta_x += speed * 0.005 * cos(angle[2] * PI / 180);                     // 每五毫秒的位移
    Delta_y += speed * 0.005 * sin(angle[2] * PI / 180);

}

// 舵机初始化
void SERVO_Init(void)
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));// 引脚，舵机频率，占空比
}

// 舵机驱动测试
void Servo_SetTest(int16 *angle)
{
    // 舵机限幅
    if(*angle > SERVO_MOTOR_LMAX) {*angle = SERVO_MOTOR_LMAX;}
    if(*angle < SERVO_MOTOR_RMAX) {*angle = SERVO_MOTOR_RMAX;}

    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(*angle));
}

// 舵机驱动
void Servo_Set(int16 angle)
{
    // 舵机限幅
    if(angle > SERVO_MOTOR_LMAX) {angle = SERVO_MOTOR_LMAX;}
    if(angle < SERVO_MOTOR_RMAX) {angle = SERVO_MOTOR_RMAX;}

    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(angle));
}

// 蜂鸣器初始化
void Buzzer_Init()
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

// 蜂鸣器自检函数
void Buzzer_Check(int time)
{
    gpio_set_level(BUZZER_PIN, 1);
    system_delay_ms(time);
    gpio_set_level(BUZZER_PIN, 0);
}

// 按键初始化
void KEY_Init()
{
    gpio_init(LED1, GPO,GPIO_HIGH, GPO_PUSH_PULL);// 初始化  LED1输出 默认高电平 推挽输出模式
    gpio_init(LED2, GPO,GPIO_HIGH, GPO_PUSH_PULL);// 初始化  LED2输出 默认高电平 推挽输出模式
    gpio_init(LED3, GPO,GPIO_HIGH, GPO_PUSH_PULL);// 初始化  LED3输出 默认高电平 推挽输出模式
    gpio_init(LED4, GPO,GPIO_HIGH, GPO_PUSH_PULL);// 初始化  LED4输出 默认高电平 推挽输出模式

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);// 初始化  KEY1输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);// 初始化  KEY2输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);// 初始化  KEY3输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);// 初始化  KEY4输入 默认高电平 上拉输入

    gpio_init(SWITCH1, GPI,GPIO_HIGH, GPI_FLOATING_IN);// 初始化  SWITCH1输出 默认高电平 浮空输入
    gpio_init(SWITCH2, GPI,GPIO_HIGH, GPI_FLOATING_IN);// 初始化  SWITCH2输出 默认高电平 浮空输入

}


void System_Time_Count()
{
    System_Time += 0.005;
}

void LED_Buzzer_Flag_Ctrl(gpio_pin_enum pin)
{
    switch(pin)
    {
        case LED1      : LED_Buzzer_Flag = 1; break;
        case LED2      : LED_Buzzer_Flag = 2; break;
        case LED3      : LED_Buzzer_Flag = 3; break;
        case LED4      : LED_Buzzer_Flag = 4; break;
        case BUZZER_PIN: LED_Buzzer_Flag = 5; break;
        default: break;
    }
}

void LED_Buzzer_Ctrl()
{
    switch (LED_Buzzer_Flag)
    {
        case 1: gpio_set_level(LED1, 0); system_delay_ms(500); gpio_set_level(LED1, 1); LED_Buzzer_Flag = 0; break;
        case 2: gpio_set_level(LED2, 0); system_delay_ms(500); gpio_set_level(LED2, 1); LED_Buzzer_Flag = 0; break;
        case 3: gpio_set_level(LED3, 0); system_delay_ms(500); gpio_set_level(LED3, 1); LED_Buzzer_Flag = 0; break;
        case 4: gpio_set_level(LED4, 0); system_delay_ms(500); gpio_set_level(LED4, 1); LED_Buzzer_Flag = 0; break;
        case 5: Buzzer_Check(500); LED_Buzzer_Flag = 0; break;
        default: break;
    }
}
