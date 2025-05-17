/*
 * GUI.c
 *
 *  Created on: 2025年1月10日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

Parameter_set Parameter_set0=
{
    {1.34, 0.0, 0.7},            // 舵机PID
    {1.0, 0.0, 0.0},            // 速度PID
    3000,                       // 调试的速度
    1.5,                        // 换点距离
    SERVO_MOTOR_MID             // 舵机机械可调中值
};

int    key_value;
int    Point0         = 0;    // 菜单点位
int    Point1         = 0;
int    Point2         = 0;
int    Point3         = 0;
int    Point4         = 0;
int8   Map_Flag       = 1;
int    Task_Point_Set = 1;
int16  Task_Flag      = 0;
int    CameraPoint    = 0;
double Test_Angle     = 0;    // 调试用
int16  Test_Encoder   = 0;    // 调试用
uint8  Start_Flag     = 0;    // 发车标志
float  Star_Time      = 0;
float  Stop_Time      = 0;
double Actual_Dist    = 0;
seekfree_assistant_oscilloscope_struct oscilloscope_data;
gui_menu_enum func_index = 0;
uint8_t data_buffer[32];
uint8_t data_len;

menu_table table[41]=
{
    // current, up, down, back, enter

    // 菜单0
    {enum_first_menu00, enum_first_menu11, enum_first_menu01, enum_first_menu00, enum_secon_menu00, main_menu0},       // 踩点GPS一层
    {enum_secon_menu00, enum_secon_menu00, enum_secon_menu00, enum_first_menu00, enum_secon_menu00, CaiDian_menu},     // 踩点GPS二层

    // 菜单1
    {enum_first_menu01, enum_first_menu00, enum_first_menu02, enum_first_menu01, enum_secon_menu01, main_menu1},       // PID一层
    {enum_secon_menu01, enum_secon_menu02, enum_secon_menu02, enum_first_menu01, enum_third_menu00, ServoPID},         // ServoPID二层
    {enum_secon_menu02, enum_secon_menu01, enum_secon_menu01, enum_first_menu01, enum_third_menu03, MotorPID},         // MotorPID二层
    {enum_third_menu00, enum_third_menu02, enum_third_menu01, enum_secon_menu01, enum_third_menu00, ServoP_menu},      // ServoP
    {enum_third_menu01, enum_third_menu00, enum_third_menu02, enum_secon_menu01, enum_third_menu01, ServoI_menu},      // ServoI
    {enum_third_menu02, enum_third_menu01, enum_third_menu00, enum_secon_menu01, enum_third_menu02, ServoD_menu},      // ServoD
    {enum_third_menu03, enum_third_menu05, enum_third_menu04, enum_secon_menu02, enum_third_menu03, MotorP_menu},      // MotorP
    {enum_third_menu04, enum_third_menu03, enum_third_menu05, enum_secon_menu02, enum_third_menu04, MotorI_menu},      // MotorI
    {enum_third_menu05, enum_third_menu04, enum_third_menu03, enum_secon_menu02, enum_third_menu05, MotorD_menu},      // MotorD

    // 菜单2
    {enum_first_menu02, enum_first_menu01, enum_first_menu03, enum_first_menu02, enum_secon_menu03, main_menu2},        // 运行GPS显示一层
    {enum_secon_menu03, enum_secon_menu03, enum_secon_menu03, enum_first_menu02, enum_secon_menu03, GPS_menu},          // 运行GPS显示二层

    // 菜单3
    {enum_first_menu03, enum_first_menu02, enum_first_menu04, enum_first_menu03, enum_secon_menu04, main_menu3},        // 调速,舵机机械中值，换点距离
    {enum_secon_menu04, enum_secon_menu06, enum_secon_menu05, enum_first_menu03, enum_secon_menu04, spd_menu},          // 调速
    {enum_secon_menu05, enum_secon_menu04, enum_secon_menu06, enum_first_menu03, enum_secon_menu05, Distance_menu},     // 换点距离
    {enum_secon_menu06, enum_secon_menu05, enum_secon_menu04, enum_first_menu03, enum_secon_menu06, TaskPoint},         // 任务点

    // 菜单4
    {enum_first_menu04, enum_first_menu03, enum_first_menu05, enum_first_menu04, enum_secon_menu07, main_menu4},        // 遥控模式一层
    {enum_secon_menu07, enum_secon_menu07, enum_secon_menu07, enum_first_menu04, enum_secon_menu07, RemoteCtrl_menu},   // 遥控模式二层

    // 菜单5
    {enum_first_menu05, enum_first_menu04, enum_first_menu06, enum_first_menu05, enum_secon_menu08, main_menu5},        // GPS点位查看一层
    {enum_secon_menu08, enum_secon_menu08, enum_secon_menu08, enum_first_menu05, enum_secon_menu08, Points_menu},       // GPS点位查看二层

    // 菜单6
    {enum_first_menu06, enum_first_menu05, enum_first_menu07, enum_first_menu06, enum_secon_menu09, main_menu6},        // 摄像头图像一层
    {enum_secon_menu09, enum_secon_menu09, enum_secon_menu09, enum_first_menu06, enum_secon_menu09, ZongZuanF} ,        // 摄像头图像二层

    // 菜单7
    {enum_first_menu07, enum_first_menu06, enum_first_menu08, enum_first_menu07, enum_secon_menu10, main_menu7},        // 陀螺仪一层
    {enum_secon_menu10, enum_secon_menu10, enum_secon_menu10, enum_first_menu07, enum_secon_menu10, Imu963_menu},       // 陀螺仪二层

    // 菜单8
    {enum_first_menu08, enum_first_menu07, enum_first_menu09, enum_first_menu08, enum_secon_menu11, main_menu8},        // Flash一层
    {enum_secon_menu11, enum_secon_menu11, enum_secon_menu11, enum_first_menu08, enum_secon_menu11, Flash_menu},        // Flash二层

    // 菜单9
    {enum_first_menu09, enum_first_menu08, enum_first_menu10, enum_first_menu09, enum_secon_menu12, main_menu9},        // 舵机测试一层
    {enum_secon_menu12, enum_secon_menu12, enum_secon_menu12, enum_first_menu09, enum_secon_menu12, Servo_menu},        // 舵机测试二层

    // 菜单10
    {enum_first_menu10, enum_first_menu09, enum_first_menu11, enum_first_menu10, enum_secon_menu13, main_menu10},       // 参数设置一层
    {enum_secon_menu13, enum_secon_menu15, enum_secon_menu14, enum_first_menu10, enum_third_menu06, LoopEnable_Param},  // 闭环参数设置二层
    {enum_secon_menu14, enum_secon_menu13, enum_secon_menu15, enum_first_menu10, enum_third_menu07, LoopDisable_Param}, // 开环参数设置二层
    {enum_secon_menu15, enum_secon_menu14, enum_secon_menu13, enum_first_menu10, enum_third_menu08, Param_Set},         // 其他参数设置二层
    {enum_third_menu06, enum_third_menu06, enum_third_menu06, enum_secon_menu13, enum_third_menu06, LoopEnable_menu},   // 闭环参数设置三层
    {enum_third_menu07, enum_third_menu07, enum_third_menu07, enum_secon_menu14, enum_third_menu07, LoopDisable_menu},  // 闭环参数设置三层
    {enum_third_menu08, enum_third_menu08, enum_third_menu08, enum_secon_menu15, enum_third_menu08, Param_Set_menu},    // 闭环参数设置三层

    // 菜单11
    {enum_first_menu11, enum_first_menu10, enum_first_menu00, enum_first_menu11, enum_secon_menu16, main_menu11},       // 任务选择一层
    {enum_secon_menu16, enum_secon_menu17, enum_secon_menu17, enum_first_menu11, enum_third_menu09, Task_Select_menu},  // 任务选择二层, 科目一、二、三
    {enum_secon_menu17, enum_secon_menu16, enum_secon_menu16, enum_first_menu11, enum_third_menu10, Task_Four_menu},    // 任务选择二层, 科目四
    {enum_third_menu09, enum_third_menu09, enum_third_menu09, enum_secon_menu16, enum_third_menu09, Task_Select},       // 任务选择三层, 科目一、二、三
    {enum_third_menu10, enum_third_menu10, enum_third_menu10, enum_secon_menu17, enum_third_menu10, Task_Four},         // 任务选择三层, 科目四
};

/////////////////////////////////一层菜单-------------------------------------------------
void main_menu0(void)
{
    ips200_show_string(  0, 16 * 0, "-->CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu1(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "-->PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu2(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "-->GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu3(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "-->Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu4(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "-->RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu5(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "-->Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu6(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "-->Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu7(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "-->Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu8(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "-->Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu9(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "-->SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu10(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "-->ParamSet  ");
    ips200_show_string(136, 16 * 3, "   TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}

void main_menu11(void)
{
    ips200_show_string(  0, 16 * 0, "   CaiDian   ");
    ips200_show_string(  0, 16 * 1, "   PID       ");
    ips200_show_string(  0, 16 * 2, "   GPS Show  ");
    ips200_show_string(  0, 16 * 3, "   Duty      ");
    ips200_show_string(  0, 16 * 4, "   RemoteCtrl");
    ips200_show_string(  0, 16 * 5, "   Points    ");
    ips200_show_string(  0, 16 * 6, "   Camera    ");
    ips200_show_string(  0, 16 * 7, "   Imu963    ");
    ips200_show_string(136, 16 * 0, "   Flash     ");
    ips200_show_string(136, 16 * 1, "   SevroTest ");
    ips200_show_string(136, 16 * 2, "   ParamSet  ");
    ips200_show_string(136, 16 * 3, "-->TaskSelect");
    ips200_draw_line  (108,   0, 108, 184, RGB565_PURPLE);
    ips200_draw_line  (132,   0, 132, 184, RGB565_PURPLE);
    ips200_draw_line  (  0, 184, 239, 184, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 0, 16, TeamName0[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 1, 16, TeamName1[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 2, 16, TeamName2[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 3, 16, TeamName3[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 4, 16, TeamName4[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 5, 16, TeamName5[0], 1, RGB565_PURPLE);
    ips200_show_chinese(112, 16 * 6, 16, TeamName6[0], 1, RGB565_PURPLE);
    ips200_show_rgb565_image(114, 185, (const uint16 *)my_picture, 126, 135, 126, 135, 1);
}
/////////////////////////////////二层菜单-------------------------------------------------
void CaiDian_menu(void)
{
    float Distance;
    ips200_show_string(0, 16 * 0, "Now_Lat:");
    ips200_show_string(0, 16 * 1, "Now_Lon:");

    ips200_show_float(96, 16 * 0, gnss.latitude, 4, 6);
    ips200_show_float(96, 16 * 1, gnss.longitude, 4, 6);

    ips200_show_string(  0, 16 * 2, "Point_NUM:");
    ips200_show_uint  ( 96, 16 * 2, Point_NUM, 3);
    ips200_show_string(  0, 16 * 3, "Last Saved Point:");

    if(Point_NUM >= 0)
    {
        ips200_show_float( 0, 16 * 4, lat_union[Point_NUM - 1].double_type, 4, 6);
        ips200_show_float( 0, 16 * 5, lon_union[Point_NUM - 1].double_type, 4, 6);

        Distance = get_two_points_distance (gnss.latitude, gnss.longitude, lat_union[Point_NUM - 1].double_type, lon_union[Point_NUM - 1].double_type);
        ips200_show_string(0, 16 * 6, "Distance:");
        ips200_show_float(96, 16 * 6, Distance, 4, 6);

    }
    ips200_show_string(  0, 16 *  7, "KEY1:Get  Point");
    ips200_show_string(  0, 16 *  8, "KEY2:Save Point");
    ips200_show_string(  0, 16 *  9, "KEY3:Point+1");
    ips200_show_string(  0, 16 * 10, "KEY4:Point-1");

}

void ServoPID(void)
{
    ips200_show_string(0, 16 * 0, "-->ServePID");
    ips200_show_string(0, 16 * 1, "   MotorPID");
}

void MotorPID(void)
{
    ips200_show_string(0, 16 * 0, "   ServePID");
    ips200_show_string(0, 16 * 1, "-->MotorPID");
}


void GPS_menu(void)
{
    ips200_show_uint(    0, 16 * 0, gnss.time.year  , 4);
    ips200_draw_line(   32, 16 * 0, 32, 16 * 1, RGB565_PURPLE);
    ips200_show_uint(   40, 16 * 0, gnss.time.month , 2);
    ips200_draw_line(   48, 16 * 0, 48, 16 * 1, RGB565_PURPLE);
    ips200_show_uint(   64, 16 * 0, gnss.time.day   , 2);
    ips200_show_uint(  176, 16 * 0, gnss.time.hour  , 2);
    ips200_show_string(192, 16 * 0, ":");
    ips200_show_uint(  200, 16 * 0, gnss.time.minute, 2);
    ips200_show_string(216, 16 * 0, ":");
    ips200_show_uint(  224, 16 * 0, gnss.time.second, 2);

    ips200_show_string( 16, 16 *  1, "Lat:");
    ips200_show_string(128, 16 *  1, "Lon:");
    ips200_show_string(168, 16 *  1, "state:");
    ips200_show_string(  0, 16 *  3, "Delta_x:");
    ips200_show_string(  0, 16 *  4, "Delta_y:");
    ips200_show_string(  0, 16 *  5, "Speed:");
    ips200_show_string(  0, 16 *  6, "MaxSpeed:");
    ips200_show_string(  0, 16 *  7, "Accel:");
    ips200_show_string(  0, 16 *  8, "MaxAccel:");
    ips200_show_string(  0, 16 *  9, "Angle:");
    ips200_show_string(  0, 16 * 10, "SateNumber:");


    ips200_show_uint  (216, 16 *  1, gnss.state     , 1);
    ips200_show_float ( 16, 16 *  2, gnss.latitude  , 4, 6);
    ips200_show_float (128, 16 *  2, gnss.longitude , 4, 6);
    ips200_show_float ( 80, 16 *  3, Delta_x, 4, 6);
    ips200_show_float ( 80, 16 *  4, Delta_y, 4, 6);
    ips200_show_float ( 48, 16 *  5, gnss.speed     , 3, 3);
    ips200_show_float ( 72, 16 *  6, GpsMaxSpeed    , 3, 3);
    ips200_show_float ( 48, 16 *  7, GpsAccel       , 3, 3);
    ips200_show_float ( 72, 16 *  8, GpsMaxAccel    , 3, 3);
    ips200_show_float ( 48, 16 *  9, Angle          , 4, 6);
    ips200_show_uint  ( 88, 16 * 10, gnss.satellite_used, 2);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = gnss.latitude;
    oscilloscope_data.data[1] = gnss.longitude;
    oscilloscope_data.data[2] = Angle;
    oscilloscope_data.data[3] = gnss.direction;
    oscilloscope_data.data[4] = gnss.speed;
    oscilloscope_data.data[5] = GpsAccel;
#endif
}

void spd_menu(void)
{
    ips200_show_string( 0, 16 * 0, "-->Duty    :");
    ips200_show_string( 0, 16 * 1, "   Distance:");
    ips200_show_string( 0, 16 * 2, "   TasPoint:");    

    ips200_show_uint(184, 16 * 0, Point1, 3);
    ips200_draw_line(208, 16 * 0, 208, 16 * 1, RGB565_PURPLE);
    ips200_show_uint(216, 16 * 0, NUM_GPS_DATA - 1, 3);

    int Page = Point1 / Page_Point_Num;
    int RightArrow = Point1 % Page_Point_Num + 1;
    ips200_show_string(168, 16 * RightArrow, "-->");
    for(int i = 1; i <= Page_Point_Num; i++)
    {
        ips200_show_uint (192, 16 * i, GpsTgtEncod[i - 1 + Page * Page_Point_Num], 5);
    }

    ips200_show_string(  0, 16 *  9, "KEY1:Point-1");
    ips200_show_string(  0, 16 * 10, "KEY2:Point+1");
    ips200_show_string(120, 16 *  9, "KEY3:Duty+100");
    ips200_show_string(120, 16 * 10, "KEY4:Duty-100");


}

void Distance_menu(void)
{

    ips200_show_string( 0, 16 * 0, "   Duty    :");
    ips200_show_string( 0, 16 * 1, "-->Distance:");
    ips200_show_string( 0, 16 * 2, "   TasPoint:");

    ips200_show_uint(184, 16 * 0, Point1, 3);
    ips200_draw_line(208, 16 * 0, 208, 16 * 1, RGB565_PURPLE);
    ips200_show_uint(216, 16 * 0, NUM_GPS_DATA - 1, 3);

    int Page = Point1 / Page_Point_Num;
    int RightArrow = Point1 % Page_Point_Num + 1;
    ips200_show_string(160, 16 * RightArrow, "-->");
    for(int i = 1; i <= Page_Point_Num; i++)
    {
        ips200_show_float (184, 16 * i, GpsDistance[i - 1 + Page * Page_Point_Num], 2, 2);
    }

    ips200_show_string(  0, 16 *  9, "KEY1:Point-1");
    ips200_show_string(  0, 16 * 10, "KEY2:Point+1");
    ips200_show_string(120, 16 *  9, "KEY3:Dist+0.10");
    ips200_show_string(120, 16 * 10, "KEY4:Dist-0.10");

}

void TaskPoint(void)
{
    ips200_show_string(0, 16 * 0, "   Duty    ");
    ips200_show_string(0, 16 * 1, "   Distance");
    ips200_show_string(0, 16 * 2, "-->TasPoint");

    if(Task_Point_Set == 1)
    {
        ips200_show_string(0, 16 * 3, "-->Task1Points:");
        ips200_show_string(0, 16 * 4, "   Task2Bucket:");
        ips200_show_string(0, 16 * 5, "   Task3Points:");
        ips200_show_int (120, 16 * 3, Task1_Points, 3);
        ips200_show_int (120, 16 * 4, Task2_Bucket, 3);
        ips200_show_int (120, 16 * 5, Task3_Points, 3);
    }
    if(Task_Point_Set == 2)
    {
        ips200_show_string(0, 16 * 3, "   Task1Points:");
        ips200_show_string(0, 16 * 4, "-->Task2Bucket:");
        ips200_show_string(0, 16 * 5, "   Task3Points:");
        ips200_show_int (120, 16 * 3, Task1_Points, 3);
        ips200_show_int (120, 16 * 4, Task2_Bucket, 3);
        ips200_show_int (120, 16 * 5, Task3_Points, 3);
    }
    if(Task_Point_Set == 3)
    {
        ips200_show_string(0, 16 * 3, "   Task1Points:");
        ips200_show_string(0, 16 * 4, "   Task2Bucket:");
        ips200_show_string(0, 16 * 5, "-->Task3Points:");
        ips200_show_int (120, 16 * 3, Task1_Points, 3);
        ips200_show_int (120, 16 * 4, Task2_Bucket, 3);
        ips200_show_int (120, 16 * 5, Task3_Points, 3);
    }

    ips200_show_string(  0, 16 *  9, "KEY1:Task-1");
    ips200_show_string(  0, 16 * 10, "KEY2:Task+1");
    ips200_show_string(120, 16 *  9, "KEY3:Point+1");
    ips200_show_string(120, 16 * 10, "KEY4:Point-1");

}

void RemoteCtrl_menu(void)
{
    ips200_show_string(80, 16 * 0, "RemoteCtrl:");
    if(1 == uart_receiver.finsh_flag)                                   // 帧完成标志判断
    {
        if(1 == uart_receiver.state)                                    // 遥控器失控状态判断
        {
            ips200_show_string(  0, 16 * 1, "CH1:");
            ips200_show_string(  0, 16 * 2, "CH2:");
            ips200_show_string(  0, 16 * 3, "CH3:");
            ips200_show_string(110, 16 * 1, "CH4:");
            ips200_show_string(110, 16 * 2, "CH5:");
            ips200_show_string(110, 16 * 3, "CH6:");
            ips200_show_uint(   40, 16 * 1, uart_receiver.channel[0], 4);
            ips200_show_uint(   40, 16 * 2, uart_receiver.channel[1], 4);
            ips200_show_uint(   40, 16 * 3, uart_receiver.channel[2], 4);
            ips200_show_uint(  150, 16 * 1, uart_receiver.channel[3], 4);
            ips200_show_uint(  150, 16 * 2, uart_receiver.channel[4], 4);
            ips200_show_uint(  150, 16 * 3, uart_receiver.channel[5], 4);
        }
        else
        {
            ips200_show_string(  0, 16 * 1, "RemoteCtrl disconnected"); // 串口输出失控提示
        }
        ips200_show_string(  0, 16 * 4, "finsh_flag:");
        ips200_show_uint(   88, 16 * 4, uart_receiver.finsh_flag, 1);
        uart_receiver.finsh_flag = 0;                                   // 帧完成标志复位
    }
    ips200_show_string(  0, 16 * 5, "state:");
    ips200_show_uint(   48, 16 * 5, uart_receiver.state, 1);
    ips200_show_string( 64, 16 * 5, "finsh_flag:");
    ips200_show_uint(  152, 16 * 5, uart_receiver.finsh_flag, 1);
    ips200_show_string(  0, 16 * 6, "Control_Flag:");
    ips200_show_uint(  104, 16 * 6, Control_Flag, 1);
    ips200_show_string(  0, 16 * 7, "Motor_PWM:");
    ips200_show_int (   80, 16 * 7 , RemoteCtrl_Speed, 4);
    ips200_show_string(  0, 16 * 8, "Encoder:");
    ips200_show_int   ( 64, 16 * 8, Encoder, 5);
    
}

void Points_menu(void)
{
    if(Map_Flag == 1)
    {
        ips200_show_string( 16, 16 * 0, "Lat:");
        ips200_show_string(128, 16 * 0, "Lon:");
        ips200_show_uint  (184, 16 * 0, Point0, 3);
        ips200_show_string(208, 16 * 0, "/");
        ips200_show_uint  (216, 16 * 0, NUM_GPS_DATA - 1, 3);
    
        int Page = Point0 / Page_Point_Num;
        int RightArrow = Point0 % Page_Point_Num + 1;
    
        ips200_show_string(0, 16 * RightArrow, "->");
        for(int i = 1; i <= Page_Point_Num; i++)
        {
            ips200_show_float ( 16, 16 * i, Point[i - 1 + Page * Page_Point_Num].latitude, 3, 6);
            ips200_show_float (128, 16 * i, Point[i - 1 + Page * Page_Point_Num].lonitude, 3, 6);
            if(i  + Page * 10 == NUM_GPS_DATA)
            {
                break;
            }
        }
    
        ips200_show_string(  0, 16 *  9, "KEY1:Up  /Lat+North");
        ips200_show_string(  0, 16 * 10, "KEY2:Down/Lat-South");
        ips200_show_string(  0, 16 * 11, "KEY3:Fix /Lon+East ");
        ips200_show_string(  0, 16 * 12, "KEY4:Swit/Lon-West ");
    }
    if(Map_Flag == -1)
    {
        drawGrid();
        drawPoints();
    }
    updateCarPosition();

}

void ZongZuanF(void)
{
#if MT9V03X_ENABLE
    Process_Image();
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        // memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        // seekfree_assistant_camera_send();
        ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    }

    ips200_show_string(0, 16 * 8, "Left Line:");
    ips200_show_string(0, 16 * 9, "RightLine:");
    ips200_show_int( 216, 16 * 8, clip_value, 2);
    if(LeftLineNum > 0)
    {
        for(int i = 0; i < MT9V03X_H; i++)
        {
            ips200_draw_point(IntClip(LeftLine_x[i], 0, ips200_width_max - 1), IntClip(LeftLine_y[i], 0, ips200_height_max - 1), RGB565_RED);
        }
        ips200_show_string(80, 16 * 8, "FoundLine");
    }
    else
    {
        ips200_show_string(80, 16  * 8, "Not Found");
    }
    if(RightLineNum > 0)
    {
        for(int i = 0; i < MT9V03X_H; i++)
        {
            ips200_draw_point(IntClip(RightLine_x[i], 0, ips200_width_max - 1), IntClip(RightLine_y[i], 0, ips200_height_max - 1), RGB565_RED);
        }
        ips200_show_string(80, 16 * 9, "FoundLine");
    }
    else
    {
        ips200_show_string(80, 16  * 9, "Not Found");
    }
    ips200_show_string(  0, 16 * 10, "Angle_Error");
    ips200_show_float(  88, 16 * 10, CalculateAngleError(LeftLine), 3, 3);
    ips200_clear();       // 清屏，填充背景色
#endif
}
void Imu963_menu()
{
    ips200_show_string( 0, 16 * 0, "IMU_Angle:");
    ips200_show_string( 0, 16 * 1, "IMU_Data.gyro_z:");
    ips200_show_string( 0, 16 * 2, "MAX_IMU_Data_gyro_z:");
    ips200_show_string( 0, 16 * 3, "angle[0]:");
    ips200_show_string( 0, 16 * 4, "angle[1]:");
    ips200_show_string( 0, 16 * 5, "angle[2]:");
    ips200_show_string( 0, 16 * 6, "Kp_Ah:");
    ips200_show_string( 0, 16 * 7, "Ki_Ah:");
    ips200_show_string( 0, 16 * 8, "System_Time:");

    ips200_show_float( 80, 16 * 0, Z_360 > 180 ? Z_360 - 360 : Z_360, 3, 3);
    ips200_show_float(128, 16 * 1, IMU_Data.gyro_z, 3, 3);
    ips200_show_float(160, 16 * 2, MAX_IMU_Data_gyro_z, 3, 3);
    ips200_show_float( 72, 16 * 3, angle[0], 3, 3);
    ips200_show_float( 72, 16 * 4, angle[1], 3, 3);
    ips200_show_float( 72, 16 * 5, angle[2], 3, 3);
    ips200_show_float( 48, 16 * 6, Kp_Ah, 3, 3);
    ips200_show_float( 48, 16 * 7, Ki_Ah, 3, 3);
    ips200_show_float( 96, 16 * 8, System_Time, 3, 3);
    ips200_show_float( 96, 16 * 9, ((int16)System_Time / 1) * 0.0122, 3, 3);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = IMU_Data.gyro_z;
    oscilloscope_data.data[1] = Z_360 > 180 ? Z_360 - 360 : Z_360;
    oscilloscope_data.data[2] = imu963ra_mag_x;
    oscilloscope_data.data[3] = imu963ra_mag_y;
    oscilloscope_data.data[4] = imu963ra_mag_z;
    oscilloscope_data.data[5] = angle[0];
    oscilloscope_data.data[6] = angle[1];
    oscilloscope_data.data[7] = angle[2];
#endif
}

void Flash_menu()
{
    ips200_show_string( 80, 16 * 0, "Flash Menu");
    ips200_show_string(  0, 16 * 1, "KEY1:Obtain");
    ips200_show_string(  0, 16 * 2, "KEY2:Save");
    ips200_show_string(  0, 16 * 3, "KEY3:Delete");
    ips200_show_string(  0, 16 * 4, "KEY4:Check");

}

void Servo_menu()
{
    ips200_show_string( 80, 16 * 0, "ServoTest:");
    ips200_show_string(  0, 16 * 1, "KEY1:Servo Angle + 10");
    ips200_show_string(  0, 16 * 2, "KEY2:Servo Angle - 10");
    ips200_show_string(  0, 16 * 3, "KEY3:Servo Angle + 1");
    ips200_show_string(  0, 16 * 4, "KEY4:Servo Angle - 1");
    ips200_show_string(  0, 16 * 5, "Servo Angle:");
    ips200_show_uint(   96, 16 * 5, Servo_Angle, 3);
}

void LoopEnable_Param()
{
    ips200_show_string( 0, 16 * 0, "-->LoopEnablePar");
    ips200_show_string( 0, 16 * 1, "   LoopDisablePa");
    ips200_show_string( 0, 16 * 2, "   OtherParamSet");
}

void LoopDisable_Param()
{
    ips200_show_string( 0, 16 * 0, "   LoopEnablePar");
    ips200_show_string( 0, 16 * 1, "-->LoopDisablePa");
    ips200_show_string( 0, 16 * 2, "   OtherParamSet");
}

void Param_Set()
{
    ips200_show_string( 0, 16 * 0, "   LoopEnablePar");
    ips200_show_string( 0, 16 * 1, "   LoopDisablePa");
    ips200_show_string( 0, 16 * 2, "-->OtherParamSet");
}

void Task_Select_menu()
{
    ips200_show_string(  0, 16 * 0, "-->Task1To3");
    ips200_show_string(  0, 16 * 1, "   TaskFour");
}

void Task_Four_menu()
{
    ips200_show_string(  0, 16 * 0, "   Task1To3");
    ips200_show_string(  0, 16 * 1, "-->TaskFour");
}

/////////////////////////////////三层菜单-------------------------------------------------

void ServoP_menu(void)
{
    ips200_show_string(  0, 16 * 0, "-->ServoP:");
    ips200_show_string(  0, 16 * 1, "   ServoI:");
    ips200_show_string(  0, 16 * 2, "   ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "angle[2]:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:P+0.01");
    ips200_show_string(120, 16 * 8, "KEY2:P-0.01");
    ips200_show_string(  0, 16 * 9, "KEY3:SavePar");
    ips200_show_string(120, 16 * 9, "KEY4:Get Par");
    ips200_show_float ( 80, 16 * 0, Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1, Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2, Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 72, 16 * 4, angle[2], 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;
#endif
}

void ServoI_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   ServoP:");
    ips200_show_string(  0, 16 * 1, "-->ServoI:");
    ips200_show_string(  0, 16 * 2, "   ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "angle[2]:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:I+0.01");
    ips200_show_string(120, 16 * 8, "KEY2:I-0.01");
    ips200_show_string(  0, 16 * 9, "KEY3:SavePar");
    ips200_show_string(120, 16 * 9, "KEY4:Get Par");
    ips200_show_float ( 80, 16 * 0, Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1, Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2, Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 72, 16 * 4, angle[2], 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;
#endif
}

void ServoD_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   ServoP:");
    ips200_show_string(  0, 16 * 1, "   ServoI:");
    ips200_show_string(  0, 16 * 2, "-->ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "angle[2]:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:D+0.01");
    ips200_show_string(120, 16 * 8, "KEY2:D-0.01");
    ips200_show_string(  0, 16 * 9, "KEY3:SavePar");
    ips200_show_string(120, 16 * 9, "KEY4:Get Par");
    ips200_show_float ( 80, 16 * 0, Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1, Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2, Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 72, 16 * 4, angle[2], 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;
#endif
}

void MotorP_menu(void)
{
    ips200_show_string(  0, 16 *  0, "-->MotorP:");
    ips200_show_string(  0, 16 *  1, "   MotorI:");
    ips200_show_string(  0, 16 *  2, "   MotorD:");
    ips200_show_string(  0, 16 *  3, "Test_Encoder:");
    ips200_show_string(  0, 16 *  4, "Tagt_Encoder:");
    ips200_show_string(  0, 16 *  5, "ReCtrl_Speed:");
    ips200_show_string(  0, 16 *  6, "Encoder:");
    ips200_show_string(  0, 16 *  7, "PID.error:");
    ips200_show_string(  0, 16 *  8, "PID.output:");
    ips200_show_string(  0, 16 *  9, "KEY1:P+0.1");
    ips200_show_string(120, 16 *  9, "KEY2:P-0.1");
    ips200_show_string(  0, 16 * 10, "KEY3:Enco+100");
    ips200_show_string(120, 16 * 10, "KEY4:Enco-100");
    ips200_show_float ( 80, 16 *  0, Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 *  1, Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 *  2, Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_int   (104, 16 *  3, Test_Encoder, 5);
    ips200_show_int   (104, 16 *  4, Target_Encoder, 5);
    ips200_show_int   (104, 16 *  5, RemoteCtrl_Speed, 5);
    ips200_show_int   ( 64, 16 *  6, Encoder, 5);
    ips200_show_float ( 80, 16 *  7, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 *  8, PID_MOTOR.output, 5, 3);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;

    data_len = (uint8_t)wireless_uart_read_buffer(data_buffer, 32);
    if(data_len != 0)
    {
        int32 get_encoder = func_str_to_int((char *)data_buffer);
        if(get_encoder <= 1500)
        {
            Test_Encoder = (int16)get_encoder;
        }
        else
        {
            Test_Encoder = 0;
        }
        memset(data_buffer, 0, 32);
    }
#endif
}

void MotorI_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   MotorP:");
    ips200_show_string(  0, 16 * 1, "-->MotorI:");
    ips200_show_string(  0, 16 * 2, "   MotorD:");
    ips200_show_string(  0, 16 * 3, "Test_Encoder:");
    ips200_show_string(  0, 16 * 4, "Encoder:");
    ips200_show_string(  0, 16 * 5, "PID.error:");
    ips200_show_string(  0, 16 * 6, "PID.output:");
    ips200_show_string(  0, 16 * 7, "KEY1:I+0.1");
    ips200_show_string(120, 16 * 7, "KEY2:I-0.1");
    ips200_show_string(  0, 16 * 8, "KEY3:Enco+100");
    ips200_show_string(120, 16 * 8, "KEY4:Enco-100");
    ips200_show_float ( 80, 16 * 0, Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1, Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2, Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_int   (104, 16 * 3, Test_Encoder, 5);
    ips200_show_int   ( 64, 16 * 4, Encoder, 5);
    ips200_show_float ( 80, 16 * 5, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 * 6, PID_MOTOR.output, 5, 3);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;

    data_len = (uint8_t)wireless_uart_read_buffer(data_buffer, 32);
    if(data_len != 0)
    {
        int32 get_encoder = func_str_to_int((char *)data_buffer);
        if(get_encoder <= 1500)
        {
            Test_Encoder = (int16)get_encoder;
        }
        else
        {
            Test_Encoder = 0;
        }
        memset(data_buffer, 0, 32);
    }
#endif
}

void MotorD_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   MotorP:");
    ips200_show_string(  0, 16 * 1, "   MotorI:");
    ips200_show_string(  0, 16 * 2, "-->MotorD:");
    ips200_show_string(  0, 16 * 3, "Test_Encoder:");
    ips200_show_string(  0, 16 * 4, "Encoder:");
    ips200_show_string(  0, 16 * 5, "PID.error:");
    ips200_show_string(  0, 16 * 6, "PID.output:");
    ips200_show_string(  0, 16 * 7, "KEY1:D+0.1");
    ips200_show_string(120, 16 * 7, "KEY2:D-0.1");
    ips200_show_string(  0, 16 * 8, "KEY3:Enco+100");
    ips200_show_string(120, 16 * 8, "KEY4:Enco-100");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_int   (104, 16 * 3, Test_Encoder, 5);
    ips200_show_int   ( 64, 16 * 4, Encoder, 5);
    ips200_show_float ( 80, 16 * 5, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 * 6, PID_MOTOR.output, 5, 3);

#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;

    data_len = (uint8_t)wireless_uart_read_buffer(data_buffer, 32);
    if(data_len != 0)
    {
        int32 get_encoder = func_str_to_int((char *)data_buffer);
        if(get_encoder <= 1500)
        {
            Test_Encoder = (int16)get_encoder;
        }
        else
        {
            Test_Encoder = 0;
        }
        memset(data_buffer, 0, 32);
    }
#endif
}

void Task_Select(void)
{
    if(!gpio_get_level(SWITCH1))
    {
        ips200_show_string(  0, 16 *  0, "Track_Point:");
        ips200_show_uint(   96, 16 *  0, Track_Points_NUM, 3);
        ips200_show_string(  0, 16 *  1, "Distance:");
        ips200_show_float(  72, 16 *  1, Distance, 3, 3);
        ips200_show_string(  0, 16 *  2, "GPS_Angle:");
        ips200_show_float(  80, 16 *  2, Angle, 3, 3);
        ips200_show_string(  0, 16 *  3, "Yaw:");
        ips200_show_float(  32, 16 *  3, angle[2], 3, 3);
        ips200_show_string(  0, 16 *  4, "Angle_Error:");
        ips200_show_float(  96, 16 *  4, Angle_Error, 3, 6);
        ips200_show_string(  0, 16 *  5, "Delta_Angle:");
        ips200_show_float(  96, 16 *  5, Delta_Angle, 3, 6);
        ips200_show_string(  0, 16 *  6, "Delta_Lat:");
        ips200_show_float(  80, 16 *  6, Delta_Lat, 3, 6);
        ips200_show_string(  0, 16 *  7, "Delta_Lon:");
        ips200_show_float(  80, 16 *  7, Delta_Lon, 3, 6);
        ips200_show_float(   0, 16 *  8, gnss.latitude, 3, 6);
        ips200_show_float(  96, 16 *  8, gnss.longitude, 3, 6);
        ips200_show_string(  0, 16 *  9, "Servo Angle:");
        ips200_show_uint(   96, 16 *  9, Servo_Angle, 3);
        ips200_show_string(  0, 16 * 10, "TagtEncoder:");
        ips200_show_int(    96, 16 * 10, Target_Encoder, 4);
        ips200_show_string(  0, 16 * 11, "KEY1:Task1");
        ips200_show_string(120, 16 * 11, "KEY2:Task2");
        ips200_show_string(  0, 16 * 12, "KEY3:Task3");
        ips200_show_string(120, 16 * 12, "KEY4:Start");
    }
    if(gpio_get_level(SWITCH1))
    {
        drawGrid();
        drawPoints();
    }
    updateCarPosition();
#if WIRELESS_UART_ENABLE
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = Angle_Error;
    oscilloscope_data.data[1] = Distance;
    oscilloscope_data.data[2] = GpsDistance[Track_Points_NUM];
    oscilloscope_data.data[3] = Track_Points_NUM;
    oscilloscope_data.data[4] = Angle;
    oscilloscope_data.data[5] = angle[2];
#endif
}

void Task_Four(void)
{
    ips200_draw_line(  4,   4, 235,   4, RGB565_PURPLE);
    ips200_draw_line(  4,  40, 235,  40, RGB565_PURPLE);
    ips200_draw_line(  4,   4,   4,  40, RGB565_PURPLE);
    ips200_draw_line(235,   4, 235,  40, RGB565_PURPLE);

    ips200_draw_line(  4,  44, 235,  44, RGB565_PURPLE);
    ips200_draw_line(  4, 284, 235, 284, RGB565_PURPLE);
    ips200_draw_line(  4,  44,   4, 284, RGB565_PURPLE);
    ips200_draw_line(235,  44, 235, 284, RGB565_PURPLE);
    ips200_draw_line(  4, 267, 235, 267, RGB565_PURPLE);

    ips200_draw_line(  0, 318, 239, 318, RGB565_CYAN);
    ips200_draw_line(  0, 319, 239, 319, RGB565_CYAN);
    ips200_show_chinese( 56, 286, 32, Chinese04[0], 1, RGB565_CYAN);
    ips200_show_chinese( 88, 286, 32, Chinese05[0], 1, RGB565_CYAN);
    ips200_show_chinese(120, 286, 32, Chinese35[0], 1, RGB565_CYAN);
    ips200_show_chinese(152, 286, 32, Chinese36[0], 1, RGB565_CYAN);

    ips200_show_int   ( 24, 268, Action_Flag[0], 2);
    ips200_show_int   ( 64, 268, Action_Flag[1], 2);
    ips200_show_int   (104, 268, Action_Flag[2], 2);
    ips200_show_int   (144, 268, Action_Flag[3], 2);
    ips200_show_int   (184, 268, Action_Flag[4], 2);
    if(Task4_Start_Direc == 0)
    {
        ips200_show_string(224, 268, "N");
    }
    else if(Task4_Start_Direc == 90)
    {
        ips200_show_string(224, 268, "E");
    }
    else if(Task4_Start_Direc == 180)
    {
        ips200_show_string(224, 268, "S");
    }
    else if(Task4_Start_Direc == 270)
    {
        ips200_show_string(224, 268, "W");
    }
    else
    {
        ips200_show_string(224, 268, "?");
    }

    process_string(Dictation_Result);
    if(Start_Flag == 0)
    {
        Recognize_Command();
    }

    if(!audio_start_flag && !audio_server_link_flag)
    {
        // 按键获取语音
        ips200_show_chinese(  5,   5, 32, Chinese00[0], 1, RGB565_CYAN);
        ips200_show_chinese( 37,   5, 32, Chinese01[0], 1, RGB565_CYAN);
        ips200_show_chinese( 69,   5, 32, Chinese02[0], 1, RGB565_CYAN);
        ips200_show_chinese(101,   5, 32, Chinese03[0], 1, RGB565_CYAN);
        ips200_show_chinese(133,   5, 32, Chinese04[0], 1, RGB565_CYAN);
        ips200_show_chinese(165,   5, 32, Chinese05[0], 1, RGB565_CYAN);
        ips200_show_chinese(197,   5, 32, Chinese57[0], 1, RGB565_CYAN);
    }
    if(audio_start_flag && !audio_server_link_flag)
    {
        // 正在连接服务器
        ips200_show_chinese(  5,   5, 32, Chinese58[0], 1, RGB565_CYAN);
        ips200_show_chinese( 37,   5, 32, Chinese59[0], 1, RGB565_CYAN);
        ips200_show_chinese( 69,   5, 32, Chinese60[0], 1, RGB565_CYAN);
        ips200_show_chinese(101,   5, 32, Chinese61[0], 1, RGB565_CYAN);
        ips200_show_chinese(133,   5, 32, Chinese62[0], 1, RGB565_CYAN);
        ips200_show_chinese(165,   5, 32, Chinese63[0], 1, RGB565_CYAN);
        ips200_show_chinese(197,   5, 32, Chinese64[0], 1, RGB565_CYAN);
    }
    if(audio_start_flag && audio_server_link_flag)
    {
        // 录音识别中
        ips200_show_chinese(  5,   5, 32, Chinese37[0], 1, RGB565_CYAN);
        ips200_show_chinese( 37,   5, 32, Chinese05[0], 1, RGB565_CYAN);
        ips200_show_chinese( 69,   5, 32, Chinese35[0], 1, RGB565_CYAN);
        ips200_show_chinese(101,   5, 32, Chinese36[0], 1, RGB565_CYAN);
        ips200_show_chinese(133,   5, 32, Chinese38[0], 1, RGB565_CYAN);
        ips200_show_chinese(165,   5, 32, Chinese57[0], 1, RGB565_CYAN);
        ips200_show_chinese(197,   5, 32, Chinese57[0], 1, RGB565_CYAN);
    }
    if(!audio_start_flag && audio_server_link_flag)
    {
        // 命令解析并执行
        ips200_show_chinese(  5,   5, 32, Chinese41[0], 1, RGB565_CYAN);
        ips200_show_chinese( 37,   5, 32, Chinese42[0], 1, RGB565_CYAN);
        ips200_show_chinese( 69,   5, 32, Chinese43[0], 1, RGB565_CYAN);
        ips200_show_chinese(101,   5, 32, Chinese44[0], 1, RGB565_CYAN);
        ips200_show_chinese(133,   5, 32, Chinese45[0], 1, RGB565_CYAN);
        ips200_show_chinese(165,   5, 32, Chinese46[0], 1, RGB565_CYAN);
        ips200_show_chinese(197,   5, 32, Chinese14[0], 1, RGB565_CYAN);
    }
}

void LoopEnable_menu()
{
    int8 Page = Point2 / Page_Point_Num;
    int8 RightArrow = Point2 % Page_Point_Num;
    ips200_show_string(0, 16 * RightArrow, "-->");
    if(Page == 0)
    {
        ips200_show_string( 24, 16 * 0, "0100EncoderKp:");
        ips200_show_string( 24, 16 * 1, "0100EncoderKd:");
        ips200_show_string( 24, 16 * 2, "0200EncoderKp:");
        ips200_show_string( 24, 16 * 3, "0200EncoderKd:");
        ips200_show_string( 24, 16 * 4, "0300EncoderKp:");
        ips200_show_string( 24, 16 * 5, "0300EncoderKd:");
        ips200_show_string( 24, 16 * 6, "0400EncoderKp:");
        ips200_show_string( 24, 16 * 7, "0400EncoderKd:");
        ips200_show_float( 136, 16 * 0, Encoder0100_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 1, Encoder0100_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 2, Encoder0200_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 3, Encoder0200_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 4, Encoder0300_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 5, Encoder0300_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 6, Encoder0400_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 7, Encoder0400_ServoPD.Kd, 1, 2);
    }
    if(Page == 1)
    {
        ips200_show_string( 24, 16 * 0, "0500EncoderKp:");
        ips200_show_string( 24, 16 * 1, "0500EncoderKd:");
        ips200_show_string( 24, 16 * 2, "0600EncoderKp:");
        ips200_show_string( 24, 16 * 3, "0600EncoderKd:");
        ips200_show_string( 24, 16 * 4, "0700EncoderKp:");
        ips200_show_string( 24, 16 * 5, "0700EncoderKd:");
        ips200_show_string( 24, 16 * 6, "0800EncoderKp:");
        ips200_show_string( 24, 16 * 7, "0800EncoderKd:");
        ips200_show_float( 136, 16 * 0, Encoder0500_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 1, Encoder0500_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 2, Encoder0600_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 3, Encoder0600_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 4, Encoder0700_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 5, Encoder0700_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 6, Encoder0800_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 7, Encoder0800_ServoPD.Kd, 1, 2);
    }
    if(Page == 2)
    {
        ips200_show_string( 24, 16 * 0, "0900EncoderKp:");
        ips200_show_string( 24, 16 * 1, "0900EncoderKd:");
        ips200_show_string( 24, 16 * 2, "1000EncoderKp:");
        ips200_show_string( 24, 16 * 3, "1000EncoderKd:");
        ips200_show_string( 24, 16 * 4, "1100EncoderKp:");
        ips200_show_string( 24, 16 * 5, "1100EncoderKd:");
        ips200_show_string( 24, 16 * 6, "1200EncoderKp:");
        ips200_show_string( 24, 16 * 7, "1200EncoderKd:");
        ips200_show_float( 136, 16 * 0, Encoder0900_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 1, Encoder0900_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 2, Encoder1000_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 3, Encoder1000_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 4, Encoder1100_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 5, Encoder1100_ServoPD.Kd, 1, 2);
        ips200_show_float( 136, 16 * 6, Encoder1200_ServoPD.Kp, 1, 2);
        ips200_show_float( 136, 16 * 7, Encoder1200_ServoPD.Kd, 1, 2);
    }
    ips200_show_string(  0, 16 *  9, "KEY1:Up");
    ips200_show_string(  0, 16 * 10, "KEY2:Down");
    ips200_show_string(120, 16 *  9, "KEY3:+0.01");
    ips200_show_string(120, 16 * 10, "KEY4:-0.01");
}

void LoopDisable_menu()
{
    int8 Page = Point3 / Page_Point_Num;
    int8 RightArrow = Point3 % Page_Point_Num;
    ips200_show_string(0, 16 * RightArrow, "-->");
    if(Page == 0)
    {
        ips200_show_string( 24, 16 * 0, "2000DutyServoKp:");
        ips200_show_string( 24, 16 * 1, "2000DutyServoKd:");
        ips200_show_string( 24, 16 * 2, "3000DutyServoKp:");
        ips200_show_string( 24, 16 * 3, "3000DutyServoKd:");
        ips200_show_string( 24, 16 * 4, "4500DutyServoKp:");
        ips200_show_string( 24, 16 * 5, "4500DutyServoKd:");
        ips200_show_string( 24, 16 * 6, "5500DutyServoKp:");
        ips200_show_string( 24, 16 * 7, "5500DutyServoKd:");
        ips200_show_float( 152, 16 * 0, From_0000_To_2000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 1, From_0000_To_2000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 2, From_2000_To_4000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 3, From_2000_To_4000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 4, From_4000_To_5000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 5, From_4000_To_5000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 6, From_5000_To_6000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 7, From_5000_To_6000_ServoPD.Kd, 1, 2);
    }
    if(Page == 1)
    {
        ips200_show_string( 24, 16 * 0, "6500DutyServoKp:");
        ips200_show_string( 24, 16 * 1, "6500DutyServoKd:");
        ips200_show_string( 24, 16 * 2, "7500DutyServoKp:");
        ips200_show_string( 24, 16 * 3, "7500DutyServoKd:");
        ips200_show_string( 24, 16 * 4, "8500DutyServoKp:");
        ips200_show_string( 24, 16 * 5, "8500DutyServoKd:");
        ips200_show_string( 24, 16 * 6, "9500DutyServoKp:");
        ips200_show_string( 24, 16 * 7, "9500DutyServoKd:");
        ips200_show_float( 152, 16 * 0, From_6000_To_7000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 1, From_6000_To_7000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 2, From_7000_To_8000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 3, From_7000_To_8000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 4, From_8000_To_9000_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 5, From_8000_To_9000_ServoPD.Kd, 1, 2);
        ips200_show_float( 152, 16 * 6, From_9000_To_9900_ServoPD.Kp, 1, 2);
        ips200_show_float( 152, 16 * 7, From_9000_To_9900_ServoPD.Kd, 1, 2);
    }
    ips200_show_string(  0, 16 *  9, "KEY1:Up");
    ips200_show_string(  0, 16 * 10, "KEY2:Down");
    ips200_show_string(120, 16 *  9, "KEY3:+0.01");
    ips200_show_string(120, 16 * 10, "KEY4:-0.01");
}

void Param_Set_menu()
{
    int8 Page = Point4 / Page_Point_Num;
    int8 RightArrow = Point4 % Page_Point_Num;
    ips200_show_string(0, 16 * RightArrow, "-->");
    if(Page == 0)
    {
        ips200_show_string( 24, 16 * 0, "Fly_Slope_Alpha:");
        ips200_show_string( 24, 16 * 1, "K_Straight     :");
        ips200_show_string( 24, 16 * 2, "Task2_Scales:");
        ips200_show_string( 24, 16 * 3, "Advan_Scales:");
        ips200_show_string( 24, 16 * 4, "Turn_Point:");
        ips200_show_string( 24, 16 * 5, "Snack_Advance:");
        ips200_show_string( 24, 16 * 6, "Snack_Back:");
        ips200_show_string( 24, 16 * 7, "Bucket_Dista:");
        ips200_show_int(   152, 16 * 0, Fly_Slope_Alpha, 4);
        ips200_show_float( 152, 16 * 1, K_Straight, 1, 3);
        ips200_show_int(   128, 16 * 2, Task2_Scales, 3);
        ips200_show_int(   128, 16 * 3, Advan_Scales, 3);
        ips200_show_int(   120, 16 * 4, Turn_Point  , 3);
        ips200_show_float( 136, 16 * 5, Snack_Advance, 2, 1);
        ips200_show_float( 112, 16 * 6, Snack_Back ,2, 1);
        ips200_show_float( 128, 16 * 7, Bucket_Dista, 3, 3);
    }
    if(Page == 1)
    {
        ips200_show_string(  24, 16 * 0, "Start_To_Bucket:");
        ips200_show_string(  24, 16 * 1, "Task3_Width:");
        ips200_show_float(  152, 16 * 0, Start_To_Bucket, 1, 3);
        ips200_show_float(  120, 16 * 1, Task3_Width, 1, 3);
    }
    ips200_show_string(  0, 16 *  9, "KEY1:Up");
    ips200_show_string(  0, 16 * 10, "KEY2:Down");
    ips200_show_string(120, 16 *  9, "KEY3:K+");
    ips200_show_string(120, 16 * 10, "KEY4:K-");
}

void Key_Ctrl_Menu()
{
    if(!gpio_get_level(SWITCH2))           // 当拨码开关在上表示按键切换菜单
    {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
          {
              key_value = 1;
          }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
          {
              key_value = 2;
          }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
          {
              key_value = 3;
          }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
          {
              key_value = 4;
          }
    }
    // flash存储GPS点位数据
    if(gpio_get_level(SWITCH2))           // 拨码开关在下表示执行存储数据
    {
        // GPS采点
        if(func_index == enum_secon_menu00)
        {
            // KEY1或者通道3按下都可以记录点位
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS || Channal_3_Press_Flag)
            {
                if(gnss.state == 1)
                {
                    lat_union[Point_NUM].double_type = gnss.latitude; // 偶数储存纬度latitude
                    lon_union[Point_NUM].double_type = gnss.longitude;// 奇数储存经度longitude
                    Point[Point_NUM].latitude = gnss.latitude;
                    Point[Point_NUM].lonitude = gnss.longitude;
                    Point_NUM++;
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_GPS();
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)           // KEY2和KEY3配合以实现对某个目标点的重采集
            {

                if(Point_NUM >= 0)
                {
                    Point_NUM += 1;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Point_NUM > 0)
                {
                    Point_NUM -= 1;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                Point_NUM = Task1_Start_Point;
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                Point_NUM = Task2_Start_Point;
            }
            if(key_get_state(KEY_3) == KEY_LONG_PRESS)
            {
                Point_NUM = Task3_Start_Point;
            }
        }

        // ServoP调节
        if(func_index == enum_third_menu00)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[0] += 0.01;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[0] -= 0.01;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                FLASH_GET_PAR();
            }
        }

        // ServoI调节
        if(func_index == enum_third_menu01)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[1] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[1] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                FLASH_GET_PAR();
            }
        }

        // ServoD调节
        if(func_index == enum_third_menu02)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[2] += 0.01;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[2] -= 0.01;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                FLASH_GET_PAR();
            }
        }

        // MotorP调节
        if(func_index == enum_third_menu03)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[0] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[0] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder < 1000)
                {
                    Test_Encoder += 100;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -1000)
                {
                    Test_Encoder -= 100;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        // MotorI调节
        if(func_index == enum_third_menu04)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[1] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[1] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder < 1000)
                {
                    Test_Encoder += 100;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -1000)
                {
                    Test_Encoder -= 100;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        //MotorD调节
        if(func_index == enum_third_menu05)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[2] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.SpeedPID[2] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder < 1000)
                {
                    Test_Encoder += 100;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -1000)
                {
                    Test_Encoder -= 100;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        if(func_index == enum_secon_menu03)
        {
        }

        // 调试速度
        if(func_index == enum_secon_menu04)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Point1 > 0)
                {
                    Point1 = Point1 - 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Point1 < NUM_GPS_DATA - 1)
                {
                    Point1 = Point1 + 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(GpsTgtEncod[Point1] < PWM_DUTY_MAX)
                {
                    GpsTgtEncod[Point1] += 100;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(GpsTgtEncod[Point1] > 0)
                {
                    GpsTgtEncod[Point1] -= 100;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                Point1 = Task1_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                Point1 = Task2_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_3) == KEY_LONG_PRESS)
            {
                Point1 = Task3_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_4) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        // 换点距离
        if(func_index == enum_secon_menu05)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Point1 > 0)
                {
                    Point1 = Point1 - 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Point1 < NUM_GPS_DATA - 1)
                {
                    Point1 = Point1 + 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                GpsDistance[Point1] += 0.10;
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(GpsDistance[Point1] > 0.10)
                {
                    GpsDistance[Point1] -= 0.10;
                }
                if(GpsDistance[Point1] < 0.10)
                {
                    GpsDistance[Point1] = 0;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                Point1 = Task1_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                Point1 = Task2_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_3) == KEY_LONG_PRESS)
            {
                Point1 = Task3_Start_Point;
                ips200_clear();
            }
            if(key_get_state(KEY_4) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        // 任务点位设置
        if(func_index == enum_secon_menu06)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Task_Point_Set > 1)
                {
                    Task_Point_Set -= 1;
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Task_Point_Set < 3)
                {
                    Task_Point_Set += 1;
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Task_Point_Set == 1)
                {
                    Task1_Points += 1;
                }
                if(Task_Point_Set == 2)
                {
                    Task2_Bucket += 1;
                }
                if(Task_Point_Set == 3)
                {
                    Task3_Points += 1;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Task_Point_Set == 1)
                {
                    Task1_Points -= 1;
                }
                if(Task_Point_Set == 2)
                {
                    Task2_Bucket -= 1;
                }
                if(Task_Point_Set == 3)
                {
                    Task3_Points -= 1;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        //点位查看调节
        if(func_index == enum_secon_menu08)
        {
            // 拨码开关在上表示点位切换
            if(!gpio_get_level(SWITCH1))
            {
                if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
                {
                    Map_Flag = -Map_Flag;
                    ips200_clear();
                }
                if(Map_Flag == 1)
                {
                    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
                    {
                        if(Point0 > 0)
                        {
                            Point0 = Point0 - 1;
                            ips200_clear();
                        }
                    }
                    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
                    {
                        if(Point0 < NUM_GPS_DATA - 1)
                        {
                            Point0 = Point0 + 1;
                            ips200_clear();
                        }
                    }
                    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
                    {
                        FLASH_FIX_GPS();
                    }
                }
                if(Map_Flag == -1)
                {
                    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
                    {
                        Track_Points_NUM = Task1_Start_Point;
                        Task_Flag = 1;
                        Turn_Angle = get_two_points_azimuth(Point[Task1_Start_Point].latitude, Point[Task1_Start_Point].lonitude, Point[Task1_Start_Point + 2].latitude, Point[Task1_Start_Point + 2].lonitude);
                        Task1_Road_Fix();
                        initCoordinateSystem();
                        LED_Buzzer_Flag_Ctrl(LED1);
                        ips200_clear();
                    }
                    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
                    {
                        Track_Points_NUM = Task2_Start_Point;
                        Task_Flag = 2;
                        Turn_Angle = get_two_points_azimuth(Point[Task2_Start_Point + Task2_Bucket + 2].latitude, Point[Task2_Start_Point + Task2_Bucket + 2].lonitude, Point[Task2_Start_Point + Task2_Bucket + 3].latitude, Point[Task2_Start_Point + Task2_Bucket + 3].lonitude);
                        Task2_Road_Gen();
                        initCoordinateSystem();
                        LED_Buzzer_Flag_Ctrl(LED1);
                        ips200_clear();
                    }
                    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
                    {
                        Track_Points_NUM = Task3_Start_Point;
                        Task_Flag = 3;
                        Turn_Angle = get_two_points_azimuth(Point[Task3_Start_Point].latitude, Point[Task3_Start_Point].lonitude, Point[Turn_Point + 1].latitude, Point[Turn_Point + 1].lonitude);
                        Task3_Road_Fix();
                        initCoordinateSystem();
                        LED_Buzzer_Flag_Ctrl(LED1);
                        ips200_clear();
                    }
                }
            }
            // 拨码开关在下表示点位设置
            if(gpio_get_level(SWITCH1))
            {
                if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
                {
                    Point[Point0].latitude += 0.000001;
                }
                if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
                {
                    Point[Point0].latitude -= 0.000001;
                }
                if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
                {
                    Point[Point0].lonitude += 0.000001;
                }
                if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
                {
                    Point[Point0].lonitude -= 0.000001;
                }
            }

        }

        if(func_index == enum_secon_menu09)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                clip_value += 1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                clip_value -= 1;
            }
        }

        if(func_index == enum_secon_menu10)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Kp_Ah += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Kp_Ah -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Ki_Ah += 0.01;
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Ki_Ah -= 0.01;
            }
        }

        if(func_index == enum_secon_menu11)
        {
            // 按下KEY1从Flash中获取经纬度数据
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                FLASH_GET_GPS();
            }
            // 按下KEY2保存经纬度数据到Flash中
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_GPS();
            }
            // 按下KEY3删除Flash中的经纬度数据
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_DEL_GPS();
            }
            // 按下KEY4查看经纬度信息
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                ips200_clear();
                func_index = 20;
                table[func_index].current_operation();
            }
        }

        if(func_index == enum_secon_menu12)
        {
            // 要测试时请注释掉PDLocServoCtrl()中的Servo_Set
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Servo_Angle += 10;
                Servo_Set(Servo_Angle);     // 舵机角度控制
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Servo_Angle -= 10;
                Servo_Set(Servo_Angle);     // 舵机角度控制
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Servo_Angle += 1;
                Servo_Set(Servo_Angle);     // 舵机角度控制
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Servo_Angle -= 1;
                Servo_Set(Servo_Angle);     // 舵机角度控制
            }
        }

        if(func_index == enum_third_menu06)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Point2 > 0)
                {
                    Point2 = Point2 - 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Point2 < 23)
                {
                    Point2 = Point2 + 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Point2 == 0)
                {
                    Encoder0100_ServoPD.Kp += 0.01;
                }
                if(Point2 == 1)
                {
                    Encoder0100_ServoPD.Kd += 0.01;
                }
                if(Point2 == 2)
                {
                    Encoder0200_ServoPD.Kp += 0.01;
                }
                if(Point2 == 3)
                {
                    Encoder0200_ServoPD.Kd += 0.01;
                }
                if(Point2 == 4)
                {
                    Encoder0300_ServoPD.Kp += 0.01;
                }
                if(Point2 == 5)
                {
                    Encoder0300_ServoPD.Kd += 0.01;
                }
                if(Point2 == 6)
                {
                    Encoder0400_ServoPD.Kp += 0.01;
                }
                if(Point2 == 7)
                {
                    Encoder0400_ServoPD.Kd += 0.01;
                }
                if(Point2 == 8)
                {
                    Encoder0500_ServoPD.Kp += 0.01;
                }
                if(Point2 == 9)
                {
                    Encoder0500_ServoPD.Kd += 0.01;
                }
                if(Point2 == 10)
                {
                    Encoder0600_ServoPD.Kp += 0.01;
                }
                if(Point2 == 11)
                {
                    Encoder0600_ServoPD.Kd += 0.01;
                }
                if(Point2 == 12)
                {
                    Encoder0700_ServoPD.Kp += 0.01;
                }
                if(Point2 == 13)
                {
                    Encoder0700_ServoPD.Kd += 0.01;
                }
                if(Point2 == 14)
                {
                    Encoder0800_ServoPD.Kp += 0.01;
                }
                if(Point2 == 15)
                {
                    Encoder0800_ServoPD.Kd += 0.01;
                }
                if(Point2 == 16)
                {
                    Encoder0900_ServoPD.Kp += 0.01;
                }
                if(Point2 == 17)
                {
                    Encoder0900_ServoPD.Kd += 0.01;
                }
                if(Point2 == 18)
                {
                    Encoder1000_ServoPD.Kp += 0.01;
                }
                if(Point2 == 19)
                {
                    Encoder1000_ServoPD.Kd += 0.01;
                }
                if(Point2 == 20)
                {
                    Encoder1100_ServoPD.Kp += 0.01;
                }
                if(Point2 == 21)
                {
                    Encoder1100_ServoPD.Kd += 0.01;
                }
                if(Point2 == 22)
                {
                    Encoder1200_ServoPD.Kp += 0.01;
                }
                if(Point2 == 23)
                {
                    Encoder1200_ServoPD.Kd += 0.01;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Point2 == 0)
                {
                    Encoder0100_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 1)
                {
                    Encoder0100_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 2)
                {
                    Encoder0200_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 3)
                {
                    Encoder0200_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 4)
                {
                    Encoder0300_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 5)
                {
                    Encoder0300_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 6)
                {
                    Encoder0400_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 7)
                {
                    Encoder0400_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 8)
                {
                    Encoder0500_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 9)
                {
                    Encoder0500_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 10)
                {
                    Encoder0600_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 11)
                {
                    Encoder0600_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 12)
                {
                    Encoder0700_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 13)
                {
                    Encoder0700_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 14)
                {
                    Encoder0800_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 15)
                {
                    Encoder0800_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 16)
                {
                    Encoder0900_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 17)
                {
                    Encoder0900_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 18)
                {
                    Encoder1000_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 19)
                {
                    Encoder1000_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 20)
                {
                    Encoder1100_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 21)
                {
                    Encoder1100_ServoPD.Kd -= 0.01;
                }
                if(Point2 == 22)
                {
                    Encoder1200_ServoPD.Kp -= 0.01;
                }
                if(Point2 == 23)
                {
                    Encoder1200_ServoPD.Kd -= 0.01;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                FLASH_PRI_PAR();
            }
        }

        if(func_index == enum_third_menu07)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Point3 > 0)
                {
                    Point3 = Point3 - 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Point3 < 15)
                {
                    Point3 = Point3 + 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Point3 == 0)
                {
                    From_0000_To_2000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 1)
                {
                    From_0000_To_2000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 2)
                {
                    From_2000_To_4000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 3)
                {
                    From_2000_To_4000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 4)
                {
                    From_4000_To_5000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 5)
                {
                    From_4000_To_5000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 6)
                {
                    From_5000_To_6000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 7)
                {
                    From_5000_To_6000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 8)
                {
                    From_6000_To_7000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 9)
                {
                    From_6000_To_7000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 10)
                {
                    From_7000_To_8000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 11)
                {
                    From_7000_To_8000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 12)
                {
                    From_8000_To_9000_ServoPD.Kp += 0.01;
                }
                if(Point3 == 13)
                {
                    From_8000_To_9000_ServoPD.Kd += 0.01;
                }
                if(Point3 == 14)
                {
                    From_9000_To_9900_ServoPD.Kp += 0.01;
                }
                if(Point3 == 15)
                {
                    From_9000_To_9900_ServoPD.Kd += 0.01;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Point3 == 0)
                {
                    From_0000_To_2000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 1)
                {
                    From_0000_To_2000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 2)
                {
                    From_2000_To_4000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 3)
                {
                    From_2000_To_4000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 4)
                {
                    From_4000_To_5000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 5)
                {
                    From_4000_To_5000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 6)
                {
                    From_5000_To_6000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 7)
                {
                    From_5000_To_6000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 8)
                {
                    From_6000_To_7000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 9)
                {
                    From_6000_To_7000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 10)
                {
                    From_7000_To_8000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 11)
                {
                    From_7000_To_8000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 12)
                {
                    From_8000_To_9000_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 13)
                {
                    From_8000_To_9000_ServoPD.Kd -= 0.01;
                }
                if(Point3 == 14)
                {
                    From_9000_To_9900_ServoPD.Kp -= 0.01;
                }
                if(Point3 == 15)
                {
                    From_9000_To_9900_ServoPD.Kd -= 0.01;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                FLASH_PRI_PAR();
            }
        }

        if(func_index == enum_third_menu08)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(Point4 > 0)
                {
                    Point4 = Point4 - 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(Point4 < 9)
                {
                    Point4 = Point4 + 1;
                    ips200_clear();
                }
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                if(Point4 == 0)
                {
                    Fly_Slope_Alpha += 100;
                }
                if(Point4 == 1)
                {
                    K_Straight += 0.1;
                }
                if(Point4 == 2)
                {
                    Task2_Scales += 1;
                }
                if(Point4 == 3)
                {
                    Advan_Scales += 1;
                }
                if(Point4 == 4)
                {
                    Turn_Point += 1;
                }
                if(Point4 == 5)
                {
                    Snack_Advance += 0.5f;
                }
                if(Point4 == 6)
                {
                    Snack_Back += 0.5f;
                }
                if(Point4 == 7)
                {
                    Bucket_Dista += 0.05f;
                }
                if(Point4 == 8)
                {
                    Start_To_Bucket += 0.05f;
                }
                if(Point4 == 9)
                {
                    Task3_Width += 0.05f;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Point4 == 0)
                {
                    Fly_Slope_Alpha -= 100;
                }
                if(Point4 == 1)
                {
                    K_Straight -= 0.1;
                }
                if(Point4 == 2)
                {
                    Task2_Scales -= 1;
                }
                if(Point4 == 3)
                {
                    Advan_Scales -= 1;
                }
                if(Point4 == 4)
                {
                    Turn_Point -= 1;
                }
                if(Point4 == 5)
                {
                    Snack_Advance -= 0.5f;
                }
                if(Point4 == 6)
                {
                    Snack_Back -= 0.5f;
                }
                if(Point4 == 7)
                {
                    Bucket_Dista -= 0.05f;
                }
                if(Point4 == 8)
                {
                    Start_To_Bucket -= 0.05f;
                }
                if(Point4 == 9)
                {
                    Task3_Width -= 0.05f;
                }
            }
            if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                FLASH_SAV_PAR();
            }
            if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                FLASH_PRI_PAR();
            }
            if(key_get_state(KEY_3) == KEY_LONG_PRESS)
            {
                if(Point4 == 8)
                {
                    Start_To_Bucket = -Start_To_Bucket;
                }
                if(Point4 == 9)
                {
                    Task3_Width = -Task3_Width;
                }
                LED_Buzzer_Flag_Ctrl(LED1);
                system_delay_ms(500);
            }
        }
        
        if(func_index == enum_third_menu09)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = Task1_Start_Point;
                Task_Flag = 1;
                Turn_Angle = get_two_points_azimuth(Point[Task1_Start_Point].latitude, Point[Task1_Start_Point].lonitude, Point[Task1_Start_Point + 2].latitude, Point[Task1_Start_Point + 2].lonitude);
                initCoordinateSystem();
                ips200_clear();
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = Task2_Start_Point;
                Task_Flag = 2;
                Turn_Angle = get_two_points_azimuth(Point[Task2_Start_Point + Task2_Bucket + 2].latitude, Point[Task2_Start_Point + Task2_Bucket + 2].lonitude, Point[Task2_Start_Point + Task2_Bucket + 3].latitude, Point[Task2_Start_Point + Task2_Bucket + 3].lonitude);
                initCoordinateSystem();
                ips200_clear();
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = Task3_Start_Point;
                Task_Flag = 3;
                Turn_Angle = get_two_points_azimuth(Point[Task3_Start_Point].latitude, Point[Task3_Start_Point].lonitude, Point[Turn_Point + 1].latitude, Point[Turn_Point + 1].lonitude);
                initCoordinateSystem();
                ips200_clear();
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Start_Flag = 1;
                LED_Buzzer_Flag_Ctrl(LED3);
            }
        }

        if(func_index == enum_third_menu10)
        {
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                // 初始化识别结果
                Dictation_Result[0] = '\0';
                memset(Action_Flag, 0, ACTION_COUNT);
                ips200_clear();
                LED_Buzzer_Flag_Ctrl(LED4);
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = Task4_Start_Point;
                if(Task_Flag == 4)
                {
                    Task4_Start_Direc = LimitFabs360(Task4_Start_Direc + 90);
                    FLASH_SAV_PAR();
                }
                Task_Flag = 4;
            }
            if(key_get_state(KEY_3) == KEY_LONG_PRESS)
            {
                Recognize_Command();
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Start_Flag = 1;
                LED_Buzzer_Flag_Ctrl(LED3);
            }
        }
    }
}













// GUI菜单切换
void Menu_Control()
{
        switch(key_value)
        {
            case 1 : ips200_clear(); func_index = table[func_index].up;    key_value = 0; break;
            case 2 : ips200_clear(); func_index = table[func_index].down;  key_value = 0; break;
            case 3 : ips200_clear(); func_index = table[func_index].back;  key_value = 0; break;
            case 4 : ips200_clear(); func_index = table[func_index].enter; key_value = 0; break;
        }
        table[func_index].current_operation();
}





