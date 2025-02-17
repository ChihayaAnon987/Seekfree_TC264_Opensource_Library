/*
 * GUI.c
 *
 *  Created on: 2025年1月10日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

Parameter_set Parameter_set0=
{
    {2.0, 0.0, 1.0},            // 舵机PID
    {1.0, 0.0, 0.0},            // 速度PID
    3000,                       // 调试的速度
    2.5,                        // 换点距离
    SERVO_MOTOR_MID             // 舵机机械可调中值
};

int    func_index   = 0;
int    key_value;
int    Point        = 0;    // 菜单点位
double Test_Angle   = 0;    // 调试用
int16  Test_Encoder = 0;    // 调试用
uint8  Start_Flag   = 0;    // 发车标志
seekfree_assistant_oscilloscope_struct oscilloscope_data;


menu_table table[31]=
{
    // current, up, down, back, enter

    // 菜单0
    { 0, 29,  2,  0,  1, main_menu0},       // 踩点GPS一层
    { 1,  1,  1,  0,  1, CaiDian_menu},     // 踩点GPS二层

    // 菜单1
    { 2,  0, 11,  2,  3, main_menu1},       // PID一层
    { 3,  4,  4,  2,  5, ServoPID},         // ServoPID二层
    { 4,  3,  3,  2,  8, MotorPID},         // MotorPID二层
    { 5,  7,  6,  3,  5, ServoP_menu},      // ServoP
    { 6,  5,  7,  3,  6, ServoI_menu},      // ServoI
    { 7,  6,  5,  3,  7, ServoD_menu},      // ServoD
    { 8,  9, 10,  4,  8, MotorP_menu},      // MotorP
    { 9, 10,  8,  4,  9, MotorD_menu},      // MotorI
    {10,  8,  9,  4, 10, MotorI_menu},      // MotorD

    // 菜单2
    {11,  2, 13, 11, 12, main_menu2},        // 运行GPS显示一层
    {12, 12, 12, 11, 12, GPS_menu},          // 运行GPS显示二层

    // 菜单3
    {13, 11, 17, 13, 14, main_menu3},        // 调速,舵机机械中值，换点距离
    {14, 16, 15, 13, 14, spd_menu},          // 调速
    {15, 14, 16, 13, 15, Distance_menu},     // 换点距离
    {16, 15, 14, 13, 16, serve_mid_menu},    // 舵机中值

    // 菜单4
    {17, 13, 19, 17, 18, main_menu4},        // 遥控模式一层
    {18, 18, 18, 17, 18, RemoteCtrl_menu},   // 遥控模式二层

    // 菜单5
    {19, 17, 21, 19, 20, main_menu5},        // GPS点位查看一层
    {20, 20, 20, 19, 20, Points_menu},       // GPS点位查看二层

    // 菜单6
    {21, 19, 23, 21, 22, main_menu6},        // 摄像头图像一层
    {22, 22, 22, 21, 22, ZongZuanF} ,        // 摄像头图像二层

    // 菜单7
    {23, 21, 25, 23, 24, main_menu7},        // 陀螺仪一层
    {24, 24, 24, 23, 24, Imu963_menu},       // 陀螺仪二层

    // 菜单8
    {25, 23, 27, 25, 26, main_menu8},        // Flash一层
    {26, 26, 26, 25, 26, Flash_menu},        // Flash二层

    // 菜单9
    {27, 25, 29, 27, 28, main_menu9},        // 舵机测试一层
    {28, 28, 28, 27, 28, Servo_menu},        // 舵机测试二层

    // 菜单10
    {29, 27,  0, 29, 30, main_menu10},       // 任务选择一层
    {30, 30, 30, 29, 30, Task_Select}        // 任务选择二层

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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "-->Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "-->SevroTest ");
    ips200_show_string(110, 16 * 2, "   TaskSelect");
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
    ips200_show_string(110, 16 * 0, "   Flash     ");
    ips200_show_string(110, 16 * 1, "   SevroTest ");
    ips200_show_string(110, 16 * 2, "-->TaskSelect");
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

        seekfree_assistant_oscilloscope_send(&oscilloscope_data);
        oscilloscope_data.data[0] = Distance;
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
    ips200_show_string( 32, 16 * 0, "/");
    ips200_show_uint(   40, 16 * 0, gnss.time.month , 2);
    ips200_show_string( 56, 16 * 0, "/");
    ips200_show_uint(   64, 16 * 0, gnss.time.day   , 2);
    ips200_show_uint(  176, 16 * 0, gnss.time.hour  , 2);
    ips200_show_string(192, 16 * 0, ":");
    ips200_show_uint(  200, 16 * 0, gnss.time.minute, 2);
    ips200_show_string(216, 16 * 0, ":");
    ips200_show_uint(  224, 16 * 0, gnss.time.second, 2);

    ips200_show_string(  0, 16 * 1, "Now_Lat:");
    ips200_show_string(  0, 16 * 2, "Now_Lon:");
    ips200_show_string(168, 16 * 1, "state:");
    ips200_show_string(168, 16 * 2, "K_G:");
    ips200_show_string(  0, 16 * 3, "Filter_Lat:");
    ips200_show_string(  0, 16 * 4, "Filter_Lon:");
    ips200_show_string(  0, 16 * 5, "Gps_Yaw:");
    ips200_show_string(  0, 16 * 6, "Gps_Yaw2:");
    ips200_show_string(  0, 16 * 7, "Yaw:");
    ips200_show_string(  0, 16 * 8, "Angle:");
    ips200_show_string(  0, 16 * 9, "KEY1:K+0.05");
    ips200_show_string(120, 16 * 9, "KEY2:K-0.05");
    ips200_show_float ( 64, 16 * 1, gnss.latitude  , 4, 6);
    ips200_show_float ( 64, 16 * 2, gnss.longitude , 4, 6);
    ips200_show_uint  (216, 16 * 1, gnss.state     , 1);
    ips200_show_float (200, 16 * 2, K_Gps          , 1, 2);
    ips200_show_float ( 88, 16 * 3, FilterPoint_Lat, 4, 6);
    ips200_show_float ( 88, 16 * 4, FilterPoint_Lon, 4, 6);
    ips200_show_float ( 72, 16 * 5, Gps_Yaw        , 4, 6);
    ips200_show_float ( 72, 16 * 6, Gps_Yaw2       , 4, 6);
    ips200_show_float ( 72, 16 * 7, Yaw            , 4, 6);
    ips200_show_float ( 48, 16 * 8, Angle          , 4, 6);

   seekfree_assistant_oscilloscope_send(&oscilloscope_data);
   oscilloscope_data.data[0] = gnss.latitude;
   oscilloscope_data.data[1] = gnss.longitude;
   oscilloscope_data.data[2] = Angle;
}

void spd_menu(void)
{
    ips200_show_string( 0, 16 * 0, "-->Duty    :");
    ips200_show_string( 0, 16 * 1, "   Distance:");
    ips200_show_string( 0, 16 * 2, "   ServeMid:");
    ips200_show_string( 0, 16 * 3, "KEY1:Duty+1000");
    ips200_show_string( 0, 16 * 4, "KEY2:Duty-1000");
    ips200_show_string( 0, 16 * 5, "KEY3:Save");
    ips200_show_int(   96, 16 * 0, Parameter_set0.Speed_Duty, 5);
    ips200_show_float( 96, 16 * 1, Parameter_set0.Distance, 2, 2);
    ips200_show_int(   96, 16 * 2, Parameter_set0.Serve_Mid, 4);

}

void Distance_menu(void)
{

    ips200_show_string( 0, 16 * 0, "   Duty    :");
    ips200_show_string( 0, 16 * 1, "-->Distance:");
    ips200_show_string( 0, 16 * 2, "   ServeMid:");
    ips200_show_string( 0, 16 * 3, "KEY1:Distance+0.5");
    ips200_show_string( 0, 16 * 4, "KEY2:Distance-0.5");
    ips200_show_string( 0, 16 * 5, "KEY3:Save");
    ips200_show_int(   96, 16 * 0, Parameter_set0.Speed_Duty, 5);
    ips200_show_float( 96, 16 * 1, Parameter_set0.Distance, 2, 2);
    ips200_show_int(   96, 16 * 2, Parameter_set0.Serve_Mid, 4);


}

void serve_mid_menu(void)
{
    ips200_show_string(0, 16 * 0, "   Duty    :");
    ips200_show_string(0, 16 * 1, "   Distance:");
    ips200_show_string(0, 16 * 2, "-->ServeMid:");
    ips200_show_int(   96, 16 * 0, Parameter_set0.Speed_Duty, 5);
    ips200_show_float( 96, 16 * 1, Parameter_set0.Distance, 2, 2);
    ips200_show_int(   96, 16 * 2, Parameter_set0.Serve_Mid, 4);

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
}

void  Points_menu(void)
{
    ips200_show_string( 16, 16 * 0, "Lat:");
    ips200_show_string(128, 16 * 0, "Lon:");
    ips200_show_uint  (184, 16 * 0, Point + 1, 3);
    ips200_show_string(208, 16 * 0, "/");
    ips200_show_uint  (216, 16 * 0, Point_NUM, 3);

    int Page = Point / Page_Point_Num;
    int RightArrow = Point % Page_Point_Num + 1;
    if(Point_NUM > 0)
    {
        ips200_show_string(0, 16 * RightArrow, "->");
        for(int i = 1; i <= Page_Point_Num; i++)
        {
            ips200_show_float ( 16, 16 * i, GPS_GET_LAT[i - 1 + Page * Page_Point_Num], 3, 6);
            ips200_show_float (128, 16 * i, GPS_GET_LOT[i - 1 + Page * Page_Point_Num], 3, 6);
            if(i  + Page * 10 == Point_NUM)
            {
                break;
            }
        }
    }
    ips200_show_string(  0, 16 *  9, "1:Point-1/Lat+");
    ips200_show_string(  0, 16 * 10, "2:Point+1/Lat-");
    ips200_show_string(120, 16 *  9, "3:Save   /Lot+");
    ips200_show_string(120, 16 * 10, "4:Print  /Lot-");

    // double Direction = get_two_points_azimuth(GPS_GET_LAT[0], GPS_GET_LOT[0], GPS_GET_LAT[1], GPS_GET_LOT[1]);
    // double Distance = get_two_points_distance(GPS_GET_LAT[0], GPS_GET_LOT[0], GPS_GET_LAT[1], GPS_GET_LOT[1]);
    // ips200_show_string(0, 16 * 4, "Direction:");
    // ips200_show_string(0, 16 * 5, "Distance:");
    // ips200_show_float(96, 16 * 4, Direction, 3, 3);
    // ips200_show_float(96, 16 * 5, Distance , 5, 6);


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

    ips200_show_float( 80, 16 * 0, Z_360 > 180 ? Z_360 - 360 : Z_360, 3, 3);
    ips200_show_float(128, 16 * 1, IMU_Data.gyro_z, 3, 3);
    ips200_show_float(160, 16 * 2, MAX_IMU_Data_gyro_z, 3, 3);
    ips200_show_float( 72, 16 * 3, angle[0], 3, 3);
    ips200_show_float( 72, 16 * 4, angle[1], 3, 3);
    ips200_show_float( 72, 16 * 5, angle[2], 3, 3);
    ips200_show_float( 48, 16 * 6, Kp_Ah, 3, 3);
    ips200_show_float( 48, 16 * 7, Ki_Ah, 3, 3);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = IMU_Data.gyro_z;
    oscilloscope_data.data[1] = Z_360 > 180 ? Z_360 - 360 : Z_360;
    oscilloscope_data.data[2] = imu963ra_mag_x;
    oscilloscope_data.data[3] = imu963ra_mag_y;
    oscilloscope_data.data[4] = imu963ra_mag_z;
    oscilloscope_data.data[5] = angle[0];
    oscilloscope_data.data[6] = angle[1];
    oscilloscope_data.data[7] = angle[2];

    system_delay_ms(10);
}

void Flash_menu()
{
    ips200_show_string( 80, 16 * 0, "Flash Menu");
    ips200_show_string(  0, 16 * 1, "KEY1:Obtain");
    ips200_show_string(  0, 16 * 2, "KEY2:Save");
    ips200_show_string(  0, 16 * 3, "KEY3:Delete");
    ips200_show_string(  0, 16 * 4, "KEY4:Check");

}

void Task_Select()
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

    ips200_show_string(  0, 16 *  9, "KEY1:Task1");
    ips200_show_string(120, 16 *  9, "KEY2:Task2");
    ips200_show_string(  0, 16 * 10, "KEY3:Task3");
    ips200_show_string(120, 16 * 10, "KEY4:Start");
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

/////////////////////////////////三层菜单-------------------------------------------------

void ServoP_menu(void)
{
    ips200_show_string(  0, 16 * 0, "-->ServoP:");
    ips200_show_string(  0, 16 * 1, "   ServoI:");
    ips200_show_string(  0, 16 * 2, "   ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "Z_360:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:P+0.1");
    ips200_show_string(120, 16 * 8, "KEY2:P-0.1");
    ips200_show_string(  0, 16 * 9, "KEY3:Angle+10");
    ips200_show_string(120, 16 * 9, "KEY4:Angle-10");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 48, 16 * 4, Z_360 > 180 ? Z_360 - 360 : Z_360, 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;

}

void ServoI_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   ServoP:");
    ips200_show_string(  0, 16 * 1, "-->ServoI:");
    ips200_show_string(  0, 16 * 2, "   ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "Z_360:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:I+0.1");
    ips200_show_string(120, 16 * 8, "KEY2:I-0.1");
    ips200_show_string(  0, 16 * 9, "KEY3:Save");
    ips200_show_string(120, 16 * 9, "KEY4:Get PID");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 48, 16 * 4, Z_360 > 180 ? Z_360 - 360 : Z_360, 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;
}
void ServoD_menu(void)
{
    ips200_show_string(  0, 16 * 0, "   ServoP:");
    ips200_show_string(  0, 16 * 1, "   ServoI:");
    ips200_show_string(  0, 16 * 2, "-->ServoD:");
    ips200_show_string(  0, 16 * 3, "Angle:");
    ips200_show_string(  0, 16 * 4, "Z_360:");
    ips200_show_string(  0, 16 * 5, "Angle_Error:");
    ips200_show_string(  0, 16 * 6, "Servo_Angle:");
    ips200_show_string(  0, 16 * 7, "PID.output:");
    ips200_show_string(  0, 16 * 8, "KEY1:D+0.1");
    ips200_show_string(120, 16 * 8, "KEY2:D-0.1");
    ips200_show_string(  0, 16 * 9, "KEY3:Angle+10");
    ips200_show_string(120, 16 * 9, "KEY4:Angle-10");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.ServePID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.ServePID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.ServePID[2], 2, 3);
    ips200_show_float ( 48, 16 * 3, Angle, 4, 6);
    ips200_show_float ( 48, 16 * 4, Z_360 > 180 ? Z_360 - 360 : Z_360, 3, 3);
    ips200_show_float ( 96, 16 * 5, Angle_Error, 3, 6);
    ips200_show_float ( 96, 16 * 6, Servo_Angle, 3, 3);
    ips200_show_float ( 88, 16 * 7, PID_SERVO.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_SERVO.output;
    oscilloscope_data.data[1] = Servo_Angle;
    oscilloscope_data.data[2] = Angle_Error;
    oscilloscope_data.data[3] = Angle;
}

void MotorP_menu(void)
{
    ips200_show_string(  0, 16 * 0, "-->MotorP:");
    ips200_show_string(  0, 16 * 1, "   MotorI:");
    ips200_show_string(  0, 16 * 2, "   MotorD:");
    ips200_show_string(  0, 16 * 3, "Test_Encoder:");
    ips200_show_string(  0, 16 * 4, "Encoder:");
    ips200_show_string(  0, 16 * 5, "PID.error:");
    ips200_show_string(  0, 16 * 6, "PID.output:");
    ips200_show_string(  0, 16 * 7, "KEY1:P+0.1");
    ips200_show_string(120, 16 * 7, "KEY2:P-0.1");
    ips200_show_string(  0, 16 * 8, "KEY3:Duty+1000");
    ips200_show_string(120, 16 * 8, "KEY4:Duty-1000");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_uint  (104, 16 * 3, Test_Encoder, 5);
    ips200_show_int   ( 64, 16 * 4, Encoder, 5);
    ips200_show_float ( 80, 16 * 5, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 * 6, PID_MOTOR.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;
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
    ips200_show_string(  0, 16 * 7, "KEY1:P+0.1");
    ips200_show_string(120, 16 * 7, "KEY2:P-0.1");
    ips200_show_string(  0, 16 * 8, "KEY3:Duty+1000");
    ips200_show_string(120, 16 * 8, "KEY4:Duty-1000");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_uint  (104, 16 * 3, Test_Encoder, 5);
    ips200_show_int   ( 64, 16 * 4, Encoder, 5);
    ips200_show_float ( 80, 16 * 5, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 * 6, PID_MOTOR.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;
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
    ips200_show_string(  0, 16 * 7, "KEY1:P+0.1");
    ips200_show_string(120, 16 * 7, "KEY2:P-0.1");
    ips200_show_string(  0, 16 * 8, "KEY3:Duty+1000");
    ips200_show_string(120, 16 * 8, "KEY4:Duty-1000");
    ips200_show_float ( 80, 16 * 0,Parameter_set0.SpeedPID[0], 2, 3);
    ips200_show_float ( 80, 16 * 1,Parameter_set0.SpeedPID[1], 2, 3);
    ips200_show_float ( 80, 16 * 2,Parameter_set0.SpeedPID[2], 2, 3);
    ips200_show_uint  (104, 16 * 3, Test_Encoder, 5);
    ips200_show_int   ( 64, 16 * 4, Encoder, 5);
    ips200_show_float ( 80, 16 * 5, PID_MOTOR.current_error, 3, 3);
    ips200_show_float ( 88, 16 * 6, PID_MOTOR.output, 3, 6);

    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    oscilloscope_data.data[0] = PID_MOTOR.output;
    oscilloscope_data.data[1] = Encoder;
    oscilloscope_data.data[2] = Test_Encoder;
}

void ZongZuanF(void)
{
//   if(mt9v03x_finish_flag)
//   {
//        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//        mt9v03x_finish_flag = 0;
//   }
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
        if(func_index == 1)
        {
            GL_CRC();

        }
        // ServoP调节
        if(func_index == 5)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[0] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[0] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Test_Angle += 10;
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Test_Angle -= 10;
            }
        }

        // ServoI调节
        if(func_index == 6)
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
        if(func_index == 7)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[2] += 0.1;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.ServePID[2] -= 0.1;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Test_Angle += 10;
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                Test_Angle -= 10;
            }
        }

        // MotorP调节
        if(func_index == 8)
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
                if(Test_Encoder < PWM_DUTY_MAX)
                {
                    Test_Encoder += 1000;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -PWM_DUTY_MAX)
                {
                    Test_Encoder -= 1000;
                }
            }
        }

        // MotorI调节
        if(func_index == 9)
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
                if(Test_Encoder < PWM_DUTY_MAX)
                {
                    Test_Encoder += 1000;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -PWM_DUTY_MAX)
                {
                    Test_Encoder -= 1000;
                }
            }
        }

        //MotorD调节
        if(func_index == 10)
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
                if(Test_Encoder < PWM_DUTY_MAX)
                {
                    Test_Encoder += 1000;
                }
            }
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                if(Test_Encoder > -PWM_DUTY_MAX)
                {
                    Test_Encoder -= 1000;
                }
            }
        }

        if(func_index == 12)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                if(K_Gps < 1)
                {
                    K_Gps += 0.05;
                }
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                if(K_Gps > 0)
                {
                    K_Gps -= 0.05;
                }
            }
        }

        // 调试速度
        if(func_index == 14)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.Speed_Duty += 1000;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.Speed_Duty -= 1000;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        // 换点距离
        if(func_index == 15)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Parameter_set0.Distance += 0.5;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Parameter_set0.Distance -= 0.5;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                FLASH_SAV_PAR();
            }
        }

        //点位查看调节
        if(func_index == 20)
        {
            // 拨码开关在上表示点位切换
            if(!gpio_get_level(SWITCH1))
            {
                if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
                {
                    if(Point_NUM > 0)
                    {
                        if(Point > 0)
                        {
                            Point = Point - 1;
                            ips200_clear();
                        }
                    }
                }
                if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
                {
                    if(Point_NUM > 0)
                    {
                        if(Point < Point_NUM - 1)
                        {
                            Point = Point + 1;
                            ips200_clear();
                        }
                    }
                }
                if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
                {
                    FLASH_FIX_GPS();
                }
                if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
                {
                    for(int i = 0; i < Point_NUM; i++)
                    {
                        printf("{ lat: %lf, lng: %lf, name: \"%d\"}\r\n",GPS_GET_LAT[i], GPS_GET_LOT[i], i);
                    }
                }
            }
            // 拨码开关在下表示点位设置
            if(gpio_get_level(SWITCH1))
            {
                if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
                {
                    GPS_GET_LAT[Point] += 0.000001;
                }
                if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
                {
                    GPS_GET_LAT[Point] -= 0.000001;
                }
                if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
                {
                    GPS_GET_LOT[Point] += 0.000001;
                }
                if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
                {
                    GPS_GET_LOT[Point] -= 0.000001;
                }
            }

        }

        if(func_index == 24)
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

        if(func_index == 26)
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

        if(func_index == 28)
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

        if(func_index == 30)
        {
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = 0;
            }
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = 20;
            }
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                Track_Points_NUM = 50;
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





