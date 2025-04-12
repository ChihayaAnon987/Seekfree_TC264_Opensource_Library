/*
 * GUI.h
 *
 *  Created on: 2025年1月10日
 *      Author: 20483
 */

#ifndef CODE_GUI_H_
#define CODE_GUI_H_


//===================================================宏定义BEG===================================================
#define Page_Point_Num          (     8     )                // 每页显示点的个数（1-8）
//===================================================宏定义END===================================================


//===================================================枚举定义BEG===================================================
typedef enum
{
    enum_first_menu00,                                         // 一级菜单00, CaiDian
    enum_secon_menu00,                                         // 二级菜单00, 踩点

    enum_first_menu01,                                         // 一级菜单01, PID
    enum_secon_menu01,                                         // 二级菜单01, ServoPID
    enum_secon_menu02,                                         // 二级菜单02, MotorPID
    enum_third_menu00,                                         // 三级菜单00, ServoP
    enum_third_menu01,                                         // 三级菜单01, ServoI
    enum_third_menu02,                                         // 三级菜单02, ServoD
    enum_third_menu03,                                         // 三级菜单03, MotorP
    enum_third_menu04,                                         // 三级菜单04, MotorI
    enum_third_menu05,                                         // 三级菜单05, MotorD

    enum_first_menu02,                                         // 一级菜单02, GPS Show
    enum_secon_menu03,                                         // 二级菜单03, GPS

    enum_first_menu03,                                         // 一级菜单03, Duty
    enum_secon_menu04,                                         // 二级菜单04, 速度
    enum_secon_menu05,                                         // 二级菜单05, 换点距离
    enum_secon_menu06,                                         // 二级菜单06, 任务点设置

    enum_first_menu04,                                         // 一级菜单04, RemoteCtrl
    enum_secon_menu07,                                         // 二级菜单07, 遥控

    enum_first_menu05,                                         // 一级菜单05, Points
    enum_secon_menu08,                                         // 二级菜单08, 点

    enum_first_menu06,                                         // 一级菜单06, Camera
    enum_secon_menu09,                                         // 二级菜单09, 总钻风

    enum_first_menu07,                                         // 一级菜单07, Imu963
    enum_secon_menu10,                                         // 二级菜单10, IMU963

    enum_first_menu08,                                         // 一级菜单08, Flash
    enum_secon_menu11,                                         // 二级菜单11, Flash

    enum_first_menu09,                                         // 一级菜单09, SevroTest
    enum_secon_menu12,                                         // 二级菜单12, 舵机

    enum_first_menu10,                                         // 一级菜单10, ParamSet
    enum_secon_menu13,                                         // 二级菜单13, 闭环参数
    enum_secon_menu14,                                         // 二级菜单14，开环参数
    enum_secon_menu15,                                         // 二级菜单15，其他参数
    enum_third_menu06,                                         // 三级菜单06, 闭环参数
    enum_third_menu07,                                         // 三级菜单07, 开环参数
    enum_third_menu08,                                         // 三级菜单08, 其他参数

    enum_first_menu11,                                         // 一级菜单11, TaskSelect
    enum_secon_menu16,                                         // 二级菜单16, 科目一到三
    enum_secon_menu17,                                         // 二级菜单17, 科目四
    enum_third_menu09,                                         // 三级菜单09, 科目一、二、三
    enum_third_menu10,                                         // 三级菜单10, 科目四
}gui_menu_enum;
//===================================================枚举定义END===================================================


//===================================================类型定义BEG===================================================
typedef struct
{
    gui_menu_enum current;                                              // 当前显示层数
    gui_menu_enum up;                                                   // 上翻
    gui_menu_enum down;                                                 // 下翻
    gui_menu_enum back;                                                 // 返回
    gui_menu_enum enter;                                                // 确认
    void (*current_operation)(void);                           // 当前显示函数
} menu_table;

typedef struct
{
    float       ServePID[3];                                    // 舵机PID
    float       SpeedPID[3];                                    // 速度PID
    int         Speed_Duty;                                     // 调试的速度
    float       Distance;                                       // 换点距离
    int         Serve_Mid;                                      // 舵机机械可调中值（根据上次跑车的情况来调节下次跑车的中值）
}Parameter_set;                                                 // 参数集合
//===================================================类型定义END===================================================


//===================================================全局变量BEG===================================================
extern Parameter_set Parameter_set0;                            // 参数集合
extern seekfree_assistant_oscilloscope_struct oscilloscope_data;// 初始化逐飞助手示波器的结构体
extern double Test_Angle;                                       // 调试用
extern int16  Test_Encoder;                                     // 调试用
extern int16  Task_Flag;                                        // 任务标志
extern uint8  Start_Flag;                                       // 发车标志
extern float  Star_Time;                                        // 开始时间
extern float  Stop_Time;                                        // 结束时间
extern double Actual_Dist;                                      // 实际路程
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
//===================================================一级菜单BEG===================================================
void main_menu0(void);                                         // 主菜单0
void main_menu1(void);                                         // 主菜单1
void main_menu2(void);                                         // 主菜单2
void main_menu3(void);                                         // 主菜单3
void main_menu4(void);                                         // 主菜单4
void main_menu5(void);                                         // 主菜单5
void main_menu6(void);                                         // 主菜单6
void main_menu7(void);                                         // 主菜单7
void main_menu8(void);                                         // 主菜单8
void main_menu9(void);                                         // 主菜单9
void main_menu10(void);                                        // 主菜单10
void main_menu11(void);                                        // 主菜单11
//===================================================一级菜单END===================================================


//===================================================二级菜单BEG===================================================
void CaiDian_menu(void);                                       // 菜单：踩点
void ServoPID(void);                                           // 菜单：ServoPID
void MotorPID(void);                                           // 菜单：MotorPID
void GPS_menu(void);                                           // 菜单：GPS
void spd_menu(void);                                           // 菜单：速度
void Distance_menu(void);                                      // 菜单：换点距离
void TaskPoint(void);                                          // 菜单：任务点设置
void RemoteCtrl_menu(void);                                    // 菜单：遥控
void Points_menu(void);                                        // 菜单：点
void ZongZuanF(void);                                          // 菜单：总钻风
void Imu963_menu(void);                                        // 菜单：IMU963
void Flash_menu(void);                                         // 菜单：Flash
void Servo_menu(void);                                         // 菜单：舵机
void LoopEnable_Param(void);                                   // 菜单：闭环参数
void LoopDisable_Param(void);                                  // 菜单：开环参数
void Param_Set(void);                                          // 菜单：其他参数
void Task_Select_menu(void);                                   // 菜单：科目一、二、三
void Task_Four_menu(void);                                     // 菜单：科目四
//===================================================二级菜单END===================================================


//===================================================三级菜单BEG===================================================
void ServoP_menu(void);                                        // 菜单：ServoP
void ServoI_menu(void);                                        // 菜单：ServoI
void ServoD_menu(void);                                        // 菜单：ServoD
void MotorP_menu(void);                                        // 菜单：MotorP
void MotorI_menu(void);                                        // 菜单：MotorI
void MotorD_menu(void);                                        // 菜单：MotorD
void LoopEnable_menu(void);                                    // 菜单：闭环参数
void LoopDisable_menu(void);                                   // 菜单：开环参数
void Param_Set_menu(void);                                     // 菜单：其他参数
void Task_Select(void);                                        // 菜单：科目一、二、三
void Task_Four(void);                                          // 菜单：科目四
//===================================================三级菜单END===================================================


//===================================================菜单控制BEG===================================================
void Key_Ctrl_Menu(void);                                      // 按键控制菜单
void Menu_Control(void);                                       // 菜单控制
//===================================================菜单控制END===================================================
//===================================================函数声明END===================================================

#endif /* CODE_GUI_H_ */
