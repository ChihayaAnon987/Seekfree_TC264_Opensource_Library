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


//===================================================类型定义BEG===================================================
typedef struct
{
    char current;                                              // 当前显示层数
    char up;                                                   // 上翻
    char down;                                                 // 下翻
    char back;                                                 // 返回
    char enter;                                                // 确认
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
void Param_Set(void);                                          // 菜单：参数设置
void Task_Select(void);                                        // 菜单：任务选择
//===================================================二级菜单END===================================================


//===================================================三级菜单BEG===================================================
void ServoP_menu(void);                                        // 菜单：ServoP
void ServoI_menu(void);                                        // 菜单：ServoI
void ServoD_menu(void);                                        // 菜单：ServoD
void MotorP_menu(void);                                        // 菜单：MotorP
void MotorI_menu(void);                                        // 菜单：MotorI
void MotorD_menu(void);                                        // 菜单：MotorD
//===================================================三级菜单END===================================================


//===================================================菜单控制BEG===================================================
void Key_Ctrl_Menu(void);                                      // 按键控制菜单
void Menu_Control(void);                                       // 菜单控制
//===================================================菜单控制END===================================================


//===================================================函数声明END===================================================

#endif /* CODE_GUI_H_ */
