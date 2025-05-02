/*
 * Position_Calculation.h
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#ifndef CODE_POSITION_CALCULATION_H_
#define CODE_POSITION_CALCULATION_H_

//===================================================宏定义BEG===================================================
#define FLASHING_LIGHT                   (     1    )        // 打开双闪灯
#define LEFTTURN_LIGHT                   (     2    )        // 打开左转灯
#define RIGHTURN_LIGHT                   (     3    )        // 打开右转灯
#define LOWBEAN_HLIGHT                   (     4    )        // 打开近光灯
#define HIGBEAN_HLIGHT                   (     5    )        // 打开远光灯
#define FOG_LIGHT                        (     6    )        // 打开雾灯
#define HEAD_STRAIGHT                    (     7    )        // 向前直行十米
#define BACK_STRAIGHT                    (     8    )        // 后退直行十米
#define SNAKE_ADVANCE                    (     9    )        // 蛇形前进十米
#define SNAKE_BACK                       (    10    )        // 蛇形后退十米
#define ROTATE_ANTICLOCK                 (    11    )        // 逆时针转一圈
#define ROTATE_CLOCKWISE                 (    12    )        // 顺时针转一圈
#define PARK_AREA_ONE                    (    13    )        // 停进停车区一
#define PARK_AREA_TWO                    (    14    )        // 停进停车区二
#define PARK_AREA_THREE                  (    15    )        // 停进停车区三
#define ACTION_COUNT                     (     5    )        // 科目四命令数量
//===================================================宏定义END===================================================


//===================================================全局变量BEG===================================================
extern int16  Track_Points_NUM;                                // 当前追踪第几个点
extern double Angle_Error;                                     // 角度误差
extern int16  Target_Encoder;                                  // 转速
extern int16  Fly_Slope_Alpha;                                 // 飞坡系数
extern float  K_Straight;                                      // 走直线系数
extern int8   Hole_Point;                                      // 标记桥洞点位
extern int8   Ramp_Point;                                      // 标记坡道点位
extern int8   Turn_Point;                                      // 标记掉头点位
extern double Turn_Angle;                                      // 掉头方向
extern int8   Action_Flag[ACTION_COUNT];                       // 科目四动作标志位
extern int8   Task_Four_Turn_Flag;                             // 科目四转圈标志位
extern float  Snack_Advance;                                   // 蛇形前进偏移
extern float  Snack_Back;                                      // 蛇形后退偏移
extern float  Task4_Start_Direc;                               // 科目四发车角度
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
void Track_Follow(void);                                       // 核心循迹程序
void Task4_Finish(void);                                       // 完成科目四
void Point_Switch(void);                                       // 点位切换
float LimitFabs180(float angle);                               // 角度限制(-180到180)
//===================================================函数声明END===================================================

#endif /* CODE_POSITION_CALCULATION_H_ */
