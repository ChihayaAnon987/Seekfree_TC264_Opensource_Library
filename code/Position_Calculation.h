/*
 * Position_Calculation.h
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#ifndef CODE_POSITION_CALCULATION_H_
#define CODE_POSITION_CALCULATION_H_


//===================================================宏定义BEG===================================================
//#define Points_Num       (     14     )                      // 踩点个数
//===================================================宏定义END===================================================


//===================================================全局变量BEG===================================================
extern int16  Track_Points_NUM;                                // 当前追踪第几个点
extern double Angle_Error;                                     // 角度误差
extern float  Fusion_angle;                                    // GPS和IMU互补滤波后的角度
extern float  Fusion_alpha;                                    // GPS和IMU互补滤波的权重
extern int16  Target_Encoder;                                  // 转速
extern int16  Fly_Slope_Alpha;                                 // 飞坡系数
extern float  K_Straight;                                      // 走直线系数
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
void GPS_IMU_Complementary_Filtering(void);                    // GPS与IMU互补滤波
void Track_Follow(void);                                       // 核心循迹程序
void Point_Switch(void);                                       // 点位切换
//===================================================函数声明END===================================================

#endif /* CODE_POSITION_CALCULATION_H_ */
