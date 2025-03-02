/*
 * GPS.h
 *
 *  Created on: 2025年1月8日
 *      Author: 20483
 */

#ifndef CODE_GPS_H_
#define CODE_GPS_H_

//===================================================宏定义BEG===================================================
#define NUM_GPS_DATA        (     100     )                  // GPS 采集点数
#define Task1_Start_Point   (      0      )                  // 科目一起始点位
#define Task2_Start_Point   (     10      )                  // 科目二起始点位
#define Task3_Start_Point   (     50      )                  // 科目三起始点位
#define QS                  (   8.99266   )                  // 坐标变换常数
//===================================================宏定义END===================================================


//===================================================全局变量BEG===================================================
extern uint32 Point_NUM;                                       // 当前采集的 GPS 点数量
extern float  K_Gps;                                           // 衔接部分的权重
extern double FilterPoint_Lat;                                 // 滤波后的纬度
extern double FilterPoint_Lon;                                 // 滤波的经度
extern double Start_Lat;                                       // 发车的经度
extern double Start_Lon;                                       // 发车的纬度
extern double Straight_Lat;                                    // 直行10-20m的经度
extern double Straight_Lon;                                    // 直行10-20m的纬度
extern double Delta_Lat;                                       // 漂移经度
extern double Delta_Lon;                                       // 漂移纬度
extern double Angle;                                           // 方位角
extern double Delta_Angle;                                     // GPS与陀螺仪的正方向偏差角
extern double Distance;                                        // 自身距下一个点的距离
extern float  Yaw;                                             // 偏航角
extern float  Gps_Yaw;                                         // GPS直接得到的偏航角
extern float  Gps_Yaw2;                                        // GPS得到的偏航角（累加和）
extern float  Lat_Fix;                                         // 纬度修正系数
extern float  Lon_Fix;                                         // 经度修正系数
extern double Delta_x;                                         // 位移
extern double Delta_y;                                         // 位移
extern double GPS_GET_LAT[NUM_GPS_DATA];                       // 存储纬度数据的数组
extern double GPS_GET_LOT[NUM_GPS_DATA];                       // 存储经度数据的数组
extern int8   Task1_Points;                                    // 科目一所用点位数量
extern int8   Task2_Points;                                    // 科目二所用点位数量
extern int8   Task3_Points;                                    // 科目三所用点位数量
extern float  GpsDistance[NUM_GPS_DATA];                       // 存储换点距离的数组
extern int16  GpsTgtEncod[NUM_GPS_DATA];                       // 存储点位速度的数组
extern float  GpsAccel;                                        // 加速度
extern float  GpsMaxSpeed;                                     // 最大速度
extern float  GpsMaxAccel;                                     // 最大加速度
//===================================================全局变量END===================================================


//===================================================函数声明BEG===================================================
void Get_Gps(void);                                            // 获取坐标等信息
void Get_Gps_Yaw(void);                                        // 获取GPS偏航角
void Get_Physicla_Parameter(void);                             // 获取物理参数
//===================================================函数声明END===================================================


#endif /* CODE_GPS_H_ */
