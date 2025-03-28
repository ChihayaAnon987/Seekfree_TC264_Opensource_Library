/*
 * GPS.c
 *
 *  Created on: 2025年1月8日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

//1.确定坐标点
//2.利用坐标点代入角度计算函数----方向角
//3.IMU---------------------------航向角
//4.方向角-航向角=角度误差
//存坐标点-----1.用FLASH（可以断电）      2.数组

/*
    参考推文：
    - 17届推文：[链接](https://mp.weixin.qq.com/s/vwganbbwu0eX2j-ZmxWk_A)
    - 18届推文：[链接](https://mp.weixin.qq.com/s/7Ezc6coE8QljQRkr4Ab2sg)
    - 20届推文：[链接](https://mp.weixin.qq.com/s/dY3w1BQd_STeICCmykcyXg)

    关键点总结如下：

    1. **17届方案**：
       - 使用GPS获取当前车头指向的方位角（`gnss.direction`）。
       - 由于GPS更新频率慢，无法完成方向闭环，因此通过陀螺仪计算转向角。
       - 已经被18届方案替代。

    2. **18届方案**：
       - 通过陀螺仪计算方位角（`Z_360`）。
       - 行驶过程中，车模抖动会导致方位角漂移，因此采用互补滤波结合GPS方向角和陀螺仪角度值，得到更稳定的方向角。
       - 同步GPS和陀螺仪的正方向，确保两者方向一致。

    3. **具体实现**：
       - 采用逐飞方案的第一种方法：以陀螺仪初始方向为正方向，方向指向车头。具体实现在函数`Stright_Some_Distance`中。

    4. **20届方案**：
       - 解决GPS导航中的点位漂移问题。
       - 记录发车位置的坐标（`GPS_GET_LAT[0]`, `GPS_GET_LON[0]`），并依次采集路径坐标（`GPS_GET_LAT[i]`, `GPS_GET_LON[i]`）。
       - 发车时，获取当前车的坐标（`Start_Lat`, `Start_Lon`），计算静态漂移值（`Delta_Lat`, `Delta_Lon`）。
       - 将后续所有采集的GPS坐标加上这个偏差值，避免GPS漂移。
       - 如果踩点时GPS数据已经偏移，可以通过显示GPS点位到屏幕，根据路径图手动修正部分偏移点位，结合GPS矫正导航，实现精准导航效果。
*/

/*
(深圳)
    Lat 0.000001 = 0.111319m
    Lon 0.000001 = 0.102907m
    Lat+0.000001 是 0°  (北)
    Lat-0.000001 是 180°(南)
    Lon+0.000001 是 90° (东)
    Lon-0.000001 是 270°(西)
*/


uint32_t Point_NUM = 0;             // 已采集点数
float    K_Gps     = 0.4;           // 衔接部分的权重
double FilterPoint_Lat = 0;         // 滤波后的纬度
double FilterPoint_Lon = 0;         // 滤波后的经度
double Start_Lat;                   // 发车的经度
double Start_Lon;                   // 发车的纬度
double Straight_Lat;                // 直行10-20m的经度
double Straight_Lon;                // 直行10-20m的纬度
double Delta_Lat    = 0;            // 漂移经度
double Delta_Lon    = 0;            // 漂移纬度
double Angle        = 0;            // 方位角
double Delta_Angle  = 0;            // GPS与陀螺仪的正方向偏差角
float  Gps_Yaw      = 0;            // GPS直接得到的偏航角
float  Gps_Yaw2     = 0;            // GPS得到的偏航角（累加和）
float  Yaw          = 0;            // 偏航角
float  Lat_Fix    = 1.0;            // 纬度修正系数
float  Lon_Fix    = 1.0;            // 经度修正系数
double Delta_x      = 0;            // 位移
double Delta_y      = 0;            // 位移
double Distance     = 0;            // 自身距下一个点的距离
double GPS_GET_LAT[NUM_GPS_DATA];   // 纬度
double GPS_GET_LOT[NUM_GPS_DATA];   // 经度
float  GpsSpeed    = 0;             // 速度
float  GpsAccel    = 0;             // 加速度
float  GpsMaxSpeed = 0;             // 最大速度
float  GpsMaxAccel = 0;             // 最大加速度
int8   Task1_Points = 5;            // 科目一所用点位数量
int8   Task2_Bucket = 4;            // 科目二锥桶数量
int8   Task2_Points = 13;           // 科目二所用点位数量
int8   Task3_Points = 9;            // 科目三所用点位数量
int8   Task2_Scales = 6;            // 科目二标尺
int8   Advan_Scales = 1;            // 预测标尺
double min_dx, max_dx, min_dy, max_dy;
double range_x, range_y;
double scale;                       // 缩放因子（单位：像素/米）
int8 start_point = 0;
int8 point_count = 0;
float  GpsDistance[NUM_GPS_DATA] = 
{
    5.0, 3.0, 2.5, 4.0,   0,   0,   0,   0, 1.5,   0,  // 0 - 9

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 10 - 19
    2.5, 1.5, 1.5, 1.5, 1.5, 1.0, 1.5, 1.5, 1.5, 1.5,  // 20 - 29
    1.5, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 30 - 39
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 40 - 49

    4.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5, 4.5, 0.0,  // 50 - 59
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 60 - 69
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 70 - 79
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 80 - 89
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   // 90 - 99
};  // 存储换点距离的数组
int16  GpsTgtEncod[NUM_GPS_DATA] = 
{
    500, 600, 600, 400, 600,   0,   0,   0, 300,   0,  // 0 - 9

    300, 300, 300, 300, 300, 300, 300, 300, 300, 300,  // 10 - 19
    300, 300,   0,   0,   0,   0,   0,   0,   0,   0,  // 20 - 29
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  // 30 - 39
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  // 40 - 49

    300, 300, 300, 300, 300, 300, 300, 300, 300,   0,  // 50 - 59
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  // 60 - 69
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  // 70 - 79
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  // 80 - 89
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0   // 90 - 99
};  // 存储点位速度的数组


void Get_Gps()
{
    // gps数据接收与解析都是通过串口中断调用gps_uart_callback函数进行实现的
    // 数据解析完毕之后gnss_flag标志位会置1
    // static double last_latitude  = 0;
    // static double last_longitude = 0;
    if(gnss_flag)
    {
        gnss_flag = 0;
        gnss_data_parse();           //开始解析数据
        // FilterPoint_Lat = K_Gps * FilterPoint_Lat + (1 - K_Gps) * gnss.latitude;
        // FilterPoint_Lon = K_Gps * FilterPoint_Lon + (1 - K_Gps) * gnss.longitude;
        // GpsSpeed = get_two_points_distance(gnss.latitude, gnss.longitude, last_latitude, last_longitude) * 10.0f;
        // last_latitude  = gnss.latitude ;
        // last_longitude = gnss.longitude;

    }
}

void Get_Physicla_Parameter()
{
    static float Last_Speed = 0;
    float dT = 0.1;
    GpsAccel = (gnss.speed - Last_Speed) / dT;
    Last_Speed = gnss.speed;

    if(GpsAccel > GpsMaxAccel)
    {
        GpsMaxAccel = GpsAccel;
    }

    if(gnss.speed > GpsMaxSpeed)
    {
        GpsMaxSpeed = gnss.speed;
    }
}


void initCoordinateSystem()
{
    int i;
    // 初始化：第0个点相对于自身为 0
    min_dx = max_dx = 0.0;
    min_dy = max_dy = 0.0;
    
    if(Task_Flag == 1)
    {
        start_point = Task1_Start_Point;
        point_count = Task1_Points;
        for(i = Task1_Start_Point; i < (Task1_Start_Point + Task1_Points); i++)
        {
            // 将经度差和纬度差转换为实际距离（米）
            double dx = (GPS_GET_LOT[i] - GPS_GET_LOT[Task1_Start_Point]) * LON_TO_METER;
            double dy = (GPS_GET_LAT[i] - GPS_GET_LAT[Task1_Start_Point]) * LAT_TO_METER;
            if(dx < min_dx) min_dx = dx;
            if(dx > max_dx) max_dx = dx;
            if(dy < min_dy) min_dy = dy;
            if(dy > max_dy) max_dy = dy;
        }
    }
    else if(Task_Flag == 2)
    {
        start_point = Task2_Start_Point;
        point_count = Task2_Points;
        for(i = Task2_Start_Point; i < (Task2_Start_Point + Task2_Points); i++)
        {
            double dx = (GPS_GET_LOT[i] - GPS_GET_LOT[Task2_Start_Point]) * LON_TO_METER;
            double dy = (GPS_GET_LAT[i] - GPS_GET_LAT[Task2_Start_Point]) * LAT_TO_METER;
            if(dx < min_dx) min_dx = dx;
            if(dx > max_dx) max_dx = dx;
            if(dy < min_dy) min_dy = dy;
            if(dy > max_dy) max_dy = dy;
        }
    }
    else if(Task_Flag == 3)
    {
        start_point = Task3_Start_Point;
        point_count = Task3_Points;
        for(i = Task3_Start_Point; i < (Task3_Start_Point + Task3_Points); i++)
        {
            double dx = (GPS_GET_LOT[i] - GPS_GET_LOT[Task3_Start_Point]) * LON_TO_METER;
            double dy = (GPS_GET_LAT[i] - GPS_GET_LAT[Task3_Start_Point]) * LAT_TO_METER;
            if(dx < min_dx) min_dx = dx;
            if(dx > max_dx) max_dx = dx;
            if(dy < min_dy) min_dy = dy;
            if(dy > max_dy) max_dy = dy;
        }
    }

    range_x = max_dx - min_dx;
    range_y = max_dy - min_dy;
    
    // 根据屏幕尺寸（减去左右边距）确定缩放因子，保持整个区域能显示在屏幕内
    double scale_x = (ips200_width_max - 2 * MARGIN) / range_x;
    double scale_y = (ips200_height_max - 2 * MARGIN) / range_y;
    scale = (scale_x < scale_y) ? scale_x : scale_y;
}


void drawGrid()
{
    for(uint16_t i = 0; i < 11; i++)
    {
        uint16_t y = (i == 0) ? 0 : (i * 32 - 1);
        if(i != 5)
        {
            ips200_draw_line(0, y, ips200_width_max - 1, y, RGB565_BLUE);
        }
    }
    for(uint16_t i = 0; i < 11; i++)
    {
        uint16_t y = (i == 0) ? 0 : (i * 24 - 1);
        if(i != 5)
        {
            ips200_draw_line(y, 0, y, ips200_height_max - 1, RGB565_BLUE);
        }
    }
    ips200_draw_line(0, 5 * 32 - 1, ips200_width_max - 1, 5 * 32 - 1, RGB565_WHITE);
    ips200_draw_line(5 * 24 - 1, 0, 5 * 24 - 1, ips200_height_max - 1, RGB565_WHITE);
}

void drawPoints()
{
    // 遍历所有点进行绘制
    for(int i = start_point; i < start_point + point_count; i++)
    {

        uint16_t screen_x, screen_y;
        static uint16_t last_screen_x = 0, last_screen_y = 0;
        if(GPS_GET_LAT[i] != 0 && GPS_GET_LOT[i] != 0)
        {
            gpsToScreen(GPS_GET_LAT[i], GPS_GET_LOT[i], &screen_x, &screen_y, start_point);
        }
        else
        {
            screen_x = ips200_width_max / 2 - 1;
            screen_y = ips200_height_max / 2 - 1;
        }
        // 绘制点（小方块）
        for(int ox = -POINT_SIZE; ox <= POINT_SIZE; ox++)
        {
            for(int oy = -POINT_SIZE; oy <= POINT_SIZE; oy++)
            {
                uint16_t px = (uint16_t)IntClip(screen_x + ox, MARGIN, ips200_width_max - MARGIN);
                uint16_t py = (uint16_t)IntClip(screen_y + oy, MARGIN, ips200_height_max - MARGIN);
                ips200_draw_point(px, py, POINT_COLOR);
            }
        }
        
        // 可选：显示点编号
        char label[4];
        snprintf(label, sizeof(label), "%d", i - start_point);
        ips200_show_string(IntClip(screen_x + 30, 0, ips200_width_max - 1), IntClip(screen_y - 8, 0, ips200_height_max - 1), label);

        if(i != start_point)
        {
            ips200_draw_line(screen_x, screen_y, last_screen_x, last_screen_y, RGB565_WHITE);
        }
        last_screen_x = screen_x;
        last_screen_y = screen_y;
    }
    if(start_point == Task1_Start_Point)
    {
        double distance = get_two_points_distance(GPS_GET_LAT[Task1_Start_Point], GPS_GET_LOT[Task1_Start_Point], GPS_GET_LAT[Task1_Start_Point + 1], GPS_GET_LOT[Task1_Start_Point]);
        static double max_latitude = 0;
        static double max_distance = 0;
        if(GPS_GET_LAT[Task1_Start_Point] < GPS_GET_LAT[Task1_Start_Point + 1]) // 向北发车
        {
            if(gnss.latitude > max_latitude)
            {
                max_latitude = gnss.latitude;
                max_distance = get_two_points_distance(GPS_GET_LAT[Task1_Start_Point], GPS_GET_LOT[Task1_Start_Point], max_latitude, GPS_GET_LOT[Task1_Start_Point]);
            }
        }
        else // 向南发车
        {
            if(gnss.latitude < max_latitude)
            {
                max_latitude = gnss.latitude;
                max_distance = get_two_points_distance(GPS_GET_LAT[Task1_Start_Point], GPS_GET_LOT[Task1_Start_Point], max_latitude, GPS_GET_LOT[Task1_Start_Point]);
            }
        }
        ips200_show_float(192, 16 * 1, distance, 3, 1);
        ips200_show_float(192, 16 * 2, max_distance, 3, 1);
    }
    if(start_point == Task2_Start_Point)
    {
        for(int8 i = Task2_Start_Point; i < Task2_Start_Point + Task2_Bucket + 1; i++)
        {
            double distance = get_two_points_distance(GPS_GET_LAT[i], GPS_GET_LOT[i], GPS_GET_LAT[i + 1], GPS_GET_LOT[i]);
            ips200_show_float(192, 16 * (i - Task2_Start_Point), distance, 2, 2);
        }
    }
    if(start_point == Task3_Start_Point)
    {
        double distance = get_two_points_distance(GPS_GET_LAT[Turn_Point], GPS_GET_LOT[Turn_Point], GPS_GET_LAT[Turn_Point], GPS_GET_LOT[Turn_Point + 1]);
        ips200_show_float(192, 16 * 1, distance, 2, 2);
    }
    ips200_show_float(176, 16 * 17, Actual_Dist, 3, 3);
    ips200_show_float(176, 16 * 18, Stop_Time - Star_Time, 3, 3);
    ips200_show_float(176, 16 * 19, (Stop_Time != 0) ? Actual_Dist / (Stop_Time - Star_Time) : 0.0f, 3, 3);
}

void gpsToScreen(double lat, double lon, uint16_t *screen_x, uint16_t *screen_y, int start_point)
{
    // 计算任务起始点作为原点
    const double origin_lon = GPS_GET_LOT[start_point];
    const double origin_lat = GPS_GET_LAT[start_point];

    double dx = (lon - origin_lon) * LON_TO_METER;
    double dy = (lat - origin_lat) * LAT_TO_METER;

    *screen_x = (uint16_t)IntClip(MARGIN + (uint16_t)((dx - min_dx) * scale), 0, ips200_width_max - 1);
    *screen_y = (uint16_t)IntClip(ips200_height_max - MARGIN - (uint16_t)((dy - min_dy) * scale), 0, ips200_height_max - 1);
}


void updateCarPosition()
{
    if(Star_Time != 0 && Stop_Time == 0)
    {
        uint16_t x = 0, y = 0;
        static uint16_t last_x = 0, last_y = 0;
        static double last_Lat = 0, last_Lon = 0;
        gpsToScreen(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, &x, &y, start_point);
        ips200_draw_point(x, y, RGB565_YELLOW);
        if(last_x != 0 && last_y != 0)
        {
            ips200_draw_line (x, y, last_x, last_y, RGB565_YELLOW);
            Actual_Dist += get_two_points_distance(last_Lat, last_Lon, gnss.latitude, gnss.longitude);
        }
        last_x = x;
        last_y = y;
        last_Lat = gnss.latitude;
        last_Lon = gnss.longitude;
    }
}

void Road_Generator_Init()
{
    Task2_Points = 1 + 2 * Task2_Bucket + 3 + 1;
    int8 Differ1 = Task2_Start_Point - Task2_Road_Genera;
    int8 Differ2 = Task2_Start_Point + Task2_Points + Task2_Road_Genera - 1;
    // 第一步: 计算发车朝向
    // 北: 1    南: -1
    int8 Toward;
    double Azimuth = get_two_points_azimuth(GPS_GET_LAT[Task2_Start_Point], GPS_GET_LOT[Task2_Start_Point], GPS_GET_LAT[Task2_Start_Point + Task2_Bucket + 1], GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 1]);
    if(Azimuth > 350 || Azimuth < 10)
    {
        Toward = 1;
    }
    if(Azimuth > 170 && Azimuth < 190)
    {
        Toward = -1;
    }
    
    // 起始点赋值
    GPS_GET_LAT[Task2_Start_Point] = GPS_GET_LAT[Task2_Road_Genera];
    GPS_GET_LOT[Task2_Start_Point] = GPS_GET_LOT[Task2_Road_Genera];

    
    // 锥桶点
    int8 sign = 1;
    for(int i = Task2_Road_Genera + 1; i < Task2_Road_Genera + Task2_Bucket + 1; i++)
    {
        GPS_GET_LAT[i + Differ1] = GPS_GET_LAT[i] - Toward * 0.000001 * Advan_Scales;
        GPS_GET_LOT[i + Differ1] = GPS_GET_LOT[Task2_Road_Genera] - Toward * sign * 0.000001 * Task2_Scales;
        GPS_GET_LAT[Differ2 - i] = GPS_GET_LAT[i] + Toward * 0.000001 * Advan_Scales;
        GPS_GET_LOT[Differ2 - i] = GPS_GET_LOT[Task2_Road_Genera] + Toward * sign * 0.000001 * Task2_Scales;
        sign = -sign;
    }

    // 掉头点
    GPS_GET_LAT[Task2_Start_Point + Task2_Bucket + 1] = GPS_GET_LAT[Task2_Road_Genera + Task2_Bucket + 1];
    GPS_GET_LAT[Task2_Start_Point + Task2_Bucket + 2] = GPS_GET_LAT[Task2_Road_Genera + Task2_Bucket + 1] + Toward * 0.000004;
    GPS_GET_LAT[Task2_Start_Point + Task2_Bucket + 3] = GPS_GET_LAT[Task2_Road_Genera + Task2_Bucket + 1];

    GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 1] = GPS_GET_LOT[Task2_Road_Genera + Task2_Bucket + Differ1];
    GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 2] = GPS_GET_LOT[Task2_Road_Genera];
    GPS_GET_LOT[Task2_Start_Point + Task2_Bucket + 3] = GPS_GET_LOT[Differ2 - Task2_Road_Genera - Task2_Bucket];
    
    // 终点赋值
    GPS_GET_LAT[Task2_Start_Point + Task2_Points - 1] = GPS_GET_LAT[Task2_Road_Genera];
    GPS_GET_LOT[Task2_Start_Point + Task2_Points - 1] = GPS_GET_LOT[Task2_Start_Point + Task2_Points - 2];

    Point_NUM = Task2_Start_Point + Task2_Points;
    FLASH_FIX_GPS();
}

void Task3_Road_Fix()
{
    for(int16 i = Task3_Start_Point; i < Turn_Point + 1; i++)
    {
        GPS_GET_LOT[i] = GPS_GET_LOT[Task3_Start_Point];
    }
    GPS_GET_LAT[Turn_Point + 1] = GPS_GET_LAT[Turn_Point];
    for(int16 i = Turn_Point + 1; i < Task3_Start_Point + Task3_Points; i++)
    {
        GPS_GET_LOT[i] = GPS_GET_LOT[Turn_Point + 1];
    }
    Point_NUM = Task3_Start_Point + Task3_Points;
    FLASH_FIX_GPS();
}