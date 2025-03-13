/*
 * GPS.c
 *
 *  Created on: 2025��1��8��
 *      Author: 20483
 */

#include "zf_common_headfile.h"

//1.ȷ�������
//2.������������Ƕȼ��㺯��----�����
//3.IMU---------------------------�����
//4.�����-�����=�Ƕ����
//�������-----1.��FLASH�����Զϵ磩      2.����

/*
    �ο����ģ�
    - 17�����ģ�[����](https://mp.weixin.qq.com/s/vwganbbwu0eX2j-ZmxWk_A)
    - 18�����ģ�[����](https://mp.weixin.qq.com/s/7Ezc6coE8QljQRkr4Ab2sg)
    - 20�����ģ�[����](https://mp.weixin.qq.com/s/dY3w1BQd_STeICCmykcyXg)

    �ؼ����ܽ����£�

    1. **17�췽��**��
       - ʹ��GPS��ȡ��ǰ��ͷָ��ķ�λ�ǣ�`gnss.direction`����
       - ����GPS����Ƶ�������޷���ɷ���ջ������ͨ�������Ǽ���ת��ǡ�
       - �Ѿ���18�췽�������

    2. **18�췽��**��
       - ͨ�������Ǽ��㷽λ�ǣ�`Z_360`����
       - ��ʻ�����У���ģ�����ᵼ�·�λ��Ư�ƣ���˲��û����˲����GPS����Ǻ������ǽǶ�ֵ���õ����ȶ��ķ���ǡ�
       - ͬ��GPS�������ǵ�������ȷ�����߷���һ�¡�

    3. **����ʵ��**��
       - ������ɷ����ĵ�һ�ַ������������ǳ�ʼ����Ϊ�����򣬷���ָ��ͷ������ʵ���ں���`Stright_Some_Distance`�С�

    4. **20�췽��**��
       - ���GPS�����еĵ�λƯ�����⡣
       - ��¼����λ�õ����꣨`GPS_GET_LAT[0]`, `GPS_GET_LON[0]`���������βɼ�·�����꣨`GPS_GET_LAT[i]`, `GPS_GET_LON[i]`����
       - ����ʱ����ȡ��ǰ�������꣨`Start_Lat`, `Start_Lon`�������㾲̬Ư��ֵ��`Delta_Lat`, `Delta_Lon`����
       - ���������вɼ���GPS����������ƫ��ֵ������GPSƯ�ơ�
       - ����ȵ�ʱGPS�����Ѿ�ƫ�ƣ�����ͨ����ʾGPS��λ����Ļ������·��ͼ�ֶ���������ƫ�Ƶ�λ�����GPS����������ʵ�־�׼����Ч����
*/

/*
    Lat 0.000001 = 0.111319m
    Lon 0.000001 = 0.061010m
    Lat+0.000001 �� 0��  (��)
    Lat-0.000001 �� 180��(��)
    Lon+0.000001 �� 90�� (��)
    Lon-0.000001 �� 270��(��)
*/


uint32_t Point_NUM = 0;             // �Ѳɼ�����
float    K_Gps     = 0.5;           // �νӲ��ֵ�Ȩ��
double FilterPoint_Lat = 0;         // �˲����γ��
double FilterPoint_Lon = 0;         // �˲���ľ���
double Start_Lat;                   // �����ľ���
double Start_Lon;                   // ������γ��
double Straight_Lat;                // ֱ��10-20m�ľ���
double Straight_Lon;                // ֱ��10-20m��γ��
double Delta_Lat    = 0;            // Ư�ƾ���
double Delta_Lon    = 0;            // Ư��γ��
double Angle        = 0;            // ��λ��
double Delta_Angle  = 0;            // GPS�������ǵ�������ƫ���
float  Gps_Yaw      = 0;            // GPSֱ�ӵõ���ƫ����
uint8  Gps_Yaw_Flag = 0;            // GPSֱ�ӵõ���ƫ���Ǳ�־λ
float  Gps_Yaw2     = 0;            // GPS�õ���ƫ���ǣ��ۼӺͣ�
float  Yaw          = 0;            // ƫ����
uint8  Yaw_Times    = 0;            // ƫ���Ǽ���
float  Lat_Fix    = 1.0;            // γ������ϵ��
float  Lon_Fix    = 1.0;            // ��������ϵ��
double Delta_x      = 0;            // λ��
double Delta_y      = 0;            // λ��
double Distance     = 0;            // �������һ����ľ���
double GPS_GET_LAT[NUM_GPS_DATA];   // γ��
double GPS_GET_LOT[NUM_GPS_DATA];   // ����
float  GpsAccel;                    // ���ٶ�
float  GpsMaxSpeed = 0;             // ����ٶ�
float  GpsMaxAccel = 0;             // �����ٶ�
int8   Task1_Points = 5;            // ��Ŀһ���õ�λ����
int8   Task2_Points = 12;           // ��Ŀ�����õ�λ������4��Ͱ, ÿ��һ��Ͱ����������λ��
int8   Task3_Points = 9;            // ��Ŀ�����õ�λ����
double min_dx, max_dx, min_dy, max_dy;
double range_x, range_y;
double scale;                       // �������ӣ���λ������/�ף�
int8 start_point = 0;
int8 point_count = 0;
float  GpsDistance[NUM_GPS_DATA] = 
{
    5.0, 3.0, 2.5, 4.0,   0,   0,   0,   0, 1.5,   0,  // 0 - 9

    2.5, 1.5, 1.5, 1.5, 1.5, 1.0, 1.5, 1.5, 1.5, 1.5,  // 10 - 19
    1.5, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 20 - 29
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 30 - 39
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 40 - 49

    4.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5, 4.5, 0.0,  // 50 - 59
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 60 - 69
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 70 - 79
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 80 - 89
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   // 90 - 99
};  // �洢������������
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
};  // �洢��λ�ٶȵ�����


void Get_Gps()
{
    // gps���ݽ������������ͨ�������жϵ���gps_uart_callback��������ʵ�ֵ�
    // ���ݽ������֮��gnss_flag��־λ����1
    if(gnss_flag)
    {
        gnss_flag = 0;
        gnss_data_parse();           //��ʼ��������
        FilterPoint_Lat = K_Gps * FilterPoint_Lat + (1 - K_Gps) * gnss.latitude;
        FilterPoint_Lon = K_Gps * FilterPoint_Lon + (1 - K_Gps) * gnss.longitude;
        Angle = get_two_points_azimuth(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, GPS_GET_LAT[Track_Points_NUM], GPS_GET_LOT[Track_Points_NUM]);
        Angle -= Delta_Angle;
        if(Angle > 180)
        {
            Angle -= 360;
        }
        if(Angle < -180)
        {
            Angle += 360;
        }

        // if(gnss.direction < 180)
        // {
        //     Gps_Yaw = gnss.direction;
        // }
        // if(gnss.direction > 180)
        // {
        //     Gps_Yaw = gnss.direction - 360;
        // }
        // Gps_Yaw_Flag = 1;
    }
    FilterPoint_Lat += (Delta_y * QS * 0.000000001 * Lat_Fix);
    FilterPoint_Lon += (Delta_x * QS * 0.000000001 * Lon_Fix) / (cos(FilterPoint_Lat * PI / 180));
}

void Get_Gps_Yaw()
{
    if(Gps_Yaw_Flag == 1)
    {
        Yaw_Times++;
        Gps_Yaw2 += Gps_Yaw;
        Gps_Yaw_Flag = 0;
    }
    if(Yaw_Times == 10)
    {
        Yaw = Gps_Yaw2 / 10;
        Gps_Yaw2 = 0;
        Yaw_Times = 0;
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
    // ��ʼ������0�������������Ϊ 0
    min_dx = max_dx = 0.0;
    min_dy = max_dy = 0.0;
    
    if(Task_Flag == 1)
    {
        start_point = Task1_Start_Point;
        point_count = Task1_Points;
        for(i = Task1_Start_Point; i < (Task1_Start_Point + Task1_Points); i++)
        {
            // �����Ȳ��γ�Ȳ�ת��Ϊʵ�ʾ��루�ף�
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
    
    // ������Ļ�ߴ磨��ȥ���ұ߾ࣩȷ���������ӣ�����������������ʾ����Ļ��
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
    // �������е���л���
    for(int i = start_point; i < start_point + point_count; i++)
    {

        uint16_t screen_x, screen_y;
        gpsToScreen(GPS_GET_LAT[i], GPS_GET_LOT[i], &screen_x, &screen_y, start_point);
        // ���Ƶ㣨С���飩
        for(int ox = -POINT_SIZE; ox <= POINT_SIZE; ox++)
        {
            for(int oy = -POINT_SIZE; oy <= POINT_SIZE; oy++)
            {
                int px = IntClip(screen_x + ox, MARGIN, ips200_width_max - MARGIN);
                int py = IntClip(screen_y + oy, MARGIN, ips200_height_max - MARGIN);
                ips200_draw_point((uint16)px, (uint16)py, POINT_COLOR);
            }
        }
        
        // ��ѡ����ʾ����
        char label[4];
        snprintf(label, sizeof(label), "%d", i - start_point);
        ips200_show_string(IntClip(screen_x + 30, 0, ips200_width_max - 1), IntClip(screen_y - 8, 0, ips200_height_max - 1), label);
    }
}

void gpsToScreen(double lat, double lon, uint16_t *screen_x, uint16_t *screen_y, int start_point)
{
    // ����������ʼ����Ϊԭ��
    const double origin_lon = GPS_GET_LOT[start_point];
    const double origin_lat = GPS_GET_LAT[start_point];

    double dx = (lon - origin_lon) * LON_TO_METER;
    double dy = (lat - origin_lat) * LAT_TO_METER;

    *screen_x = (uint16_t)IntClip(MARGIN + (uint16_t)((dx - min_dx) * scale), 0, ips200_width_max - 1);
    *screen_y = (uint16_t)IntClip(ips200_height_max - MARGIN - (uint16_t)((dy - min_dy) * scale), 0, ips200_height_max - 1);
}


void updateCarPosition()
{
    if(gnss.state == 1 && Start_Flag == 1)
    {
        uint16_t x = 0, y = 0;
        static uint16_t last_x = 0, last_y = 0;
        gpsToScreen(gnss.latitude - Delta_Lat, gnss.longitude - Delta_Lon, &x, &y, start_point);
        ips200_draw_point(x, y, RGB565_YELLOW);
        if(last_x != 0 && last_y != 0)
        {
            ips200_draw_line (x, y, last_x, last_y, RGB565_YELLOW);
        }
        last_x = x;
        last_y = y;
    }
}
