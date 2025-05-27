/*
 * Flash.c
 *
 *  Created on: 2025年1月9日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

double_to_uint32_union lat_union[NUM_GPS_DATA];
double_to_uint32_union lon_union[NUM_GPS_DATA];

// 存储数据到Flash
void FLASH_SAV_GPS()
{
    flash_buffer_clear();
    if(Point_NUM != 0)
    {
        flash_union_buffer[0].uint32_type = Point_NUM;
        if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
        {
            flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // 擦除这一页
        }
        for (int i = 0; i < NUM_GPS_DATA; i++)
        {
            flash_union_buffer[i * 4 + 1].uint32_type = lat_union[i].uint32_type[0];  // 纬度高 32 位
            flash_union_buffer[i * 4 + 2].uint32_type = lat_union[i].uint32_type[1];  // 纬度低 32 位

            flash_union_buffer[i * 4 + 3].uint32_type = lon_union[i].uint32_type[0];  // 经度高 32 位
            flash_union_buffer[i * 4 + 4].uint32_type = lon_union[i].uint32_type[1];  // 经度低 32 位
        }
        // 写入 Flash 页面
        flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        flash_buffer_clear();

        LED_Buzzer_Flag_Ctrl(LED3); // 暂时替换
    }
}

// 从 Flash 读取数据到数组
void FLASH_GET_GPS()
{
    flash_buffer_clear();                                                       // 确保缓冲区干净
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        Point_NUM = flash_union_buffer[0].uint32_type;
        for(int i = 0; i < NUM_GPS_DATA; i++)
        {
            if(flash_union_buffer[i * 4 + 1].uint32_type != 0)
            {
                if(flash_union_buffer[i * 4 + 2].uint32_type != 0)
                {
                    lat_union[i].uint32_type[0] = flash_union_buffer[i * 4 + 1].uint32_type;  // 纬度高 32 位
                    lat_union[i].uint32_type[1] = flash_union_buffer[i * 4 + 2].uint32_type;  // 纬度低 32 位
                    Point[i].latitude = lat_union[i].double_type;                                // 纬度
                }
            }
            if(flash_union_buffer[i * 4 + 3].uint32_type != 0)
            {
                if(flash_union_buffer[i * 4 + 4].uint32_type != 0)
                {
                    lon_union[i].uint32_type[0] = flash_union_buffer[i * 4 + 3].uint32_type;  // 经度高 32 位
                    lon_union[i].uint32_type[1] = flash_union_buffer[i * 4 + 4].uint32_type;  // 经度低 32 位
                    Point[i].lonitude = lon_union[i].double_type;                                // 经度
                }
            }
        }
        flash_buffer_clear();                                                    // 清空缓冲区

        LED_Buzzer_Flag_Ctrl(LED3); // 暂时替换
    }
    else
    {
        LED_Buzzer_Flag_Ctrl(LED2);
    }
}

// 修正 Gps 数据
void FLASH_FIX_GPS()
{
    for(int i = 0; i < NUM_GPS_DATA; i++)
    {
        lat_union[i].double_type = Point[i].latitude;
        lon_union[i].double_type = Point[i].lonitude;
    }
    FLASH_SAV_GPS();
}

// 清除Flash数据
void FLASH_DEL_GPS()
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        gpio_set_level(LED1, 0);                                                    //指示灯亮，表明已删除
        gpio_set_level(LED2, 0);
        gpio_set_level(LED3, 0);                                                    //指示灯亮，表明已删除
        gpio_set_level(LED4, 0);
        system_delay_ms(500);
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 1);
        gpio_set_level(LED3, 1);
        gpio_set_level(LED4, 1);
    }
}


void FLASH_SAV_PAR()
{
    flash_buffer_clear();
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX - 2))                      // 判断是否有数据
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX - 2);                // 擦除这一页
    }
    // 舵机PID
    flash_union_buffer[0].float_type = Parameter_set0.ServePID[0];
    flash_union_buffer[1].float_type = Parameter_set0.ServePID[1];
    flash_union_buffer[2].float_type = Parameter_set0.ServePID[2];

    //电机PID
    flash_union_buffer[3].float_type = Parameter_set0.SpeedPID[0];
    flash_union_buffer[4].float_type = Parameter_set0.SpeedPID[1];
    flash_union_buffer[5].float_type = Parameter_set0.SpeedPID[2];
    // 调试速度和换点距离
    flash_union_buffer[6].int32_type = Parameter_set0.Speed_Duty;
    flash_union_buffer[7].float_type = Parameter_set0.Distance;
    // 任务点
    flash_union_buffer[8].int8_type  = Task1_Points;
    flash_union_buffer[9].int8_type  = Task2_Bucket;
    flash_union_buffer[10].int8_type = Task3_Points;

    // 系数
    flash_union_buffer[11].int16_type = Fly_Slope_Alpha;
    flash_union_buffer[12].float_type = K_Straight;

    flash_union_buffer[13].float_type = From_0000_To_2000_ServoPD.Kp;
    flash_union_buffer[14].float_type = From_0000_To_2000_ServoPD.Kd;
    flash_union_buffer[15].float_type = From_2000_To_4000_ServoPD.Kp;
    flash_union_buffer[16].float_type = From_2000_To_4000_ServoPD.Kd;
    flash_union_buffer[17].float_type = From_4000_To_5000_ServoPD.Kp;
    flash_union_buffer[18].float_type = From_4000_To_5000_ServoPD.Kd;
    flash_union_buffer[19].float_type = From_5000_To_6000_ServoPD.Kp;
    flash_union_buffer[20].float_type = From_5000_To_6000_ServoPD.Kd;
    flash_union_buffer[21].float_type = From_6000_To_7000_ServoPD.Kp;
    flash_union_buffer[22].float_type = From_6000_To_7000_ServoPD.Kd;
    flash_union_buffer[23].float_type = From_7000_To_8000_ServoPD.Kp;
    flash_union_buffer[24].float_type = From_7000_To_8000_ServoPD.Kd;
    flash_union_buffer[25].float_type = From_8000_To_9000_ServoPD.Kp;
    flash_union_buffer[26].float_type = From_8000_To_9000_ServoPD.Kd;
    flash_union_buffer[27].float_type = From_9000_To_9900_ServoPD.Kp;
    flash_union_buffer[28].float_type = From_9000_To_9900_ServoPD.Kd;
    flash_union_buffer[29].int8_type  = Task2_Scales;
    flash_union_buffer[30].int8_type  = Advan_Scales;
    flash_union_buffer[31].int8_type  = Turn_Point;
    flash_union_buffer[32].float_type  = Encoder0100_ServoPD.Kp;
    flash_union_buffer[33].float_type  = Encoder0100_ServoPD.Kd;
    flash_union_buffer[34].float_type  = Encoder0200_ServoPD.Kp;
    flash_union_buffer[35].float_type  = Encoder0200_ServoPD.Kd;
    flash_union_buffer[36].float_type  = Encoder0300_ServoPD.Kp;
    flash_union_buffer[37].float_type  = Encoder0300_ServoPD.Kd;
    flash_union_buffer[38].float_type  = Encoder0400_ServoPD.Kp;
    flash_union_buffer[39].float_type  = Encoder0400_ServoPD.Kd;
    flash_union_buffer[40].float_type  = Encoder0500_ServoPD.Kp;
    flash_union_buffer[41].float_type  = Encoder0500_ServoPD.Kd;
    flash_union_buffer[42].float_type  = Encoder0600_ServoPD.Kp;
    flash_union_buffer[43].float_type  = Encoder0600_ServoPD.Kd;
    flash_union_buffer[44].float_type  = Encoder0700_ServoPD.Kp;
    flash_union_buffer[45].float_type  = Encoder0700_ServoPD.Kd;
    flash_union_buffer[46].float_type  = Encoder0800_ServoPD.Kp;
    flash_union_buffer[47].float_type  = Encoder0800_ServoPD.Kd;
    flash_union_buffer[48].float_type  = Encoder0900_ServoPD.Kp;
    flash_union_buffer[49].float_type  = Encoder0900_ServoPD.Kd;
    flash_union_buffer[50].float_type  = Encoder1000_ServoPD.Kp;
    flash_union_buffer[51].float_type  = Encoder1000_ServoPD.Kd;
    flash_union_buffer[52].float_type  = Encoder1100_ServoPD.Kp;
    flash_union_buffer[53].float_type  = Encoder1100_ServoPD.Kd;
    flash_union_buffer[54].float_type  = Encoder1200_ServoPD.Kp;
    flash_union_buffer[55].float_type  = Encoder1200_ServoPD.Kd;
    
    flash_union_buffer[56].float_type  = Snack_Advance;
    flash_union_buffer[57].float_type  = Snack_Back;
    flash_union_buffer[58].float_type  = Task4_Start_Direc;
    flash_union_buffer[59].float_type  = Bucket_Dista;
    flash_union_buffer[60].float_type  = Start_To_Bucket;
    flash_union_buffer[61].float_type  = Task3_Width;
    
    // 换点距离和速度数组
    for(int i = 100; i < 100 + NUM_GPS_DATA; i++)
    {
        flash_union_buffer[i].float_type = GpsDistance[i - 100];
    }
    for(int i = 100 + NUM_GPS_DATA; i < 100 + 2 * NUM_GPS_DATA; i++)
    {
        flash_union_buffer[i].int16_type = GpsTgtEncod[i - 100 - NUM_GPS_DATA];
    }
    // 指示灯亮，表明已读取
    LED_Buzzer_Flag_Ctrl(LED3); // 暂时替换


    // 写入 Flash 页面
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX - 2);
    flash_buffer_clear();
}

void FLASH_GET_PAR()
{
    flash_buffer_clear();                                                       // 确保缓冲区干净
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX - 2))                      // 判断是否有数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX - 2);
        // 速度PID
        Parameter_set0.ServePID[0] = flash_union_buffer[0].float_type;
        Parameter_set0.ServePID[1] = flash_union_buffer[1].float_type;
        Parameter_set0.ServePID[2] = flash_union_buffer[2].float_type;

        // 电机PID
        Parameter_set0.SpeedPID[0] = flash_union_buffer[3].float_type;
        Parameter_set0.SpeedPID[1] = flash_union_buffer[4].float_type;
        Parameter_set0.SpeedPID[2] = flash_union_buffer[5].float_type;
        // 调试速度和换点距离
        Parameter_set0.Speed_Duty = flash_union_buffer[6].int32_type;
        Parameter_set0.Distance   = flash_union_buffer[7].float_type;
        // 任务点
        Task1_Points = flash_union_buffer[8].int8_type ;
        Task2_Bucket = flash_union_buffer[9].int8_type ;
        Task3_Points = flash_union_buffer[10].int8_type;

        // 系数
        Fly_Slope_Alpha = flash_union_buffer[11].int16_type;
        K_Straight      = flash_union_buffer[12].float_type;

        From_0000_To_2000_ServoPD.Kp = flash_union_buffer[13].float_type;
        From_0000_To_2000_ServoPD.Kd = flash_union_buffer[14].float_type;
        From_2000_To_4000_ServoPD.Kp = flash_union_buffer[15].float_type;
        From_2000_To_4000_ServoPD.Kd = flash_union_buffer[16].float_type;
        From_4000_To_5000_ServoPD.Kp = flash_union_buffer[17].float_type;
        From_4000_To_5000_ServoPD.Kd = flash_union_buffer[18].float_type;
        From_5000_To_6000_ServoPD.Kp = flash_union_buffer[19].float_type;
        From_5000_To_6000_ServoPD.Kd = flash_union_buffer[20].float_type;
        From_6000_To_7000_ServoPD.Kp = flash_union_buffer[21].float_type;
        From_6000_To_7000_ServoPD.Kd = flash_union_buffer[22].float_type;
        From_7000_To_8000_ServoPD.Kp = flash_union_buffer[23].float_type;
        From_7000_To_8000_ServoPD.Kd = flash_union_buffer[24].float_type;
        From_8000_To_9000_ServoPD.Kp = flash_union_buffer[25].float_type;
        From_8000_To_9000_ServoPD.Kd = flash_union_buffer[26].float_type;
        From_9000_To_9900_ServoPD.Kp = flash_union_buffer[27].float_type;
        From_9000_To_9900_ServoPD.Kd = flash_union_buffer[28].float_type;
        Task2_Scales = flash_union_buffer[29].int8_type;
        Advan_Scales = flash_union_buffer[30].int8_type;
        Turn_Point   = flash_union_buffer[31].int8_type;
        Encoder0100_ServoPD.Kp = flash_union_buffer[32].float_type;
        Encoder0100_ServoPD.Kd = flash_union_buffer[33].float_type;
        Encoder0200_ServoPD.Kp = flash_union_buffer[34].float_type;
        Encoder0200_ServoPD.Kd = flash_union_buffer[35].float_type;
        Encoder0300_ServoPD.Kp = flash_union_buffer[36].float_type;
        Encoder0300_ServoPD.Kd = flash_union_buffer[37].float_type;
        Encoder0400_ServoPD.Kp = flash_union_buffer[38].float_type;
        Encoder0400_ServoPD.Kd = flash_union_buffer[39].float_type;
        Encoder0500_ServoPD.Kp = flash_union_buffer[40].float_type;
        Encoder0500_ServoPD.Kd = flash_union_buffer[41].float_type;
        Encoder0600_ServoPD.Kp = flash_union_buffer[42].float_type;
        Encoder0600_ServoPD.Kd = flash_union_buffer[43].float_type;
        Encoder0700_ServoPD.Kp = flash_union_buffer[44].float_type;
        Encoder0700_ServoPD.Kd = flash_union_buffer[45].float_type;
        Encoder0800_ServoPD.Kp = flash_union_buffer[46].float_type;
        Encoder0800_ServoPD.Kd = flash_union_buffer[47].float_type;
        Encoder0900_ServoPD.Kp = flash_union_buffer[48].float_type;
        Encoder0900_ServoPD.Kd = flash_union_buffer[49].float_type;
        Encoder1000_ServoPD.Kp = flash_union_buffer[50].float_type;
        Encoder1000_ServoPD.Kd = flash_union_buffer[51].float_type;
        Encoder1100_ServoPD.Kp = flash_union_buffer[52].float_type;
        Encoder1100_ServoPD.Kd = flash_union_buffer[53].float_type;
        Encoder1200_ServoPD.Kp = flash_union_buffer[54].float_type;
        Encoder1200_ServoPD.Kd = flash_union_buffer[55].float_type;

        Snack_Advance = flash_union_buffer[56].float_type;
        Snack_Back    = flash_union_buffer[57].float_type;
        Task4_Start_Direc = flash_union_buffer[58].float_type;
        Bucket_Dista = flash_union_buffer[59].float_type;
        Start_To_Bucket = flash_union_buffer[60].float_type;
        Task3_Width = flash_union_buffer[61].float_type;
        
        // 换点距离和速度数组
        for(int i = 100; i < 100 + NUM_GPS_DATA; i++)
        {
            GpsDistance[i - 100] = flash_union_buffer[i].float_type;
        }
        for(int i = 100 + NUM_GPS_DATA; i < 100 + 2 * NUM_GPS_DATA; i++)
        {
            GpsTgtEncod[i - 100 - NUM_GPS_DATA] = flash_union_buffer[i].int16_type;
        }

        flash_buffer_clear();                                                    // 清空缓冲区
    }
}

void FLASH_PRI_PAR()
{
    printf("ServoPID[0]:%f\r\n", Parameter_set0.ServePID[0]);
    printf("ServoPID[1]:%f\r\n", Parameter_set0.ServePID[1]);
    printf("ServoPID[2]:%f\r\n", Parameter_set0.ServePID[2]);
    
    printf("MotorPID[0]:%f\r\n", Parameter_set0.SpeedPID[0]);
    printf("MotorPID[1]:%f\r\n", Parameter_set0.SpeedPID[1]);
    printf("MotorPID[2]:%f\r\n", Parameter_set0.SpeedPID[2]);

    printf("Task1Points:%d\r\n", Task1_Points);
    printf("Task2Bucket:%d\r\n", Task2_Bucket);
    printf("Task3Points:%d\r\n", Task3_Points);

    printf("Fly_Slope_Alpha:%d\r\n", Fly_Slope_Alpha);
    printf("K_Straight:%f\r\n", K_Straight);
    printf("Task2_Scales:%d\r\n", Task2_Scales);
    printf("Advan_Scales:%d\r\n", Advan_Scales);
    printf("Turn_Point:%d\r\n", Turn_Point);

    printf("From_0000_To_2000_ServoPD: Kp = %f, Kd = %f\r\n", From_0000_To_2000_ServoPD.Kp, From_0000_To_2000_ServoPD.Kd);
    printf("From_2000_To_4000_ServoPD: Kp = %f, Kd = %f\r\n", From_2000_To_4000_ServoPD.Kp, From_2000_To_4000_ServoPD.Kd);
    printf("From_4000_To_5000_ServoPD: Kp = %f, Kd = %f\r\n", From_4000_To_5000_ServoPD.Kp, From_4000_To_5000_ServoPD.Kd);
    printf("From_5000_To_6000_ServoPD: Kp = %f, Kd = %f\r\n", From_5000_To_6000_ServoPD.Kp, From_5000_To_6000_ServoPD.Kd);
    printf("From_6000_To_7000_ServoPD: Kp = %f, Kd = %f\r\n", From_6000_To_7000_ServoPD.Kp, From_6000_To_7000_ServoPD.Kd);
    printf("From_7000_To_8000_ServoPD: Kp = %f, Kd = %f\r\n", From_7000_To_8000_ServoPD.Kp, From_7000_To_8000_ServoPD.Kd);
    printf("From_8000_To_9000_ServoPD: Kp = %f, Kd = %f\r\n", From_8000_To_9000_ServoPD.Kp, From_8000_To_9000_ServoPD.Kd);
    printf("From_9000_To_9900_ServoPD: Kp = %f, Kd = %f\r\n", From_9000_To_9900_ServoPD.Kp, From_9000_To_9900_ServoPD.Kd);
    printf("Encoder0100_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0100_ServoPD.Kp, Encoder0100_ServoPD.Kd);
    printf("Encoder0200_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0200_ServoPD.Kp, Encoder0200_ServoPD.Kd);
    printf("Encoder0300_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0300_ServoPD.Kp, Encoder0300_ServoPD.Kd);
    printf("Encoder0400_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0400_ServoPD.Kp, Encoder0400_ServoPD.Kd);
    printf("Encoder0500_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0500_ServoPD.Kp, Encoder0500_ServoPD.Kd);
    printf("Encoder0600_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0600_ServoPD.Kp, Encoder0600_ServoPD.Kd);
    printf("Encoder0700_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0700_ServoPD.Kp, Encoder0700_ServoPD.Kd);
    printf("Encoder0800_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0800_ServoPD.Kp, Encoder0800_ServoPD.Kd);
    printf("Encoder0900_ServoPD: Kp = %f, Kd = %f\r\n", Encoder0900_ServoPD.Kp, Encoder0900_ServoPD.Kd);
    printf("Encoder1000_ServoPD: Kp = %f, Kd = %f\r\n", Encoder1000_ServoPD.Kp, Encoder1000_ServoPD.Kd);
    printf("Encoder1100_ServoPD: Kp = %f, Kd = %f\r\n", Encoder1100_ServoPD.Kp, Encoder1100_ServoPD.Kd);
    printf("Encoder1200_ServoPD: Kp = %f, Kd = %f\r\n", Encoder1200_ServoPD.Kp, Encoder1200_ServoPD.Kd);
    printf("Snack_Advance: %f\r\n", Snack_Advance);
    printf("Snack_Back: %f\r\n", Snack_Back);

    for(int16 i = 0; i < NUM_GPS_DATA; i++)
    {
        if(GpsDistance[i] != 0)
        {
            printf("GpsDistance[%d]:%f\r\n", i, GpsDistance[i]);
        }
    }
    for(int16 i = 0; i < NUM_GPS_DATA; i++)
    {
        if(GpsTgtEncod[i] != 0)
        {
            printf("GpsTgtEncod[%d]:%d\r\n", i, GpsTgtEncod[i]);
        }
    }
    LED_Buzzer_Flag_Ctrl(LED3); // 暂时替换
    system_delay_ms(1000);
}
