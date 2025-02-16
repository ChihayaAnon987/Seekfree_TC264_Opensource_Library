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
        for (int i = 0; i < Point_NUM; i++)
        {
            flash_union_buffer[i * 4 + 1].uint32_type = lat_union[i].uint32_type[0];  // 纬度高 32 位
            flash_union_buffer[i * 4 + 2].uint32_type = lat_union[i].uint32_type[1];  // 纬度低 32 位

            flash_union_buffer[i * 4 + 3].uint32_type = lon_union[i].uint32_type[0];  // 经度高 32 位
            flash_union_buffer[i * 4 + 4].uint32_type = lon_union[i].uint32_type[1];  // 经度低 32 位
        }
        // 写入 Flash 页面
        flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        flash_buffer_clear();

        LED_Buzzer_Flag_Ctrl(LED1);
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
        for(int i = 0; i < Point_NUM; i++)
        {
            if(flash_union_buffer[i * 4 + 1].uint32_type != 0)
            {
                if(flash_union_buffer[i * 4 + 2].uint32_type != 0)
                {
                    lat_union[i].uint32_type[0] = flash_union_buffer[i * 4 + 1].uint32_type;  // 纬度高 32 位
                    lat_union[i].uint32_type[1] = flash_union_buffer[i * 4 + 2].uint32_type;  // 纬度低 32 位
                    GPS_GET_LAT[i] = lat_union[i].double_type;                                // 纬度
                }
            }
            if(flash_union_buffer[i * 4 + 3].uint32_type != 0)
            {
                if(flash_union_buffer[i * 4 + 4].uint32_type != 0)
                {
                    lon_union[i].uint32_type[0] = flash_union_buffer[i * 4 + 3].uint32_type;  // 经度高 32 位
                    lon_union[i].uint32_type[1] = flash_union_buffer[i * 4 + 4].uint32_type;  // 经度低 32 位
                    GPS_GET_LOT[i] = lon_union[i].double_type;                                // 经度
                }
            }
        }
        flash_buffer_clear();                                                    // 清空缓冲区

        LED_Buzzer_Flag_Ctrl(LED1);
    }
//    Point_NUM = 6;
//    GPS_GET_LAT[0] = 22.590801;
//    GPS_GET_LAT[1] = 22.590741;
//    GPS_GET_LAT[2] = 22.590671;
//    GPS_GET_LAT[3] = 22.590666;
//    GPS_GET_LAT[4] = 22.590737;
//    GPS_GET_LAT[5] = 22.590791;
//
//    GPS_GET_LOT[0] = 113.961823;
//    GPS_GET_LOT[1] = 113.961822;
//    GPS_GET_LOT[2] = 113.961823;
//    GPS_GET_LOT[3] = 113.961864;
//    GPS_GET_LOT[4] = 113.961859;
//    GPS_GET_LOT[5] = 113.961853;
}

// 修正 Gps 数据
void FLASH_FIX_GPS()
{
    flash_buffer_clear();
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // 擦除这一页
    }

    for(int i = 0; i < Point_NUM; i++)
    {
        lat_union[i].double_type = GPS_GET_LAT[i];
        lon_union[i].double_type = GPS_GET_LOT[i];
    }
    for (int i = 0; i < Point_NUM; i++)
    {
        flash_union_buffer[i * 4 + 1].uint32_type = lat_union[i].uint32_type[0];  // 纬度高 32 位
        flash_union_buffer[i * 4 + 2].uint32_type = lat_union[i].uint32_type[1];  // 纬度低 32 位

        flash_union_buffer[i * 4 + 3].uint32_type = lon_union[i].uint32_type[0];  // 经度高 32 位
        flash_union_buffer[i * 4 + 4].uint32_type = lon_union[i].uint32_type[1];  // 经度低 32 位
    }
    // 写入 Flash 页面
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_buffer_clear();

    LED_Buzzer_Flag_Ctrl(LED1);
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

    // 指示灯亮，表明已读取
    LED_Buzzer_Flag_Ctrl(LED1);


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
        flash_buffer_clear();                                                    // 清空缓冲区
    }

}
