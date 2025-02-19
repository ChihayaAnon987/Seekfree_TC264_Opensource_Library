/*
 * Camera.c
 *
 *  Created on: 2025年2月18日
 *      Author: 20483
 */

#include "zf_common_headfile.h"

uint8 block_size = 5;
float clip_value = 15;




int   LeftLine    [MT9V03X_H][2];
int   RightLine   [MT9V03X_H][2];
float CenterLine  [MT9V03X_H][2];
int   LeftLine_x  [MT9V03X_H];
int   LeftLine_y  [MT9V03X_H];
int   RightLine_x [MT9V03X_H];
int   RightLine_y [MT9V03X_H];
float CenterLine_x[MT9V03X_H];
float CenterLine_y[MT9V03X_H];
int   LeftLineNum;
int   RightLineNum;
int   CenterLineNum;


// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];




//-------------------------------------------------------------------------------------------------------------------
//  @brief      返回图片(x,y)的灰度值
//  @param      x                x坐标
//  @param      y                y坐标
//  @return     int型            图片(x,y)的灰度值
//  @since
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int AT_IMAGE(int x, int y)
{
    return mt9v03x_image[IntClip(y, 0, MT9V03X_H - 1)][IntClip(x, 0, MT9V03X_W - 1)];
}

uint8_t adaptiveThresholdPoint(int x, int y, float block_size, float clip_value)
{
    // block必须为奇数
    // 取中间像素点
    int half_block = block_size / 2;
    // 计算局部阈值
    int thres = 0;
    for(int dy = -half_block; dy <= half_block; dy++)
    {
        for(int dx = -half_block; dx <= half_block; dx++)
        {
            thres += AT_IMAGE(x + dx, y + dy);
        }
    }
    thres = thres / (block_size * block_size) - clip_value;
    // 进行二值化
    return (AT_IMAGE(x, y) > thres ? IMG_WHITE  : IMG_BLACK);
}


void Process_Image()
{
    int x1 = MT9V03X_W / 2, y1 = MT9V03X_W - 10;
    LeftLineNum = sizeof(LeftLine) / sizeof(LeftLine[0]);
    for (; x1 > (block_size / 2 + 1); x1--)
    {
        if (adaptiveThresholdPoint(x1 - 1, y1, block_size, clip_value) == IMG_BLACK) 
        {
            break;
        }
    }
    if (x1 == (block_size / 2 + 1))
    {
        for (; y1 > (block_size / 2 + 1); y1--)
        {
            if (adaptiveThresholdPoint(x1, y1 - 1, block_size, clip_value) == IMG_BLACK) break; // 左边线没有找到，则向上方寻找
        }
    }
    // 迷宫法求左边线
    if (adaptiveThresholdPoint(x1, y1, block_size, clip_value) == IMG_WHITE)
    {
        findline_lefthand_adaptive(x1, y1, block_size, clip_value, LeftLine, &LeftLineNum);
    }
    else
    {
        LeftLineNum = 0;
    }
    // findline_lefthand_adaptive(x1, y1, block_size, clip_value, LeftLine, &LeftLineNum);


    for(int i = 0; i < LeftLineNum; i++)
    {
        LeftLine_x[i] = LeftLine[i][0];
        LeftLine_y[i] = LeftLine[i][1];
    }
}



const int dir_front[4][2]      = {{0,  -1},  {1,  0},  {0,  1},  {-1,  0}};
const int dir_frontleft[4][2]  = {{-1, -1},  {1, -1},  {1,  1},  {-1,  1}};
const int dir_frontright[4][2] = {{1,  -1},  {1,  1},  {-1, 1},  {-1, -1}};


//-------------------------------------------------------------------------------------------------------------------
//  @brief      左手迷宫巡线
//  @param      x                     起始点横坐标
//  @param      y                     起始点纵坐标
//  @param      block_size            巡线区域大小
//  @param      clip_value            阈值修正值 
//  @param      pts[][2]              左边线数组
//  @param      *num                  左边线数组大小
//  @return     void
//  @since      
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void findline_lefthand_adaptive(int x, int y, int block_size, int clip_value, int pts[][2], int *num) {

    // 计算局部阈值
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while (step < *num && half < x && x <  MT9V03X_W - half - 1 && half < y && y < MT9V03X_H - half - 1 && turn < 4) {
        // 计算局部阈值
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(x + dx, y + dy);
            }
        }
        local_thres /= (block_size * block_size);
        local_thres -= clip_value;

        int current_value = AT_IMAGE(x, y);
        int front_value = AT_IMAGE(x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = AT_IMAGE(x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    } 
    *num = step;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      右手迷宫巡线
//  @param      x                     起始点横坐标
//  @param      y                     起始点纵坐标
//  @param      block_size            巡线区域大小
//  @param      clip_value            阈值修正值 
//  @param      pts[][2]              右边线数组
//  @param      *num                  右边线数组大小
//  @return     void
//  @since      
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void findline_righthand_adaptive(int x, int y, int block_size, int clip_value, int pts[][2], int *num) {

    // 计算局部阈值
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while (step < *num && half < x && x < MT9V03X_W - half - 1 && half < y && y < MT9V03X_H - half - 1 && turn < 4) {
        // 计算局部阈值
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT_IMAGE(x + dx, y + dy);
            }
        }
        local_thres /= (block_size * block_size);
        local_thres -= clip_value;
        
        int current_value = AT_IMAGE(x, y);
        int front_value = AT_IMAGE(x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT_IMAGE(x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取偏差角数组
//  @param      pts_in[][2]            中线数组
//  @param      num1                   中线数组大小
//  @param      angle_out[]            偏差角数组
//  @param      *num2                  偏差角数组大小
//  @return     void
//  @since      
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_error(float pts_in[][2],int num1, float angle_out[])
{
    // 获取起点坐标
    float begin_x = pts_in[0][0];
    float begin_y = pts_in[0][1];
    // 计算偏差角
    for (int i = 0; i < num1; i++)
    {
        if (pts_in[i][0] != 0 && pts_in[i][1] != 0)
        {
            float dx = pts_in[i][0] - begin_x;
            float dy = pts_in[i][1] - begin_y;
            angle_out[i] = atan2(dx, -dy) * 180 / PI;
        }
        else
        {
            angle_out[i] = 0;
        }
    }

}
