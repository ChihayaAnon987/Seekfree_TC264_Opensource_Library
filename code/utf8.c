/*
 * utf8.c
 *
 *  Created on: 2025年4月14日
 *      Author: 20483
 */

#include "zf_common_headfile.h"


// char   Dictation_Result[1024] = {'\0'};    // 语音听写结果
char Dictation_Result[1024] = "顺时针，10，绕，1，2，3。";

FuzzyCommand fuzzyTable[] =
{
    {"打开双闪灯", FLASHING_LIGHT},
    {"双闪灯", FLASHING_LIGHT},
    {"打开左转灯", LEFTTURN_LIGHT},
    {"左转灯", LEFTTURN_LIGHT},
    {"打开右转灯", RIGHTURN_LIGHT},
    {"右转灯", RIGHTURN_LIGHT},
    {"打开近光灯", LOWBEAN_HLIGHT},
    {"近光灯", LOWBEAN_HLIGHT},
    {"打开远光灯", HIGBEAN_HLIGHT},
    {"远光灯", HIGBEAN_HLIGHT},
    {"打开雾灯", FOG_LIGHT},
    {"雾灯", FOG_LIGHT},
    {"向前直行十米", HEAD_STRAIGHT},
    {"向前直行10米", HEAD_STRAIGHT},
    {"向前直行", HEAD_STRAIGHT},
    {"直行十米", HEAD_STRAIGHT},
    {"后退直行十米", BACK_STRAIGHT},
    {"后退直行10米", BACK_STRAIGHT},
    {"直行十米", BACK_STRAIGHT},
    {"蛇形前进十米", SNAKE_ADVANCE},
    {"蛇形前进10米", SNAKE_ADVANCE},
    {"前进十米", SNAKE_ADVANCE},
    {"蛇形后退十米", SNAKE_BACK},
    {"蛇形后退10米", SNAKE_BACK},
    {"蛇形后退", SNAKE_BACK},
    {"逆时针转一圈", ROTATE_ANTICLOCK},
    {"逆时针", ROTATE_ANTICLOCK},
    {"顺时针转一圈", ROTATE_CLOCKWISE},
    {"顺时针", ROTATE_CLOCKWISE},
    {"停车区一", PARK_AREA_ONE},
    {"停车区二", PARK_AREA_TWO},
    {"停车区三", PARK_AREA_THREE},
};

void Recognize_Command()
{
    int16 FUZZY_COUNT = 32;
    int32 outputLen = strlen(Dictation_Result);
    int16 pos = 0;          // 当前扫描位置
    int16 cmdFound = 0;     // 已经找到的命令数
    printf("%s\n", Dictation_Result);
    while (cmdFound < ACTION_COUNT && pos < outputLen)
    {
        int16 bestOffset = 10000;
        int16 bestIndex = -1;
        // 遍历所有模糊匹配表的命令，查找离当前位置最近的匹配项
        for (int16 i = 0; i < FUZZY_COUNT; i++)
        {
            char *p = strstr(&Dictation_Result[pos], fuzzyTable[i].fuzzyStr);
            if (p != NULL)
            {
                int16 offset = p - &Dictation_Result[pos];
                // 如果该匹配比前面找到的更靠前，则更新
                if (offset < bestOffset)
                {
                    bestOffset = offset;
                    bestIndex = i;
                }
            }
            printf("%d\r\n", i);
        }

        // 如果没有匹配到任何命令，则结束解析
        if (bestIndex == -1)
        {
            printf("在位置 %d 无法匹配到任何命令，解析终止！\n", pos);
            break;
        }

        // 如果匹配不在当前位置，则说明前面的部分未识别
        if (bestOffset > 0)
        {
            printf("提示：在位置 %d 有 %d 个字符未能匹配，自动跳过这些字符。\n", pos, bestOffset);
            pos += bestOffset;
        }
        // 记录匹配到的命令的标志位
        Action_Flag[cmdFound] = fuzzyTable[bestIndex].flag;
        cmdFound++;

        // 更新 pos，跳过匹配到的字符串
        pos += strlen(fuzzyTable[bestIndex].fuzzyStr);
    }

    // 输出解析结果
    printf("解析得到的 Action_Flag 标志位如下：\n");
    for (int16 i = 0; i < cmdFound; i++)
    {
        printf("Action_Flag[%d] = %d\n", i, Action_Flag[i]);
    }
}

int raw = 0;  // 行
int column = 0;  // 列

// 声明处理函数类型
typedef void (*HandlerFunc)(const char*);

// 具体处理函数实现
void handle_da(const char *p)
{
    // 打
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese06[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_kai(const char *p)
{
    // 开
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese07[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_jin(const char *p)
{
    // 近
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese08[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_guang(const char *p)
{
    // 光
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese09[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_deng(const char *p)
{
    // 灯
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese10[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_yuan(const char *p)
{
    // 远
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese11[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_wu(const char *p)
{
    // 雾
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese12[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhi(const char *p)
{
    // 直
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese13[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_xing(const char *p)
{
    // 行
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese14[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhongwenshi(const char *p)
{
    // 十
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese15[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_mi(const char *p)
{
    // 米
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese16[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_hou(const char *p)
{
    // 后
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese17[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_tui(const char *p)
{
    // 退
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese18[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_she(const char *p)
{
    // 蛇
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese19[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_xing1(const char *p)
{
    // 形
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese20[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_qian(const char *p)
{
    // 前
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese21[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_jin1(const char *p)
{
    // 进
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese22[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_ni(const char *p)
{
    // 逆
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese23[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_shijian(const char *p)
{
    // 时
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese24[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhen(const char *p)
{
    // 针
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese25[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhuang(const char *p)
{
    // 转
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese26[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhongwenyi(const char *p)
{
    // 一
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese27[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_quan(const char *p)
{
    // 圈
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese28[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_shun(const char *p)
{
    // 顺
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese29[1], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_ting(const char *p)
{
    // 停
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese30[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_che(const char *p)
{
    // 车
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese31[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_qu(const char *p)
{
    // 区
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese32[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhongwener(const char *p)
{
    // 二
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese33[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zhongwensan(const char *p)
{
    // 三
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese34[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_xiang(const char *p)
{
    // 向
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese47[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_shuang(const char *p)
{
    // 双
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese49[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_shan(const char *p)
{
    // 闪
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese50[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_zuo(const char *p)
{
    // 左
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese51[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_you(const char *p)
{
    // 右
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese52[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_douhao(const char *p)
{
    // ，
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese56[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_juhao(const char *p)
{
    // 。
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese57[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

void handle_unknown(const char *p)
{
    // ??
    ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese99[0], 1, RGB565_CYAN);
    if(column == 6) raw++;
    column = (column + 1) % 7;
}

bool is_chinese(const char *p)
{
    return (*p & 0xF0) == 0xE0 && strlen(p) >= 3;
}

// 判断是否为 ASCII 数字
bool is_number(char c)
{
    return c >= '0' && c <= '9';
}

// 数字处理示例
void handle_number(const char *p)
{
    static int num_buffer = 0;
    num_buffer = num_buffer * 10 + (*p - '0');
    
    // 检测数字结束
    if (!is_number(*(p+1)))
    {
        // printf("获取到数值: %d\n", num_buffer);
        if(num_buffer == 1)
        {
            ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese53[0], 1, RGB565_CYAN);
        }
        if(num_buffer == 2)
        {
            ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese54[0], 1, RGB565_CYAN);
        }
        if(num_buffer == 3)
        {
            ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese55[0], 1, RGB565_CYAN);
        }
        if(num_buffer == 10)
        {
            ips200_show_chinese(5 + 32 * column, 45 + 32 * raw, 32, Chinese48[0], 1, RGB565_CYAN);
        }
        num_buffer = 0;  // 重置缓冲
        
        if(column == 6) raw++;
        column = (column + 1) % 7;
    }
}

// 预定义字符处理映射
struct
{
    const char *utf8;  // UTF-8编码
    HandlerFunc handler;
}
handlers[] =
{
    {"打", handle_da},
    {"开", handle_kai},
    {"近", handle_jin},
    {"光", handle_guang},
    {"灯", handle_deng},
    {"远", handle_yuan},
    {"雾", handle_wu},
    {"直", handle_zhi},
    {"行", handle_xing},
    {"十", handle_zhongwenshi},
    {"米", handle_mi},
    {"后", handle_hou},
    {"退", handle_tui},
    {"蛇", handle_she},
    {"形", handle_xing1},
    {"前", handle_qian},
    {"进", handle_jin1},
    {"逆", handle_ni},
    {"时", handle_shijian},
    {"针", handle_zhen},
    {"转", handle_zhuang},
    {"一", handle_zhongwenyi},
    {"圈", handle_quan},
    {"顺", handle_shun},
    {"停", handle_ting},
    {"车", handle_che},
    {"区", handle_qu},
    {"二", handle_zhongwener},
    {"三", handle_zhongwensan},
    {"向", handle_xiang},
    {"双", handle_shuang},
    {"闪", handle_shan},
    {"左", handle_zuo},
    {"右", handle_you},
    {"，", handle_douhao},
    {"。", handle_juhao},
    {"??", handle_unknown}
};

// 改进的字符处理逻辑
void process_string(const char *str)
{
    int index = 0;
    raw = 0;
    column = 0;
    while (str[index])
    {
        if (is_chinese(&str[index]))
        {
            bool matched = false;
            const int handler_count = sizeof(handlers)/sizeof(handlers[0]);
            // 遍历所有预定义处理项
            for (int i = 0; i < handler_count; i++)
            {
                if (memcmp(&str[index], handlers[i].utf8, 3) == 0)
                {
                    handlers[i].handler(&str[index]);
                    matched = true;
                    break;
                }
            }
            // 未匹配时执行最后一个处理器
            if (!matched && handler_count > 0)
            {
                handlers[handler_count - 1].handler(&str[index]);
            }
            index += 3;
        }
        else if(is_number(str[index]))
        {
            handle_number(&str[index]);
            index++;
        }
        else
        {
            index++;
        }
    }
}
