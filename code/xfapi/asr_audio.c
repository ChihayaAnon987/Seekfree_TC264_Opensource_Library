#include "zf_common_headfile.h"

#define     ASR_SEND_DATA_MAX_LENTH     (4000)                          // 语音识别发送的最长语音数据

#pragma section all "cpu0_dsram"
char        time_now_data[32];                                          // 当前时间戳
char        asr_url_out[512];                                           // websocket_url
uint8       wifi_uart_get_data_buffer[1024];                            // wifi接收缓冲区
char        http_request[1024];                                         // http协议缓冲区

int16       adc_get_one_data;                                           // 获取麦克风一次的ADC值
int         audio_get_count = -1;                                       // 音频数据计数

fifo_struct adc_data_fifo;                                              // 麦克风数据fifo
uint32      fifo_data_count;                                            // fifo一次接收数据变量
int16       adc_get_data[ASR_SEND_DATA_MAX_LENTH * 2];                  // fifo接收数据缓冲区
int16       fifo_get_data[ASR_SEND_DATA_MAX_LENTH];                     // fifo获取缓冲区

char        json_data[ASR_SEND_DATA_MAX_LENTH * 3];                     // json发送缓冲区
char        temp_data[ASR_SEND_DATA_MAX_LENTH * 3 + 2000];              // 临时数据缓冲区
uint8       websocket_receive_buffer[4096];                             // 接收websocket数据

uint8       audio_start_flag = 0;                                       // 语音识别开始标志位
uint8       audio_need_net_flag = 0;                                    // 语音识别需要联网标志位
uint8       audio_server_link_flag = 0;                                 // 语音识别连接服务器标志位
uint8       audio_send_data_flag = 0;                                   // 语音识别发送数据标志位
uint32      asr_max_time = 0;

#pragma section all restore
// 解析时间戳
void time_parse_data(char* input, char* time_str)
{
    const char* dateField = strstr(input, "Date: ");
    if(dateField == NULL) return;
    dateField += 6;
    const char* end = strchr(dateField, '\n');
    if(end == NULL) return;
    int length = end - dateField;
    strncpy(time_str, dateField, length);
    time_str[length - 1] = '\0';
}

// 获取时间戳
void time_get_data()
{
    while(wifi_uart_init(ASR_WIFI_SSID, ASR_WIFI_PASSWORD, WIFI_UART_STATION))
    {
        system_delay_ms(500);
    }
    while(wifi_uart_connect_tcp_servers(
                ASR_TARGET_IP,
                ASR_TARGET_PORT,
                WIFI_UART_COMMAND))
    {
        printf("\r\n Connect TCP Servers error, try again.");
        system_delay_ms(500);
    }
    memset(http_request, 0, sizeof(http_request));
    snprintf(http_request, sizeof(http_request), "1");

    while(wifi_uart_send_buffer((const uint8*)http_request, strlen(http_request)));
    memset(wifi_uart_get_data_buffer, 0, sizeof(wifi_uart_get_data_buffer));
    system_delay_ms(1000);
    wifi_uart_read_buffer(wifi_uart_get_data_buffer, sizeof(wifi_uart_get_data_buffer));
    time_parse_data((char*)wifi_uart_get_data_buffer, time_now_data);
//    printf("now_time: %s\r\n", time_now_data);
    wifi_uart_disconnect_link();
}

// 解析语音识别字段
void extract_content_fields(const char* input, char* output)
{
    const char* ptr = input;
    char w_value[256];
    while(*ptr)
    {
        // 查找 "w":" 模式
        const char* w_start = strstr(ptr, "\"w\":\"");
        if(!w_start)
        {
            break;
        }
        w_start += 5; // 跳过 "\"w\":\""
        const char* w_end = strchr(w_start, '"');
        if(!w_end)
        {
            break;
        }
        // 提取 w 字段的值
        int length = w_end - w_start;
        strncpy(w_value, w_start, length);
        w_value[length] = '\0';

        // 打印 w 字段的值
        // 拼接结果到 output
        strcat(output, w_value);
        printf("%s", w_value);
        ptr = w_end + 1;
    }
}

// 计算websocket url
void websocket_get_url(const char* base_url, char* url_out)
{
    uint8_t APISecret[] = {ASR_APISecret};
    uint8_t APIKey[] = {ASR_APIKey};
    uint8_t indata[128] = {0};
    uint8_t sha256_out[128];
    char base64_out[512];

    char host_domain[256] = {0};
    char path[256] = {0};

    const char* domain_start = strstr(base_url, "://");
    if(domain_start)
    {
        domain_start += 3;
        const char* path_start = strchr(domain_start, '/');
        if(path_start)
        {
            strncpy(host_domain, domain_start, path_start - domain_start);
            host_domain[path_start - domain_start] = '\0';
            strcpy(path, path_start);
        }
    }

    snprintf((char*)indata, sizeof(indata),
             "host: %s\n"
             "date: %s\n"
             "GET %s HTTP/1.1", host_domain, time_now_data, path);
    
    hmac_sha256(APISecret, strlen((char*)APISecret),
                indata, strlen((char*)indata),
                sha256_out);

    base64_encode((uint8_t*)sha256_out, base64_out, SHA256_DIGESTLEN);

    char authorization_origin[256] = {0};
    snprintf(authorization_origin, sizeof(authorization_origin),
             "api_key=\"%s\", algorithm=\"hmac-sha256\", headers=\"host date request-line\", signature=\"%s\"",
             APIKey, base64_out);
    base64_encode((uint8_t*)authorization_origin, base64_out, strlen(authorization_origin));
    char encoded_authorization[256];
    url_encode(base64_out, encoded_authorization);
    char encoded_date[256];
    url_encode(time_now_data, encoded_date);
    char encoded_host[256];
    url_encode(host_domain, encoded_host);
    uint16 total_len = strlen(base_url) + strlen(encoded_authorization) + strlen(encoded_date) + strlen(encoded_host) + 100;
    snprintf(url_out, total_len, "%s?authorization=%s&date=%s&host=%s",
             base_url, encoded_authorization, encoded_date, encoded_host);
//    printf("WebSocket URL: %s \t %d\n", url_out, strlen(url_out));
}

// 语音识别定时器回调函数
void audio_callback()
{
    if(gpio_get_level(ASR_BUTTON) == 0 && !audio_start_flag && audio_get_count == -1 && func_index == enum_third_menu10)
    {
        audio_get_count = 0;
        audio_server_link_flag = 0;
        audio_start_flag = 1;
        asr_max_time = 0;
        printf("正在连接服务器...\r\n");
    }
    if(audio_start_flag)
    {
        adc_get_one_data = (adc_convert(ASR_AUDIO_ADC) - 1870);
        if(audio_server_link_flag)
        {
            audio_get_count++;
            fifo_write_buffer(&adc_data_fifo, &adc_get_one_data, 1);
        }
        if((audio_get_count + 1) % (ASR_SEND_DATA_MAX_LENTH - 500) == 0)
        {
            if(audio_server_link_flag)
            {
                asr_max_time += (ASR_SEND_DATA_MAX_LENTH - 500);
                audio_send_data_flag = 1;
            }
        }
    }
    if((gpio_get_level(ASR_BUTTON) == 0 || asr_max_time > (60 * 8000)) && audio_server_link_flag)
    {
        audio_start_flag = 0;
        audio_get_count = 0;
        audio_send_data_flag = 0;
        audio_need_net_flag = 1;
    }
}

// 语音数据发送
void audio_data_send(uint8 status)
{
    if(status != 0)
    {
        fifo_data_count = fifo_used(&adc_data_fifo);
        if(fifo_data_count > ASR_SEND_DATA_MAX_LENTH)
        {
            fifo_data_count = ASR_SEND_DATA_MAX_LENTH;
        }
        memset(fifo_get_data, 0, sizeof(fifo_get_data));
        fifo_read_buffer(&adc_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
        memset(temp_data, 0, sizeof(temp_data));
        base64_encode((uint8*)fifo_get_data, temp_data, fifo_data_count * 2);
        memset(json_data, 0, sizeof(json_data));
    }
    else
    {
        memset(temp_data, 0, sizeof(temp_data));
    }
    snprintf(json_data, (ASR_SEND_DATA_MAX_LENTH * 3),
             "{  \"common\":                                                                                                            \
                        {\"app_id\": \"%s\"},                                                                                           \
                    \"business\":                                                                                                       \
                        {\"domain\": \"iat\", \"language\": \"zh_cn\", \"accent\": \"mandarin\", \"vinfo\": 1, \"vad_eos\": 10000},     \
                    \"data\":                                                                                                           \
                        {\"status\": %d, \"format\": \"audio/L16;rate=8000\", \"audio\": \"%s\", \"encoding\": \"raw\"}}",              \
             ASR_APIID, status, temp_data);
}

// 语音识别处理
void audio_loop()
{
    if(audio_start_flag && !audio_server_link_flag)
    {
        // 连接服务器
        while(!websocket_client_connect(asr_url_out))
        {
            printf("服务器连接失败\n");
            system_delay_ms(500); // 初始化失败，等待 500ms
        }
        // 语音数据第一帧数据包
        audio_data_send(0);
        // 发送第一帧数据包
        websocket_client_send((uint8_t*)json_data, strlen(json_data));
        audio_server_link_flag = 1;
        printf("服务器连接成功，开始识别语音，最长识别时长为60秒，可手动按键停止\r\n");
        printf("语音识别结果:");
    }
    if(audio_start_flag && audio_server_link_flag && audio_send_data_flag)
    {
        audio_send_data_flag = 0;
        // 语音数据中间帧数据包
        audio_data_send(1);
        // 发送中间帧数据包
        websocket_client_send((uint8_t*)json_data, strlen(json_data));
        // 等待语音结果
        system_delay_ms(300);
        // 接收语音结果
        websocket_client_receive(websocket_receive_buffer);
        // 解析语音结果
        extract_content_fields((const char*)websocket_receive_buffer, Dictation_Result);
    }
    if(audio_need_net_flag && !audio_send_data_flag)
    {
        // 语音数据结束帧数据包
        audio_data_send(2);
        // 发送结束帧数据包
        websocket_client_send((uint8_t*)json_data, strlen(json_data));
        // 等待语音结果
        system_delay_ms(1000);
        // 接收语音结果
        websocket_client_receive(websocket_receive_buffer);
        // 解析语音结果
        extract_content_fields((const char*)websocket_receive_buffer, Dictation_Result);

        if(asr_max_time > (60 * 8000))
        {
            printf("\r\n语音识别达到最大时长60秒\r\n");
            asr_max_time = 0;
        }
        pit_disable(ASR_PIT);
        while(wifi_uart_init(ASR_WIFI_SSID, ASR_WIFI_PASSWORD, WIFI_UART_STATION))
        {
            system_delay_ms(500); // 初始化失败，等待 500ms
        }
        pit_enable(ASR_PIT);
        printf("\r\n语音识别完成，可再次按下按键开始识别\r\n");
        audio_get_count = -1;
        audio_need_net_flag = 0;
    }
}

// 语音识别初始化
void audio_init()
{
    printf("等待初始化...\r\n");
    adc_init(RANDOM_NUM_ADC, ADC_12BIT);                                                            // 使用ADC生成随机数
    adc_init(ASR_AUDIO_ADC, ADC_12BIT);                                                             // 初始化麦克风ADC
    // gpio_init(ASR_BUTTON, GPI, GPIO_HIGH, GPI_PULL_UP);                                             // 初始话录音按键

    time_get_data();                                                                                // 获取当前时间戳
    websocket_get_url(ASR_DOMAIN, asr_url_out);                                                     // 计算websocket_url，需要利用当前时间戳

    pit_us_init(ASR_PIT, 125);                                                                      // 开启麦克风接收定时器，频率为8K
    fifo_init(&adc_data_fifo, FIFO_DATA_16BIT, adc_get_data, (ASR_SEND_DATA_MAX_LENTH * 2));        // 初始化音频接收fifo

    pit_disable(ASR_PIT);                                                                           // 连接wifi前先关闭定时器
    while(wifi_uart_init(ASR_WIFI_SSID, ASR_WIFI_PASSWORD, WIFI_UART_STATION))                      // 连接 WiFi 模块到指定的 WiFi 网络
    {
        printf("wifi连接失败，开始重连...\r\n");
        system_delay_ms(500);                                                                       // 初始化失败，等待 500ms
    }
    pit_enable(ASR_PIT);                                                                            // 连接wifi后打开定时器
    printf("按下P11.3按键开始连接服务器，连接完成后开始识别，开始识别后再次按下按键停止识别\r\n");    // 按下按键开始录音识别，最长仅能录制60秒
}
