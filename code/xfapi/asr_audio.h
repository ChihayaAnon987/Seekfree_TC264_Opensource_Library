#include "zf_common_headfile.h"
#include "hmac_sha256.h" 
#include "base64.h"  
#include "asr_ctrl.h"

extern uint8       audio_start_flag;                                      // 语音识别开始标志位
extern uint8       audio_server_link_flag;                                // 语音识别连接服务器标志位

void audio_init(void);
void audio_callback(void);
void audio_loop(void);
