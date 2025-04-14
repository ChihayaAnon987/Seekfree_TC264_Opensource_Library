#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H
#include "zf_common_headfile.h"
#include "hmac_sha256.h" 
#include "base64.h"  



void url_encode(const char* str, char* encoded);
// 连接到 WebSocket 服务器
bool websocket_client_connect(const char* url);
// 发送数据到 WebSocket 服务器
bool websocket_client_send(const uint8_t* data, uint32_t len);

bool websocket_client_receive(uint8_t* buffer);
// 关闭 WebSocket 连接
void websocket_client_close();
// WebSocket 数据帧创建函数
uint32_t websocket_create_frame(uint8_t* frame, const uint8_t* payload, uint64_t payload_len, uint8_t type, bool mask);

#endif // WEBSOCKET_CLIENT_H
