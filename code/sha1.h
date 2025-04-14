#ifndef SHA1_H
#define SHA1_H

#include <stdint.h>
#include <stddef.h>

// 定义 SHA-1 哈希结果长度
#define SHA1_HASH_SIZE 20

// SHA-1 上下文结构体
typedef struct {
    uint32_t state[5];  // 当前哈希状态
    uint32_t count[2];  // 消息长度（以比特为单位）
    uint8_t buffer[64]; // 输入缓冲区
} sha1_context;

// 初始化 SHA-1 上下文
void sha1_init(sha1_context *ctx);

// 更新 SHA-1 上下文，处理输入数据
void sha1_update(sha1_context *ctx, const uint8_t *data, size_t len);

// 完成 SHA-1 计算并输出最终哈希值
void sha1_final(sha1_context *ctx, uint8_t digest[SHA1_HASH_SIZE]);

// 方便使用的单步函数：计算整个数据的 SHA-1 哈希值
void sha1_hash(const uint8_t *data, size_t len, uint8_t digest[SHA1_HASH_SIZE]);

#endif // SHA1_H