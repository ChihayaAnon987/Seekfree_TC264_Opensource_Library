#include "zf_common_headfile.h"

// SHA-1 常量
#define SHA1_K0 0x5A827999
#define SHA1_K1 0x6ED9EBA1
#define SHA1_K2 0x8F1BBCDC
#define SHA1_K3 0xCA62C1D6

// 循环左移操作
#define SHA1_ROTLEFT(a, b) (((a) << (b)) | ((a) >> (32 - (b))))

// SHA-1 基本函数
#define SHA1_F1(B, C, D) ((B & C) | ((~B) & D))
#define SHA1_F2(B, C, D) (B ^ C ^ D)
#define SHA1_F3(B, C, D) ((B & C) | (B & D) | (C & D))
#define SHA1_F4(B, C, D) (B ^ C ^ D)

// SHA-1 处理一个 512 位块
static void sha1_process_block(sha1_context* ctx, const uint8_t data[64])
{
    uint32_t W[80];
    uint32_t A, B, C, D, E, temp;
    int t;

    // 将 512 位数据分成 16 个 32 位字
    for(t = 0; t < 16; t++)
    {
        W[t] = (data[t * 4] << 24) | (data[t * 4 + 1] << 16) |
               (data[t * 4 + 2] << 8) | (data[t * 4 + 3]);
    }

    // 扩展为 80 个字
    for(t = 16; t < 80; t++)
    {
        W[t] = SHA1_ROTLEFT(W[t - 3] ^ W[t - 8] ^ W[t - 14] ^ W[t - 16], 1);
    }

    // 初始化哈希值
    A = ctx->state[0];
    B = ctx->state[1];
    C = ctx->state[2];
    D = ctx->state[3];
    E = ctx->state[4];

    // 主循环
    for(t = 0; t < 80; t++)
    {
        if(t < 20)
        {
            temp = SHA1_ROTLEFT(A, 5) + SHA1_F1(B, C, D) + E + W[t] + SHA1_K0;
        }
        else if(t < 40)
        {
            temp = SHA1_ROTLEFT(A, 5) + SHA1_F2(B, C, D) + E + W[t] + SHA1_K1;
        }
        else if(t < 60)
        {
            temp = SHA1_ROTLEFT(A, 5) + SHA1_F3(B, C, D) + E + W[t] + SHA1_K2;
        }
        else
        {
            temp = SHA1_ROTLEFT(A, 5) + SHA1_F4(B, C, D) + E + W[t] + SHA1_K3;
        }
        E = D;
        D = C;
        C = SHA1_ROTLEFT(B, 30);
        B = A;
        A = temp;
    }

    // 更新哈希值
    ctx->state[0] += A;
    ctx->state[1] += B;
    ctx->state[2] += C;
    ctx->state[3] += D;
    ctx->state[4] += E;
}

// 初始化 SHA-1 上下文
void sha1_init(sha1_context* ctx)
{
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count[0] = 0;
    ctx->count[1] = 0;
}

// 更新 SHA-1 上下文，处理输入数据
void sha1_update(sha1_context* ctx, const uint8_t* data, size_t len)
{
    uint32_t i, index, partLen;

    index = (ctx->count[0] >> 3) & 0x3F;
    ctx->count[0] += len << 3;
    if(ctx->count[0] < (len << 3))
    {
        ctx->count[1]++;
    }
    ctx->count[1] += len >> 29;

    partLen = 64 - index;

    if(len >= partLen)
    {
        memcpy(&ctx->buffer[index], data, partLen);
        sha1_process_block(ctx, ctx->buffer);
        for(i = partLen; i + 63 < len; i += 64)
        {
            sha1_process_block(ctx, &data[i]);
        }
        index = 0;
    }
    else
    {
        i = 0;
    }

    memcpy(&ctx->buffer[index], &data[i], len - i);
}

// 完成 SHA-1 计算并输出最终哈希值
void sha1_final(sha1_context* ctx, uint8_t digest[SHA1_HASH_SIZE])
{
    uint8_t finalcount[8];
    uint8_t c;

    for(int i = 0; i < 8; i++)
    {
        finalcount[i] = (uint8_t)((ctx->count[(i >= 4 ? 0 : 1)] >> ((3 - (i & 3)) * 8)) & 0xFF);
    }

    c = 0x80;
    sha1_update(ctx, &c, 1);

    while((ctx->count[0] & 0x1F8) != 0x1C0)
    {
        c = 0x00;
        sha1_update(ctx, &c, 1);
    }

    sha1_update(ctx, finalcount, 8);

    for(int i = 0; i < SHA1_HASH_SIZE; i++)
    {
        digest[i] = (uint8_t)((ctx->state[i >> 2] >> ((3 - (i & 3)) * 8)) & 0xFF);
    }
}

// 方便使用的单步函数：计算整个数据的 SHA-1 哈希值
void sha1_hash(const uint8_t* data, size_t len, uint8_t digest[SHA1_HASH_SIZE])
{
    sha1_context ctx;
    sha1_init(&ctx);
    sha1_update(&ctx, data, len);
    sha1_final(&ctx, digest);
}