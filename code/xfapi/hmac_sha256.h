#ifndef _HMAC_SHA256_H_
#define _HMAC_SHA256_H_

#include <stdint.h>

#define SHA256_BLOCKLEN  64ul // size of message block buffer
#define SHA256_DIGESTLEN 32ul // size of digest in uint8_t
#define SHA256_DIGESTINT 8ul  //size of digest in uint32_t

typedef struct sha256_ctx_t {
    uint64_t len;                 // processed message length
    uint32_t h[8];                // hash state (SHA256_DIGESTINT = 8)
    uint8_t buf[SHA256_BLOCKLEN]; // message block buffer
} SHA256_CTX;

typedef struct hmac_sha256_ctx_t {
    uint8_t buf[SHA256_BLOCKLEN]; // key block buffer
    uint32_t h_inner[8];
    uint32_t h_outer[8];
    SHA256_CTX sha;
} HMAC_SHA256_CTX;

void hmac_sha256(const uint8_t *key, uint32_t keylen, 
                const uint8_t *data, uint32_t datalen,
                uint8_t *digest);
                
#endif // _HMAC_SHA256_H_