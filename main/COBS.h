// COBS.h dùng cho telemetry
#ifndef COBS_H
#define COBS_H

#include <stdint.h>
#include <stddef.h>

// Mã hóa COBS, đầu ra tối đa len + (len/254) + 1
size_t cobs_encode(const uint8_t *in, size_t in_len, uint8_t *out);
// Giải mã COBS, trả về độ dài dữ liệu sau giải mã, -1 nếu lỗi
int cobs_decode(const uint8_t *in, size_t in_len, uint8_t *out);

#endif