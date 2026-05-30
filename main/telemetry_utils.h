#pragma once

#include <stdint.h>
#include <stddef.h>

// ==========================================
// CRC16 CCITT
// ==========================================
inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// ==========================================
// COBS ENCODING / DECODING
// ==========================================
// Mã hóa COBS, đầu ra tối đa len + (len/254) + 1
inline size_t cobs_encode(const uint8_t *in, size_t in_len, uint8_t *out) {
    size_t out_idx = 1;  // byte đầu tiên chứa khoảng cách đến 0x00 tiếp theo
    size_t code_idx = 0;
    uint8_t code = 1;
    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == 0) {
            out[code_idx] = code;
            code = 1;
            code_idx = out_idx++;
        } else {
            out[out_idx++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_idx] = code;
                code = 1;
                code_idx = out_idx++;
            }
        }
    }
    out[code_idx] = code;
    return out_idx;
}

// Giải mã COBS, trả về độ dài dữ liệu sau giải mã, -1 nếu lỗi
inline int cobs_decode(const uint8_t *in, size_t in_len, uint8_t *out) {
    size_t out_idx = 0;
    size_t in_idx = 0;
    
    while (in_idx < in_len) {
        uint8_t code = in[in_idx++];
        
        // Code byte không bao giờ được phép bằng 0 trong chuẩn COBS
        if (code == 0) return -1; 
        
        for (uint8_t i = 1; i < code; i++) {
            // Fix Dữ liệu bị cắt cụt Overrun, đang đọc dở thì hết độ dài
            if (in_idx >= in_len) return -1; 
            out[out_idx++] = in[in_idx++];
        }
        
        if (code < 0xFF && in_idx < in_len) {
            out[out_idx++] = 0;
        }
    }
    return out_idx; // Ép kiểu ngầm định về int, trả về số byte hợp lệ
}

