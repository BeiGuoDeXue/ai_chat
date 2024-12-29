/**
 * @file inmp441.h
 * @brief 定义INMP441麦克风相关的常量和函数
 */

#ifndef INMP441_H
#define INMP441_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

// // I2S 引脚定义
// #define INMP441_WS  4    // Word Select (WS) 引脚，也称为 LRCLK
// #define INMP441_SCK 5    // Serial Clock (SCK) 引脚，也称为 BCLK
// #define INMP441_SD  6    // Serial Data (SD) 引脚，用于数据输入

#define INMP441_SCK 9  // 时钟
#define INMP441_SD  10  // 数据
#define INMP441_WS  14  // 字选择


// 采样率设置（Hz）
#define INMP441_SAMPLE_RATE 16000

// 噪声阈值，低于此值的声音将被过滤，需要根据实际环境调整
#define NOISE_THRESHOLD 500

esp_err_t inmp441_init(void);
esp_err_t stop_mic(void);
esp_err_t start_mic(void);
esp_err_t inmp441_deinit(void);
esp_err_t read_mic_data(int16_t *data_in, int data_len, size_t *bytes_read);
void mic_data_handle(int16_t *data_in, size_t bytes_read);

#endif // INMP441_H
