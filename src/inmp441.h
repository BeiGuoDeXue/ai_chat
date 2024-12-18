/**
 * @file inmp441.h
 * @brief 定义INMP441麦克风相关的常量和函数
 */

#ifndef INMP441_H
#define INMP441_H

#include <Arduino.h>

// I2S 引脚定义
#define INMP441_WS  4    // Word Select (WS) 引脚，也称为 LRCLK
#define INMP441_SCK 5    // Serial Clock (SCK) 引脚，也称为 BCLK
#define INMP441_SD  6    // Serial Data (SD) 引脚，用于数据输入

// 采样率设置（Hz）
#define INMP441_SAMPLE_RATE 16000

// 噪声阈值，低于此值的声音将被过滤，需要根据实际环境调整
#define NOISE_THRESHOLD 500

int inmp441_init();
void stop_mic();
void start_mic();
void inmp441_deinit();
int16_t read_mic_data(int16_t *data_in, int data_len);
void mic_data_handle(int16_t *data_in, size_t bytes_read);



#endif // INMP441_H
