#include <stdio.h>
#include <string.h>
#include <math.h>  // 添加数学库头文件
#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>
#include "esp_log.h"

static const char* TAG = "OPUS_TEST";

// 添加PI的定义（如果math.h中没有提供）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MIN(a,b) ((a) < (b) ? (a) : (b))

// 测试参数
#define SAMPLE_RATE 16000
#define CHANNELS 1
#define FRAME_SIZE 160  // 10ms at 16kHz
#define MAX_PACKET_SIZE 1500
#define TEST_DURATION_MS 1000  // 1秒的测试数据

void run_opus_test(void) {
    int err;
    
    // 创建编码器和解码器
    OpusEncoder* encoder = opus_encoder_create(SAMPLE_RATE, CHANNELS, OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK || encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder: %d", err);
        return;
    }

    OpusDecoder* decoder = opus_decoder_create(SAMPLE_RATE, CHANNELS, &err);
    if (err != OPUS_OK || decoder == NULL) {
        opus_encoder_destroy(encoder);
        ESP_LOGE(TAG, "Failed to create decoder: %d", err);
        return;
    }

    // 设置编码器参数
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(12000));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(4));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(encoder, OPUS_SET_VBR(1));

    // 打印编码器参数
    opus_int32 bitrate;
    opus_encoder_ctl(encoder, OPUS_GET_BITRATE(&bitrate));
    ESP_LOGI(TAG, "编码器比特率: %d", (int)bitrate);

    // 生成测试数据时打印一些样本
    ESP_LOGI(TAG, "生成测试信号...");
    int16_t input_signal[FRAME_SIZE];
    for (int i = 0; i < FRAME_SIZE; i++) {
        double angle = 2.0 * M_PI * 440.0 * i / SAMPLE_RATE;
        input_signal[i] = (int16_t)(32767.0 * sin(angle));
        if (i < 5) {
            ESP_LOGI(TAG, "测试信号[%d] = %d", i, input_signal[i]);
        }
    }

    // 编码缓冲区
    unsigned char encoded_packet[MAX_PACKET_SIZE];
    
    // 解码缓冲区
    int16_t decoded_signal[FRAME_SIZE];

    // 进行编解码测试
    ESP_LOGI(TAG, "开始编解码测试...");

    // 编码
    ESP_LOGI(TAG, "开始编码...");
    int encoded_bytes = opus_encode(encoder, input_signal, FRAME_SIZE, encoded_packet, MAX_PACKET_SIZE);
    if (encoded_bytes < 0) {
        ESP_LOGE(TAG, "编码错误: %d", encoded_bytes);
    } else {
        ESP_LOGI(TAG, "编码成功，编码后大小: %d bytes", encoded_bytes);
        ESP_LOGI(TAG, "编码数据:");
        for (int i = 0; i < MIN(encoded_bytes, 16); i++) {
            ESP_LOGI(TAG, "byte[%d] = 0x%02X", i, encoded_packet[i]);
        }

        // 解码
        ESP_LOGI(TAG, "开始解码...");
        int decoded_samples = opus_decode(decoder, encoded_packet, encoded_bytes, 
                                        decoded_signal, FRAME_SIZE, 0);
        
        if (decoded_samples < 0) {
            ESP_LOGE(TAG, "解码错误: %d", decoded_samples);
        } else {
            ESP_LOGI(TAG, "解码成功，解码后样本数: %d", decoded_samples);

            // 打印更多解码信息
            for (int i = 0; i < MIN(decoded_samples, 5); i++) {
                ESP_LOGI(TAG, "解码信号[%d] = %d", i, decoded_signal[i]);
            }

            // 确保解码样本数正确
            if (decoded_samples == FRAME_SIZE) {
                // 比较原始信号和解码后的信号
                int32_t max_diff = 0;
                int64_t total_diff = 0;
                
                for (int i = 0; i < FRAME_SIZE; i++) {
                    int32_t diff = abs((int32_t)input_signal[i] - (int32_t)decoded_signal[i]);
                    total_diff += diff;
                    if (diff > max_diff) max_diff = diff;
                }
                int32_t avg_diff = (int32_t)(total_diff / FRAME_SIZE);

                ESP_LOGI(TAG, "信号比较:");
                ESP_LOGI(TAG, "最大差异: %d", (int)max_diff);
                ESP_LOGI(TAG, "平均差异: %d", (int)avg_diff);
                
                // 打印部分样本值进行比较
                ESP_LOGI(TAG, "原始信号前5个样本: %d %d %d %d %d", 
                         input_signal[0], input_signal[1], input_signal[2], 
                         input_signal[3], input_signal[4]);
                ESP_LOGI(TAG, "解码信号前5个样本: %d %d %d %d %d", 
                         decoded_signal[0], decoded_signal[1], decoded_signal[2], 
                         decoded_signal[3], decoded_signal[4]);
            } else {
                ESP_LOGW(TAG, "解码样本数与输入不匹配: %d != %d", decoded_samples, FRAME_SIZE);
            }
        }
    }

    // 清理
    opus_encoder_destroy(encoder);
    opus_decoder_destroy(decoder);
    
    ESP_LOGI(TAG, "测试完成");
} 