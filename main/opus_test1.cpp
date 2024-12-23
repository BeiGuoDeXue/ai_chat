#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "opus_encoder.h"
#include "opus_decoder.h"

static const char* TAG = "OPUS_TEST1";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MIN(a,b) ((a) < (b) ? (a) : (b))

// 测试参数
#define SAMPLE_RATE 16000
#define CHANNELS 1
#define DURATION_MS 60  // 改为60ms以匹配解码器的帧大小
#define FRAME_SIZE (SAMPLE_RATE * DURATION_MS / 1000)  // 960 samples

void run_opus_test(void) {
    ESP_LOGI(TAG, "开始编解码测试...");

    // 创建编码器和解码器
    OpusEncoderWrapper encoder(SAMPLE_RATE, CHANNELS, DURATION_MS);
    OpusDecoderWrapper decoder(SAMPLE_RATE, CHANNELS);

    // 生成测试数据 - 正弦波
    std::vector<int16_t> input_signal;
    input_signal.reserve(FRAME_SIZE);

    ESP_LOGI(TAG, "生成 %d 个采样点的测试数据 (%.1fms @ %dHz)", 
             FRAME_SIZE, (float)FRAME_SIZE * 1000 / SAMPLE_RATE, SAMPLE_RATE);
    
    for (int i = 0; i < FRAME_SIZE; i++) {
        double angle = 2.0 * M_PI * 440.0 * i / SAMPLE_RATE;  // 440Hz音调
        input_signal.push_back((int16_t)(32767.0 * sin(angle)));
    }

    // 保存一份原始信号的副本用于后续比较
    std::vector<int16_t> original_signal = input_signal;

    // 打印原始信号的前几个样本
    ESP_LOGI(TAG, "原始信号大小: %d 采样点", input_signal.size());
    ESP_LOGI(TAG, "原始信号前5个样本:");
    for (int i = 0; i < 5 && i < input_signal.size(); i++) {
        ESP_LOGI(TAG, "sample[%d] = %d", i, input_signal[i]);
    }

    // 编码
    ESP_LOGI(TAG, "开始编码...");
    std::vector<uint8_t> encoded_data;
    bool encode_success = false;

    encoder.Encode(std::move(input_signal), [&](std::vector<uint8_t>&& opus) {
        encoded_data = std::move(opus);
        encode_success = true;
        ESP_LOGI(TAG, "编码成功，编码后大小: %d bytes", encoded_data.size());
        
        // 打印编码数据的前几个字节
        if (!encoded_data.empty()) {
            ESP_LOGI(TAG, "编码数据前8字节:");
            for (int i = 0; i < 8 && i < encoded_data.size(); i++) {
                ESP_LOGI(TAG, "byte[%d] = 0x%02X", i, encoded_data[i]);
            }
        }
    });

    if (!encode_success) {
        ESP_LOGE(TAG, "编码失败");
        return;
    }

    // 解码
    ESP_LOGI(TAG, "开始解码...");
    std::vector<int16_t> decoded_signal;
    bool decode_success = decoder.Decode(std::move(encoded_data), decoded_signal);

    if (!decode_success) {
        ESP_LOGE(TAG, "解码失败");
        return;
    }

    ESP_LOGI(TAG, "解码成功，解码后样本数: %d", decoded_signal.size());
    
    // 打印解码后的前几个样本
    if (!decoded_signal.empty()) {
        ESP_LOGI(TAG, "解码信号前5个样本:");
        for (int i = 0; i < 5 && i < decoded_signal.size(); i++) {
            ESP_LOGI(TAG, "sample[%d] = %d", i, decoded_signal[i]);
        }
    }

    // 比较原始信号和解码后的信号
    if (original_signal.size() == decoded_signal.size()) {
        int32_t max_diff = 0;
        int64_t total_diff = 0;
        
        for (size_t i = 0; i < original_signal.size(); i++) {
            int32_t diff = abs((int32_t)original_signal[i] - (int32_t)decoded_signal[i]);
            total_diff += diff;
            if (diff > max_diff) max_diff = diff;
        }
        
        double avg_diff = (double)total_diff / original_signal.size();
        ESP_LOGI(TAG, "信号比较:");
        ESP_LOGI(TAG, "最大差异: %d", (int)max_diff);
        ESP_LOGI(TAG, "平均差异: %.2f", avg_diff);
        ESP_LOGI(TAG, "信噪比: %.2f dB", 
                 20 * log10(32767.0 / (avg_diff > 0 ? avg_diff : 1)));
    } else {
        ESP_LOGW(TAG, "输入和输出样本数不匹配: %d != %d", 
                 original_signal.size(), decoded_signal.size());
    }

    ESP_LOGI(TAG, "测试完成");
}