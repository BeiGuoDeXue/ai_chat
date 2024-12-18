#include "inmp441.h"
#include <driver/i2s.h>
#include <hal/i2s_types.h>


// I2S驱动配置结构体
i2s_config_t inmp441_i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),              // 主模式 + 接收模式
    .sample_rate = INMP441_SAMPLE_RATE,                             // 采样率
    .bits_per_sample = i2s_bits_per_sample_t(16),                   // 16位采样精度
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                    // 仅使用左声道
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), // I2S标准通信格式
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                       // 中断优先级设置
    .dma_buf_count = 8,                                             // DMA缓冲区数量
    .dma_buf_len = 128,                                             // 每个DMA缓冲区长度
    .use_apll = false                                               // 不使用APLL
};

// I2S引脚配置结构体
const i2s_pin_config_t inmp441_gpio_config = {
    .bck_io_num = INMP441_SCK, // 位时钟引脚
    .ws_io_num = INMP441_WS,   // 字选择引脚
    .data_out_num = -1,        // 不使用数据输出
    .data_in_num = INMP441_SD  // 数据输入引脚
};

/**
 * @brief 初始化INMP441麦克风
 * @return ESP_OK: 成功, 其他: 失败
 */
int inmp441_init()
{
    esp_err_t ret;

    // 安装I2S驱动
    ret = i2s_driver_install(I2S_NUM_0, &inmp441_i2s_config, 0, NULL);
    if (ret != ESP_OK)
    {
        Serial.println("i2s_driver_install failed");
        return ret;
    }
    // 设置I2S引脚
    ret = i2s_set_pin(I2S_NUM_0, &inmp441_gpio_config);
    if (ret != ESP_OK)
    {
        i2s_driver_uninstall(I2S_NUM_0); // 清理已安装的驱动
        Serial.println("i2s_set_pin failed");
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief 停止麦克风采集
 */
void stop_mic()
{
    i2s_stop(I2S_NUM_0);
}

/**
 * @brief 启动麦克风采集
 */
void start_mic()
{
    i2s_start(I2S_NUM_0);
}

/**
 * @brief 读取麦克风数据
 * @param data_in 数据缓冲区指针
 * @param data_len 要读取的数据长度（字节）
 * @return 实际读取的字节数，-1表示错误
 */
int16_t read_mic_data(int16_t *data_in, int data_len)
{
    if (data_in == NULL || data_len <= 0)
    {
        return -1;
    }

    size_t bytes_read = 0;
    esp_err_t result = i2s_read(I2S_NUM_0, data_in, data_len, &bytes_read, portMAX_DELAY);
    if (result != ESP_OK)
    {
        Serial.printf("Error reading I2S data: %d\n", result);
        return -1;
    }
    return bytes_read;
}

/**
 * @brief 处理麦克风数据，包括降噪、音量调节等
 * @param data_in 输入数据缓冲区，处理后的数据也存储在此
 * @param bytes_read 数据字节数
 */
void mic_data_handle(int16_t *data_in, size_t bytes_read) {
    // 降噪处理
    // 处理音频数据
    int samples_read = bytes_read / 2; // 16位数据，每个样本2字节
    for(int i = 0; i < samples_read; i++) {
        // 1. 降噪处理
        if(abs(data_in[i]) < NOISE_THRESHOLD) {
            data_in[i] = 0;
        }
        
        // 2. 音量调节
        data_in[i] = data_in[i] * 4;
        
        // 3. 削波保护
        if(data_in[i] > 32767) data_in[i] = 32767;
        if(data_in[i] < -32767) data_in[i] = -32767;
    }
}

/**
 * @brief 释放INMP441相关资源
 */
void inmp441_deinit()
{
    i2s_stop(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
    Serial.println("INMP441 deinitialized");
}
