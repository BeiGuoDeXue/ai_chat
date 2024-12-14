#include "max98357.h"
#include <driver/i2s.h>
#include <hal/i2s_types.h>

/* 变量区域 */
// 标记MAX98357模块的初始化状态
static bool is_max98357_initialized = false;

/* I2S配置区域 */

// I2S通信配置参数
i2s_config_t max98357_i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),                    // 主模式，仅发送数据
    .sample_rate = MAX98357_SAMPLE_RATE,                                  // 采样率16kHz
    .bits_per_sample = i2s_bits_per_sample_t(16),                         // 16位采样精度
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                          // 仅使用左声道
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_MSB), // 标准MSB格式
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                             // 中断优先级设置
    .dma_buf_count = 8,                                                   // DMA缓冲区数量
    .dma_buf_len = 128                                                    // 每个DMA缓冲区长度
};

// I2S引脚配置
const i2s_pin_config_t max98357_gpio_config = {
    .bck_io_num = MAX98357_BCLK,  // BCLK引脚
    .ws_io_num = MAX98357_LRC,    // LRCLK引脚
    .data_out_num = MAX98357_DIN, // 数据输出引脚
    .data_in_num = -1             // 不使用数据输入
};

/**
 * 初始化MAX98357模块
 * 配置GPIO和I2S驱动
 */
int max98357_init()
{
    // 检查是否已经初始化
    if (is_max98357_initialized)
    {
        Serial.println("max98357 is already initialized");
        return ESP_OK;
    }

    // 配置SD_MODE引脚为输出模式，并默认关闭功放
    pinMode(MAX98357_SD_MODE_PIN, OUTPUT);
    digitalWrite(MAX98357_SD_MODE_PIN, LOW); // 默认关闭状态

    esp_err_t ret;
    // 安装I2S驱动
    ret = i2s_driver_install(I2S_NUM_1, &max98357_i2s_config, 0, NULL);
    if (ret != ESP_OK)
    {
        Serial.println("i2s_driver_install failed");
        return ret;
    }

    // 设置I2S引脚
    ret = i2s_set_pin(I2S_NUM_1, &max98357_gpio_config);
    if (ret != ESP_OK)
    {
        Serial.println("i2s_set_pin failed");
        return ret;
    }
    is_max98357_initialized = true;
    return ESP_OK;
}

/**
 * 通过I2S接口写入音频数据
 * @param data_in 音频数据缓冲区
 * @param bytes_read 要写入的字节数
 * @return 实际写入的字节数，失败返回ESP_FAIL
 */
int write_max98357_data(int16_t *data_in, size_t bytes_read)
{
    // 检查初始化状态
    if (!is_max98357_initialized)
    {
        Serial.println("max98357 is not initialized");
        return ESP_FAIL;
    }

    size_t bytes_write = 0;
    // 通过I2S接口写入数据
    esp_err_t result = i2s_write(I2S_NUM_1, data_in, bytes_read, &bytes_write, portMAX_DELAY);
    if (result != ESP_OK)
    {
        Serial.println("Error writing I2S data");
        return ESP_FAIL;
    }
    return bytes_write;
}

/**
 * 关闭MAX98357输出
 * 通过SD_MODE引脚将芯片置于低功耗模式
 */
void max98357_close()
{
    if (!is_max98357_initialized)
    {
        Serial.println("max98357 is not initialized");
        return;
    }
    // 由于开关太快，因此这里只关功放，不清空缓冲区
    // 先清空缓冲区
    // i2s_zero_dma_buffer(I2S_NUM_1);

    // // 停止I2S驱动
    // i2s_stop(I2S_NUM_1);

    // 最后关闭功放
    digitalWrite(MAX98357_SD_MODE_PIN, LOW);
}

/**
 * 打开MAX98357输出
 * 通过SD_MODE引脚使能芯片
 */
void max98357_open()
{
    if (!is_max98357_initialized)
    {
        Serial.println("max98357 is not initialized");
        return;
    }
    
    // 先启动I2S驱动
    i2s_start(I2S_NUM_1);
    
    // 清空可能存在的残留数据
    i2s_zero_dma_buffer(I2S_NUM_1);
    
    // 最后打开功放
    digitalWrite(MAX98357_SD_MODE_PIN, HIGH);
}
