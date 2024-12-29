#include "max98357.h"
#include <driver/i2s.h>
#include <driver/gpio.h>
#include "esp_log.h"

static const char *TAG = "MAX98357";
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
    .dma_buf_count = 4,                                                   // DMA缓冲区数量
    .dma_buf_len = 960                                                    // 每个DMA缓冲区长度
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
esp_err_t max98357_init(void)
{
    if (is_max98357_initialized) {
        ESP_LOGW(TAG, "MAX98357 is already initialized");
        return ESP_OK;
    }

    // 配置 SD_MODE GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX98357_SD_MODE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO");
        return ret;
    }

    // 默认关闭功放
    gpio_set_level(MAX98357_SD_MODE_PIN, 0);

    ret = i2s_driver_install(I2S_NUM_1, &max98357_i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S driver");
        return ret;
    }

    ret = i2s_set_pin(I2S_NUM_1, &max98357_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S pins");
        i2s_driver_uninstall(I2S_NUM_1);
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
esp_err_t write_max98357_data(int16_t *data_in, size_t bytes_to_write, size_t *bytes_written)
{
    if (!is_max98357_initialized) {
        ESP_LOGE(TAG, "MAX98357 is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return i2s_write(I2S_NUM_1, data_in, bytes_to_write, bytes_written, portMAX_DELAY);
}

/**
 * 关闭MAX98357输出
 * 通过SD_MODE引脚将芯片置于低功耗模式
 */
esp_err_t max98357_close(void)
{
    if (!is_max98357_initialized) {
        ESP_LOGE(TAG, "MAX98357 is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return gpio_set_level(MAX98357_SD_MODE_PIN, 0);
}

/**
 * 打开MAX98357输出
 * 通过SD_MODE引脚使能芯片
 */
esp_err_t max98357_open(void)
{
    if (!is_max98357_initialized) {
        ESP_LOGE(TAG, "MAX98357 is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // esp_err_t ret = i2s_start(I2S_NUM_1);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // ret = i2s_zero_dma_buffer(I2S_NUM_1);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    return gpio_set_level(MAX98357_SD_MODE_PIN, 1);
}
