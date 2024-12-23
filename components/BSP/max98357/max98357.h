#ifndef _MAX98357_H_
#define _MAX98357_H_

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * MAX98357A I2S 功放模块引脚定义
 * 该模块用于将I2S数字音频信号转换为模拟信号并放大输出
 */

// I2S通信引脚定义
#define MAX98357_LRC 18  // LRCLK: 左/右声道时钟信号，用于区分左右声道数据
#define MAX98357_BCLK 17 // BCLK: 位时钟信号，控制数据传输的速率
#define MAX98357_DIN 16  // DIN: 数据输入引脚，用于传输音频数据

#define MAX98357_SD_MODE_PIN GPIO_NUM_3 // SD_MODE: 控制芯片的开关，HIGH-使能，LOW-关闭

// 音频采样率设置 (Hz)
#define MAX98357_SAMPLE_RATE 16000

/**
 * 初始化MAX98357A模块
 * 配置I2S接口和GPIO引脚
 * @return ESP_OK: 初始化成功
 *         其他: 初始化失败的错误代码
 */
esp_err_t max98357_init(void);

/**
 * 向MAX98357A写入音频数据
 * @param data_in 要播放的音频数据缓冲区
 * @param bytes_to_write 要写入的字节数
 * @param bytes_written 实际写入的字节数
 * @return ESP_OK: 写入成功
 *         ESP_ERR_INVALID_STATE: 模块未初始化
 *         ESP_ERR_INVALID_ARG: 参数无效
 *         ESP_ERR_TIMEOUT: 写入超时
 */
esp_err_t write_max98357_data(int16_t *data_in, size_t bytes_to_write, size_t *bytes_written);

/**
 * 关闭MAX98357A输出
 * 通过SD_MODE引脚将芯片置于关闭状态
 * @return ESP_OK: 关闭成功
 *         ESP_ERR_INVALID_STATE: 模块未初始化
 */
esp_err_t max98357_close(void);

/**
 * 打开MAX98357A输出
 * 通过SD_MODE引脚使能芯片
 * @return ESP_OK: 打开成功
 *         ESP_ERR_INVALID_STATE: 模块未初始化
 */
esp_err_t max98357_open(void);

#endif
