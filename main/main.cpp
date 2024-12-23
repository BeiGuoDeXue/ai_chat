#include <stdio.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../components/BSP/inmp441/inmp441.h"
#include "../components/BSP/max98357/max98357.h"
#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>
#include <wifi_station.h>
#include <wifi_configuration_ap.h>
#include <esp_websocket_client.h>
#include "esp_timer.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define MAX_AUDIO_SAMPLES 1024
#define MAX_AUDIO_BUFFER_SIZE (MAX_AUDIO_SAMPLES * 2)
#define MAX_PACKET_SIZE 1024  // 适当调整包大小
#define OPUS_FRAME_SIZE 160  // 与服务器编码帧大小保持一致
#define OPUS_CODE_SIZE 20  // 与服务器编码帧大小保持一致
#define DECODE_BUFFER_SIZE (MAX_PACKET_SIZE * 6)  // 增大环形缓冲区大小

static const char *TAG = "MAIN";

// 音频数据结构
typedef struct {
    int16_t buffer[MAX_AUDIO_SAMPLES];  // 使用采样点数
    size_t size;
} audio_data_t;

typedef struct {
    uint8_t buffer[MAX_PACKET_SIZE];
    size_t size;
} encoded_data_t;

// WebSocket配置
static esp_websocket_client_handle_t client = NULL;
static const char* WS_URI = "ws://192.168.31.79:80/ws/esp32-client";

// WiFi配置
#define WIFI_SSID "2806"
#define WIFI_PASS "zhaokangxu"

// Opus编码器
static OpusEncoder *encoder = NULL;
static OpusDecoder *decoder = NULL;

// 队列句柄
static QueueHandle_t audioOutputQueue;
static QueueHandle_t wsOutQueue;
static QueueHandle_t audioEncodeQueue;
static QueueHandle_t audioDecodeQueue;

// 任务句柄
static TaskHandle_t audioInputTaskHandle;
static TaskHandle_t audioOutputTaskHandle;
static TaskHandle_t wsTaskHandle;

// 全局变量
static bool wsConnected = false;
static uint8_t encoded_data[MAX_PACKET_SIZE];
static int16_t decoded_buffer[DECODE_BUFFER_SIZE];

// 在文件开头添加环形缓冲区的结构定义
typedef struct {
    uint8_t *buffer;
    size_t size;
    size_t read_pos;
    size_t write_pos;
    size_t data_size;
} ring_buffer_t;

// 环形缓冲区操作函数
static void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, size_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->data_size = 0;
}

static size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, size_t len) {
    if (rb->data_size + len > rb->size) {
        return 0;  // 缓冲区空间不足
    }

    size_t first_chunk = MIN(len, rb->size - rb->write_pos);
    memcpy(rb->buffer + rb->write_pos, data, first_chunk);
    
    if (first_chunk < len) {
        // 需要回环到缓冲区开始处
        memcpy(rb->buffer, data + first_chunk, len - first_chunk);
    }
    
    rb->write_pos = (rb->write_pos + len) % rb->size;
    rb->data_size += len;
    return len;
}

static size_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, size_t len) {
    len = MIN(len, rb->data_size);
    if (len == 0) return 0;

    size_t first_chunk = MIN(len, rb->size - rb->read_pos);
    memcpy(data, rb->buffer + rb->read_pos, first_chunk);
    
    if (first_chunk < len) {
        // 需要回环到缓冲区开始处
        memcpy(data + first_chunk, rb->buffer, len - first_chunk);
    }
    
    rb->read_pos = (rb->read_pos + len) % rb->size;
    rb->data_size -= len;
    return len;
}

// 初始化Opus编码器和解码器
static esp_err_t init_opus(void) {
    int err;
    
    // 保持16kHz采样率
    encoder = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK || encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder: %d", err);
        return ESP_FAIL;
    }

    decoder = opus_decoder_create(16000, 1, &err);
    if (err != OPUS_OK || decoder == NULL) {
        opus_encoder_destroy(encoder);
        ESP_LOGE(TAG, "Failed to create decoder: %d", err);
        return ESP_FAIL;
    }

    // 因为帧变大，可以适当降低比特率
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(12000));  // 12kbps也能保证质量
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(5));
    opus_encoder_ctl(encoder, OPUS_SET_DTX(1));
    // opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    // opus_encoder_ctl(encoder, OPUS_SET_VBR(1));

    
    return ESP_OK;
}

// WebSocket事件处理
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    // 移动变量声明到 switch 语句之外
    const char* test_data = "hello";
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            wsConnected = true;
            // 连接成功后立即发送一些测试数据
            esp_websocket_client_send_text(client, test_data, strlen(test_data), portMAX_DELAY);
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            wsConnected = false;
            // 断开连接时清空所有队列
            xQueueReset(wsOutQueue);
            xQueueReset(audioOutputQueue);
            break;
            
        case WEBSOCKET_EVENT_DATA:
        {
            if (!wsConnected || !data->data_len) {
                ESP_LOGW(TAG, "WebSocket未连接或数据长度为0");
                return;
            }

            ESP_LOGI(TAG, "Received data len: %d", data->data_len);
            
            // 每次处理160字节
            const uint8_t* data_ptr = (const uint8_t*)data->data_ptr;
            size_t remaining = data->data_len;
            // TODO: 这了数据可能没发送完
            while (remaining >= OPUS_CODE_SIZE) {
                encoded_data_t encoded_data = {0};
                memcpy(encoded_data.buffer, data_ptr, OPUS_CODE_SIZE);
                encoded_data.size = OPUS_CODE_SIZE;
                
                // 发送到解码队列
                if (xQueueSend(audioDecodeQueue, &encoded_data, 0) != pdPASS) {
                    ESP_LOGW(TAG, "解码队列已满");
                    break;
                }
                
                data_ptr += OPUS_CODE_SIZE;
                remaining -= OPUS_CODE_SIZE;
            }
            break;
        }
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WEBSOCKET_EVENT_ERROR");
            if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "TCP transport error, errno: %d", errno);
            }
            break;

        case WEBSOCKET_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_BEFORE_CONNECT");
            break;

        default:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ANY");
            break;
    }
}

// 音频输入任务
static void audio_input_task(void *parameter)
{
    audio_data_t inputData;
    size_t bytes_read;
    
    while (1) {
        esp_err_t ret = read_mic_data(inputData.buffer, 
                                    sizeof(inputData.buffer), 
                                    &bytes_read);
        if (ret == ESP_OK && bytes_read > 0) {
            inputData.size = bytes_read;
            // mic_data_handle(inputData.buffer, inputData.size);
            // if (xQueueSend(audioEncodeQueue, &inputData, 0) != pdPASS) {
            //     ESP_LOGV(TAG, "Failed to queue input audio");
            // }
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 音频输出任务
static void audio_output_task(void *parameter)
{
    audio_data_t outputData;
    size_t bytes_written;
    
    while (1) {
        if (xQueueReceive(audioOutputQueue, &outputData, portMAX_DELAY)) {
            // ESP_LOGI(TAG, "len: %d", outputData.size);
            // ESP_LOGI(TAG, "quenn size: %d", uxQueueMessagesWaiting(audioOutputQueue));
            max98357_open();
            esp_err_t ret = write_max98357_data(outputData.buffer, 
                                              outputData.size,
                                              &bytes_written);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Error write I2S data");
            }
            // if (uxQueueMessagesWaiting(audioOutputQueue) == 0) {
            //     max98357_close();
            // }
        }
    }
}

// WebSocket数据发送任务
static void websocket_send_task(void *parameter)
{
    while (1) {
        if (!wsConnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        encoded_data_t encoded_data = {0};
        if (xQueueReceive(wsOutQueue, &encoded_data, portMAX_DELAY) == pdPASS) {
            esp_websocket_client_send_bin(client, (const char*)encoded_data.buffer, encoded_data.size, portMAX_DELAY);
        }
    }
}

// WiFi初始化
static void wifi_init(void)
{
    // 初始化默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 初始化 NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 直接使用预设的WiFi凭据连接
    ESP_LOGI(TAG, "Connecting to WiFi network %s...", WIFI_SSID);
    WifiStation::GetInstance().SetAuth(WIFI_SSID, WIFI_PASS);
    WifiStation::GetInstance().Start();

    // 等待WiFi连接成功
    int retry_count = 0;
    while (!WifiStation::GetInstance().IsConnected() && retry_count < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry_count++;
        ESP_LOGI(TAG, "Waiting for WiFi connection... (%d/20)", retry_count);
    }

    if (WifiStation::GetInstance().IsConnected()) {
        ESP_LOGI(TAG, "WiFi connected successfully");
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        // 如果连接失败，再启动配置模式
        ESP_LOGI(TAG, "Starting configuration AP mode");
        WifiConfigurationAp::GetInstance().Start();
    }
}

// 编码任务
static void audio_encode_task(void *parameter)
{
    audio_data_t inputData = {0};
    encoded_data_t encoded_data = {0};
    while (1) {
        // 从输入队列接收音频数据
        if (xQueueReceive(audioEncodeQueue, &inputData, portMAX_DELAY) == pdPASS) {
            // 编码
            uint64_t start_time = esp_timer_get_time();
            int encoded_size = opus_encode(encoder, inputData.buffer, inputData.size / 2, encoded_data.buffer, MAX_PACKET_SIZE);
            uint64_t end_time = esp_timer_get_time();
            ESP_LOGI(TAG, "Encoding time: %llu microseconds", end_time - start_time);

            if (encoded_size < 0) {
                ESP_LOGE(TAG, "Opus编码错误: %d", encoded_size);
                continue;
            }

            encoded_data.size = encoded_size;
            if (xQueueSend(wsOutQueue, &encoded_data, 0) != pdPASS) {
                ESP_LOGW(TAG, "WebSocket输出队列已满");
            }
        }
    }
}

// 解码任务
static void audio_decode_task(void *parameter)
{
    static audio_data_t outputData = {0};
    static encoded_data_t encoded_data = {0};


    while (1) {
        if (xQueueReceive(audioDecodeQueue, &encoded_data, portMAX_DELAY) == pdPASS) {
            // 验证数据大小
            if (encoded_data.size != OPUS_CODE_SIZE) {
                ESP_LOGW(TAG, "无效的数据包大小: %d", encoded_data.size);
                continue;
            }

            // 打印调试信息
            ESP_LOGI(TAG, "解码长度: %d, 解码前数据前8字节: %d %d %d %d %d %d %d %d",
                encoded_data.size,
                encoded_data.buffer[0], encoded_data.buffer[1], encoded_data.buffer[2], encoded_data.buffer[3],
                encoded_data.buffer[4], encoded_data.buffer[5], encoded_data.buffer[6], encoded_data.buffer[7]);

            // 解码
            int frame_size = opus_decode(decoder,
                                       encoded_data.buffer,
                                       encoded_data.size,
                                       decoded_buffer,
                                       DECODE_BUFFER_SIZE,
                                       0);

            if (frame_size < 0) {
                ESP_LOGE(TAG, "Opus解码错误: %s\n", opus_strerror(frame_size));
                // 添加错误计数和处理
                continue;
            }

            ESP_LOGI(TAG, "解码成功，帧大小: %d", frame_size);

            // 将解码后的数据发送到音频输出队列
            outputData.size = frame_size * 2;  // 16位采样，所以乘2
            memcpy(outputData.buffer, decoded_buffer, outputData.size);

            // if (xQueueSend(audioOutputQueue, &outputData, 0) != pdPASS) {
            //     ESP_LOGW(TAG, "音频输出队列已满");
            // }
            // 添加短暂延时避免CPU占用过高
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void)
{
    // 初始化日志
    ESP_LOGI(TAG, "Starting application...");

    // 初始化 WiFi
    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(2000));  // 当前代码中没有WiFi连接成功的事件，所以需要等待5秒
    // 初始化音频设备
    ESP_ERROR_CHECK(max98357_init());
    ESP_ERROR_CHECK(inmp441_init());

    // 初始化Opus编码器和解码器
    ESP_ERROR_CHECK(init_opus());

    // 创建队列
    audioOutputQueue = xQueueCreate(10, sizeof(audio_data_t));
    wsOutQueue = xQueueCreate(5, sizeof(encoded_data_t));
    audioEncodeQueue = xQueueCreate(5, sizeof(audio_data_t));
    audioDecodeQueue = xQueueCreate(10, sizeof(encoded_data_t));


    // 初始化 WebSocket 客户端配置
    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = WS_URI;
    websocket_cfg.reconnect_timeout_ms = 10000;  // 重连超时时间
    websocket_cfg.network_timeout_ms = 10000;    // 网络超时时间
    websocket_cfg.disable_auto_reconnect = false; // 启用自动重连
    
    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

    client = esp_websocket_client_init(&websocket_cfg);
    ESP_ERROR_CHECK(esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL));
    ESP_ERROR_CHECK(esp_websocket_client_start(client));

    // 创建编码和解码任务
    xTaskCreate(audio_encode_task, "AudioEncode", 8192, NULL, 3, NULL);
    xTaskCreate(audio_decode_task, "AudioDecode", 8192 * 2, NULL, 3, NULL);

    // 创建音频输入和输出任务
    xTaskCreate(audio_input_task, "AudioInput", 8192, NULL, 2, &audioInputTaskHandle);
    xTaskCreate(audio_output_task, "AudioOutput", 8192, NULL, 2, &audioOutputTaskHandle);
    xTaskCreate(websocket_send_task, "WSSend", 8192, NULL, 3, NULL);
}

#ifdef __cplusplus
}
#endif
