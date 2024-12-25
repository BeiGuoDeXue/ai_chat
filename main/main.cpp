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

#define MAX_PACKET_SIZE 150  // 适当调整包大小
#define OPUS_FRAME_SIZE 960  // 与服务器编码帧大小保持一致

static const char *TAG = "MAIN";

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
static QueueHandle_t audioEncodeQueue;

// 任务句柄
static TaskHandle_t audioInputTaskHandle;
static TaskHandle_t audioOutputTaskHandle;
static TaskHandle_t wsTaskHandle;

// 全局变量
static bool wsConnected = false;

// 音频数据结构
typedef struct {
    int16_t buffer[OPUS_FRAME_SIZE];  // 使用采样点数
    size_t size;
} audio_data_t;

// 数据包头结构（整个传输包的头部）
typedef struct __attribute__((packed)) {
    uint32_t total_length;    // 4字节
    uint16_t total_packets;   // 2字节
} packet_header_t;

// 实际数据包结构
typedef struct __attribute__((packed)) {
    uint16_t data_length;     // 2字节
    uint8_t data[MAX_PACKET_SIZE];
} packet_data_t;

// 解码数据的环形缓冲区结构（用于存储编码后的数据包）
typedef struct {
    packet_data_t *packets;   // packet_data_t数组
    size_t capacity;          // 缓冲区能容纳的包数量
    size_t read_pos;
    size_t write_pos;
    size_t packet_count;      // 当前缓冲区中的包数量
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t data_ready;
} decoded_packets_buffer_t;

// 音频输出的环形缓冲区结构
typedef struct {
    uint8_t *buffer;         // 字节数组
    size_t size;            // 缓冲区总大小
    size_t read_pos;
    size_t write_pos;
    size_t data_size;       // 当前数据量
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t data_ready;
} audio_ring_buffer_t;

// 在全局变量部分添加
static struct {
    uint8_t buffer[1024];  // 足够大的缓冲区存储多个数据包
    size_t current_size;   // 当前已存储的数据大小
    uint16_t packet_count; // 当前包含的数据包数量
} ws_send_buffer;

// 全局变量
static decoded_packets_buffer_t decoded_packets_buffer;
static audio_ring_buffer_t audio_ring_buffer;

// 在全局变量部分添加编码数据的ring buffer
static struct {
    packet_data_t *packets;   // packet_data_t数组
    size_t capacity;          // 缓冲区能容纳的包数量
    size_t read_pos;
    size_t write_pos;
    size_t packet_count;      // 当前缓冲区中的包数量
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t data_ready;
} encoded_packets_buffer;

// 在全局变量部分
static packet_data_t encoded_packets[20];  // 存储20个编码后的数据包

// 初始化环形缓冲区
static packet_data_t decoded_packets[50];  // 存储50个待解码的数据包
static uint8_t audio_buffer[65536];        // 64KB音频输出缓冲区

// 初始化码数据环形缓冲区
static void decoded_packets_buffer_init(decoded_packets_buffer_t *rb, packet_data_t *buffer, size_t packet_count) {
    rb->packets = buffer;
    rb->capacity = packet_count;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->packet_count = 0;
    rb->mutex = xSemaphoreCreateMutex();
    rb->data_ready = xSemaphoreCreateBinary();
}

// 初始化音频输出环形缓冲区
static void audio_ring_buffer_init(audio_ring_buffer_t *rb, uint8_t *buffer, size_t buffer_size) {
    rb->buffer = buffer;
    rb->size = buffer_size;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->data_size = 0;
    rb->mutex = xSemaphoreCreateMutex();
    rb->data_ready = xSemaphoreCreateBinary();
}

// 写入解码数据包
static bool decoded_packets_buffer_write(decoded_packets_buffer_t *rb, const packet_data_t *packet) {
    if (rb->packet_count >= rb->capacity) {
        return false;
    }
    memcpy(&rb->packets[rb->write_pos], packet, sizeof(packet_data_t));
    rb->write_pos = (rb->write_pos + 1) % rb->capacity;
    rb->packet_count++;
    return true;
}

// 读取解码数据包
static bool decoded_packets_buffer_read(decoded_packets_buffer_t *rb, packet_data_t *packet) {
    if (rb->packet_count == 0) {
        return false;
    }
    memcpy(packet, &rb->packets[rb->read_pos], sizeof(packet_data_t));
    rb->read_pos = (rb->read_pos + 1) % rb->capacity;
    rb->packet_count--;
    return true;
}

// 写入音频数据
static size_t audio_ring_buffer_write(audio_ring_buffer_t *rb, const uint8_t *data, size_t len) {
    size_t available = rb->size - rb->data_size;
    size_t write_len = len < available ? len : available;
    
    if (write_len == 0) {
        return 0;
    }

    size_t first_chunk = rb->size - rb->write_pos;
    if (first_chunk >= write_len) {
        memcpy(rb->buffer + rb->write_pos, data, write_len);
    } else {
        memcpy(rb->buffer + rb->write_pos, data, first_chunk);
        memcpy(rb->buffer, data + first_chunk, write_len - first_chunk);
    }

    rb->write_pos = (rb->write_pos + write_len) % rb->size;
    rb->data_size += write_len;
    return write_len;
}

// 读取音频数据
static size_t audio_ring_buffer_read(audio_ring_buffer_t *rb, uint8_t *data, size_t len) {
    size_t read_len = len < rb->data_size ? len : rb->data_size;
    
    if (read_len == 0) {
        return 0;
    }

    size_t first_chunk = rb->size - rb->read_pos;
    if (first_chunk >= read_len) {
        memcpy(data, rb->buffer + rb->read_pos, read_len);
    } else {
        memcpy(data, rb->buffer + rb->read_pos, first_chunk);
        memcpy(data + first_chunk, rb->buffer, read_len - first_chunk);
    }

    rb->read_pos = (rb->read_pos + read_len) % rb->size;
    rb->data_size -= read_len;
    return read_len;
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
    
    return ESP_OK;
}

// WebSocket事件处理
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_DATA: {
            if (!wsConnected || !data->data_len) {
                ESP_LOGW(TAG, "WebSocket未连接或数据长度为0");
                return;
            }

            // 首先读取头部
            if (data->data_len < sizeof(packet_header_t)) {
                ESP_LOGE(TAG, "数据包太小，无法包含头部");
                for (int i = 0; i < data->data_len; i++) {
                    printf("%c ", data->data_ptr[i]);
                    printf("%02x ", data->data_ptr[i]);
                }
                printf("\n");
                return;
            }

            const uint8_t* current_ptr = (const uint8_t*)data->data_ptr;
            packet_header_t header;
            memcpy(&header, current_ptr, sizeof(packet_header_t));
            
            // 移动指针到数据包开始位置
            current_ptr += sizeof(packet_header_t);
            size_t remaining_len = data->data_len - sizeof(packet_header_t);

            ESP_LOGI(TAG, "Received packet: total_length=%ld, total_packets=%d", 
                    header.total_length, header.total_packets);

            // 循环处理所有数据包
            for (int i = 0; i < header.total_packets && remaining_len >= sizeof(uint16_t); i++) {
                // 读取数据长度
                uint16_t data_length;
                memcpy(&data_length, current_ptr, sizeof(uint16_t));
                current_ptr += sizeof(uint16_t);
                remaining_len -= sizeof(uint16_t);

                // 验证数据包长度
                if (data_length > MAX_PACKET_SIZE || data_length > remaining_len) {
                    ESP_LOGE(TAG, "Invalid packet length: %d", data_length);
                    break;
                }

                // 创建数据包
                packet_data_t packet;
                packet.data_length = data_length;
                memcpy(packet.data, current_ptr, data_length);

                // 写入编码数据环形缓冲区
                if (xSemaphoreTake(decoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    if (decoded_packets_buffer_write(&decoded_packets_buffer, &packet)) {
                        xSemaphoreGive(decoded_packets_buffer.data_ready);
                        // ESP_LOGI(TAG, "Packet written: %d/%d, length=%d", 
                        //         i + 1, header.total_packets, data_length);
                    } else {
                        ESP_LOGW(TAG, "Ring buffer full, packet dropped");
                    }
                    xSemaphoreGive(decoded_packets_buffer.mutex);
                }

                // 移动到下一个数据包
                current_ptr += data_length;
                remaining_len -= data_length;
            }
            ESP_LOGI(TAG, "ring buffer size: %d", decoded_packets_buffer.packet_count);

            if (remaining_len > 0) {
                ESP_LOGW(TAG, "Incomplete packet data remaining: %d bytes", remaining_len);
            }
            break;
        }
        
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            wsConnected = true;
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            wsConnected = false;
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WEBSOCKET_EVENT_ERROR");
            break;

        default:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ANY");
            break;
    }
}

// 音频输入任务
static void audio_input_task(void *parameter)
{
    static audio_data_t inputData = {};
    size_t bytes_read;
    
    while (1) {
        ESP_LOGI(TAG, "Reading mic data");
        esp_err_t ret = read_mic_data(inputData.buffer, 
                                    sizeof(inputData.buffer), 
                                    &bytes_read);
        if (ret == ESP_OK && bytes_read > 0) {
            inputData.size = bytes_read;
            mic_data_handle(inputData.buffer, inputData.size);

            // 将数据写入队列
            if (xQueueSend(audioEncodeQueue, &inputData, 0) != pdPASS) {
                ESP_LOGW(TAG, "Audio encode queue is full");
            }

            // if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
            //     audio_ring_buffer_write(&audio_ring_buffer, 
            //                     (uint8_t*)inputData.buffer, 
            //                     inputData.size);
            //     xSemaphoreGive(audio_ring_buffer.mutex);
            //     xSemaphoreGive(audio_ring_buffer.data_ready);
            // }
        } else {
            ESP_LOGE(TAG, "Read mic data error");
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 音频输出任务
static void audio_output_task(void *parameter)
{
    static audio_data_t output_data = {};
    size_t bytes_written;
    max98357_open();

    while (1) {
        // 等待解码数据就绪
        if (xSemaphoreTake(audio_ring_buffer.data_ready, portMAX_DELAY) == pdTRUE) {
            // 持续读取并输出数据，直到缓冲区为空
            
            while (1) {
                size_t read_size = 0;
                
                // 从解码环形缓冲区读取数据
                if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    read_size = audio_ring_buffer_read(&audio_ring_buffer, 
                                                     (uint8_t*)output_data.buffer,
                                                     sizeof(output_data.buffer));
                    xSemaphoreGive(audio_ring_buffer.mutex);
                }

                // 如果没有更多数据可读，退出循环
                if (read_size == 0) {
                    // max98357_close();
                    break;
                }

                output_data.size = read_size;
                ESP_LOGI(TAG, "output data size: %d, ring buffer len: %d", 
                        output_data.size, audio_ring_buffer.data_size);

                // 输出到喇叭
                esp_err_t ret = write_max98357_data(output_data.buffer, 
                                                   output_data.size,
                                                   &bytes_written);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Error writing I2S data");
                }
            }
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

// 初始化编码数据环形缓冲区
static void encoded_packets_buffer_init(void) {
    encoded_packets_buffer.packets = encoded_packets;
    encoded_packets_buffer.capacity = 20;
    encoded_packets_buffer.read_pos = 0;
    encoded_packets_buffer.write_pos = 0;
    encoded_packets_buffer.packet_count = 0;
    encoded_packets_buffer.mutex = xSemaphoreCreateMutex();
    encoded_packets_buffer.data_ready = xSemaphoreCreateBinary();
}

// 修改编码任务
static void audio_encode_task(void *parameter)
{
    static audio_data_t inputData = {};
    static packet_data_t packet_data = {};
    static uint8_t encoded_data[1000];
    
    while (1) {
        // 从输入队列接收音频数据
        if (xQueueReceive(audioEncodeQueue, &inputData, portMAX_DELAY) == pdPASS) {
            if (inputData.size != OPUS_FRAME_SIZE * 2) {
                ESP_LOGE(TAG, "Input data size is not valid: %d", inputData.size);
                continue;
            }
            // // 编码
            // uint64_t start_time = esp_timer_get_time();
            // int encoded_size = opus_encode(encoder, inputData.buffer, OPUS_FRAME_SIZE, 
            //                             packet_data.data, sizeof(packet_data.data));
            // uint64_t end_time = esp_timer_get_time();
            // ESP_LOGI(TAG, "Encoding time: %llu microseconds", end_time - start_time);

            // if (encoded_size < 0) {
            //     ESP_LOGE(TAG, "Opus编码错误: %d", encoded_size);
            //     continue;
            // } else if (encoded_size > MAX_PACKET_SIZE) {
            //     ESP_LOGE(TAG, "Encoded packet size exceeds maximum: %d", encoded_size);
            //     continue;
            // }

            // packet_data.data_length = encoded_size;
            
            // // 写入编码数据环形缓冲区
            // if (xSemaphoreTake(encoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
            //     if (encoded_packets_buffer.packet_count < encoded_packets_buffer.capacity) {
            //         memcpy(&encoded_packets_buffer.packets[encoded_packets_buffer.write_pos], 
            //                &packet_data, sizeof(packet_data_t));
            //         encoded_packets_buffer.write_pos = 
            //             (encoded_packets_buffer.write_pos + 1) % encoded_packets_buffer.capacity;
            //         encoded_packets_buffer.packet_count++;
            //         xSemaphoreGive(encoded_packets_buffer.data_ready);
            //     } else {
            //         ESP_LOGW(TAG, "Encoded packets buffer full");
            //     }
            //     xSemaphoreGive(encoded_packets_buffer.mutex);
            // }
        }
    }
}

// 解码任务
static void audio_decode_task(void *parameter) {
    static packet_data_t encoded_packet;
    static audio_data_t decoded_data = {};

    while (1) {
        // 等待编码数据就绪
        if (xSemaphoreTake(decoded_packets_buffer.data_ready, portMAX_DELAY) == pdTRUE) {
            // 持续解码直到缓冲区为空
            while (1) {
                bool has_data = false;
                
                // 从编码环形缓冲区读取数据
                if (xSemaphoreTake(decoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    has_data = decoded_packets_buffer_read(&decoded_packets_buffer, &encoded_packet);
                    xSemaphoreGive(decoded_packets_buffer.mutex);
                }

                if (!has_data) {
                    break;  // 缓冲区已空
                }

                // 解码
                int frame_size = opus_decode(decoder,
                                          encoded_packet.data,
                                          encoded_packet.data_length,
                                          (int16_t*)decoded_data.buffer,
                                          OPUS_FRAME_SIZE,
                                          0);

                if (frame_size > 0) {
                    decoded_data.size = frame_size * 2;  // 16位采样，所以乘2
                    // 写入解码数据环形缓冲区
                    if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                        audio_ring_buffer_write(&audio_ring_buffer, 
                                        (uint8_t*)decoded_data.buffer, 
                                        decoded_data.size);
                        xSemaphoreGive(audio_ring_buffer.mutex);
                        xSemaphoreGive(audio_ring_buffer.data_ready);
                    }
                    // ESP_LOGI(TAG, "decode success: %d", frame_size);
                } else {
                    ESP_LOGE(TAG, "decode error: %s", opus_strerror(frame_size));
                }
            }
        }
    }
}

// 修改websocket发送任务
static void websocket_send_task(void *parameter)
{
    static packet_data_t packet_data = {};
    const size_t MAX_SEND_SIZE = 1024;

    while (1) {
        if (!wsConnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 等待编码数据就绪
        if (xSemaphoreTake(encoded_packets_buffer.data_ready, portMAX_DELAY) == pdTRUE) {
            // 持续处理，直到缓冲区为空
            while (1) {
                bool has_data = false;
                
                // 从编码环形缓冲区读取数据
                if (xSemaphoreTake(encoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    if (encoded_packets_buffer.packet_count > 0) {
                        memcpy(&packet_data, 
                               &encoded_packets_buffer.packets[encoded_packets_buffer.read_pos],
                               sizeof(packet_data_t));
                        encoded_packets_buffer.read_pos = 
                            (encoded_packets_buffer.read_pos + 1) % encoded_packets_buffer.capacity;
                        encoded_packets_buffer.packet_count--;
                        has_data = true;
                    }
                    xSemaphoreGive(encoded_packets_buffer.mutex);
                }

                // 如果没有更多数据，退出循环
                if (!has_data) {
                    break;
                }

                // 处理读取到的数据包
                size_t packet_total_size = sizeof(uint16_t) + packet_data.data_length;
                
                // 检查是否需要发送当前缓冲区
                if ((ws_send_buffer.current_size + packet_total_size) > sizeof(ws_send_buffer.buffer) ||
                    (ws_send_buffer.current_size + packet_total_size - sizeof(packet_header_t)) >= MAX_SEND_SIZE) {
                    
                    if (ws_send_buffer.packet_count > 0) {
                        // 填充头部并发送
                        packet_header_t *header = (packet_header_t *)ws_send_buffer.buffer;
                        header->total_length = ws_send_buffer.current_size - sizeof(packet_header_t);
                        header->total_packets = ws_send_buffer.packet_count;

                        esp_websocket_client_send_bin(client, 
                                                    (const char*)ws_send_buffer.buffer,
                                                    ws_send_buffer.current_size,
                                                    portMAX_DELAY);
                        
                        ESP_LOGI(TAG, "Sent buffer: size=%d, packets=%d", 
                                ws_send_buffer.current_size,
                                ws_send_buffer.packet_count);

                        // 重置发送缓冲区
                        ws_send_buffer.current_size = sizeof(packet_header_t);
                        ws_send_buffer.packet_count = 0;
                    }
                }

                // 添加新的数据包到发送缓冲区
                memcpy(ws_send_buffer.buffer + ws_send_buffer.current_size,
                      &packet_data.data_length,
                      sizeof(uint16_t));
                ws_send_buffer.current_size += sizeof(uint16_t);

                memcpy(ws_send_buffer.buffer + ws_send_buffer.current_size,
                      packet_data.data,
                      packet_data.data_length);
                ws_send_buffer.current_size += packet_data.data_length;
                ws_send_buffer.packet_count++;
            }

            // 发送最后剩余的数据（如果有的话）
            if (ws_send_buffer.packet_count > 0) {
                packet_header_t *header = (packet_header_t *)ws_send_buffer.buffer;
                header->total_length = ws_send_buffer.current_size - sizeof(packet_header_t);
                header->total_packets = ws_send_buffer.packet_count;

                esp_websocket_client_send_bin(client, 
                                            (const char*)ws_send_buffer.buffer,
                                            ws_send_buffer.current_size,
                                            portMAX_DELAY);
                
                ESP_LOGI(TAG, "Sent final buffer: size=%d, packets=%d", 
                        ws_send_buffer.current_size,
                        ws_send_buffer.packet_count);

                // 重置发送缓冲区
                ws_send_buffer.current_size = sizeof(packet_header_t);
                ws_send_buffer.packet_count = 0;
            }
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
    audioEncodeQueue = xQueueCreate(5, sizeof(audio_data_t));

    // 初始化缓冲区
    decoded_packets_buffer_init(&decoded_packets_buffer, decoded_packets, 50);
    audio_ring_buffer_init(&audio_ring_buffer, audio_buffer, sizeof(audio_buffer));
    encoded_packets_buffer_init();
    
    // 初始化发送缓冲区
    ws_send_buffer.current_size = sizeof(packet_header_t);
    ws_send_buffer.packet_count = 0;

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

    // 创建所有任务，并检查是否创建成功
    BaseType_t xReturned;
    
    // 创建音频输入任务
    xReturned = xTaskCreate(audio_input_task, "AudioInput", 8192, NULL, 5, &audioInputTaskHandle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio input task");
    }

    // 创建编码任务
    xReturned = xTaskCreate(audio_encode_task, "AudioEncode", 8192, NULL, 4, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio encode task");
    }

    // 创建解码任务
    xReturned = xTaskCreate(audio_decode_task, "AudioDecode", 8192 * 2, NULL, 4, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio decode task");
    }

    // 创建音频输出任务
    xReturned = xTaskCreate(audio_output_task, "AudioOutput", 4096, NULL, 3, &audioOutputTaskHandle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio output task");
    }

    // 创建WebSocket发送任务
    xReturned = xTaskCreate(websocket_send_task, "WSSend", 8192, NULL, 4, &wsTaskHandle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create websocket send task");
    }

    ESP_LOGI(TAG, "All tasks created successfully");
}

#ifdef __cplusplus
}
#endif
