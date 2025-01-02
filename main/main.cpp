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
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "cJSON.h"

// 定义监控间隔（5秒）
#define MONITOR_INTERVAL_MS 5000
#define MAX_PACKET_SIZE 150  // 适当调整包大小
#define OPUS_FRAME_SIZE 960  // 与服务器编码帧大小保持一致
#define AUDIO_BUFFER_SIZE (32 * 1024)
#define ENCODED_PACKETS_BUFFER_SIZE 64  // 64个包，由于网络卡顿，这里需要多一些
#define DECODED_PACKETS_BUFFER_SIZE 64
#define FLOW_CONTROL_THRESHOLD_HIGH 0.75  // 75%容量时发送暂停
#define FLOW_CONTROL_THRESHOLD_LOW  0.25  // 25%容量时发送恢复
#define FLOW_CONTROL_CHECK_INTERVAL_MS 100  // 每100ms检查一次缓冲区状态
#define MAX_BATCH_SIZE 1024

// #define TEST_MIC_AUDIO 1

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
static bool flow_control_paused = false;
static uint32_t sequence_id = 0;

// 音频数据结构
typedef struct {
    int16_t buffer[OPUS_FRAME_SIZE];  // 使用采样点数
    int16_t size;
} audio_data_t;

// 数据包头结构（整个传输包的头部）
typedef struct __attribute__((packed)) {
    uint32_t total_length;    // 4字节
    uint16_t total_packets;   // 2字节
    uint32_t sequence_id;     // 4字节 - 新增
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

// 在全局变量部分添加编码数据的ring buffer
typedef struct {
    packet_data_t *packets;   // packet_data_t数组
    size_t capacity;          // 缓冲区能容纳的包数量
    size_t read_pos;
    size_t write_pos;
    size_t packet_count;      // 当前缓冲区中的包数量
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t data_ready;
} encoded_packets_buffer_t;

// ring buffer
static audio_ring_buffer_t audio_ring_buffer;
static decoded_packets_buffer_t decoded_packets_buffer;
static encoded_packets_buffer_t encoded_packets_buffer;

// Base64解码表
static const unsigned char base64_table[256] = {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64, 64, 63,
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
    64,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 64,
    64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

// Base64解码函数
static unsigned char* base64_decode(const unsigned char* input, size_t input_len, size_t* output_len) {
    if (input_len % 4 != 0) return NULL;
    
    *output_len = input_len / 4 * 3;
    if (input[input_len - 1] == '=') (*output_len)--;
    if (input[input_len - 2] == '=') (*output_len)--;
    
    unsigned char* output = (unsigned char*)malloc(*output_len);
    if (!output) return NULL;
    
    size_t i, j;
    for (i = 0, j = 0; i < input_len;) {
        uint32_t a = input[i] == '=' ? 0 : base64_table[input[i]]; i++;
        uint32_t b = input[i] == '=' ? 0 : base64_table[input[i]]; i++;
        uint32_t c = input[i] == '=' ? 0 : base64_table[input[i]]; i++;
        uint32_t d = input[i] == '=' ? 0 : base64_table[input[i]]; i++;
        
        uint32_t triple = (a << 18) + (b << 12) + (c << 6) + d;
        
        if (j < *output_len) output[j++] = (triple >> 16) & 0xFF;
        if (j < *output_len) output[j++] = (triple >> 8) & 0xFF;
        if (j < *output_len) output[j++] = triple & 0xFF;
    }
    
    return output;
}

// 在全局定义部分添加 base64 编码相关函数
static const char base64_chars[] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static char* base64_encode(const uint8_t* input, size_t length, size_t* output_length) {
    *output_length = 4 * ((length + 2) / 3);  // 计算编码后的长度
    char* encoded_data = (char*)malloc(*output_length + 1);  // +1 用于 null 终止符
    if (!encoded_data) return NULL;

    size_t i = 0, j = 0;
    size_t remaining = length;

    while (remaining >= 3) {
        encoded_data[j++] = base64_chars[(input[i] >> 2) & 0x3F];
        encoded_data[j++] = base64_chars[((input[i] & 0x3) << 4) | ((input[i + 1] >> 4) & 0xF)];
        encoded_data[j++] = base64_chars[((input[i + 1] & 0xF) << 2) | ((input[i + 2] >> 6) & 0x3)];
        encoded_data[j++] = base64_chars[input[i + 2] & 0x3F];
        i += 3;
        remaining -= 3;
    }

    if (remaining > 0) {
        encoded_data[j++] = base64_chars[(input[i] >> 2) & 0x3F];
        if (remaining == 1) {
            encoded_data[j++] = base64_chars[(input[i] & 0x3) << 4];
            encoded_data[j++] = '=';
        } else {  // remaining == 2
            encoded_data[j++] = base64_chars[((input[i] & 0x3) << 4) | ((input[i + 1] >> 4) & 0xF)];
            encoded_data[j++] = base64_chars[(input[i + 1] & 0xF) << 2];
        }
        encoded_data[j++] = '=';
    }

    encoded_data[j] = '\0';  // 添加 null 终止符
    return encoded_data;
}

// 监控任务
static void monitor_task(void* arg)
{
    char *task_stats_buffer = (char*)malloc(2048);
    
    while(1) {
        // 内存使用情况监控
        multi_heap_info_t heap_info;
        heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        
        uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        uint32_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        uint32_t total_heap = heap_info.total_allocated_bytes + heap_info.total_free_bytes;
        
        ESP_LOGI(TAG, "Memory Info:");
        ESP_LOGI(TAG, "Total Heap: %ld bytes", total_heap);
        ESP_LOGI(TAG, "Free Heap: %ld bytes", free_heap);
        ESP_LOGI(TAG, "Minimum Free Heap: %ld bytes", min_free_heap);
        ESP_LOGI(TAG, "Largest Free Block: %u", heap_info.largest_free_block);
        
        // 系统运行时间
        ESP_LOGI(TAG, "System uptime: %llu seconds", esp_timer_get_time() / 1000000);
        
        // CPU使用率监控
        if (task_stats_buffer) {
            vTaskGetRunTimeStats(task_stats_buffer);
            ESP_LOGI(TAG, "Raw stats:\n%s", task_stats_buffer);
            
            float cpu_usage_0 = 0.0f;
            float cpu_usage_1 = 0.0f;
            
            // 解析原始数据中显示的百分比
            char *line = strtok(task_stats_buffer, "\n");
            while (line != NULL) {
                char task_name[32] = {0};
                uint32_t abs_time = 0;
                float percentage = 0.0f;
                
                // 尝试解析包含百分比的行
                if (sscanf(line, "%31s %lu %f%%", task_name, &abs_time, &percentage) >= 2) {
                    if (strcmp(task_name, "IDLE0") == 0) {
                        cpu_usage_0 = 100.0f - percentage;
                        ESP_LOGI(TAG, "IDLE0 percentage: %.1f%%", percentage);
                    } else if (strcmp(task_name, "IDLE1") == 0) {
                        cpu_usage_1 = 100.0f - percentage;
                        ESP_LOGI(TAG, "IDLE1 percentage: %.1f%%", percentage);
                    }
                }
                line = strtok(NULL, "\n");
            }
            
            // 确保结果在0-100范围内
            cpu_usage_0 = (cpu_usage_0 < 0.0f) ? 0.0f : (cpu_usage_0 > 100.0f) ? 100.0f : cpu_usage_0;
            cpu_usage_1 = (cpu_usage_1 < 0.0f) ? 0.0f : (cpu_usage_1 > 100.0f) ? 100.0f : cpu_usage_1;
            
            ESP_LOGI(TAG, "CPU Usage - Core 0: %.1f%%, Core 1: %.1f%%", cpu_usage_0, cpu_usage_1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
    
    free(task_stats_buffer);
}

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
    opus_encoder_ctl(encoder, OPUS_SET_DTX(0));
    
    return ESP_OK;
}

// 添加流控制消息发送函数
static void send_flow_control_message(bool pause) {
    if (!wsConnected || flow_control_paused == pause) {
        return;
    }
    
    // 创建JSON对象
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "创建JSON对象失败");
        return;
    }

    // 添加字段
    cJSON_AddStringToObject(root, "type", "flow_control");
    cJSON_AddBoolToObject(root, "pause", pause);

    // 生成JSON字符串
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str != NULL) {
        // 发送消息
        int msg_id = esp_websocket_client_send_text(client, json_str, strlen(json_str), portMAX_DELAY);
        if (msg_id >= 0) {
            flow_control_paused = pause;
            ESP_LOGI(TAG, "发送流控制消息成功: %s, msg_id=%d", json_str, msg_id);
        } else {
            ESP_LOGE(TAG, "发送流控制消息失败: %s", esp_err_to_name(msg_id));
        }
        
        // 释放JSON字符串
        free(json_str);
    } else {
        ESP_LOGE(TAG, "JSON字符串生成失败");
    }

    // 释放JSON对象
    cJSON_Delete(root);
}

// 添加流控制任务
static void flow_control_task(void* arg) {
    while (1) {
        if (wsConnected) {
            if (xSemaphoreTake(decoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                float usage = (float)decoded_packets_buffer.packet_count / decoded_packets_buffer.capacity;
                bool should_pause = !flow_control_paused && usage > FLOW_CONTROL_THRESHOLD_HIGH;
                bool should_resume = flow_control_paused && usage < FLOW_CONTROL_THRESHOLD_LOW;
                
                xSemaphoreGive(decoded_packets_buffer.mutex);

                // 在释放互斥锁后发送消息
                if (should_pause || should_resume) {
                    send_flow_control_message(should_pause);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(FLOW_CONTROL_CHECK_INTERVAL_MS));
    }
}

// WebSocket事件处理
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    static uint32_t last_sequence_id = 0;
    static int64_t last_time = 0;
    static char *message_buffer = NULL;
    static size_t message_len = 0;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            wsConnected = true;
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            if (message_buffer) {
                free(message_buffer);
                message_buffer = NULL;
            }
            message_len = 0;
            wsConnected = false;
            break;
            
        case WEBSOCKET_EVENT_DATA:
        {
            if (!wsConnected || !data->data_len) {
                return;
            }
            
            // 检查是否是控制帧
            if (data->op_code == 0x9) {  // PING
                ESP_LOGI(TAG, "收到PING帧");
                // 自动回复PONG
                const uint8_t pong_data[] = {0x0A};
                esp_websocket_client_send_with_opcode(client, WS_TRANSPORT_OPCODES_PONG, pong_data, 1, portMAX_DELAY);  // 0xA是PONG
                return;
            } else if (data->op_code == 0xA) {  // PONG
                ESP_LOGI(TAG, "收到PONG帧");
                return;
            }

            // 打印详细的帧信息
            ESP_LOGD(TAG, "WebSocket帧信息: op_code=0x%x, fin=%d, payload_offset=%d, data_len=%d, payload_len=%d", 
                     data->op_code, data->fin, data->payload_offset, data->data_len, data->payload_len);

            if (data->payload_offset != 0) {
                ESP_LOGW(TAG, "payload_offset != 0, 分帧");
            }
            // 如果是新消息的开始
            if (data->payload_offset == 0) {
                if (message_buffer) {
                    free(message_buffer);
                }
                message_buffer = (char *)malloc(data->payload_len + 1);
                message_len = 0;
            }

            // 确保有足够的缓冲区空间
            if (!message_buffer || (message_len + data->data_len) > data->payload_len) {
                ESP_LOGE(TAG, "缓冲区错误");
                if (message_buffer) {
                    free(message_buffer);
                    message_buffer = NULL;
                }
                return;
            }
            // 只处理文本数据
            if (data->op_code != 0x1) {  // 0x1 是文本帧
                ESP_LOGW(TAG, "非文本数据，跳过处理");
                return;
            }
            // 复制数据到缓冲区
            memcpy(message_buffer + data->payload_offset, data->data_ptr, data->data_len);
            message_len = data->payload_offset + data->data_len;

            // 如果收到了完整的消息
            if (message_len == data->payload_len) {
                message_buffer[message_len] = '\0';
                
                // 解析JSON
                cJSON *root = cJSON_Parse(message_buffer);
                if (root) {
                    cJSON *type = cJSON_GetObjectItem(root, "type");
                    if (type && cJSON_IsString(type)) {
                        if (strcmp(type->valuestring, "audio_batch") == 0) {
                            cJSON *sequence = cJSON_GetObjectItem(root, "sequence");
                            cJSON *batch = cJSON_GetObjectItem(root, "batch");
                            cJSON *total_packets = cJSON_GetObjectItem(root, "total_packets");
                            cJSON *packets = cJSON_GetObjectItem(root, "packets");

                            if (sequence && batch && total_packets && packets &&
                                cJSON_IsNumber(sequence) && cJSON_IsNumber(batch) &&
                                cJSON_IsNumber(total_packets) && cJSON_IsArray(packets)) {
                                
                                uint32_t current_sequence = sequence->valueint;
                                
                                // 检查序列号连续性
                                if (last_sequence_id != 0 && current_sequence != last_sequence_id + 1) {
                                    ESP_LOGW(TAG, "检测到批次丢失: 期望 %lu, 收到 %lu", 
                                        last_sequence_id + 1, current_sequence);
                                }
                                last_sequence_id = current_sequence;

                                // 处理批次中的所有包
                                int packet_count = cJSON_GetArraySize(packets);
                                for (int i = 0; i < packet_count; i++) {
                                    cJSON *packet = cJSON_GetArrayItem(packets, i);
                                    if (!packet) continue;

                                    cJSON *audio_data = cJSON_GetObjectItem(packet, "data");
                                    cJSON *length = cJSON_GetObjectItem(packet, "length");

                                    if (audio_data && length && 
                                        cJSON_IsString(audio_data) && cJSON_IsNumber(length)) {
                                        
                                        // 解码base64数据
                                        size_t decoded_len;
                                        unsigned char *decoded_data = base64_decode(
                                            (unsigned char*)audio_data->valuestring,
                                            strlen(audio_data->valuestring),
                                            &decoded_len
                                        );

                                        if (decoded_data && decoded_len == length->valueint) {
                                            // 创建数据包
                                            packet_data_t packet;
                                            packet.data_length = decoded_len;
                                            memcpy(packet.data, decoded_data, decoded_len);

                                            // 写入解码数据环形缓冲区
                                            if (xSemaphoreTake(decoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                                                if (decoded_packets_buffer_write(&decoded_packets_buffer, &packet)) {
                                                    xSemaphoreGive(decoded_packets_buffer.data_ready);
                                                } else {
                                                    ESP_LOGE(TAG, "decoded_packets_buffer_write failed");
                                                }
                                                xSemaphoreGive(decoded_packets_buffer.mutex);
                                            }
                                            ESP_LOGD(TAG, "ring buffer len: %d", decoded_packets_buffer.packet_count);
                                        }
                                        if (decoded_data) {
                                            free(decoded_data);
                                        }
                                    }
                                }
                                ESP_LOGD(TAG, "处理批次 %d: 成功处理 %d 个包", 
                                    batch->valueint, packet_count);
                                int64_t current_time = esp_timer_get_time() / 1000;
                                if (current_time - last_time > 1000) {
                                    ESP_LOGW(TAG, "websocket_event_handler time overtime: %lld ms", current_time - last_time);
                                }
                                last_time = current_time;
                            }
                        }
                    }
                    cJSON_Delete(root);
                } else {
                    ESP_LOGW(TAG, "JSON解析失败: %.*s", 
                            message_len > 100 ? 100 : message_len, 
                            message_buffer);
                }

                // 清理缓冲区
                free(message_buffer);
                message_buffer = NULL;
                message_len = 0;
            }
            break;
        }
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WEBSOCKET_EVENT_ERROR");
            break;
    }
}

// 音频输入任务
static void audio_input_task(void *parameter)
{
    static audio_data_t inputData = {};
    size_t bytes_read;
    int64_t last_time = 0;
    while (1) {
        // 读取一次数据（阻塞式）
        esp_err_t ret = read_mic_data(inputData.buffer, 
                                    sizeof(inputData.buffer), 
                                    &bytes_read);

        if (ret == ESP_OK && bytes_read > 0) {
            inputData.size = bytes_read;
            mic_data_handle(inputData.buffer, inputData.size);
#ifndef TEST_MIC_AUDIO
            // 将数据写入队列
            if (xQueueSend(audioEncodeQueue, &inputData, 0) != pdPASS) {
                ESP_LOGW(TAG, "Audio encode queue is full");
            }
#else
            if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                size_t write_size = audio_ring_buffer_write(&audio_ring_buffer, 
                                (uint8_t*)inputData.buffer, 
                                inputData.size);
                if (write_size <= 0) {
                    ESP_LOGE(TAG, "audio output ring buffer full");
                }
                xSemaphoreGive(audio_ring_buffer.mutex);
                xSemaphoreGive(audio_ring_buffer.data_ready);
            }
#endif
        } else {
            ESP_LOGE(TAG, "Read mic data error: %d", ret);
        }

        int64_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_time > 100) {
            ESP_LOGE(TAG, "audio_input_task time overtime: %lld ms", current_time - last_time);
        }
        last_time = current_time;

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
            // max98357_open();
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
                // ESP_LOGI(TAG, "output data size: %d, ring buffer len: %d", 
                //         output_data.size, audio_ring_buffer.data_size);
                max98357_data_handle(output_data.buffer, output_data.size, 0.25);
                // 输出到喇叭
                esp_err_t ret = write_max98357_data(output_data.buffer, 
                                                   output_data.size,
                                                   &bytes_written);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Error writing I2S data");
                }
            }
            // max98357_close();
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
static void encoded_packets_buffer_init(encoded_packets_buffer_t *rb, packet_data_t *buffer, size_t packet_count) {
    rb->packets = buffer;
    rb->capacity = packet_count;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->packet_count = 0;
    rb->mutex = xSemaphoreCreateMutex();
    rb->data_ready = xSemaphoreCreateBinary();
}

// 写入编码数据包
static bool encoded_packets_buffer_write(encoded_packets_buffer_t *rb, const packet_data_t *packet) {
    if (rb->packet_count >= rb->capacity) {
        return false;
    }
    memcpy(&rb->packets[rb->write_pos], packet, sizeof(packet_data_t));
    rb->write_pos = (rb->write_pos + 1) % rb->capacity;
    rb->packet_count++;
    return true;
}

// 读取编码数据包
static bool encoded_packets_buffer_read(encoded_packets_buffer_t *rb, packet_data_t *packet) {
    if (rb->packet_count == 0) {
        return false;
    }
    memcpy(packet, &rb->packets[rb->read_pos], sizeof(packet_data_t));
    rb->read_pos = (rb->read_pos + 1) % rb->capacity;
    rb->packet_count--;
    return true;
}

// 修改编码任务
static void audio_encode_task(void *parameter)
{
    static audio_data_t inputData = {};
    static packet_data_t packet_data = {};
    int64_t last_time = 0;
    while (1) {
        // 从输入队列接收音频数据
        if (xQueueReceive(audioEncodeQueue, &inputData, portMAX_DELAY) == pdPASS) {
            if (inputData.size != OPUS_FRAME_SIZE * 2) {
                ESP_LOGE(TAG, "Input data size is not valid: %d", inputData.size);
                continue;
            }
            // 编码
            int encoded_size = opus_encode(encoder, inputData.buffer, OPUS_FRAME_SIZE, 
                                        packet_data.data, sizeof(packet_data.data));
            if (encoded_size < 0) {
                ESP_LOGE(TAG, "Opus编码错误: %d", encoded_size);
                continue;
            } else if (encoded_size > MAX_PACKET_SIZE) {
                ESP_LOGE(TAG, "Encoded packet size exceeds maximum: %d", encoded_size);
                continue;
            }

            packet_data.data_length = encoded_size;
            
            // 写入编码数据环形缓冲区
            if (xSemaphoreTake(encoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                if (encoded_packets_buffer_write(&encoded_packets_buffer, &packet_data)) {
                    xSemaphoreGive(encoded_packets_buffer.data_ready);
                } else {
                    ESP_LOGW(TAG, "Encoded packets buffer full");
                }
                xSemaphoreGive(encoded_packets_buffer.mutex);
            }
        }
        int64_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_time > 100) {
            ESP_LOGE(TAG, "audio_encode_task time overtime: %lld ms", current_time - last_time);
        }
        last_time = current_time;
    }
}

// 解码任务
static void audio_decode_task(void *parameter) {
    static packet_data_t encoded_packet;
    static audio_data_t decoded_data = {};
    const TickType_t xMaxWait = pdMS_TO_TICKS(100); // 最大等待100ms

    while (1) {
        bool has_data = false;
        
        // 先检查是否有数据，不管是否收到信号量
        if (xSemaphoreTake(decoded_packets_buffer.mutex, portMAX_DELAY) == pdTRUE) {
            has_data = (decoded_packets_buffer.packet_count > 0);
            xSemaphoreGive(decoded_packets_buffer.mutex);
        }

        if (has_data) {
            // 有数据就处理，不依赖信号量
            process_decode_data:
            while (1) {
                // 检查音频输出缓冲区状态
                bool buffer_full = false;
                if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    // 当缓冲区使用超过90%时，认为已满
                    buffer_full = (audio_ring_buffer.data_size >= (audio_ring_buffer.size * 9 / 10));
                    xSemaphoreGive(audio_ring_buffer.mutex);
                }

                if (buffer_full) {
                    // 如果音频缓冲区满，暂停解码一段时间
                    ESP_LOGD(TAG, "音频输出缓冲区接近满，暂停解码");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                // 从编码环形缓冲区读取数据
                has_data = false;
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
                    bool write_success = false;
                    size_t retry_count = 0;
                    const size_t MAX_RETRIES = 10;  // 最大重试次数

                    while (!write_success && retry_count < MAX_RETRIES) {
                        if (xSemaphoreTake(audio_ring_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                            size_t write_size = audio_ring_buffer_write(&audio_ring_buffer, 
                                            (uint8_t*)decoded_data.buffer, 
                                            decoded_data.size);
                            
                            if (write_size > 0) {
                                write_success = true;
                                xSemaphoreGive(audio_ring_buffer.data_ready);
                                ESP_LOGD(TAG, "audio_ring_buffer len: %d/%d", 
                                    audio_ring_buffer.data_size, audio_ring_buffer.size);
                            } else {
                                // 如果写入失败，等待一小段时间后重试
                                retry_count++;
                                ESP_LOGD(TAG, "audio_ring_buffer 写入失败，重试 %d/%d", 
                                    retry_count, MAX_RETRIES);
                            }
                            xSemaphoreGive(audio_ring_buffer.mutex);
                        }

                        if (!write_success && retry_count < MAX_RETRIES) {
                            vTaskDelay(pdMS_TO_TICKS(20));  // 等待20ms后重试
                        }
                    }

                    if (!write_success) {
                        ESP_LOGW(TAG, "无法写入音频数据到输出缓冲区，丢弃数据包");
                    }
                } else {
                    ESP_LOGE(TAG, "decode error: %s", opus_strerror(frame_size));
                }
            }
        }

        // 等待新数据到来，但不会永远等待
        if (xSemaphoreTake(decoded_packets_buffer.data_ready, xMaxWait) == pdTRUE) {
            // 收到信号量，说明有新数据，直接跳转去处理
            goto process_decode_data;
        }
        // 如果没收到信号量，会回到主循环重新检查是否有数据
    }
}

// 修改websocket发送任务
static void websocket_send_task(void *parameter)
{
    static packet_data_t packet_data = {};
    int64_t last_time = 0;
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
                    has_data = encoded_packets_buffer_read(&encoded_packets_buffer, &packet_data);
                    xSemaphoreGive(encoded_packets_buffer.mutex);
                }

                if (!has_data) {
                    break;  // 缓冲区已空
                }

                // 创建单个数据包的JSON对象
                cJSON *packet = cJSON_CreateObject();
                if (!packet) {
                    ESP_LOGE(TAG, "创建JSON对象失败");
                    continue;
                }

                // 添加类型和序列号
                cJSON_AddStringToObject(packet, "type", "audio_batch");
                cJSON_AddNumberToObject(packet, "sequence", sequence_id++);

                // Base64编码音频数据
                size_t encoded_len = 0;
                char *base64_data = base64_encode(packet_data.data, 
                                                packet_data.data_length, 
                                                &encoded_len);
                if (!base64_data) {
                    cJSON_Delete(packet);
                    ESP_LOGE(TAG, "Base64编码失败");
                    continue;
                }

                // 添加音频数据
                cJSON_AddStringToObject(packet, "data", base64_data);
                cJSON_AddNumberToObject(packet, "length", packet_data.data_length);
                free(base64_data);

                // 转换为字符串并发送
                char *json_str = cJSON_PrintUnformatted(packet);
                if (json_str) {
                    esp_websocket_client_send_text(client, json_str, strlen(json_str), portMAX_DELAY);
                    ESP_LOGD(TAG, "发送音频数据: sequence=%lu, size=%d", 
                            sequence_id - 1, packet_data.data_length);
                    free(json_str);
                }

                cJSON_Delete(packet);
                
                // 短暂延时以避免发送过快
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        int64_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_time > 500) {
            ESP_LOGE(TAG, "websocket send task overtime: %lld ms", current_time - last_time);
        }
        last_time = current_time;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void)
{
    // 初始化日志
    ESP_LOGI(TAG, "Starting application...");

    // 检查PSRAM
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size == 0) {
        ESP_LOGE(TAG, "PSRAM not found or not initialized!");
        return;
    }
    ESP_LOGI(TAG, "PSRAM initialized in Octal mode");
    ESP_LOGI(TAG, "PSRAM size: %d bytes", psram_size);

    // 分配大型缓冲区到PSRAM
    uint8_t *audio_buffer = (uint8_t *)heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    packet_data_t *encoded_packets = (packet_data_t *)heap_caps_malloc(
        ENCODED_PACKETS_BUFFER_SIZE * sizeof(packet_data_t), 
        MALLOC_CAP_SPIRAM
    );
    packet_data_t *decoded_packets = (packet_data_t *)heap_caps_malloc(
        DECODED_PACKETS_BUFFER_SIZE * sizeof(packet_data_t), 
        MALLOC_CAP_SPIRAM
    );

    if (!audio_buffer || !encoded_packets || !decoded_packets) {
        ESP_LOGE(TAG, "Failed to allocate buffers in PSRAM");
        // 清理已分配的内存
        if (audio_buffer) heap_caps_free(audio_buffer);
        if (encoded_packets) heap_caps_free(encoded_packets);
        if (decoded_packets) heap_caps_free(decoded_packets);
        return;
    }

    // 初始化 WiFi
    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(2000));  // 当前代码中没有WiFi连接成功的事件，所以需要等待5秒
    // 初始化音频设备
    ESP_ERROR_CHECK(max98357_init());
    ESP_ERROR_CHECK(inmp441_init());
    int dec_size = opus_decoder_get_size(1);
    ESP_LOGI(TAG, "解码器所需空间大小: %d", dec_size);
    int enc_size = opus_encoder_get_size(1);
    ESP_LOGI(TAG, "编码器所需空间大小: %d", enc_size);

    // 初始化Opus编码器和解码器
    ESP_ERROR_CHECK(init_opus());

    // 创建队列
    audioEncodeQueue = xQueueCreate(5, sizeof(audio_data_t));

    // 初始化缓冲区
    audio_ring_buffer_init(&audio_ring_buffer, audio_buffer, AUDIO_BUFFER_SIZE);
    encoded_packets_buffer_init(&encoded_packets_buffer, encoded_packets, ENCODED_PACKETS_BUFFER_SIZE);
    decoded_packets_buffer_init(&decoded_packets_buffer, decoded_packets, DECODED_PACKETS_BUFFER_SIZE);

    // 打印内存使用情况
    ESP_LOGI(TAG, "DRAM free: %d bytes", 
        heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM free: %d bytes", 
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));


    // 初始化 WebSocket 客户端配置
    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = WS_URI;
    websocket_cfg.reconnect_timeout_ms = 10000;  // 重连超时时间
    websocket_cfg.network_timeout_ms = 10000;    // 网络超时时间
    websocket_cfg.disable_auto_reconnect = false; // 启用自动重连
    websocket_cfg.buffer_size = 2048;
    
    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

    client = esp_websocket_client_init(&websocket_cfg);
    ESP_ERROR_CHECK(esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL));
    ESP_ERROR_CHECK(esp_websocket_client_start(client));

    // 创建所有任务，并检查是否创建成功
    BaseType_t xReturned;
    
    // 创建音频输入任务
    xReturned = xTaskCreatePinnedToCore(audio_input_task, "AudioInput", 4096, NULL, 5, &audioInputTaskHandle, 0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio input task");
    }

    // 创建编码任务
    xReturned = xTaskCreatePinnedToCore(audio_encode_task, "AudioEncode", 26 * 1024 / sizeof(StackType_t), NULL, 4, NULL, 1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio encode task");
    }

    // 创建解码任务
    xReturned = xTaskCreatePinnedToCore(audio_decode_task, "AudioDecode", 8192 * 2, NULL, 4, NULL, 1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio decode task");
    }

    // 创建音频输出任务
    xReturned = xTaskCreatePinnedToCore(audio_output_task, "AudioOutput", 4096, NULL, 5, &audioOutputTaskHandle, 1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio output task");
    }

    // 创建WebSocket发送任务
    xReturned = xTaskCreatePinnedToCore(websocket_send_task, "WSSend", 8192, NULL, 5, &wsTaskHandle, 0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create websocket send task");
    }

    // 创建监控任务
    xReturned = xTaskCreatePinnedToCore(monitor_task, "monitor", 4096, NULL, 1, NULL, 0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
    }

    // 创建流控制任务
    xReturned = xTaskCreatePinnedToCore(flow_control_task, "flow_control", 4096, NULL, 5, NULL, 0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create flow control task");
    }

    ESP_LOGI(TAG, "All tasks created successfully");
}

// 添加资源清理函数（可以在需要时调用）
static void cleanup_resources(void)
{
    if (audio_ring_buffer.buffer)
        heap_caps_free(audio_ring_buffer.buffer);
    if (encoded_packets_buffer.packets)
        heap_caps_free(encoded_packets_buffer.packets);
    if (decoded_packets_buffer.packets)
        heap_caps_free(decoded_packets_buffer.packets);
}

#ifdef __cplusplus
}
#endif
