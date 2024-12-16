#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "inmp441.h"
#include "max98357.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"

#define MAX_AUDIO_SAMPLES 1024  // 最大采样点数
#define MAX_AUDIO_BUFFER_SIZE (MAX_AUDIO_SAMPLES * 2)  // 实际字节大小

// 音频数据结构
struct AudioData {
    int16_t buffer[MAX_AUDIO_SAMPLES];  // 使用采样点数
    size_t size;  // size 表示字节数
};

// WebSocket 客户端
WebSocketsClient webSocket;
const char* wsHost = "192.168.31.79";
const int wsPort = 80;
const char* wsPath = "/ws/esp32-client";

// WiFi 配置
const char *ssid = "2806";
const char *password = "zhaokangxu";

// 队列句柄
QueueHandle_t audioOutputQueue;  // 音频输出队列
QueueHandle_t wsOutQueue;       // WebSocket发送队列

// 任务句柄
TaskHandle_t audioInputTaskHandle;   // 音频输入任务
TaskHandle_t audioOutputTaskHandle;  // 音频输出任务
TaskHandle_t wsTaskHandle;           // WebSocket任务

// 计算base64编码后的大小：(n + 2) / 3 * 4
#define BASE64_ENCODE_SIZE(n) (((n) + 2) / 3 * 4)

// JSON文档需要的大小 = base64编码后的大小 + JSON格式开销(约100字节)
#define JSON_DOC_SIZE (BASE64_ENCODE_SIZE(MAX_AUDIO_BUFFER_SIZE * 2) + 100)

// JSON序列化后的缓冲区大小 = JSON文档大小 + 额外的格式化空间(约200字节)
#define JSON_BUFFER_SIZE (BASE64_ENCODE_SIZE(MAX_AUDIO_BUFFER_SIZE * 2) + 200)

// 全局变量区域
static StaticJsonDocument<JSON_DOC_SIZE> g_jsonDoc;
static uint8_t g_buffer[JSON_BUFFER_SIZE * 2];  // 合并后的单个缓冲区
static DynamicJsonDocument g_doc(JSON_BUFFER_SIZE);  // 将解析数据的大变量放到全局变量

// 全局变量区域添加连接状态标志
bool wsConnected = false;
time_t event_start_time = millis();
// WebSocket 事件处理
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED:
            Serial.println("WebSocket Connected");
            wsConnected = true;
            break;
            
        case WStype_DISCONNECTED:
            if (payload != NULL) {
                Serial.printf("WebSocket Disconnected, reason: %s\n", payload);  // 打印断开连接的原因
            } else {
                Serial.println("WebSocket Disconnected, reason: (null payload)");  // 处理空指针情况
            }
            wsConnected = false;
            // 断开连接时清空所有队列
            xQueueReset(wsOutQueue);
            xQueueReset(audioOutputQueue);
            break;
            
        case WStype_TEXT: {
            if (!wsConnected) {
                Serial.println("Received data but not connected, ignoring");
                return;
            }
            if (payload == NULL)
            {
                Serial.println("payload is NULL");
                break;
            }
            // 打印接收到的数据长度
            Serial.printf("Received data length: %d\n", length);
            // Serial.printf("Received data: %.*s\n", length, payload);  // 打印接收到的数据
            
            // 添加try-catch块
            try {
                // 解析 JSON 数据
                DeserializationError error = deserializeJson(g_doc, (const char*)payload);  // 使用全局变量
                if (!error) {
                    const char* type = g_doc["type"];  // 使用全局变量
                    if (strcmp(type, "audio") == 0) {
                        const char* base64Data = g_doc["data"];  // 使用全局变量
                        size_t output_length;
                        AudioData audioData;

                        // 解码音频数据
                        // TODO：后面把base64去掉
                        if (mbedtls_base64_decode(
                            (unsigned char*)audioData.buffer,
                            sizeof(audioData.buffer),
                            &output_length,
                            (const unsigned char*)base64Data,
                            strlen(base64Data)
                        ) == 0) {
                            audioData.size = output_length;
                            Serial.printf("output_length: %d\n", output_length);
                            
                            // 检查数据格式是否符合要求
                            if (output_length % 2 != 0) {
                                Serial.println("警告：PCM数据长度不是2的倍数");
                                return;
                            }
                            
                            // 发送到音频输出队列，阻塞时间改为0，避免阻塞音频数据
                            if (xQueueSend(audioOutputQueue, &audioData, 0) != pdPASS) {
                                Serial.println("音频输出队列已满");
                            }
                            time_t end_time = millis();
                            Serial.printf("event Time taken: %d ms\n", end_time - event_start_time);
                            event_start_time = end_time;
                        }
                    }
                } else {
                    Serial.println("JSON解析错误");
                }
            } catch (...) {
                Serial.println("WebSocket数据处理异常");
            }
            break;
        }
        
        case WStype_ERROR:
            Serial.println("WebSocket Error");
            wsConnected = false;
            break;

        case WStype_PING:
            Serial.println("Received PING, sending PONG");
            break;

        case WStype_PONG:
            Serial.println("Received PONG");
            break;

        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
            Serial.println("Fragment start - not supported");
            break;
    }
}

// 音频输入任务 - 处理麦克风输入
void audioInputTask(void *parameter) {
    while (true) {
        AudioData inputData;
        int bytes_read = read_mic_data(inputData.buffer, sizeof(inputData.buffer));
        if (bytes_read > 0) {
            inputData.size = bytes_read;
            if (xQueueSend(wsOutQueue, &inputData, 0) != pdPASS) {
                // Serial.println("Failed to queue input audio");
                // // 打印队列状态
                // UBaseType_t queueSize = uxQueueMessagesWaiting(wsOutQueue);
                // Serial.printf("Current queue size: %d\n", queueSize);
            }

            // if (xQueueSend(audioOutputQueue, &inputData, 0) != pdPASS) {
            //     Serial.println("Failed to audioOutputQueue queue input audio");
            // }
            // Serial.printf("inputData.size: %d\n", inputData.size);

            // mic_data_handle(inputData.buffer, inputData.size);

        } else {
            max98357_close();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 音频输出任务 - 处理扬声器输出
void audioOutputTask(void *parameter) {
    AudioData outputData;
    while (true) {
        if (xQueueReceive(audioOutputQueue, &outputData, portMAX_DELAY)) {
            max98357_open();
            int bytes_write = write_max98357_data(outputData.buffer, outputData.size);
            if (bytes_write == -1) {
                Serial.println("Error write I2S data");
            }
            // 检查输出队列是否为空，如果为空则关闭音频设备
            if (uxQueueMessagesWaiting(audioOutputQueue) == 0) {
                max98357_close();
            }
        }
    }
}

// WebSocket连接维护任务
void webSocketLoopTask(void *parameter) {
    while (true) {
        webSocket.loop();  // 只负责处理WebSocket连接维护
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// WebSocket数据发送任务
void webSocketSendTask(void *parameter) {
    AudioData wsData;
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 增加延时

    while (true) {
        // 检查WebSocket连接状态
        if (!wsConnected) {
            vTaskDelay(xDelay);
            continue;
        }

        if (xQueueReceive(wsOutQueue, &wsData, 0) == pdPASS) {
            // 检查数据大小（字节数）
            if (wsData.size <= 0 || wsData.size > MAX_AUDIO_BUFFER_SIZE) {
                Serial.printf("Invalid audio size (bytes): %d, max allowed: %d\n", 
                    wsData.size, MAX_AUDIO_BUFFER_SIZE);
                continue;
            }

            try {
                size_t buffer_half = sizeof(g_buffer) / 2;
                size_t output_length;

                // 清空缓冲区
                memset(g_buffer, 0, sizeof(g_buffer));

                int result = mbedtls_base64_encode(
                    (unsigned char*)(g_buffer + buffer_half),
                    buffer_half,
                    &output_length,
                    (unsigned char*)wsData.buffer,
                    wsData.size
                );
                
                if (result == 0 && output_length < buffer_half) {
                    g_buffer[buffer_half + output_length] = '\0';
                    
                    g_jsonDoc.clear();
                    g_jsonDoc["type"] = "audio";
                    g_jsonDoc["data"] = (const char*)(g_buffer + buffer_half);
                    
                    size_t len = serializeJson(g_jsonDoc, (char*)g_buffer, buffer_half);
                    
                    if (len > 0 && len < buffer_half && wsConnected) {
                        webSocket.sendTXT((char*)g_buffer, len);
                    }
                }
            } catch (...) {
                Serial.println("数据处理异常");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // 连接WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
    
    // 初始化音频设备
    esp_err_t ret;
    ret = inmp441_init();
    if (ret != ESP_OK) {
        Serial.println("inmp441_init failed");
        return;
    }

    ret = max98357_init();
    if (ret != ESP_OK) {
        Serial.println("max98357_init failed");
        return;
    }
    
    // 创建队列
    audioOutputQueue = xQueueCreate(32, sizeof(AudioData));
    wsOutQueue = xQueueCreate(5, sizeof(AudioData));

    // 设置WebSocket
    webSocket.begin(wsHost, wsPort, wsPath);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    webSocket.enableHeartbeat(15000, 3000, 2);  // 启用心跳检测
    
    // 创建任务
    xTaskCreate(
        audioInputTask,
        "AudioInput",
        8192,
        NULL,
        1,
        &audioInputTaskHandle
    );
    
    xTaskCreate(
        audioOutputTask,
        "AudioOutput",
        8192,
        NULL,
        1,
        &audioOutputTaskHandle
    );
    
    xTaskCreate(
        webSocketLoopTask,
        "WSLoop",
        8192,
        NULL,
        3,
        &wsTaskHandle  // 添加任务句柄
    );
    
    xTaskCreate(
        webSocketSendTask,
        "WSSend",
        16384,  // 增加堆栈大小
        NULL,
        2,
        NULL
    );
}

void loop() {
    // 主循环为空，所有工作都在任务中完成
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("loop");
    
    // 打印 WebSocket 连接状态
    if (webSocket.isConnected()) {
        Serial.println("WebSocket: Connected");
    } else {
        Serial.println("WebSocket: Disconnected");
    }
}
