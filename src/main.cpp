#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "inmp441.h"
#include "max98357.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"

#define MAX_AUDIO_BUFFER_SIZE 1024

// 音频数据结构
struct AudioData {
    int16_t buffer[MAX_AUDIO_BUFFER_SIZE];
    size_t size;
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

// WebSocket 事件处理
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED:
            Serial.println("WebSocket Connected");
            break;
            
        case WStype_DISCONNECTED:
            Serial.println("WebSocket Disconnected");
            break;
            
        case WStype_TEXT: {
            // 检查消息类型标记 (假设前5个字符是"audio")
            if (length > 5 && strncmp((char*)payload, "audio", 5) == 0) {
                // 直接从payload+5开始解码base64数据
                size_t output_length;
                if (mbedtls_base64_decode(
                    (unsigned char*)g_buffer, 
                    sizeof(g_buffer), 
                    &output_length,
                    (const unsigned char*)(payload + 5),
                    length - 5
                ) == 0) {
                    
                    AudioData audioData;
                    audioData.size = (output_length > MAX_AUDIO_BUFFER_SIZE) ? 
                                   MAX_AUDIO_BUFFER_SIZE : output_length;
                    memcpy(audioData.buffer, g_buffer, audioData.size); 
                    
                    if (xQueueSend(audioOutputQueue, &audioData, pdMS_TO_TICKS(100)) != pdPASS) {
                        Serial.println("音频输出队列已满");
                    }
                } else {
                    Serial.println("Base64解码失败");
                }
            }
            break;
        }
    }
}

// 音频输入任务 - 处理麦克风输入
void audioInputTask(void *parameter) {
    while (true) {
        AudioData inputData;
        int bytes_read = read_mic_data(inputData.buffer, sizeof(inputData.buffer));
        if(bytes_read > 0) {
            inputData.size = bytes_read;
            if (xQueueSend(wsOutQueue, &inputData, 0) != pdPASS) {
                Serial.println("Failed to queue input audio");
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
                // delay(10);
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
    while (true) {
        if (xQueueReceive(wsOutQueue, &wsData, 0) == pdPASS) {
            // 打印调试信息
            // Serial.printf("Raw audio size: %d, Buffer size: %d\n", 
            //             wsData.size, sizeof(g_buffer));
            
            // 直接使用原始音频数据进行base64编码
            size_t buffer_half = sizeof(g_buffer) / 2;
            size_t output_length;
            int result = mbedtls_base64_encode(
                (unsigned char*)(g_buffer + buffer_half),
                buffer_half,
                &output_length,
                (unsigned char*)wsData.buffer,  // 直接使用原始buffer
                wsData.size
            );
            
            if (result == 0) {
                // 打印调试信息
                // Serial.printf("Base64 length: %d\n", output_length);
                
                // 确保base64字符串正确终止
                g_buffer[buffer_half + output_length] = '\0';
                
                // 打印base64数据的开头部分
                char tempBuffer[100];
                strncpy(tempBuffer, (char*)(g_buffer + buffer_half), 50);
                tempBuffer[50] = '\0';
                // Serial.printf("Base64 start: %s\n", tempBuffer);
                
                // 使用ArduinoJson构建JSON
                g_jsonDoc.clear();
                g_jsonDoc["type"] = "audio";
                g_jsonDoc["data"] = (const char*)(g_buffer + buffer_half);
                
                // 序列化到g_buffer前半部分
                size_t len = serializeJson(g_jsonDoc, (char*)g_buffer, buffer_half);
                
                // 打印JSON数据的开头部分
                if (len > 0) {
                    memcpy(tempBuffer, (char*)g_buffer, 50);
                    tempBuffer[50] = '\0';
                    // Serial.printf("JSON start: %s\n", tempBuffer);
                    
                    webSocket.sendTXT((char*)g_buffer, len);
                    Serial.printf("JSON size: %d\n", len);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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
    audioOutputQueue = xQueueCreate(16, sizeof(AudioData));
    wsOutQueue = xQueueCreate(5, sizeof(AudioData));

    // 设置WebSocket
    webSocket.begin(wsHost, wsPort, wsPath);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    
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
        4096,
        NULL,
        2,  // 可以给连接维护更高的优先级
        NULL
    );
    
    xTaskCreate(
        webSocketSendTask,
        "WSSend",
        8192,
        NULL,
        1,
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
