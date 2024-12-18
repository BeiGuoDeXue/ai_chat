#include <WebSocketsClient.h>
#include "inmp441.h"
#include "max98357.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "adpcm.h"

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

// ADPCM编码器
ADPCMEncoder adpcmEncoder;  // 创建ADPCM编码器实例

// 队列句柄
QueueHandle_t audioOutputQueue;  // 音频输出队列
QueueHandle_t wsOutQueue;       // WebSocket发送队列

// 任务句柄
TaskHandle_t audioInputTaskHandle;   // 音频输入任务
TaskHandle_t audioOutputTaskHandle;  // 音频输出任务
TaskHandle_t wsTaskHandle;           // WebSocket任务


// 全局变量区域添加连接状态标志
bool wsConnected = false;
time_t event_start_time = millis();
static AudioData recv_audioData;
static AudioData wsData;
// WebSocket 事件处理
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    if (payload == NULL) {
        Serial.printf("payload is NULL\n");
        return;
    }
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

        case WStype_BIN: {
            if (!wsConnected) return;
            
            try {
                // ADPCM解码
                recv_audioData.size = adpcmEncoder.decodeBuffer(
                    payload,
                    length,
                    recv_audioData.buffer
                ) * 2;  // 转换为字节数
                if (xQueueSend(audioOutputQueue, &recv_audioData, 0) != pdPASS) {
                    Serial.println("音频输出队列已满");
                }
                Serial.printf("recv_audioData.size: %d\n", recv_audioData.size);
            } catch (...) {
                Serial.println("WebSocket数据处理异常");
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
        if (bytes_read > 0) {
            inputData.size = bytes_read;
            mic_data_handle(inputData.buffer, inputData.size);
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
            mic_data_handle(outputData.buffer, outputData.size);
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

uint8_t encoded_data[MAX_AUDIO_SAMPLES/2];  // ADPCM压缩后的大小
// WebSocket数据发送任务
void webSocketSendTask(void *parameter) {
    while (true) {
        if (!wsConnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (xQueueReceive(wsOutQueue, &wsData, 0) == pdPASS) {
            try {
                // 检查输入数据大小是否超出范围
                if(wsData.size > MAX_AUDIO_BUFFER_SIZE) {
                    Serial.println("Input data too large");
                    continue;
                }
                // ADPCM编码
                size_t encoded_size = adpcmEncoder.encodeBuffer(
                    wsData.buffer, 
                    wsData.size/2,  // 转换为样本数
                    encoded_data
                );

                if(encoded_size <= 0) {
                    Serial.println("Encoding failed");
                    continue;
                }
                // 发送二进制数据
                webSocket.sendBIN(encoded_data, encoded_size);
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
