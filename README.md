# 实时语音对话系统

这是一个基于ESP32和Python的实时语音对话系统，支持双向语音通信和AI对话功能。系统包含服务器端和客户端两个主要组件。

## 系统架构

### 服务器端 (Python)
- 基于WebSocket的实时通信服务器
- 支持语音识别（Speech-to-Text）
- 支持语音合成（Text-to-Speech）
- 集成大语言模型（支持智谱AI和火山引擎）
- 音频编解码（Opus）支持
- VAD（Voice Activity Detection）语音活动检测

### 客户端 (ESP32)
- 基于ESP32-S3开发
- 支持INMP441麦克风输入
- 支持MAX98357A音频输出
- 实时音频编解码（Opus）
- WebSocket实时通信
- 支持音频流控制
- PSRAM缓存支持

## 主要功能

1. 实时语音采集和播放
2. 双向语音通信
3. 语音识别和AI对话
4. 自动语音活动检测
5. 流量控制和缓冲管理
6. 音频数据压缩和解压缩

## 技术特点

- 使用Opus编解码器实现高质量低延迟的音频传输
- 采用WebSocket实现可靠的双向实时通信
- 支持音频流的自动流控制，防止缓冲区溢出
- 多核任务调度，优化性能
- 支持多种AI对话模型接入

## 系统要求

### 服务器端
- Python 3.8+
- 必要的Python包：
  - websockets
  - numpy
  - pyaudio
  - webrtcvad
  - opus
  - python-dotenv

### 客户端
- ESP32-S3开发板
- INMP441麦克风模块
- MAX98357A I2S功放模块
- 8MB+ PSRAM
- ESP-IDF 5.0+

## 配置说明

### 服务器配置
1. 设置环境变量（.env文件）：
   - API密钥配置
   - 服务器地址和端口
   - 音频参数设置

### 客户端配置
1. WiFi设置：
   ```cpp
   #define WIFI_SSID "your_ssid"
   #define WIFI_PASS "your_password"
   ```

2. WebSocket服务器地址：
   ```cpp
   #define WS_URI "ws://your_server:port/ws/esp32-client"
   ```

## 使用流程

1. 启动服务器：
   ```bash
   python server/mic_audio_server.py
   ```

2. 编译并烧录ESP32客户端程序

3. 系统启动后：
   - ESP32自动连接WiFi并建立WebSocket连接
   - 麦克风开始采集音频
   - 可以开始语音对话

## 注意事项

1. 确保网络连接稳定
2. 检查音频设备是否正确连接
3. 注意配置正确的采样率和缓冲区大小
4. 监控系统资源使用情况

## 调试信息

- 服务器端日志输出语音识别和AI响应信息
- 客户端通过ESP日志系统输出运行状态
- 支持内存和CPU使用率监控

## 待优化项目

1. 网络断开重连机制优化
2. 音频质量进一步提升
3. 降低通信延迟
4. 添加更多AI模型支持