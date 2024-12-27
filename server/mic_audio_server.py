import asyncio
import websockets
from aip import AipSpeech
import numpy as np
from collections import deque
import time
from audio_codec import AudioCodec
import threading
import struct
import pyaudio
from openai import OpenAI
import os
import wave  # 添加到文件顶部的导入语句中

class AudioPacketManager:
    def __init__(self):
        self.MAX_PACKET_SIZE = 150     # 每个数据包的最大大小
        self.MAX_SEND_SIZE = 512      # 每次发送的最大字节数
        
        # 使用 =表示原生字节序，不进行对齐
        self.HEADER_FORMAT = '=IH'     # total_length(4) + total_packets(2)
        self.DATA_HEADER_FORMAT = '=H'  # data_length(2)

    def create_packet_header(self, total_length, total_packets):
        """创建数据包头部"""
        return struct.pack(self.HEADER_FORMAT, total_length, total_packets)

    def create_data_packet(self, data):
        """创建单个数据包"""
        return struct.pack(self.DATA_HEADER_FORMAT, len(data)) + data

    def pack_audio_data(self, encoded_packets):
        """将多个编码包打包成一个完整的传输包"""
        # 计算总长度（所有数据包的长度总和）
        total_length = sum(len(packet) for packet in encoded_packets)
        total_packets = len(encoded_packets)
        
        # 创建头部
        header = self.create_packet_header(total_length, total_packets)
        
        # 创建数据包
        data_packets = [self.create_data_packet(packet) for packet in encoded_packets]
        
        # 组合所有数据
        return header + b''.join(data_packets)

    def split_packets(self, packed_data):
        """将大数据包分割成适合发送的小包，确保不破坏数据包结构
        返回: 分割后的数据包列表
        """
        header_size = struct.calcsize('<IH')  # 头部大小: 4 + 2 = 6 字节
        data_header_size = struct.calcsize('<H')  # 数据包头部大小: 2 字节
        
        # 如果总大小小于最大发送大小，直接返回
        if len(packed_data) <= self.MAX_SEND_SIZE:
            return [packed_data]
        
        # 提取原始头部信息
        total_length, total_packets = struct.unpack('<IH', packed_data[:header_size])
        
        # 开始分割数据
        result = []
        current_pos = header_size
        current_packets = []
        current_length = 0
        packet_count = 0
        
        while current_pos < len(packed_data):
            # 读取数据包长度
            data_length = struct.unpack('<H', packed_data[current_pos:current_pos + data_header_size])[0]
            packet_total_size = data_header_size + data_length
            
            # 检查是否需要创建新的发送包
            if current_length + packet_total_size > (self.MAX_SEND_SIZE - header_size) and current_packets:
                # 创建新的发送包
                sub_total_length = sum(len(p) for p in current_packets)
                sub_header = struct.pack('<IH', sub_total_length, len(current_packets))
                result.append(sub_header + b''.join(current_packets))
                
                # 重置当前包的状态
                current_packets = []
                current_length = 0
            
            # 添加当前数据包
            current_packet = packed_data[current_pos:current_pos + packet_total_size]
            current_packets.append(current_packet)
            current_length += packet_total_size
            packet_count += 1
            
            current_pos += packet_total_size
        
        # 处理最后一批数据包
        if current_packets:
            sub_total_length = sum(len(p) for p in current_packets)
            sub_header = struct.pack('<IH', sub_total_length, len(current_packets))
            result.append(sub_header + b''.join(current_packets))
        
        return result

class AudioChatServer:
    def __init__(self):
        # 百度语音配置
        self.APP_ID = '116470106'
        self.API_KEY = 'ibrjXDUQ6FgjrvSQJ4GANDBA'
        self.SECRET_KEY = '0xZe4laRgQbZrxAECCcKqeRZ1R8j3Jq1'
        self.baidu_client = AipSpeech(self.APP_ID, self.API_KEY, self.SECRET_KEY)
    
        # 音频配置
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # 初始化 PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # 初始化音频编解码器
        self.audio_codec = AudioCodec()
        # 循环缓冲区配置
        self.BUFFER_SECONDS = 10
        self.SAMPLE_RATE = 16000
        self.BYTES_PER_SAMPLE = 2
        self.BUFFER_SIZE = self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.BUFFER_SECONDS
        self.PROCESS_SIZE = 128000  # 处理单位大小(约2秒的数据)
        
        # 初始化循环缓冲区
        self.audio_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.raw_buffer = deque()  # 存储解析后的packet_data_t小包列表
        self.current_packet = None  # 当前正在处理的数据包
        self.current_packet_data = []  # 当前数据包的数据
        self.buffer_count = 0
        
        # 发送队列
        self.send_queue = asyncio.Queue(maxsize=100)
        self.is_processing = False

        # 初始化音频编解码器
        self.audio_codec = AudioCodec()
        
        # 分包大小
        self.PACKET_SIZE = 1024

        # 录音状态控制
        self.is_recording = False
        self.recording_thread = None

        # 添加锁
        self.buffer_lock = threading.Lock()
        
        # 语音状态
        self.is_speaking = False
        self.silence_frames = 0

        # 数据发送完成
        self.data_send_complete = True
        
        self.packet_manager = AudioPacketManager()

        # 初始化火山云客户端
        self.llm_client = OpenAI(
            api_key=os.environ.get("ARK_API_KEY"),
            base_url="https://ark.cn-beijing.volces.com/api/v3"
        )
        self.model_id = "ep-20241226171735-z9tvd"

    def start_recording(self):
        """开始录音"""
        if self.is_recording:
            return
            
        self.stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.recording_thread.start()
        print("开始录音...")

    def stop_recording(self):
        """停止录音"""
        self.is_recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.recording_thread:
            self.recording_thread.join()
        print("停止录音...")

    def _record_audio(self):
        """录音线程"""
        while self.is_recording:
            if self.data_send_complete:
                # print("record data_send_complete: ", self.data_send_complete)
                try:
                    # 读取音频数据
                    audio_data = self.stream.read(self.CHUNK)
                    # 转换为numpy数组
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)
                    
                    # 更新缓冲区
                    with self.buffer_lock:
                        self.update_buffer(audio_array.tobytes())

                except Exception as e:
                    print(f"录音错误: {e}")
                    break

    async def handle_websocket(self, websocket):
        """处理WebSocket连接"""
        print("新客户端连接!")
        
        # 启动发送协程
        send_task = asyncio.create_task(self.send_worker(websocket))
        # 启动处理音频的循环
        process_task = asyncio.create_task(self.process_audio_loop())
        
        self.start_recording()
        
        try:
            async for _ in websocket:  # 保持连接活跃
                pass
        except websockets.exceptions.ConnectionClosed:
            print("客户端断开连接")
        finally:
            self.stop_recording()
            send_task.cancel()
            process_task.cancel()
            try:
                await send_task
                await process_task
            except asyncio.CancelledError:
                pass

    async def send_audio_packets(self, websocket, audio_data):
        """分包发送音频数据"""
        try:
            if websocket.closed:
                print("WebSocket连接已关闭，无法发送数据")
                return
            
            # 将音频数据转换为numpy数组
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            
            # 编码音频数据
            encoded_packets, stats = self.audio_codec.encode_audio(audio_array)
            
            # 打包所有数据
            packed_data = self.packet_manager.pack_audio_data(encoded_packets)
            
            # 分割成适合发送的小包
            send_packets = self.packet_manager.split_packets(packed_data)

            self.data_send_complete = False
            # 发送所有分割后的数据包
            for i, packet in enumerate(send_packets):
                await websocket.send(packet)
                print(f"发送分包 {i+1}/{len(send_packets)}: {len(packet)} 字节")
                await asyncio.sleep(0.3)  # 短暂延迟，避免发送过快

            # 等待2秒，确保数据发送完成，音频播放完成
            await asyncio.sleep(2)
            self.data_send_complete = True
            print(f"完成发送: 总包数={len(send_packets)}, 原始数据大小={len(packed_data)}字节")

        except websockets.exceptions.ConnectionClosed as e:
            print(f"WebSocket连接关闭: {e}")
        except Exception as e:
            print(f"发送数据包错误: {e}")

    def update_buffer(self, new_data):
        """更新循环缓冲区"""
        for byte in new_data:
            self.audio_buffer.append(byte)
            self.buffer_count = min(self.buffer_count + 1, self.BUFFER_SIZE)

    def get_process_data(self):
        """获取待处理的数据，并检测语音结束"""
        if self.buffer_count < self.PROCESS_SIZE:
            return None
        
        # 从缓冲区提取数据
        data = bytearray()
        for _ in range(self.PROCESS_SIZE):
            if self.audio_buffer:
                data.append(self.audio_buffer.popleft())
                self.buffer_count -= 1
        
        # 将字节数据转换为16位整数数组
        audio_array = np.frombuffer(bytes(data), dtype=np.int16)
        
        # 计算音频能量
        energy = np.mean(np.abs(audio_array))
        
        # 根据波形图设置合适的阈值
        SPEECH_THRESHOLD = 100  # 有声音的阈值
        SILENCE_THRESHOLD = 20  # 静音阈值
        
        # 检测语音状态
        if energy > SPEECH_THRESHOLD:
            # 检测到说话
            self.is_speaking = True
            self.silence_frames = 0
            return bytes(data)
        elif energy < SILENCE_THRESHOLD:
            # 检测到静音
            if self.is_speaking:
                # 如果之前在说话，开始计数静音帧
                self.silence_frames += 1
                # 连续0.5秒静音认为说话结束（8000采样率下约4帧）
                if self.silence_frames > 4:
                    self.is_speaking = False
                    self.silence_frames = 0
                    return bytes(data)  # 返回最后一帧
            return None
        else:
            # 能量在两个阈值之间，保持当前状态
            if self.is_speaking:
                self.silence_frames = 0
                return bytes(data)
            return None

    async def process_audio_loop(self):
        """定期处理音频数据的循环"""
        while True:
            try:
                # 获取要处理的数据
                process_data = self.get_process_data()
                print("data_send_complete: ", self.data_send_complete)
                if process_data and self.data_send_complete:
                    print("start process audio")
                    # 保存音频数据为WAV文件
                    # timestamp = int(time.time())
                    # filename = f"debug_audio_{timestamp}.wav"
                    # self.save_wav(process_data, filename)
                    
                    # 播放音频数据
                    try:
                        playback_stream = self.audio.open(
                            format=self.FORMAT,
                            channels=self.CHANNELS,
                            rate=self.RATE,
                            output=True
                        )
                        playback_stream.write(process_data)
                        playback_stream.stop_stream()
                        playback_stream.close()
                        print(f"音频数据长度: {len(process_data)} 字节")
                    except Exception as e:
                        print(f"播放音频时出错: {e}")

                    # 原有的处理逻辑
                    text = self.speech_to_text(process_data)
                    if text:
                        print(f"识别到的文字: {text}")
                        # 调用大语言模型获取回复
                        response_text = await self.get_llm_response(text)
                        print(f"AI回复: {response_text}")
                        # 文字转语音
                        audio_data = self.text_to_speech(response_text)
                        if audio_data:
                            await self.send_queue.put(audio_data)

                await asyncio.sleep(0.5)

            except Exception as e:
                print(f"处理音频数据错误: {e}")
                await asyncio.sleep(1)

    async def send_worker(self, websocket):
        """独立的发送协程"""
        try:
            while True:
                # 等待发送队列中的数据
                packets = await self.send_queue.get()
                try:
                    await self.send_audio_packets(websocket, packets)
                except Exception as e:
                    print(f"发送数据包时出错: {e}")
                finally:
                    self.send_queue.task_done()
        except asyncio.CancelledError:
            print("发送协程被取消")
        except Exception as e:
            print(f"发送协程发生错误: {e}")
        finally:
            print("发送协程结束")  # 添加日志信息

    def speech_to_text(self, audio_data):
        """语音转文字"""
        try:
            result = self.baidu_client.asr(audio_data, 'pcm', 16000, {
                'dev_pid': 1537,  # 普通话(支持简单的英文识别)
            })
            
            if result['err_no'] == 0:
                return result['result'][0]
            else:
                print(f"语音识别错误: {result}")
                return None
        except Exception as e:
            print(f"语音识别异常: {e}")
            return None

    def save_pcm(self, audio_data, filename):
        """保存PCM格式音频文件"""
        with open(filename, 'wb') as f:
            f.write(audio_data)
        print(f"PCM音频已保存到: {filename}")

    def save_wav(self, audio_data, filename):
        """保存WAV格式音频文件"""
        try:
            with wave.open(filename, 'wb') as wav_file:
                wav_file.setnchannels(self.CHANNELS)
                wav_file.setsampwidth(self.audio.get_sample_size(self.FORMAT))
                wav_file.setframerate(self.RATE)
                wav_file.writeframes(audio_data)
            print(f"WAV音频已保存到: {filename}")
        except Exception as e:
            print(f"保存WAV文件时出错: {e}")


    def text_to_speech(self, text):
        """文字转语音"""
        try:
            result = self.baidu_client.synthesis(text, 'zh', 1, {
                'spd': 5,  # 语速，取值0-15
                'pit': 5,  # 音调，取值0-15
                'vol': 3,  # 音量，取值0-15
                'per': 1,  # 发音人，0为女声，1为男声，3为情感男声，4为情感女声
                'aue': 6,  # 返回PCM格式音频
            })
            
            # 增加错误处理
            if isinstance(result, dict):
                print(f"语音合成错误: {result}")
                return []
            
            # 直接返回result，不再转为字典，也不限制大小
            return result
            
        except Exception as e:
            print(f"语音合成异常: {e}")
            return []

    def read_audio(self, filename):
        """读取音频文件"""
        with open(filename, 'rb') as f:
            return f.read()  # 返回字节数据

    def cleanup(self):
        """清理资源"""
        self.stop_recording()
        self.audio.terminate()

    async def get_llm_response(self, text):
        """调用火山云大语言模型获取回复"""
        try:
            completion = self.llm_client.chat.completions.create(
                model=self.model_id,
                messages=[
                    {"role": "system", "content": "你是一个智能助手，请用简短的语言回答问题。"},
                    {"role": "user", "content": text}
                ]
            )
            return completion.choices[0].message.content
        except Exception as e:
            print(f"调用大语言模型出错: {e}")
            return "抱歉，我现在无法回答这个问题。"

async def main():
    server = AudioChatServer()
    try:
        async with websockets.serve(
            server.handle_websocket,  # 直接使用 handle_websocket 方法
            "0.0.0.0", 
            80,
            # 如果需要路径过滤，使用 process_request
            process_request=lambda path, request_headers: None if path == "/ws/esp32-client" else (404, [], b"Not Found")
        ):
            print("WebSocket server started on ws://0.0.0.0:80/ws/esp32-client")
            await asyncio.Future()
    finally:
        server.cleanup()

if __name__ == "__main__":
    asyncio.run(main())