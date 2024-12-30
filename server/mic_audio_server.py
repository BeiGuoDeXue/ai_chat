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
import os
import wave  # 添加到文件顶部的导入语句中
from llm.zhipu_client import ZhipuClient
from llm.volcengine_client import VolcengineClient
import webrtcvad
from dotenv import load_dotenv
import json
import base64
import sys

# 设置控制台输出编码
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

load_dotenv()

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
        self.data_send_complete_last = False

        # 初始化LLM客户端
        self.zhipu_client = ZhipuClient(
            server_host="qzvdrb.natappfree.cc",
            api_key="my-test-key-2024"
        )
        self.volcengine_client = VolcengineClient(
            api_key=os.environ.get("ARK_API_KEY")
        )
        
        # 默认使用智谱AI
        self.current_llm = "volcengine"  # 可以是 "zhipu" 或 "volcengine"

        # 初始化 WebRTC VAD
        self.vad = webrtcvad.Vad()
        # 设置VAD的激进程度 (0-3)，数字越大越激进
        # 0: 最不激进，容易检测到语音
        # 3: 最激进，只检测很确定的语音
        self.vad.set_mode(3)
        
        # VAD 相关参数
        self.vad_frame_duration = 30  # 每帧持续时间(ms)，可选值：10, 20, 30
        self.vad_frame_size = int(self.RATE * self.vad_frame_duration / 1000) * 2  # 每帧样本数
        self.vad_active_frames = 0  # 连续检测到语音的帧数
        self.vad_inactive_frames = 0  # 连续检测到静音的帧数
        self.speech_started = False  # 是否开始检测到语音
        self.speech_buffer = bytearray()  # 存储语音数据

        # 初始化包序列号
        self.sequence_id = 0

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
                try:
                    # 读取音频数据
                    audio_data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                    
                    # 更新缓冲区
                    with self.buffer_lock:
                        self.update_buffer(audio_data)

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
        """使用JSON格式发送音频数据"""
        try:
            if websocket.closed:
                print("WebSocket未连接，无法发送数据")
                return
            
            # 1. 将音频数据编码成opus包
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            encoded_packets, _ = self.audio_codec.encode_audio(audio_array)
            
            # 2. 将数据包分批处理
            current_batch = []
            current_size = 0
            batch_number = 0
            
            for packet in encoded_packets:
                # 计算当前包的JSON大小
                packet_data = {
                    "type": "audio",
                    "sequence": self.sequence_id,
                    "batch": batch_number,
                    "data": base64.b64encode(packet).decode('utf-8'),
                    "length": len(packet)
                }
                json_data = json.dumps(packet_data)
                packet_size = len(json_data.encode('utf-8'))
                
                # 如果添加这个包会超过512字节，先发送当前批次
                if current_size + packet_size > 1024 and current_batch:
                    batch_data = {
                        "type": "audio_batch",
                        "sequence": self.sequence_id,
                        "batch": batch_number,
                        "total_packets": len(current_batch),
                        "packets": current_batch
                    }
                    
                    try:
                        await websocket.send(json.dumps(batch_data))
                        print(f"发送批次 {batch_number}: {len(current_batch)}个包, "
                              f"sequence={self.sequence_id}, size={current_size}字节")
                        
                        # 重置当前批次
                        current_batch = []
                        current_size = 0
                        batch_number += 1
                        self.sequence_id += 1
                        
                        # 控制发送速率
                        await asyncio.sleep(0.2)  # 50ms间隔

                    except websockets.exceptions.ConnectionClosed:
                        print("WebSocket连接已关闭")
                        return
                    except Exception as e:
                        print(f"发送数据包失败: {e}")
                        continue
                
                # 添加当前包到批次
                current_batch.append({
                    "data": base64.b64encode(packet).decode('utf-8'),
                    "length": len(packet)
                })
                current_size += packet_size
            
            # 发送最后一批（如果有）
            if current_batch:
                batch_data = {
                    "type": "audio_batch",
                    "sequence": self.sequence_id,
                    "batch": batch_number,
                    "total_packets": len(current_batch),
                    "packets": current_batch
                }
                
                try:
                    await websocket.send(json.dumps(batch_data))
                    print(f"发送最后批次 {batch_number}: {len(current_batch)}个包, "
                          f"sequence={self.sequence_id}, size={current_size}字节")
                    self.sequence_id += 1
                    
                except websockets.exceptions.ConnectionClosed:
                    print("WebSocket连接已关闭")
                except Exception as e:
                    print(f"发送最后批次失败: {e}")
            
            await asyncio.sleep(0.1)  # 等待数据发送完成
            self.data_send_complete = True

        except Exception as e:
            print(f"发送数据包错误: {e}")

    def update_buffer(self, new_data):
        """更新循环缓冲区"""
        for byte in new_data:
            self.audio_buffer.append(byte)
            self.buffer_count = min(self.buffer_count + 1, self.BUFFER_SIZE)

    def get_process_data(self):
        """使用 WebRTC VAD 获取待处理的数据，并检测语音结束"""
        try:
            if self.buffer_count < self.vad_frame_size:
                return None

            # 从缓冲区提取一帧数据
            frame_data = bytearray()
            for _ in range(self.vad_frame_size):
                if self.audio_buffer:
                    frame_data.append(self.audio_buffer.popleft())
                    self.buffer_count -= 1

            # 确保帧大小正确
            if len(frame_data) != self.vad_frame_size:
                print(f"帧大小不正确: {len(frame_data)} != {self.vad_frame_size}")
                return None

            # 判断当前帧是否是语音
            try:
                is_speech = self.vad.is_speech(bytes(frame_data), self.RATE)
            except Exception as e:
                print(f"VAD 处理错误: {e}")
                print(f"帧大小: {len(frame_data)}, 采样率: {self.RATE}")
                return None

            # 语音活动检测逻辑
            if is_speech:
                self.vad_active_frames += 1
                self.vad_inactive_frames = 0
                if self.vad_active_frames >= 3:  # 连续3帧检测到语音才开始记录
                    self.speech_started = True
                    print("检测到语音开始")
            else:
                self.vad_inactive_frames += 1
                self.vad_active_frames = 0
                
            # 如果已经开始检测到语音，将数据添加到语音缓冲区
            if self.speech_started:
                self.speech_buffer.extend(frame_data)
                
            # 检查是否需要结束当前语音段
            if self.speech_started and self.vad_inactive_frames >= 15:  # 连续15帧未检测到语音则认为语音结束
                print("检测到语音结束")
                self.speech_started = False
                speech_data = bytes(self.speech_buffer)
                self.speech_buffer.clear()
                self.vad_active_frames = 0
                self.vad_inactive_frames = 0
                
                # 检查语音长度是否足够
                if len(speech_data) > self.RATE * 2:  # 至少1秒的语音 (16-bit = 2 bytes per sample)
                    return speech_data
                    
            return None

        except Exception as e:
            print(f"处理音频数据时发生错误: {e}")
            return None

    async def process_audio_loop(self):
        """定期处理音频数据的循环"""
        while True:
            try:
                # # 获取要处理的数据
                # process_data = self.get_process_data()
                # if not self.data_send_complete:
                #     self.audio_buffer.clear()
                #     self.buffer_count = 0
                # if self.data_send_complete_last != self.data_send_complete:
                #     print("data_send_complete: ", self.data_send_complete)
                # self.data_send_complete_last = self.data_send_complete
                # if process_data and self.data_send_complete:
                #     print(f"检测到语音片段，长度: {len(process_data)} 字节")
                    
                #     # 原有的处理逻辑
                #     text = self.speech_to_text(process_data)
                #     if text:
                #         print(f"识别到的文字: {text}")
                #         # 调用大语言模型获取回复
                #         response_text = await self.get_llm_response(text)
                #         print(f"AI回复: {response_text}")
                #         # 文字转语音
                #         audio_data = self.text_to_speech(response_text)
                #         if audio_data:
                #             await self.send_queue.put(audio_data)

                audio_data = self.read_audio("../server/audio_files/pcm/qiufeng.pcm")
                if audio_data:
                    await self.send_queue.put(audio_data)

                await asyncio.sleep(6)  # 缩短检测间隔，提高响应速度

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
                'vol': 1,  # 音量，取值0-15
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

    async def get_llm_response(self, text: str) -> str:
        """获取大语言模型回复"""
        if self.current_llm == "zhipu":
            return await self.zhipu_client.get_response(text)
        else:
            return await self.volcengine_client.get_response(text)

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