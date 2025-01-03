import asyncio
import websockets
import numpy as np
from collections import deque
import time
from audio_codec import AudioCodec
import threading
import pyaudio
import os
from llm.zhipu_client import ZhipuClient
from llm.volcengine_client import VolcengineClient
import webrtcvad
from dotenv import load_dotenv
import json
import base64
import sys
from audio_analyzer import analyze_audio_energy
import itertools
import datetime
import queue
from audio_recorder import AudioRecorder
from audio_player import AudioPlayer
from speech_service import SpeechService
from audio_saver import AudioSaver
from vad_detector import VadDetector

# 设置控制台输出编码
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

load_dotenv()

class AudioChatServer:
    def __init__(self):
        # 初始化语音服务
        self.speech_service = SpeechService()
        
        # 初始化录音器
        self.recorder = AudioRecorder()
        
        # 音频配置
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
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

        # 添加锁
        self.buffer_lock = threading.Lock()

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

        # 初始化 VAD 检测器
        self.vad_detector = VadDetector(sample_rate=self.RATE)
        # 初始化包序列号
        self.sequence_id = 0
        # 添加流控制相关的属性
        self.flow_control_paused = False
        self.pause_event = asyncio.Event()
        self.pause_event.set()  # 初始状态为未暂停

        # 测试模式
        self.test_mode = False

        # 初始化音频播放器
        self.player = AudioPlayer()

        # 初始化音频保存器
        self.audio_saver = AudioSaver(
            channels=self.CHANNELS,
            sample_format=self.FORMAT,
            rate=self.RATE
        )

    async def handle_websocket(self, websocket):
        """处理WebSocket连接"""
        print("新客户端连接!")
        
        # 启动发送协程
        send_task = asyncio.create_task(self.send_worker(websocket))
        # 启动处理音频的循环
        process_task = asyncio.create_task(self.process_audio_loop())
        
        # 用于临时存储音频数据的列表
        audio_packets = []
        
        try:
            async for message in websocket:
                try:
                    # 解析JSON消息
                    if isinstance(message, bytes):
                        message = message.decode('utf-8')
                    
                    data = json.loads(message)
                    
                    if data.get('type') == 'flow_control':
                        self.flow_control_paused = data.get('pause', False)
                        if self.flow_control_paused:
                            self.pause_event.clear()
                            print("收到暂停请求")
                        else:
                            self.pause_event.set()
                            print("收到恢复请求")
                    elif data.get('type') == 'audio_batch':
                        if self.data_send_complete:
                            sequence = data.get('sequence')
                            audio_data = base64.b64decode(data.get('data', ''))
                            length = data.get('length', 0)

                            # 验证数据长度
                            if len(audio_data) == length:
                                # print(f"收到音频数据: sequence={sequence}, size={length}")
                                # 将音频数据添加到临时列表
                                audio_packets.append(audio_data)
                                
                                # 当收集到足够的包时，一次性添加到raw_buffer
                                if len(audio_packets) >= 1:  # 可以调整这个数值
                                    with self.buffer_lock:
                                        self.raw_buffer.append(audio_packets)
                                    # print(f"存储音频包组: {len(audio_packets)}个包")
                                    audio_packets = []  # 重置临时列表
                            else:
                                print(f"音频数据长度不匹配: 期望{length}, 实际{len(audio_data)}")
                    
                except json.JSONDecodeError as e:
                    print(f"JSON解析错误: {e}, 消息内容: {message[:100]}")
                except Exception as e:
                    print(f"处理消息错误: {type(e).__name__}: {str(e)}")
                
        except websockets.exceptions.ConnectionClosed:
            print("客户端断开连接")
        finally:
            send_task.cancel()
            process_task.cancel()
            try:
                await send_task
                await process_task
            except asyncio.CancelledError:
                pass
            print("WebSocket连接已关闭")

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
            self.data_send_complete = False
            for packet in encoded_packets:
                # 等待恢复发送信号
                await self.pause_event.wait()
                
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
            
            await asyncio.sleep(4)  # 发送完成后等待2s用于喇叭播放
            self.data_send_complete = True

        except Exception as e:
            print(f"发送数据包错误: {e}")

    def update_buffer(self, new_data):
        """更新循环缓冲区"""
        for byte in new_data:
            if self.buffer_count < self.BUFFER_SIZE:
                self.audio_buffer.append(byte)
                self.buffer_count += 1
            else:
                self.audio_buffer.popleft()
                self.audio_buffer.append(byte)

    def get_process_data(self):
        """使用 WebRTC VAD 获取待处理的数据，并检测语音结束"""
        try:
            if self.buffer_count < self.vad_detector.vad_frame_size:
                return None

            # 计算要处理的数据量(不超过缓冲区大小)
            process_size = min(self.buffer_count, self.RATE * 2)  # 处理2秒的数据
            
            # 从缓冲区一次性提取所需数据
            frame_data = bytearray(list(itertools.islice(self.audio_buffer, 0, process_size)))
            print(f"处理数据大小: {len(frame_data)}字节, 剩余缓冲区: {self.buffer_count - len(frame_data)}字节")
            
            # 更新缓冲区和计数
            for _ in range(len(frame_data)):
                self.audio_buffer.popleft()
                self.buffer_count -= 1

            return self.vad_detector.process_audio(frame_data)

        except Exception as e:
            print(f"处理音频数据时发生错误: {e}")
            return None

    async def process_audio_loop(self):
        """定期处理音频数据的循环"""
        time1 = time.time()
        while True:
            try:
                if not self.test_mode:
                    encoded_packets = []
                    # 使用锁保护raw_buffer的读取
                    with self.buffer_lock:
                        while self.raw_buffer:
                            encoded_packets.extend(self.raw_buffer.popleft())

                    if not self.data_send_complete:
                        self.audio_buffer.clear()
                        self.buffer_count = 0
                    if self.data_send_complete_last != self.data_send_complete:
                        print("data_send_complete: ", self.data_send_complete)
                        self.data_send_complete_last = self.data_send_complete

                    if encoded_packets:
                        # 解码后的数据先累积到临时缓冲区
                        temp_buffer = bytearray()
                        for packet in encoded_packets:
                            try:
                                decoded_data, _ = self.audio_codec.decode_audio([packet])
                                if len(decoded_data) > 0:
                                    # 将解码后的数据添加到临时缓冲区
                                    # temp_buffer.extend(decoded_data.tobytes())
                                    self.update_buffer(decoded_data.tobytes())
                                else:
                                    print("解码后的音频数据为空")

                            except Exception as e:
                                print(f"解码单个包时出错: {e}, 包类型: {type(packet)}, 包大小: {len(packet)} 字节")
                                continue
                        # if len(temp_buffer) >= self.CHUNK * 2:  # 确保有足够的数据
                        #     self.play_audio(bytes(temp_buffer))
                        #     analyze_audio_energy(bytes(temp_buffer))
                        #     temp_buffer.clear()
                        # 获取要处理的数据
                        if self.data_send_complete:
                            process_data = self.get_process_data()
                            if process_data:
                                print(f"检测到语音片段，长度: {len(process_data)} 字节")
                                # 播放语音片段
                                self.play_audio(process_data)
                                # current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                                # self.save_pcm(process_data, f"../server/audio_files/pcm/{current_time}.pcm")
                                
                                # 原文字转语音有的处理逻辑
                                text = self.speech_to_text(process_data)
                                if text:
                                    print(f"识别到的文字: {text}")
                                    # 调用大语言模型获取回复
                                    response_text = await self.get_llm_response(text)
                                    print(f"AI回复: {response_text}")
                                    # 
                                    audio_data = self.text_to_speech(response_text)
                                    if audio_data:
                                        await self.send_queue.put(audio_data)
                                else:
                                    print("没有识别到语音, text: ", text)

                        time2 = time.time()
                        print("process_audio_loop period: ", time2 - time1)
                        time1 = time2
                else:
                    audio_data = self.read_audio("../server/audio_files/pcm/qiufeng.pcm")
                    if audio_data:
                        await self.send_queue.put(audio_data)

                await asyncio.sleep(0.01)  # 缩短检测间隔，提高响应速度

            except Exception as e:
                print(f"处理音频数据错误: {e}")
                # await asyncio.sleep(1)

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
        return self.speech_service.speech_to_text(audio_data)

    def save_pcm(self, audio_data, filename):
        """保存PCM格式音频文件"""
        self.audio_saver.save_pcm(audio_data, filename)

    def save_wav(self, audio_data, filename):
        """保存WAV格式音频文件"""
        self.audio_saver.save_wav(audio_data, filename)

    def text_to_speech(self, text):
        """文字转语音"""
        return self.speech_service.text_to_speech(text)

    def read_audio(self, filename):
        """读取音频文件"""
        with open(filename, 'rb') as f:
            return f.read()  # 返回字节数据

    def cleanup(self):
        """清理资源"""
        self.recorder.cleanup()
        self.player.cleanup()
        self.audio_saver.cleanup()

    async def get_llm_response(self, text: str) -> str:
        """获取大语言模型回复"""
        if self.current_llm == "zhipu":
            return await self.zhipu_client.get_response(text)
        else:
            return await self.volcengine_client.get_response(text)

    def play_audio(self, audio_data):
        """将音频数据加入播放队列"""
        self.player.play_audio(audio_data)

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