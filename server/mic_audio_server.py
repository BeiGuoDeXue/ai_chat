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
from dotenv import load_dotenv
import json
import base64
import sys
from audio_analyzer import analyze_audio_energy
import itertools
import datetime
import queue
from audio_player import AudioPlayer
from speech_service import SpeechService
from audio_saver import AudioSaver
from vad_detector import VadDetector
from queue import Queue
import concurrent.futures

# 设置控制台输出编码
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

load_dotenv()

class AudioChatServer:
    def __init__(self):
        # 初始化语音服务
        self.speech_service = SpeechService()

        # 音频配置
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # 初始化音频编解码器
        self.audio_codec = AudioCodec()
        
        # 用于线程间通信的队列
        self.audio_queue = Queue()        # 原始音频数据队列
        self.result_queue = Queue()       # 处理后的音频数据队列
        self.send_queue = Queue()         # 发送队列
        
        # 线程控制标志
        self.running = True
        
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
        
        # 初始化音频播放器和保存器
        self.player = AudioPlayer()

        # 初始化音频保存器
        self.audio_saver = AudioSaver(
            channels=self.CHANNELS,
            sample_format=self.FORMAT,
            rate=self.RATE
        )
        
        # 添加 Event 用于流控制
        self.flow_control_event = asyncio.Event()
        self.flow_control_event.set()  # 默认设置为可发送状态

        # 发送完成信号
        self.send_data_complete = True
        
        # 修改 flow_control_paused 的处理
        
        self.flow_control_paused = False
        self._websocket_lock = threading.Lock()
        self._current_websocket = None

        # 创建并启动线程
        self.start_threads()

        self.loop = None  # 添加事件循环引用

        self.test_play_music_mode = False  # 添加测试模式标志
        self.test_reply_mode = False  # 添加测试模式标志
        self.play_test_audio = True

    def start_threads(self):
        """创建并启动所有工作线程"""
        # 语音服务线程
        self.speech_thread = threading.Thread(
            target=self.speech_service_worker,
            name="Speech-Service"
        )
        
        # 发送线程
        self.send_thread = threading.Thread(
            target=self.send_worker,
            name="Send-Worker"
        )
        
        # 设置为守护线程
        self.speech_thread.daemon = True
        self.send_thread.daemon = True
        
        # 启动所有线程
        self.speech_thread.start()
        self.send_thread.start()

    def speech_service_worker(self):
        """处理语音服务的线程"""
        time0 = time.time()
        while self.running:
            try:
                audio_chunks = []
                # 一次性读取所有可用数据
                while True:
                    try:
                        audio_data = self.result_queue.get_nowait()
                        if audio_data is None:
                            return
                        audio_chunks.append(audio_data)
                    except queue.Empty:
                        break

                if not audio_chunks:
                    time.sleep(0.1)  # 避免空转
                    continue

                # 合并音频数据
                combined_audio = b''.join(audio_chunks)
                if not self.send_data_complete:
                    print("发送中，清空结果队列")
                    # self.result_queue
                    continue
                if self.test_play_music_mode:
                    audio_data = self.read_audio("../server/audio_files/pcm/qiufeng.pcm")
                    if audio_data:
                        self.send_queue.put(audio_data)
                    continue

                try:
                    # VAD检测
                    processed_audio = self.vad_detector.process_audio(combined_audio)
                    if processed_audio:
                        if self.test_reply_mode:
                            # 测试模式：直接回放音频
                            self.send_queue.put(processed_audio)
                        else:
                            # 播放音频（如果需要）
                            if self.play_test_audio and self.player:
                                self.player.play_audio(processed_audio)
                            # 正常模式：语音识别和处理
                            text = self.speech_service.speech_to_text(processed_audio)
                            if text:
                                print(f"识别到的文字: {text}")
                                response = self.get_llm_response_sync(text)
                                if response:
                                    print(f"AI回复: {response}")
                                    audio = self.speech_service.text_to_speech(response)
                                    if audio:
                                        self.send_queue.put(audio)
                                    else:
                                        print("text_to_speech 失败")
                                else:
                                    print("AI回复为空")
                            else:
                                print("语音识别失败")
                except Exception as e:
                    print(f"语音服务错误: {e}")
                
                time1 = time.time()
                if time1 - time0 > 1:
                    print("loop 1s")
                    time0 = time1
            except Exception as e:
                print(f"语音处理线程错误: {e}")

    async def handle_websocket(self, websocket):
        """WebSocket消息处理"""
        try:
            # 保存当前 websocket 连接
            self.current_websocket = websocket
            print(f"新的WebSocket连接已建立: {id(websocket)}")  # 添加日志
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    # 处理流控制消息
                    if data.get('type') == 'flow_control':
                        if 'pause' in data:
                            self.flow_control_paused = data['pause']
                            if self.flow_control_paused:
                                self.flow_control_event.clear()
                            else:
                                self.flow_control_event.set()
                            print(f"收到流控制消息: {'暂停' if self.flow_control_paused else '继续'} 发送")
                        continue
                    
                    # 处理音频数据
                    if data.get('type') == 'audio_batch':
                        sequence = data.get('sequence')
                        audio_data = base64.b64decode(data.get('data', ''))
                        length = data.get('length', 0)

                        # 验证数据长度
                        if len(audio_data) == length:
                            # print(f"收到音频数据: sequence={sequence}, size={length}")
                            try:
                                # 解码音频
                                decoded_data, _ = self.audio_codec.decode_audio([audio_data])
                                if len(decoded_data) > 0:
                                    # 直接放入结果队列供语音服务处理
                                    self.result_queue.put(decoded_data.tobytes())
                            except Exception as e:
                                print(f"处理音频包错误: {e}")
                        else:
                            print(f"音频数据长度不匹配: 收到={len(audio_data)}, 预期={length}")

                except json.JSONDecodeError as e:
                    print(f"JSON解析错误: {e}, 消息内容: {message[:100]}")
                except Exception as e:
                    print(f"处理消息错误: {type(e).__name__}: {str(e)}")

        except websockets.exceptions.ConnectionClosed:
            print("WebSocket连接关闭")
        finally:
            print(f"WebSocket连接断开: {id(websocket)}")  # 添加日志
            self.current_websocket = None

    def send_worker(self):
        """处理发送队列的线程"""
        while self.running:
            try:
                # 从发送队列获取音频数据
                audio_data = self.send_queue.get(timeout=1)
                if audio_data is None:
                    break
                    
                # 增加更详细的连接状态检查
                if not self.current_websocket:
                    print("WebSocket实例不存在")
                    continue
                    
                if not hasattr(self.current_websocket, 'open'):
                    print("WebSocket实例无效")
                    continue
                    
                print(f"当前WebSocket状态: {self.current_websocket.open}, ID: {id(self.current_websocket)}")
                
                if self.current_websocket and self.current_websocket.open:
                    print(f"准备发送音频数据: {len(audio_data)}字节")
                    if self.loop:
                        try:
                            future = asyncio.run_coroutine_threadsafe(
                                self.send_audio_packets(self.current_websocket, audio_data),
                                self.loop
                            )
                            # 添加超时机制
                            future.result(timeout=10)  # 10秒超时
                            print("音频数据发送完成")
                        except concurrent.futures.TimeoutError:
                            print("发送音频数据超时")
                        except Exception as e:
                            print(f"发送音频数据时发生错误: {e}")
                    else:
                        print("事件循环未初始化")
                else:
                    print("WebSocket未连接")

            except queue.Empty:
                continue
            except Exception as e:
                print(f"发送线程错误: {e}")

    def get_llm_response_sync(self, text: str) -> str:
        """同步方式获取LLM响应"""
        try:
            if self.current_llm == "zhipu":
                return asyncio.run(self.zhipu_client.get_response(text))
            else:
                return asyncio.run(self.volcengine_client.get_response(text))
        except Exception as e:
            print(f"获取LLM响应错误: {e}")
            return None

    async def run_websocket_server(self):
        """运行WebSocket服务器"""
        try:
            # 保存事件循环引用
            self.loop = asyncio.get_running_loop()
            print("事件循环已初始化")
            
            async with websockets.serve(
                self.handle_websocket,
                "0.0.0.0",
                80,
                process_request=lambda path, request_headers: (
                    None if path == "/ws/esp32-client" 
                    else (404, [], b"Not Found")
                )
            ):
                print("WebSocket server started on ws://0.0.0.0:80/ws/esp32-client")
                await asyncio.Future()  # 保持服务器运行
        except Exception as e:
            print(f"WebSocket服务器错误: {e}")
        finally:
            self.cleanup()

    def read_audio(self, filename):
        """读取音频文件"""
        with open(filename, 'rb') as f:
            return f.read()  # 返回字节数据

    def cleanup(self):
        """清理资源"""
        self.running = False
        # 向队列发送终止信号
        self.audio_queue.put(None)
        self.result_queue.put(None)
        self.send_queue.put(None)
        
        # 清理各个组件
        self.player.cleanup()
        self.audio_saver.cleanup()
        
        # 等待线程结束
        if hasattr(self, 'speech_thread'):
            self.speech_thread.join()
        if hasattr(self, 'send_thread'):
            self.send_thread.join()

    async def send_audio_packets(self, websocket, audio_data):
        """发送音频数据包"""
        try:
            if not websocket or not websocket.open:
                print("WebSocket未连接")
                return
            
            # 1. 编码音频
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            time0 = time.time()
            encoded_packets, _ = self.audio_codec.encode_audio(audio_array)
            time1 = time.time()
            print(f"音频编码时间: {time1 - time0}秒")
            if not encoded_packets:
                print("音频编码失败")
                return

            self.send_data_complete = False
            # 2. 发送控制消息并处理音频包
            await self._send_control(websocket, "start_playing")
            await self._send_audio_batches(websocket, encoded_packets)
            await self._send_control(websocket, "stop_playing")
            self.send_data_complete = True

            await asyncio.sleep(0.5)  # 给客户端处理时间

        except Exception as e:
            print(f"发送音频数据错误: {e}")

    async def _send_control(self, websocket, action):
        """发送控制消息"""
        await websocket.send(json.dumps({
            "type": "control",
            "action": f"{action}"
        }))
        print(f"发送{action}控制消息")

    async def _send_audio_batches(self, websocket, encoded_packets):
        """分批发送音频数据"""
        current_batch = []
        current_size = 0
        batch_number = sequence_id = 0
        
        for packet in encoded_packets:
            # 使用 Event 等待流控制
            await self.flow_control_event.wait()
            
            packet_data = {
                "data": base64.b64encode(packet).decode('utf-8'),
                "length": len(packet)
            }
            json_size = len(json.dumps(packet_data).encode('utf-8'))
            
            if current_size + json_size > 512 and current_batch:
                await self._send_batch(websocket, current_batch, batch_number, sequence_id)
                current_batch = []
                current_size = 0
                batch_number += 1
                sequence_id += 1
                await asyncio.sleep(0.1)
            
            current_batch.append(packet_data)
            current_size += json_size
        
        if current_batch:
            await self._send_batch(websocket, current_batch, batch_number, sequence_id)

    async def _send_batch(self, websocket, batch, batch_number, sequence_id):
        """发送单个音频数据批次"""
        try:
            batch_data = {
                "type": "audio_batch",
                "sequence": sequence_id,
                "batch": batch_number,
                "total_packets": len(batch),
                "packets": batch
            }
            await websocket.send(json.dumps(batch_data))
            print(f"发送批次 {batch_number}: {len(batch)}个包, sequence={sequence_id}")
        except Exception as e:
            print(f"发送批次失败: {e}")

    @property
    def current_websocket(self):
        with self._websocket_lock:
            return self._current_websocket

    @current_websocket.setter
    def current_websocket(self, websocket):
        with self._websocket_lock:
            self._current_websocket = websocket
            if websocket:
                print(f"设置新的WebSocket连接: {id(websocket)}")
            else:
                print("清除WebSocket连接")

if __name__ == "__main__":
    server = AudioChatServer()
    try:
        # 直接在主线程中运行 WebSocket 服务器
        asyncio.run(server.run_websocket_server())
    except KeyboardInterrupt:
        print("\n正在关闭服务器...")
    finally:
        server.cleanup()
