import asyncio
import websockets
from aip import AipSpeech
import numpy as np
from collections import deque
import time
from audio_codec import AudioCodec
import threading

class AudioChatServer:
    def __init__(self):
        # 百度语音配置
        self.APP_ID = '116470106'
        self.API_KEY = 'ibrjXDUQ6FgjrvSQJ4GANDBA'
        self.SECRET_KEY = '0xZe4laRgQbZrxAECCcKqeRZ1R8j3Jq1'
        self.baidu_client = AipSpeech(self.APP_ID, self.API_KEY, self.SECRET_KEY)
        
        # 循环缓冲区配置
        self.BUFFER_SECONDS = 10
        self.SAMPLE_RATE = 16000
        self.BYTES_PER_SAMPLE = 2
        self.BUFFER_SIZE = self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.BUFFER_SECONDS
        self.PROCESS_SIZE = 64000  # 处理单位大小(约2秒的数据)
        
        # 初始化循环缓冲区
        self.audio_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.raw_buffer = deque()  # 存储原始编码数据
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

        # 语音状态
        self.is_speaking = False
        self.silence_frames = 0

    async def handle_websocket(self, websocket):
        print("新客户端连接!")
        
        # 启动发送协程
        send_task = asyncio.create_task(self.send_worker(websocket))
        # 启动处理音频的循环
        process_task = asyncio.create_task(self.process_audio_loop())

        try:
            async for message in websocket:
                try:
                    if isinstance(message, bytes):
                        # 使用锁保护raw_buffer的写入
                        with self.buffer_lock:
                            self.raw_buffer.append(message)
                except Exception as e:
                    print(f"处理消息错误: {e}")
                    continue
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

    async def send_audio_packets(self, websocket, audio_data):
        """分包发送音频数据"""
        try:
            if websocket.closed:
                print("WebSocket连接已关闭，无法发送数据")
                return
            
            # 将音频数据转换为numpy数组
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            
            # 打印原始数据大小
            print(f"原始音频数据大小: {len(audio_array.tobytes())} 字节")
            
            # 编码音频数据
            encoded_packets, stats = self.audio_codec.encode_audio(audio_array)
            
            # 直接发送每个编码后的数据包
            for packet in encoded_packets:
                await websocket.send(packet)
                await asyncio.sleep(0.05)  # 短暂延迟，避免发送过快

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
                # encoded_data = []
                # # 使用锁保护raw_buffer的读取
                # with self.buffer_lock:
                #     while self.raw_buffer:
                #         encoded_data.append(self.raw_buffer.popleft())
                
                # if encoded_data:
                #     # 解码数据
                #     decoded_data, _ = self.audio_codec.decode_audio(encoded_data)
                #     # 更新缓冲区
                #     self.update_buffer(decoded_data.tobytes())

                #     # 获取要处理的数据
                #     process_data = self.get_process_data()
                #     if process_data:
                #         # # 检测到语音，进行处理
                #         # text = self.speech_to_text(process_data)
                #         # if text:
                #             # print(f"识别到的文字: {text}")
                #             # 处理响应


                response_text = "李梦丽"
                print(f"回复: {response_text}")
                audio_data = self.read_audio("server/audio_files/beijingkejiguan.pcm")
                await self.send_queue.put(audio_data)
                await asyncio.sleep(6)

            except Exception as e:
                print(f"处理音频数据错误: {e}")
                self.is_processing = False
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

    def text_to_speech(self, text):
        """文字转语音"""
        try:
            result = self.baidu_client.synthesis(text, 'zh', 1, {
                'spd': 5,  # 语速，取值0-15
                'pit': 5,  # 音调，取值0-15
                'vol': 5,  # 音量，取值0-15
                'per': 0,  # 发音人，0为女声，1为男声，3为情感男声，4为情感女声
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

async def main():
    server = AudioChatServer()
    # 创建websocket服务器
    async with websockets.serve(
        lambda websocket, path: server.handle_websocket(websocket) if path == "/ws/esp32-client" else None,
        "0.0.0.0", 
        80
    ):
        print("WebSocket server started on ws://0.0.0.0:80/ws/esp32-client")
        await asyncio.Future()  # 运行直到被中断

if __name__ == "__main__":
    asyncio.run(main())