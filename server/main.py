import asyncio
import json
import base64
import websockets
from aip import AipSpeech
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import time

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
        self.buffer_count = 0  # 当前缓冲区中的数据量
        
        # 发送队列
        self.send_queue = asyncio.Queue()
        self.is_processing = False

    async def handle_websocket(self, websocket):
        print("新客户端连接!")
        
        # 启动发送协程
        send_task = asyncio.create_task(self.send_worker(websocket))
        # 启动处理音频的循环
        process_task = asyncio.create_task(self.process_audio_loop())

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data['type'] == 'audio':
                        audio_data = base64.b64decode(data['data'])
                        # 更新缓冲区
                        self.update_buffer(audio_data)
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

    def update_buffer(self, new_data):
        """更新循环缓冲区"""
        # 将新数据逐字节添加到缓冲区
        for byte in new_data:
            self.audio_buffer.append(byte)
            self.buffer_count = min(self.buffer_count + 1, self.BUFFER_SIZE)

    def analyze_audio_energy(self, data):
        """分析音频能量并打印/绘制图表"""
        # 将字节数据转换为16位整数数组
        audio_array = np.frombuffer(data, dtype=np.int16)
        
        # 计算音频能量
        energy = np.mean(np.abs(audio_array))
        
        # 计算更详细的统计信息
        max_amplitude = np.max(np.abs(audio_array))
        min_amplitude = np.min(np.abs(audio_array))
        std_amplitude = np.std(audio_array)
        
        print(f"音频能量: {energy:.2f}")
        print(f"最大振幅: {max_amplitude}")
        print(f"最小振幅: {min_amplitude}")
        print(f"标准差: {std_amplitude:.2f}")
        
        # 绘制波形图
        try:
            plt.clf()  # 清除当前图形
            # 创建时间轴
            time_axis = np.arange(len(audio_array)) / self.SAMPLE_RATE
            plt.plot(time_axis, audio_array)
            plt.title('Audio Waveform', fontsize=12)  # 使用英文避免乱码
            plt.xlabel('Time (s)', fontsize=10)
            plt.ylabel('Amplitude', fontsize=10)
            plt.grid(True)
            
            # 实时显示
            plt.draw()
            plt.pause(0.001)  # 短暂暂停以更新显示
            
            # 保存图片（可选）
            plt.savefig(f'audio_waveform_{int(time.time())}.png')
            
        except Exception as e:
            print(f"绘制波形图错误: {e}")

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
        
        # 初始化语音状态（如果未初始化）
        if not hasattr(self, 'is_speaking'):
            self.is_speaking = False
            self.silence_frames = 0
        
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

        # 分析音频能量（用于调试）
        # self.analyze_audio_energy(bytes(data))

    async def process_audio_loop(self):
        """定期处理音频数据的循环"""
        while True:
            try:
                # 检查是否有足够的数据进行处理
                if self.buffer_count >= self.PROCESS_SIZE and not self.is_processing:
                    self.is_processing = True
                    
                    # 获取要处理的数据
                    process_data = self.get_process_data()
                    if process_data:
                        # 语音转文字
                        text = self.speech_to_text(process_data)
                        if text:
                            print(f"识别到的文字: {text}")
                            
                            # 处理响应
                            response_text = "这是一个测试回复"
                            print(f"回复: {response_text}")
                            
                            # 文字转语音
                            audio_packets = self.text_to_speech(response_text)
                            # 先注释掉，调试下数据接收部分
                            if audio_packets:
                                await self.send_queue.put(audio_packets)
                    
                    self.is_processing = False
                
                # 短暂休眠
                await asyncio.sleep(0.1)
                
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
                    print(f"发送音频数据错误: {e}")
                finally:
                    self.send_queue.task_done()
        except asyncio.CancelledError:
            print("发送协程被取消")
            raise

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
            
            # 固定每个数据包的大小为2408字节
            PACKET_SIZE = 2408
            packets = []
            total_packets = (len(result) + PACKET_SIZE - 1) // PACKET_SIZE
            
            for i in range(total_packets):
                start = i * PACKET_SIZE
                end = min(start + PACKET_SIZE, len(result))
                packet_data = result[start:end]
                
                packets.append({
                    'sequence': i,
                    'total': total_packets,
                    'data': base64.b64encode(packet_data).decode('utf-8')
                })
            
            return packets
            
        except Exception as e:
            print(f"语音合成异常: {e}")
            return []

    async def send_audio_packets(self, websocket, packets):
        """发送音频数据包"""
        try:
            for packet in packets:
                message = {
                    'type': 'audio',
                    'sequence': packet['sequence'],
                    'total': packet['total'],
                    'data': packet['data']
                }
                await websocket.send(json.dumps(message))
                # 每个包之间添加小延时，避免发送过快
                await asyncio.sleep(0.01)
        except Exception as e:
            print(f"发送数据包错误: {e}")

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