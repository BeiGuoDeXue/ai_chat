import pyaudio
import threading
from collections import deque

class AudioRecorder:
    def __init__(self):
        # 音频配置
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # 初始化 PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # 录音状态控制
        self.is_recording = False
        self.recording_thread = None
        
        # 添加锁
        self.buffer_lock = threading.Lock()
        
        # 循环缓冲区配置
        self.BUFFER_SECONDS = 10
        self.SAMPLE_RATE = 16000
        self.BYTES_PER_SAMPLE = 2
        self.BUFFER_SIZE = self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.BUFFER_SECONDS
        
        # 初始化循环缓冲区
        self.audio_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.buffer_count = 0

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
            try:
                # 读取音频数据
                audio_data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                
                # 更新缓冲区
                with self.buffer_lock:
                    self.update_buffer(audio_data)

            except Exception as e:
                print(f"录音错误: {e}")
                break

    def update_buffer(self, new_data):
        """更新循环缓冲区"""
        for byte in new_data:
            if self.buffer_count < self.BUFFER_SIZE:
                self.audio_buffer.append(byte)
                self.buffer_count += 1
            else:
                self.audio_buffer.popleft()
                self.audio_buffer.append(byte)

    def cleanup(self):
        """清理资源"""
        self.stop_recording()
        self.audio.terminate()