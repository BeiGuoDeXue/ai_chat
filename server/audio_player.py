import pyaudio
import queue
import threading

class AudioPlayer:
    def __init__(self):
        # 音频配置
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # 初始化 PyAudio
        self.audio = pyaudio.PyAudio()
        self.output_stream = None
        
        # 添加播放队列和播放线程
        self.play_queue = queue.Queue()
        self.play_thread = threading.Thread(target=self._play_audio_worker)
        self.play_thread.daemon = True
        self.play_thread.start()

    def play_audio(self, audio_data):
        """将音频数据加入播放队列"""
        try:
            self.play_queue.put(audio_data)
        except Exception as e:
            print(f"添加音频到播放队列时出错: {e}")

    def _play_audio_worker(self):
        """音频播放工作线程"""
        while True:
            try:
                audio_data = self.play_queue.get()
                if audio_data is None:  # 用于停止线程的信号
                    break
                    
                if not self.output_stream:
                    self.output_stream = self.audio.open(
                        format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        output=True
                    )
                    
                self.output_stream.write(audio_data)
                self.play_queue.task_done()
                
            except Exception as e:
                print(f"播放音频线程错误: {e}")
                self.close_output_stream()

    def close_output_stream(self):
        """关闭音频输出流"""
        if self.output_stream:
            try:
                self.output_stream.stop_stream()
                self.output_stream.close()
            except Exception as e:
                print(f"关闭输出流时出错: {e}")
            finally:
                self.output_stream = None

    def cleanup(self):
        """清理资源"""
        # 停止播放线程
        self.play_queue.put(None)
        self.play_thread.join()
        self.close_output_stream()
        self.audio.terminate() 