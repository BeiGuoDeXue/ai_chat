import wave
import pyaudio

class AudioSaver:
    def __init__(self, channels=1, sample_format=pyaudio.paInt16, rate=16000):
        self.CHANNELS = channels
        self.FORMAT = sample_format
        self.RATE = rate
        self.audio = pyaudio.PyAudio()

    def save_pcm(self, audio_data, filename):
        """保存PCM格式音频文件"""
        try:
            with open(filename, 'wb') as f:
                f.write(audio_data)
            print(f"PCM音频已保存到: {filename}")
        except Exception as e:
            print(f"保存PCM文件时出错: {e}")

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

    def cleanup(self):
        """清理资源"""
        self.audio.terminate() 