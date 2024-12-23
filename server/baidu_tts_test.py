import asyncio
import json
import base64
from aip import AipSpeech
import numpy as np
import time
import wave
import pyaudio


class AudioChatServer:
    def __init__(self):
        # 百度语音配置
        self.APP_ID = '116470106'
        self.API_KEY = 'ibrjXDUQ6FgjrvSQJ4GANDBA'
        self.SECRET_KEY = '0xZe4laRgQbZrxAECCcKqeRZ1R8j3Jq1'
        self.baidu_client = AipSpeech(self.APP_ID, self.API_KEY, self.SECRET_KEY)
    async def process_audio_loop(self):
        # while True:
            try:
                """定期处理音频数据的循环"""
                response_text = "赵康旭"
                # 文字转语音
                audio_data = self.text_to_speech(response_text)
                
                # 保存音频文件
                if audio_data:
                    # 保存为WAV格式
                    self.save_audio(audio_data, "output.wav")
                    # 保存为PCM格式
                    self.save_pcm(audio_data, "output.pcm")
                    # 播放音频
                    self.play_audio("output.wav")
                
                await asyncio.sleep(2)

            except Exception as e:
                print(f"处理音频数据错误: {e}")
                self.is_processing = False
                await asyncio.sleep(1)

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
                return None
            
            return result

        except Exception as e:
            print(f"语音合成异常: {e}")
            return None

    def save_audio(self, audio_data, filename):
        """保存WAV格式音频文件"""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)  # 单声道
            wf.setsampwidth(2)  # 2字节采样
            wf.setframerate(16000)  # 采样率16k
            wf.writeframes(audio_data)
        print(f"WAV音频已保存到: {filename}")

    def save_pcm(self, audio_data, filename):
        """保存PCM格式音频文件"""
        with open(filename, 'wb') as f:
            f.write(audio_data)
        print(f"PCM音频已保存到: {filename}")

    def play_audio(self, filename):
        """播放音频文件"""
        chunk = 1024
        wf = wave.open(filename, 'rb')
        p = pyaudio.PyAudio()

        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                       channels=wf.getnchannels(),
                       rate=wf.getframerate(),
                       output=True)

        data = wf.readframes(chunk)
        while data:
            stream.write(data)
            data = wf.readframes(chunk)

        stream.stop_stream()
        stream.close()
        p.terminate()
        print("音频播放完成")


async def main():
    server = AudioChatServer()
    await server.process_audio_loop()
if __name__ == "__main__":
    asyncio.run(main())