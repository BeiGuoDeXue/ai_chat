import wave
import struct

def pcm_to_wav(pcm_file, wav_file, channels=1, sample_width=2, sample_rate=16000):
    """
    将PCM文件转换为WAV文件
    
    参数:
    pcm_file: PCM文件路径
    wav_file: 输出WAV文件路径
    channels: 声道数（默认为1，即单声道）
    sample_width: 采样宽度（字节数，默认为2）
    sample_rate: 采样率（默认为16000Hz）
    """
    
    # 读取PCM文件数据
    with open(pcm_file, 'rb') as pcm:
        pcm_data = pcm.read()
    
    # 创建WAV文件
    with wave.open(wav_file, 'wb') as wav:
        # 设置WAV文件的参数
        wav.setnchannels(channels)        # 声道数
        wav.setsampwidth(sample_width)    # 采样宽度
        wav.setframerate(sample_rate)     # 采样率
        wav.writeframes(pcm_data)         # 写入PCM数据

# 使用示例
if __name__ == "__main__":
    # 设置输入输出文件路径
    input_pcm = "output1.pcm"
    output_wav = "output1.wav"
    
    # 转换文件
    pcm_to_wav(input_pcm, output_wav)
    print(f"转换完成：{input_pcm} -> {output_wav}")