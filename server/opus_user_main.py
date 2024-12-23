from audio_codec import AudioCodec
import soundfile as sf
import numpy as np
import os

def process_audio(input_file: str, output_file: str, sample_rate=16000) -> dict:
    """
    处理音频文件 (支持PCM和WAV格式)
    
    Args:
        input_file: 输入文件路径 (.pcm 或 .wav)
        output_file: 输出文件路径
        sample_rate: 采样率 (默认16000Hz)
    """
    try:
        # 检测输入文件格式
        file_ext = os.path.splitext(input_file)[1].lower()
        
        # 读取音频数据
        if file_ext == '.pcm':
            audio_data = np.fromfile(input_file, dtype=np.int16)
        elif file_ext == '.wav':
            audio_data, file_sample_rate = sf.read(input_file, dtype=np.int16)
            if file_sample_rate != sample_rate:
                print(f"警告：输入文件采样率 ({file_sample_rate}Hz) 与期望采样率 ({sample_rate}Hz) 不符")
        else:
            raise ValueError(f"不支持的文件格式: {file_ext}")
            
        # 确保是单声道
        if len(audio_data.shape) > 1:
            audio_data = audio_data[:, 0]

        # 创建编解码器实例
        codec = AudioCodec()
        
        # 编码
        encoded_data, encode_stats = codec.encode_audio(audio_data)
        
        # 解码
        decoded_data, decode_stats = codec.decode_audio(encoded_data, len(audio_data))
        
        # 分析质量
        quality_stats = codec.analyze_quality(audio_data, decoded_data)
        
        # 保存解码后的音频
        codec.save_wav(output_file, decoded_data)
        
        # 合并所有统计信息
        stats = {
            "encode_stats": encode_stats,
            "decode_stats": decode_stats,
            "quality_stats": quality_stats
        }
        
        return stats
        
    except Exception as e:
        print(f"处理音频错误: {e}")
        raise

# 测试代码
if __name__ == "__main__":
    try:
        stats = process_audio("beijingkejiguan.pcm", "opus_output1.wav")
        print("\n处理结果统计:")
        print(f"原始大小: {stats['encode_stats']['original_size']} 字节")
        print(f"压缩后大小: {stats['encode_stats']['encoded_size']} 字节")
        print(f"压缩率: {stats['encode_stats']['compression_ratio']:.2f}x")
        print(f"编码时间: {stats['encode_stats']['encoding_time']:.3f} 秒")
        print(f"解码时间: {stats['decode_stats']['decoding_time']:.3f} 秒")
        print("\n音频质量分析:")
        print(f"波形相关性: {stats['quality_stats']['correlation']:.4f}")
        print(f"信噪比: {stats['quality_stats']['snr_db']:.2f} dB")
        print(f"平均差异: {stats['quality_stats']['mean_diff']:.2f}")
        print(f"最大差异: {stats['quality_stats']['max_diff']}")
        
    except Exception as e:
        print(f"程序错误: {e}")

# 测试代码
# stats = process_audio("input.wav", "output.wav")  # WAV格式
# stats = process_audio("input.pcm", "output.wav")  # PCM格式