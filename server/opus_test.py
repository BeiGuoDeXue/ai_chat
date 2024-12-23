import opuslib
import numpy as np
import soundfile as sf
import time

def test_opus():
    try:
        # 1. 最优参数配置
        FRAME_SIZE = 160  # 10ms at 16kHz - 这是最稳定的帧大小
        SAMPLE_RATE = 16000  # 16kHz采样率
        CHANNELS = 1  # 单声道
        
        # 2. 读取音频
        audio_data, _ = sf.read('text2audio.wav', dtype=np.int16)
        if len(audio_data.shape) > 1:
            audio_data = audio_data[:, 0]
        
        # 3. 创建编码器和解码器 - 使用VOIP模式，因为它对语音优化最好
        encoder = opuslib.Encoder(fs=SAMPLE_RATE, channels=CHANNELS, application="voip")
        decoder = opuslib.Decoder(fs=SAMPLE_RATE, channels=CHANNELS)
        
        print(f"原始音频大小: {len(audio_data) * 2} 字节")
        print(f"采样率: {SAMPLE_RATE} Hz")
        print(f"声道数: {CHANNELS}")
        print(f"前10个样本: {audio_data[:10]}")
        print(f"音频范围: {np.min(audio_data)} 到 {np.max(audio_data)}")
        
        # 4. 编码过程
        encoded_data = []
        start_time = time.time()
        
        for i in range(0, len(audio_data), FRAME_SIZE):
            frame = audio_data[i:i + FRAME_SIZE]
            if len(frame) < FRAME_SIZE:
                frame = np.pad(frame, (0, FRAME_SIZE - len(frame)), 'constant')
            
            try:
                frame_bytes = frame.tobytes()
                encoded = encoder.encode(frame_bytes, FRAME_SIZE)
                encoded_data.append(encoded)
            except Exception as e:
                print(f"编码错误在样本 {i}: {e}")
                raise
        
        encoding_time = time.time() - start_time
        total_encoded_size = sum(len(x) for x in encoded_data)
        print(f"编码时间: {encoding_time:.3f} 秒")
        print(f"编码后总大小: {total_encoded_size} 字节")
        
        # 5. 解码过程
        decoded_samples = []
        start_time = time.time()
        
        for encoded in encoded_data:
            try:
                decoded = decoder.decode(encoded, FRAME_SIZE)
                samples = np.frombuffer(decoded, dtype=np.int16)
                decoded_samples.append(samples)
            except Exception as e:
                print(f"解码错误: {e}")
                raise
        
        decoded_data = np.concatenate(decoded_samples)
        decoded_data = decoded_data[:len(audio_data)]
        
        decoding_time = time.time() - start_time
        print(f"解码时间: {decoding_time:.3f} 秒")
        print(f"解码后大小: {len(decoded_data) * 2} 字节")
        print(f"前10个解码样本: {decoded_data[:10]}")
        print(f"解码数据范围: {np.min(decoded_data)} 到 {np.max(decoded_data)}")
        
        # 6. 计算压缩率和保存文件
        compression_ratio = (len(audio_data) * 2) / total_encoded_size
        print(f"压缩率: {compression_ratio:.2f}x")
        
        sf.write('original_copy.wav', audio_data, SAMPLE_RATE)
        sf.write('decoded.wav', decoded_data, SAMPLE_RATE)
        print("\n音频文件已保存为 'original_copy.wav' 和 'decoded.wav'")
        
        # 7. 音频质量分析
        print("\n音频质量分析:")
        correlation = np.corrcoef(audio_data, decoded_data)[0,1]
        print(f"波形相关性: {correlation:.4f}")
        
        signal_power = np.mean(np.square(audio_data.astype(np.float64)))
        noise = audio_data.astype(np.float64) - decoded_data.astype(np.float64)
        noise_power = np.mean(np.square(noise))
        if noise_power > 0:
            snr = 10 * np.log10(signal_power / noise_power)
            print(f"信噪比 (SNR): {snr:.2f} dB")
        
        diff = np.abs(audio_data - decoded_data)
        print(f"\n差异分析:")
        print(f"平均差异: {np.mean(diff):.2f}")
        print(f"中位数差异: {np.median(diff):.2f}")
        print(f"标准差: {np.std(diff):.2f}")
        print(f"最大差异: {np.max(diff)}")
        
    except Exception as e:
        print(f"错误: {e}")
        print(f"错误类型: {type(e)}")
        import traceback
        traceback.print_exc()
        return

if __name__ == "__main__":
    test_opus()