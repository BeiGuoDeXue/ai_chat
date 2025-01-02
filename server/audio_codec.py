import opuslib
import numpy as np
import soundfile as sf
import time
from typing import Tuple, List, Optional

class AudioCodec:
    def __init__(self):
        # 基本参数配置
        self.FRAME_SIZE = 960  # 60ms at 16kHz
        self.SAMPLE_RATE = 16000  # 16kHz
        self.CHANNELS = 1  # 单声道

        # 创建编码器和解码器
        self.encoder = opuslib.Encoder(
            fs=self.SAMPLE_RATE, 
            channels=self.CHANNELS, 
            application="voip"
        )
        # 设置编码器参数
        self.encoder.bitrate = 12000  # 设置比特率为12kbps
        self.encoder.complexity = 5    # 设置复杂度
        self.encoder._set_dtx = 1            # 启用DTX

        self.decoder = opuslib.Decoder(
            fs=self.SAMPLE_RATE, 
            channels=self.CHANNELS
        )

    def encode_audio(self, audio_data: np.ndarray) -> Tuple[List[bytes], dict]:
        """
        编码音频数据
        
        Args:
            audio_data: 音频数据数组
            
        Returns:
            Tuple[List[bytes], dict]: 编码后的数据和统计信息
        """
        try:
            encoded_data = []
            start_time = time.time()
            
            for i in range(0, len(audio_data), self.FRAME_SIZE):
                frame = audio_data[i:i + self.FRAME_SIZE]
                if len(frame) < self.FRAME_SIZE:
                    frame = np.pad(frame, (0, self.FRAME_SIZE - len(frame)), 'constant')
                
                frame_bytes = frame.tobytes()
                encoded = self.encoder.encode(frame_bytes, self.FRAME_SIZE)
                encoded_data.append(encoded)
                # print(f"编码后的数据大小: {len(encoded)} 字节")
            
            encoding_time = time.time() - start_time
            total_encoded_size = sum(len(x) for x in encoded_data)
            
            stats = {
                "original_size": len(audio_data) * 2,
                "encoded_size": total_encoded_size,
                "encoding_time": encoding_time,
                "compression_ratio": (len(audio_data) * 2) / total_encoded_size
            }
            
            return encoded_data, stats
            
        except Exception as e:
            print(f"编码错误: {e}")
            raise

    def decode_audio(self, encoded_data: List[bytes], original_length: Optional[int] = None) -> Tuple[np.ndarray, dict]:
        """
        解码音频数据
        
        Args:
            encoded_data: 编码后的数据列表
            original_length: 原始音频长度（可选）
            
        Returns:
            Tuple[np.ndarray, dict]: 解码后的音频数据和统计信息
        """
        try:
            decoded_samples = []
            start_time = time.time()
            
            for encoded in encoded_data:
                decoded = self.decoder.decode(encoded, self.FRAME_SIZE)
                samples = np.frombuffer(decoded, dtype=np.int16)
                decoded_samples.append(samples)
            
            decoded_data = np.concatenate(decoded_samples)
            if original_length is not None:
                decoded_data = decoded_data[:original_length]
            
            decoding_time = time.time() - start_time
            
            stats = {
                "decoded_size": len(decoded_data) * 2,
                "decoding_time": decoding_time
            }
            
            return decoded_data, stats
            
        except Exception as e:
            print(f"解码错误: {e}")
            raise

    def analyze_quality(self, original_data: np.ndarray, decoded_data: np.ndarray) -> dict:
        """
        分析音频质量
        
        Args:
            original_data: 原始音频数据
            decoded_data: 解码后的音频数据
            
        Returns:
            dict: 质量分析结果
        """
        try:
            # 计算相关性
            correlation = np.corrcoef(original_data, decoded_data)[0,1]
            
            # 计算信噪比
            signal_power = np.mean(np.square(original_data.astype(np.float64)))
            noise = original_data.astype(np.float64) - decoded_data.astype(np.float64)
            noise_power = np.mean(np.square(noise))
            snr = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else float('inf')
            
            # 计算差异
            diff = np.abs(original_data - decoded_data)
            
            return {
                "correlation": correlation,
                "snr_db": snr,
                "mean_diff": float(np.mean(diff)),
                "median_diff": float(np.median(diff)),
                "std_diff": float(np.std(diff)),
                "max_diff": float(np.max(diff))
            }
            
        except Exception as e:
            print(f"质量分析错误: {e}")
            raise

    @staticmethod
    def save_wav(filename: str, data: np.ndarray, sample_rate: int = 16000):
        """
        保存WAV文件
        
        Args:
            filename: 文件名
            data: 音频数据
            sample_rate: 采样率
        """
        try:
            sf.write(filename, data, sample_rate)
        except Exception as e:
            print(f"保存文件错误: {e}")
            raise