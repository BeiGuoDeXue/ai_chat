import numpy as np
import matplotlib.pyplot as plt
import time

def analyze_audio_energy(audio_data: bytes, sample_rate: int = 16000):
    """分析音频能量并打印/绘制图表"""
    try:
        # 将字节数据转换为16位整数数组
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
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
            time_axis = np.arange(len(audio_array)) / sample_rate
            plt.plot(time_axis, audio_array)
            plt.title('Audio Waveform', fontsize=12)
            plt.xlabel('Time (s)', fontsize=10)
            plt.ylabel('Amplitude', fontsize=10)
            plt.grid(True)
            
            # 实时显示
            plt.draw()
            plt.pause(0.001)
            
            # 保存图片
            plt.savefig(f'server/audio_files/png/audio_waveform_{int(time.time())}.png')
            
        except Exception as e:
            print(f"绘制波形图错误: {e}")
            
    except Exception as e:
        print(f"音频分析错误: {e}")