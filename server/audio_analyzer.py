import numpy as np
import matplotlib.pyplot as plt
import time
import os

def analyze_audio_energy(audio_data: bytes, sample_rate: int = 16000):
    """分析音频能量并打印/绘制图表"""
    try:
        # 初始化音频缓存
        if not hasattr(analyze_audio_energy, 'audio_buffer'):
            analyze_audio_energy.audio_buffer = np.array([], dtype=np.int16)
            analyze_audio_energy.last_save_time = 0

        # 将新的音频数据添加到缓存中
        new_audio = np.frombuffer(audio_data, dtype=np.int16)
        analyze_audio_energy.audio_buffer = np.concatenate([analyze_audio_energy.audio_buffer, new_audio])
        
        # 计算2秒对应的采样点数
        samples_per_2sec = 2 * sample_rate
        
        # 如果缓存的数据超过2秒，则进行处理和绘图
        if len(analyze_audio_energy.audio_buffer) >= samples_per_2sec:
            # 取最近2秒的数据
            audio_array = analyze_audio_energy.audio_buffer[-samples_per_2sec:]
            # 更新缓存，只保留最近2秒的数据
            analyze_audio_energy.audio_buffer = audio_array.copy()
            
            # 计算音频能量
            energy = np.mean(np.abs(audio_array))
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
                # 创建时间轴（2秒）
                time_axis = np.linspace(0, 2, len(audio_array))
                plt.plot(time_axis, audio_array)
                plt.title('Audio Waveform (2s)', fontsize=12)
                plt.xlabel('Time (s)', fontsize=10)
                plt.ylabel('Amplitude', fontsize=10)
                plt.grid(True)
                
                # 实时显示
                plt.draw()
                plt.pause(0.001)
                
                current_time = time.time()
                # 每2秒保存一次图片
                if current_time - analyze_audio_energy.last_save_time >= 2:
                    # 确保目录存在
                    os.makedirs('audio_files/png', exist_ok=True)
                    # 保存图片
                    plt.savefig(f'audio_files/png/audio_waveform_{int(current_time)}.png')
                    analyze_audio_energy.last_save_time = current_time
                
            except Exception as e:
                print(f"绘制波形图错误: {e}")
                
    except Exception as e:
        print(f"音频分析错误: {e}")