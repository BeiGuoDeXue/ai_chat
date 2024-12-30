from pydub import AudioSegment
import os
import sys

# 设置控制台输出编码
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')


def convert_mp3_to_pcm(input_dir, output_dir):
    """将目录下的所有MP3文件转换为PCM格式"""
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 遍历输入目录中的所有文件
    for filename in os.listdir(input_dir):
        if filename.lower().endswith('.mp3'):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, os.path.splitext(filename)[0] + '.pcm')
            
            print(f"正在转换: {filename}")
            try:
                # 读取MP3
                audio = AudioSegment.from_mp3(input_path)
                
                # 转换为指定格式：16kHz, 16bit, mono
                audio = audio.set_frame_rate(16000).set_channels(1).set_sample_width(2)
                
                # 导出为PCM
                with open(output_path, 'wb') as f:
                    f.write(audio.raw_data)
                    
                print(f"转换完成: {os.path.basename(output_path)}")
                
            except Exception as e:
                print(f"转换失败 {filename}: {e}")

if __name__ == "__main__":
    # 设置输入和输出目录
    input_directory = "../audio_files/mp3"
    output_directory = "../audio_files/pcm"
    
    convert_mp3_to_pcm(input_directory, output_directory)