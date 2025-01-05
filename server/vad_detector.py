import webrtcvad
import itertools

class VadDetector:
    def __init__(self, sample_rate=16000, vad_mode=3):
        # 初始化 WebRTC VAD
        self.vad = webrtcvad.Vad()
        # 设置VAD的激进程度 (0-3)，数字越大越激进
        # 0: 最不激进，容易检测到语音
        # 3: 最激进，只检测很确定的语音
        self.vad.set_mode(vad_mode)
        
        # VAD 相关参数
        self.RATE = sample_rate
        self.vad_frame_duration = 30  # 每帧持续时间(ms)，可选值：10, 20, 30
        self.vad_frame_size = int(self.RATE * self.vad_frame_duration / 1000) * 2  # 每帧样本数
        self.vad_active_frames = 0  # 连续检测到语音的帧数
        self.vad_inactive_frames = 0  # 连续检测到静音的帧数
        self.speech_started = False  # 是否开始检测到语音
        self.speech_buffer = bytearray()  # 存储语音数据

    def process_audio(self, frame_data):
        """处理音频数据并检测语音"""
        try:
            # 将数据分割成VAD帧大小的块
            frames = [frame_data[i:i+self.vad_frame_size] 
                     for i in range(0, len(frame_data), self.vad_frame_size)
                     if len(frame_data[i:i+self.vad_frame_size]) == self.vad_frame_size]
            
            speech_detected = False
            for frame in frames:
                try:
                    is_speech = self.vad.is_speech(bytes(frame), self.RATE)
                    
                    if is_speech:
                        self.vad_active_frames += 1
                        self.vad_inactive_frames = 0
                        if self.vad_active_frames >= 3:  # 连续3帧检测到语音才开始记录
                            self.speech_started = True
                            speech_detected = True
                    else:
                        self.vad_inactive_frames += 1
                        self.vad_active_frames = 0
                    
                    # 如果已经开始检测到语音，将数据添加到语音缓冲区
                    if self.speech_started:
                        self.speech_buffer.extend(frame)

                except Exception as e:
                    print(f"处理单个帧时出错: {e}")
                    continue

            if speech_detected:
                print("检测到语音开始")

            # 检查是否需要结束当前语音段
            if self.speech_started and self.vad_inactive_frames >= 15:
                print("检测到语音结束")
                self.speech_started = False
                speech_data = bytes(self.speech_buffer)
                self.speech_buffer.clear()
                self.vad_active_frames = 0
                self.vad_inactive_frames = 0
                
                # 检查语音长度是否足够
                if len(speech_data) > self.RATE * 1.5:  # 至少1.5秒的语音
                    print(f"返回语音片段，长度: {len(speech_data)}字节")
                    return speech_data
                else:
                    print(f"语音长度不足: {len(speech_data)}字节")
            
            return None

        except Exception as e:
            print(f"处理音频数据时发生错误: {e}")
            return None

    def reset(self):
        """重置VAD检测器状态"""
        self.vad_active_frames = 0
        self.vad_inactive_frames = 0
        self.speech_started = False
        self.speech_buffer.clear() 