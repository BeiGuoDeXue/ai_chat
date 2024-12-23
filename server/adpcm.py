import numpy as np

class ADPCMEncoder:
    def __init__(self):
        self.prev_sample = 0
        self.index = 0
        
        self.index_table = [
            -1, -1, -1, -1, 2, 4, 6, 8,
            -1, -1, -1, -1, 2, 4, 6, 8
        ]
        
        self.step_table = [
            7, 8, 9, 10, 11, 12, 13, 14,
            16, 17, 19, 21, 23, 25, 28, 31,
            34, 37, 41, 45, 50, 55, 60, 66,
            73, 80, 88, 97, 107, 118, 130, 143,
            157, 173, 190, 209, 230, 253, 279, 307,
            337, 371, 408, 449, 494, 544, 598, 658,
            724, 796, 876, 963, 1060, 1166, 1282, 1411,
            1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
            3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
            7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
            15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
            32767
        ]

    def encode_buffer(self, pcm_data):
        # 将字节转换为16位整数样本
        samples = np.frombuffer(pcm_data, dtype=np.int16)
        encoded_data = bytearray()
        
        for i in range(0, len(samples), 2):
            high = self.encode(samples[i])
            low = self.encode(samples[i + 1]) if i + 1 < len(samples) else 0
            encoded_data.append((high << 4) | low)
            
        return bytes(encoded_data)

    def decode_buffer(self, adpcm_data):
        decoded_samples = []
        for byte in adpcm_data:
            high = (byte >> 4) & 0x0F
            low = byte & 0x0F
            decoded_samples.append(self.decode(high))
            decoded_samples.append(self.decode(low))
        return np.array(decoded_samples, dtype=np.int16).tobytes()

    def encode(self, sample):
        diff = sample - self.prev_sample
        step = self.step_table[self.index]
        delta = abs(diff)
        code = 0

        if diff < 0:
            code = 8

        if delta >= step:
            code |= 4
            delta -= step
        step >>= 1

        if delta >= step:
            code |= 2
            delta -= step
        step >>= 1

        if delta >= step:
            code |= 1

        diff_new = self.step_table[self.index] >> 3
        if code & 4: diff_new += self.step_table[self.index]
        if code & 2: diff_new += self.step_table[self.index] >> 1
        if code & 1: diff_new += self.step_table[self.index] >> 2
        if code & 8: diff_new = -diff_new

        self.prev_sample += diff_new
        self.prev_sample = min(max(self.prev_sample, -32768), 32767)

        self.index += self.index_table[code]
        self.index = min(max(self.index, 0), 88)

        return code

    def decode(self, code):
        step = self.step_table[self.index]
        diff = step >> 3

        if code & 4: diff += step
        if code & 2: diff += step >> 1
        if code & 1: diff += step >> 2
        if code & 8: diff = -diff

        self.prev_sample += diff
        self.prev_sample = min(max(self.prev_sample, -32768), 32767)

        self.index += self.index_table[code]
        self.index = min(max(self.index, 0), 88)

        return self.prev_sample