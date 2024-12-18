#ifndef ADPCM_H
#define ADPCM_H

#include <stdint.h>
#include <stddef.h>

class ADPCMEncoder {
private:
    int prevSample = 0;
    int index = 0;
    static const int8_t indexTable[16];
    static const int16_t stepTable[89];

public:
    ADPCMEncoder() : prevSample(0), index(0) {}
    
    // 编码单个16位PCM样本为4位ADPCM
    uint8_t encode(int16_t sample);
    
    // 解码4位ADPCM为16位PCM
    int16_t decode(uint8_t code);
    
    // 批量编码PCM数据
    size_t encodeBuffer(const int16_t* pcm, size_t pcmSize, uint8_t* adpcm);
    
    // 批量解码ADPCM数据
    size_t decodeBuffer(const uint8_t* adpcm, size_t adpcmSize, int16_t* pcm);
};

#endif