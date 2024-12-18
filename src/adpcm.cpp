#include "adpcm.h"
#include <cmath>

const int8_t ADPCMEncoder::indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

const int16_t ADPCMEncoder::stepTable[89] = {
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
};

uint8_t ADPCMEncoder::encode(int16_t sample) {
    int diff = sample - prevSample;
    int step = stepTable[index];
    int delta = abs(diff);
    uint8_t code = 0;

    if (diff < 0) code = 8;

    if (delta >= step) {
        code |= 4;
        delta -= step;
    }
    step >>= 1;

    if (delta >= step) {
        code |= 2;
        delta -= step;
    }
    step >>= 1;

    if (delta >= step) {
        code |= 1;
    }

    int diff_new = stepTable[index] >> 3;
    if (code & 4) diff_new += stepTable[index];
    if (code & 2) diff_new += stepTable[index] >> 1;
    if (code & 1) diff_new += stepTable[index] >> 2;
    if (code & 8) diff_new = -diff_new;

    prevSample += diff_new;
    if (prevSample > 32767) prevSample = 32767;
    else if (prevSample < -32768) prevSample = -32768;

    index += indexTable[code];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;

    return code;
}

int16_t ADPCMEncoder::decode(uint8_t code) {
    int step = stepTable[index];
    int diff = step >> 3;

    if (code & 4) diff += step;
    if (code & 2) diff += step >> 1;
    if (code & 1) diff += step >> 2;
    if (code & 8) diff = -diff;

    prevSample += diff;
    if (prevSample > 32767) prevSample = 32767;
    else if (prevSample < -32768) prevSample = -32768;

    index += indexTable[code];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;

    return prevSample;
}

size_t ADPCMEncoder::encodeBuffer(const int16_t* pcm, size_t pcmSize, uint8_t* adpcm) {
    size_t adpcmSize = 0;
    for (size_t i = 0; i < pcmSize; i += 2) {
        uint8_t code1 = encode(pcm[i]);
        uint8_t code2 = (i + 1 < pcmSize) ? encode(pcm[i + 1]) : 0;
        adpcm[adpcmSize++] = (code1 << 4) | code2;
    }
    return adpcmSize;
}

size_t ADPCMEncoder::decodeBuffer(const uint8_t* adpcm, size_t adpcmSize, int16_t* pcm) {
    size_t pcmSize = 0;
    for (size_t i = 0; i < adpcmSize; i++) {
        pcm[pcmSize++] = decode((adpcm[i] >> 4) & 0x0F);
        pcm[pcmSize++] = decode(adpcm[i] & 0x0F);
    }
    return pcmSize;
}