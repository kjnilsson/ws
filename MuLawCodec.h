#ifndef MULAW_CODEC_H
#define MULAW_CODEC_H

#include <stdint.h>

// µ-law codec for 12-bit signed audio (-2047 .. +2047).
//
// Internally scales the 12-bit input ×4 into the G.711 13-bit range before
// compressing, so that the standard 8-segment logarithmic curve is applied
// correctly.  Segments 0–5 are used (covering the full 12-bit range);
// segments 6–7 are structurally unused but decode to valid clipped values.
//
// This gives true µ-law character: fine quantisation on quiet signals,
// coarse on loud — without the distortion of the old 3-segment version.

class MuLawCodec {
    // G.711 constants
    static constexpr int32_t BIAS  = 132;   // 0x84
    static constexpr int32_t SCALE = 4;     // 12-bit → 13-bit

    // Lookup tables built once in the constructor
    uint8_t  encodeTable[4096];   // index = sample + 2048  (0-4095)
    int16_t  decodeTable[256];

public:
    MuLawCodec() {
        for (int i = 0; i < 4096; i++)
            encodeTable[i] = encode((int16_t)(i - 2048));
        for (int i = 0; i < 256; i++)
            decodeTable[i] = decode((uint8_t)i);
    }

    // Encode a 12-bit signed sample to 8-bit µ-law
    inline uint8_t encodeSample(int16_t input) {
        return encodeTable[(uint16_t)(input + 2048) & 0xFFF];
    }

    // Decode 8-bit µ-law back to 12-bit signed
    inline int16_t decodeSample(uint8_t mulaw) {
        return decodeTable[mulaw];
    }

private:
    uint8_t encode(int16_t sample) {
        // Sign: bit 7 set = positive (G.711 convention)
        uint8_t sign = (sample >= 0) ? 0x80u : 0x00u;
        if (sample < 0) sample = -sample;

        // Scale to G.711 13-bit range and add bias
        int32_t s = (int32_t)sample * SCALE + BIAS;
        if (s > 8191) s = 8191;

        // Find segment: index of highest set bit, searching down from bit 13
        uint8_t segment = 7;
        for (int32_t mask = 0x4000; (s & mask) == 0 && segment > 0; segment--, mask >>= 1);

        // 4-bit mantissa from the bits just below the leading 1
        uint8_t mantissa = (uint8_t)((s >> (segment + 3)) & 0x0F);

        return (uint8_t)~(sign | (segment << 4) | mantissa);
    }

    int16_t decode(uint8_t mulaw) {
        uint8_t u        = ~mulaw;
        uint8_t sign     = u & 0x80;
        uint8_t segment  = (u >> 4) & 0x07;
        uint8_t mantissa = u & 0x0F;

        // G.711 reconstruct: re-insert the implicit leading 1 and the
        // half-step framing bit, then shift up by the segment exponent
        int32_t s = (((int32_t)mantissa << 3) + BIAS) << segment;
        s -= BIAS;

        // Scale back from 13-bit to 12-bit
        s /= SCALE;
        if (s > 2047) s = 2047;

        return sign ? (int16_t)s : (int16_t)-s;
    }
};

#endif
