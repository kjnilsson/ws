#ifndef MULAW_CODEC_H
#define MULAW_CODEC_H

class MuLawCodec {
    private:
        static constexpr int16_t BIAS = 132;
        static constexpr int16_t CLIP = 2047;

        // Lookup tables for speed
        uint8_t muLawEncodeTable[4096];
        int16_t muLawDecodeTable[256];

    public:
        MuLawCodec() {
            // Build encode table
            for (int i = 0; i < 4096; i++) {
                int16_t sample = i - 2048;
                muLawEncodeTable[i] = encode(sample);
            }

            // Build decode table
            for (int i = 0; i < 256; i++) {
                muLawDecodeTable[i] = decode(i);
            }
        }

        inline uint8_t encodeSample(int16_t input) {
            uint16_t index = (uint16_t)(input + 2048) & 0xFFF;
            return muLawEncodeTable[index];
        }

        inline int16_t decodeSample(uint8_t mulaw) {
            return muLawDecodeTable[mulaw];
        }

    private:
        uint8_t encode(int16_t sample) {
            uint8_t sign = (sample < 0) ? 0x00 : 0x80;
            if (sample < 0) sample = -sample;

            if (sample > CLIP) sample = CLIP;
            sample += BIAS;

            uint8_t segment = 0;
            for (int i = 0; i < 8; i++) {
                if (sample > (0xFF << (i + 1))) {
                    segment = i;
                }
            }

            uint8_t mantissa = (sample >> (segment + 3)) & 0x0F;
            return ~(sign | (segment << 4) | mantissa);
        }

        int16_t decode(uint8_t mulaw) {
            mulaw = ~mulaw;
            uint8_t sign = mulaw & 0x80;
            uint8_t segment = (mulaw >> 4) & 0x07;
            uint8_t mantissa = mulaw & 0x0F;

            int16_t sample = ((mantissa << 3) + BIAS) << segment;
            sample -= BIAS;

            // Scale down to 12-bit range
            sample = (sample * CLIP) / 8031;

            return sign ? sample : -sample;
        }
};
