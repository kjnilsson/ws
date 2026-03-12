#ifndef REVERB_H
#define REVERB_H

#include <stdint.h>
#include <string.h>

// Schroeder/Freeverb-style reverb: 4 LP-damped comb filters + 2 allpass filters.
// All arithmetic is Q15 fixed-point integer – no floats at runtime.
//
// knobY (0-4095) drives both parameters simultaneously:
//   feedback – higher Y = longer tail
//   damping  – higher Y = more HF rolloff per reflection = darker sound
//
// Total delay-line RAM: ~13.4 KB

class Reverb {
    // Delay lengths in samples at 48 kHz (Freeverb ratios scaled from 44.1 kHz)
    static constexpr int C0 = 1214, C1 = 1389, C2 = 1547, C3 = 1693;
    static constexpr int A0 =  605, A1 =  371;

    int16_t cBuf0[C0], cBuf1[C1], cBuf2[C2], cBuf3[C3];
    int16_t aBuf0[A0], aBuf1[A1];
    int32_t cLp[4];   // one-pole LP state per comb (stays in audio range)
    int     cIdx[4];
    int     aIdx[2];

    static constexpr int32_t AP_GAIN = 16384;  // allpass feedback = 0.5 in Q15

public:
    Reverb() {
        memset(cBuf0, 0, sizeof(cBuf0)); memset(cBuf1, 0, sizeof(cBuf1));
        memset(cBuf2, 0, sizeof(cBuf2)); memset(cBuf3, 0, sizeof(cBuf3));
        memset(aBuf0, 0, sizeof(aBuf0)); memset(aBuf1, 0, sizeof(aBuf1));
        memset(cLp,   0, sizeof(cLp));
        memset(cIdx,  0, sizeof(cIdx));
        memset(aIdx,  0, sizeof(aIdx));
    }

    // Process one 12-bit signed sample; returns 12-bit signed wet output.
    // knobY: 0-4095
    int16_t process(int16_t input, int knobY)
    {
        // feedback Q15: 0.50 (16384) at Y=0  →  0.95 (31130) at Y=4095
        int32_t fb   = 16384 + ((int32_t)knobY * 14746) / 4096;
        // damping Q15: 0.10 (3277)  at Y=0  →  0.65 (21299) at Y=4095
        //   higher = more HF rolloff per reflection = darker tail
        int32_t damp = 3277  + ((int32_t)knobY * 18022) / 4096;

        // Pre-attenuate input by (1-fb) to normalise the comb DC gain (= 1/(1-fb))
        // back to unity.  Without this, high feedback causes the comb buffers to
        // accumulate to ~20× the input amplitude and saturate hard.
        int32_t normIn = ((32768 - fb) * (int32_t)input) >> 15;

        // 4 comb filters in parallel, then sum
        int32_t wet = 0;
        wet += processComb(cBuf0, C0, cIdx[0], cLp[0], normIn, fb, damp);
        wet += processComb(cBuf1, C1, cIdx[1], cLp[1], normIn, fb, damp);
        wet += processComb(cBuf2, C2, cIdx[2], cLp[2], normIn, fb, damp);
        wet += processComb(cBuf3, C3, cIdx[3], cLp[3], normIn, fb, damp);
        wet >>= 2;  // average 4 combs

        // 2 allpass filters in series for diffusion
        wet = processAllpass(aBuf0, A0, aIdx[0], wet);
        wet = processAllpass(aBuf1, A1, aIdx[1], wet);

        if (wet >  2047) wet =  2047;
        if (wet < -2048) wet = -2048;
        return (int16_t)wet;
    }

private:
    // Comb filter with one-pole LP damping on the feedback path.
    // lp = (1-damp)*out + damp*lp  →  higher damp = lower cutoff = darker.
    static int32_t processComb(int16_t* buf, int len, int& idx, int32_t& lp,
                                int32_t in, int32_t fb, int32_t damp)
    {
        int32_t out = buf[idx];
        lp = ((32768 - damp) * out + damp * lp) >> 15;
        int32_t w = in + ((fb * lp) >> 15);
        if (w >  32767) w =  32767;
        if (w < -32768) w = -32768;
        buf[idx] = (int16_t)w;
        if (++idx >= len) idx = 0;
        return out;
    }

    // Schroeder allpass (gain = 0.5) for diffusion
    static int32_t processAllpass(int16_t* buf, int len, int& idx, int32_t in)
    {
        int32_t b   = buf[idx];
        int32_t out = b - in;
        int32_t w   = in + ((AP_GAIN * b) >> 15);
        if (w >  32767) w =  32767;
        if (w < -32768) w = -32768;
        buf[idx] = (int16_t)w;
        if (++idx >= len) idx = 0;
        return out;
    }
};

#endif
