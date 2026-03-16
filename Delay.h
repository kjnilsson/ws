#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include <string.h>

// Tape-style mono delay with LP-filtered feedback and interpolated time changes.
//
// Y knob (0-4095): delay time 50 ms → 1000 ms
// Feedback fixed at 0.65 (~5 audible repeats, each progressively darker).
//
// Input is scaled by INPUT_GAIN (-6 dB) before entering the feedback loop so
// the delay runs clean at normal playing levels.  Each echo decays naturally
// by FEEDBACK per repeat.
//
// When the knob moves, currentDelay slews toward the target at SLEW_Q8 per
// sample (Q8 fixed-point).  Linear interpolation between adjacent buffer
// samples gives a smooth pitch-bend rather than a click during the transition.
//
// Delay buffer RAM: 36 000 × int16 = 72 KB
// (Delay is limited to 750 ms to keep total static RAM within bounds:
//  144 KB sample buffer + 72 KB delay = 216 KB, leaving room for the USB host stack.)

class Delay {
    static constexpr int MAX_SAMPLES = 36000;   // 750 ms at 48 kHz

    int16_t buf[MAX_SAMPLES];
    int     writeIdx;
    int32_t lpState;          // one-pole LP state in feedback path
    int32_t currentDelayQ8;   // current delay in Q8 (samples × 256)

    static constexpr int32_t FEEDBACK   = 21299;  // 0.65 in Q15
    static constexpr int32_t LP_COEF    = 13107;  // 0.40 in Q15
    static constexpr int32_t INPUT_GAIN = 16384;  // 0.50 in Q15 (-6 dB)
    // Slew: 512 Q8-units/sample = 2 samples/sample → ~350 ms to sweep full range.
    static constexpr int32_t SLEW_Q8  = 512;

public:
    Delay() : writeIdx(0), lpState(0), currentDelayQ8(2400 << 8) {
        memset(buf, 0, sizeof(buf));
    }

    // input: 12-bit signed; returns 12-bit signed wet output.
    // knobY: 0-4095
    int16_t process(int16_t input, int knobY)
    {
        // Target delay in Q8
        int32_t targetQ8 = (2400 + ((int32_t)knobY * 33600) / 4096) << 8;

        // Slew currentDelay toward target
        int32_t diff = targetQ8 - currentDelayQ8;
        if      (diff >  SLEW_Q8) currentDelayQ8 += SLEW_Q8;
        else if (diff < -SLEW_Q8) currentDelayQ8 -= SLEW_Q8;
        else                      currentDelayQ8   = targetQ8;

        // Fractional read position in Q8
        int32_t readPosQ8 = (writeIdx << 8) - currentDelayQ8;
        if (readPosQ8 < 0) readPosQ8 += (MAX_SAMPLES << 8);

        int r0   = (readPosQ8 >> 8);
        int r1   = (r0 + 1 < MAX_SAMPLES) ? r0 + 1 : 0;
        int frac = readPosQ8 & 0xFF;   // 0-255

        // Linear interpolation between adjacent samples
        int32_t delayed = (int32_t)buf[r0] +
                          (((int32_t)(buf[r1] - buf[r0]) * frac) >> 8);

        // One-pole LP on the feedback path: warms each successive repeat
        lpState += (LP_COEF * (delayed - lpState)) >> 15;

        // Write: scaled input + filtered feedback, clipped to 12-bit.
        int32_t w = (((int32_t)input * INPUT_GAIN) >> 15)
                  + ((FEEDBACK * lpState) >> 15);
        if (w >  2047) w =  2047;
        if (w < -2048) w = -2048;
        buf[writeIdx] = (int16_t)w;

        if (++writeIdx >= MAX_SAMPLES) writeIdx = 0;

        // Output is the interpolated delayed signal (already in 12-bit range)
        if (delayed >  2047) delayed =  2047;
        if (delayed < -2048) delayed = -2048;
        return (int16_t)delayed;
    }
};

#endif
