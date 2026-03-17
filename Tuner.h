#pragma once
#include "ComputerCard.h"
#include <stdint.h>

// Detects pitch by measuring the period between positive-going zero crossings.
// Averages AVG_PERIODS consecutive periods for stability.
class PitchDetector {
    static const uint16_t MIN_PERIOD  = 12;    // 4 kHz max  (48000/4000)
    static const uint16_t MAX_PERIOD  = 2400;  // 20 Hz min  (48000/20)
    static const uint8_t  AVG_PERIODS = 8;

    bool     above;
    uint32_t since_last;   // samples since last positive crossing
    uint32_t timeout;      // samples since any crossing (silence detection)

    uint32_t periods[AVG_PERIODS];
    uint8_t  pi;           // ring-buffer write index
    uint8_t  count;        // valid periods accumulated

    uint32_t result;       // averaged period (samples), 0 = unknown

public:
    PitchDetector() : above(false), since_last(0), timeout(0),
                      pi(0), count(0), result(0) {
        for (uint8_t i = 0; i < AVG_PERIODS; i++) periods[i] = 0;
    }

    void addSample(int16_t s) {
        since_last++;
        timeout++;

        if (!above && s > 256) {
            above = true;
            if (since_last >= MIN_PERIOD && since_last <= MAX_PERIOD) {
                periods[pi] = since_last;
                pi = (pi + 1) % AVG_PERIODS;
                if (count < AVG_PERIODS) count++;
                if (count >= AVG_PERIODS) {
                    uint32_t sum = 0;
                    for (uint8_t i = 0; i < AVG_PERIODS; i++) sum += periods[i];
                    result = sum / AVG_PERIODS;
                }
            } else {
                // Period out of range — discard history
                count  = 0;
                pi     = 0;
                result = 0;
            }
            since_last = 0;
            timeout    = 0;
        } else if (above && s < -256) {
            above = false;
        }

        // No crossing for >100 ms: declare silence
        if (timeout > 4800) {
            result     = 0;
            count      = 0;
            pi         = 0;
            since_last = 0;
            timeout    = 0;
            above      = false;
        }
    }

    // Returns averaged period in samples, or 0 if not yet determined.
    uint32_t period() const {
        return (count >= AVG_PERIODS) ? result : 0;
    }
};

// Convert a period (samples at 48 kHz) to deviation from the nearest C note.
//
// Returns INT16_MIN if no pitch.
// Returns  0        if perfectly in tune with a C.
// Returns +2047     if >= 50 cents sharp  (above C).
// Returns -2047     if >= 50 cents flat   (below C).
static inline int16_t periodToCloseness(uint32_t period_samples) {
    if (period_samples == 0) return INT16_MIN;

    // Frequency in mHz (avoids floats; 48 MHz fits in uint32_t)
    uint32_t f = 48000000UL / period_samples;

    // Octave-reduce into [C4, C5) = [261626, 523251) mHz
    const uint32_t C4 = 261626;
    const uint32_t C5 = 523251;
    while (f < C4) f <<= 1;
    while (f >= C5) f >>= 1;

    // Cents deviation using first-order log approximation:
    //   cents ≈ (1200/ln2) × (f−C)/C  ≈  1731 × (f−C)/C
    // This is accurate to within ~2% for deviations up to ±100 cents.
    //
    // Geometric midpoint between C4 and C5 is at ~370 000 mHz (600 cents).
    // Above that, compare to C5 to get the correct (negative) sign.
    int32_t cents;
    if (f >= 370000UL) {
        cents = -(int32_t)((C5 - f) * 1731UL / C5);
    } else {
        cents = (int32_t)((f - C4) * 1731UL / C4);
    }

    // Scale ±600 cents (6 semitones) to ±2047 and clamp
    int32_t out = cents * 2047L / 600;
    if (out >  2047) out =  2047;
    if (out < -2047) out = -2047;
    return (int16_t)out;
}

// Brightness values for a tuner LED triplet (flat / center / sharp).
// Apply with: LedBrightness(ledFlat, v.flat); LedBrightness(ledCenter, v.center); etc.
struct TunerLedValues { uint16_t flat, center, sharp; };

// Compute LED brightnesses from cents deviation.
//   c range: -2047..+2047 = -600..+600 cents; INT16_MIN = no signal
//   flat  : bright when below C, fades to zero as C is approached
//   center: full at C, fades to zero towards ±600 cents
//   sharp : bright when above C, fades to zero as C is approached
static inline TunerLedValues tunerLedValues(int16_t c) {
    if (c == INT16_MIN) return {0, 0, 0};
    int16_t absC = c < 0 ? -c : c;
    uint16_t side   = (uint16_t)(4095L * absC          / 2047);
    uint16_t center = (uint16_t)(4095L * (2047 - absC) / 2047);
    return { c < 0 ? side : (uint16_t)0, center, c > 0 ? side : (uint16_t)0 };
}
