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
// Returns +2047     if ≥ 50 cents sharp  (above C).
// Returns -2047     if ≥ 50 cents flat   (below C).
static int16_t periodToCloseness(uint32_t period_samples) {
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
    // At larger deviations the display saturates anyway, so sign is what matters.
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

class Tnr : public ComputerCard {
    PitchDetector pd1, pd2;
    uint8_t counter;

    // Drive three LEDs for one tuner channel.
    //   c range: -2047..+2047 = -600..+600 cents; INT16_MIN = no signal
    //   ledSharp (LED 0): fades in from off (600 cents sharp) to full (just above C), snaps off at C
    //   ledCenter(LED 2): full at C, zero outside ±200 cents
    //   ledFlat  (LED 4): fades in from off (600 cents flat)  to full (just below C), snaps off at C
    void updateLeds(int16_t c, uint8_t ledFlat, uint8_t ledCenter, uint8_t ledSharp) {
        if (c == INT16_MIN) {
            LedBrightness(ledFlat,   0);
            LedBrightness(ledCenter, 0);
            LedBrightness(ledSharp,  0);
            return;
        }
        int16_t absC = c < 0 ? -c : c;

        // Side LEDs: bright when off-pitch, fade to zero as C is approached
        int32_t side   = 4095L * absC          / 2047;

        // Center LED: zero when off-pitch, fades to full as C is approached
        int32_t center = 4095L * (2047 - absC) / 2047;

        LedBrightness(ledCenter, (uint16_t)center);
        LedBrightness(ledFlat,   c < 0 ? (uint16_t)side : 0);
        LedBrightness(ledSharp,  c > 0 ? (uint16_t)side : 0);
    }

public:
    Tnr() : counter(0) {}

    virtual void ProcessSample() override {
        // Both CV outputs emit middle C
        CVOutMIDINote(0, 60);
        CVOutMIDINote(1, 60);

        pd1.addSample(AudioIn1());
        pd2.addSample(AudioIn2());

        // Update display at ~187 Hz (every 256 samples)
        if (!counter) {
            // ledFlat=4, ledCenter=2, ledSharp=0  (0 = above C, 4 = below C)
            updateLeds(periodToCloseness(pd1.period()), 4, 2, 0);
            // ledFlat=5, ledCenter=3, ledSharp=1
            updateLeds(periodToCloseness(pd2.period()), 5, 3, 1);
        }
        counter++;
    }
};

int main() {
    Tnr tnr;
    tnr.Run();
}
