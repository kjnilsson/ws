#include "ComputerCard.h"

#include <stdint.h>

class CNoteDetector {
private:
    static const uint16_t SAMPLE_RATE = 48000;  // Adjust based on your ADC setup
    static const uint16_t WINDOW_SIZE = 1024;    // Power of 2 for efficient operations
    static const uint8_t NUM_C_NOTES = 8;       // C0 to C7

    // Pre-calculated C note frequencies in mHz (millihertz) to avoid floats
    static const uint32_t C_FREQUENCIES_MHZ[NUM_C_NOTES];

    // Autocorrelation buffer
    int16_t samples[WINDOW_SIZE];
    uint16_t sample_index;
    bool buffer_full;

    // Fixed-point arithmetic helpers
    static const uint8_t FIXED_POINT_SHIFT = 10;  // 1024 units per 1.0

    // Calculate autocorrelation for a given lag
    int32_t autocorrelate(uint16_t lag) {
        if (!buffer_full && sample_index < WINDOW_SIZE) return 0;

        int32_t sum = 0;
        uint16_t count = WINDOW_SIZE - lag;

        for (uint16_t i = 0; i < count; i++) {
            sum += (int32_t)samples[i] * samples[i + lag];
        }

        return sum / count;
    }

    // Find fundamental frequency using autocorrelation
    uint32_t detectPitch() {
        int32_t max_correlation = 0;
        uint16_t best_lag = 0;

        // Search for lag between 20Hz and 4kHz
        uint16_t min_lag = SAMPLE_RATE / 4000;  // 4kHz max
        uint16_t max_lag = SAMPLE_RATE / 20;    // 20Hz min

        // Find the lag with maximum autocorrelation
        for (uint16_t lag = min_lag; lag < max_lag && lag < WINDOW_SIZE / 2; lag++) {
            int32_t correlation = autocorrelate(lag);

            if (correlation > max_correlation) {
                max_correlation = correlation;
                best_lag = lag;
            }
        }

        if (best_lag == 0) return 0;

        // Convert lag to frequency in mHz
        return (SAMPLE_RATE * 1000UL) / best_lag;
    }

    // Calculate cents difference between two frequencies
    int16_t frequencyToCents(uint32_t freq_mhz, uint32_t ref_freq_mhz) {
        if (freq_mhz == 0 || ref_freq_mhz == 0) return 2047;

        // Approximate log2 ratio using fixed-point arithmetic
        // cents = 1200 * log2(freq/ref)

        // Find octave difference first
        int16_t octave_diff = 0;
        uint32_t f = freq_mhz;
        uint32_t r = ref_freq_mhz;

        while (f >= r * 2) {
            f /= 2;
            octave_diff++;
        }
        while (f * 2 <= r) {
            f *= 2;
            octave_diff--;
        }

        // Now f and r are within one octave
        // Use linear approximation for small differences
        int32_t ratio = ((int32_t)f - (int32_t)r) * 1200 / (int32_t)r;

        return octave_diff * 1200 + ratio;
    }

public:
    CNoteDetector() : sample_index(0), buffer_full(false) {
        // Initialize sample buffer
        for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
            samples[i] = 0;
        }
    }

    // Add a new 12-bit sample (0-4095)
    void addSample(int16_t signed_sample) {
        samples[sample_index] = signed_sample;
        sample_index++;

        if (sample_index >= WINDOW_SIZE) {
            sample_index = 0;
            buffer_full = true;
        }
    }

    // Get closeness to nearest C note (-2048 to 2047)
    int16_t getCloseness() {
        uint32_t detected_freq_mhz = detectPitch();

        if (detected_freq_mhz == 0) {
            return -2048;  // No pitch detected
        }

        // Find closest C note
        int32_t min_cents_diff = 32767;  // Large initial value
        bool is_above = true;

        for (uint8_t i = 0; i < NUM_C_NOTES; i++) {
            int16_t cents_diff = frequencyToCents(detected_freq_mhz, C_FREQUENCIES_MHZ[i]);
            int16_t abs_cents_diff = cents_diff < 0 ? -cents_diff : cents_diff;

            if (abs_cents_diff < min_cents_diff) {
                min_cents_diff = abs_cents_diff;
                is_above = cents_diff > 0;
            }
        }

        // Convert cents to output range
        // 0 cents = 0 output (perfect match)
        // 50 cents = ±2047 output (quarter tone away)
        // Linear mapping: output = (cents * 2047) / 50

        int32_t output = (min_cents_diff * 2047L) / 600;

        // Clamp to range
        if (output > 2047) output = 2047;

        // Apply sign based on whether we're above or below the C note
        return is_above ? (int16_t)output : -(int16_t)output;
    }
    // Process a block of samples and return closeness
    int16_t processBlock(uint16_t* block, uint16_t block_size) {
        for (uint16_t i = 0; i < block_size; i++) {
            addSample(block[i]);
        }
        return getCloseness();
    }
};

// C note frequencies in millihertz (C0 to C7)
const uint32_t CNoteDetector::C_FREQUENCIES_MHZ[CNoteDetector::NUM_C_NOTES] = {
    16352,   // C0
    32703,   // C1
    65406,   // C2
    130813,  // C3
    261626,  // C4 (Middle C)
    523251,  // C5
    1046502, // C6
    2093005  // C7
};
class Tnr : public ComputerCard
{
public:
    uint8_t counter;
    CNoteDetector nd;

    Tnr()
    {
        nd = CNoteDetector();
        counter = 0;
    }

    inline int16_t abs(int16_t x) {
        int16_t mask = x >> 15;  // Sign bit replicated to all bits
        return (x ^ mask) - mask;
    }

	virtual void ProcessSample()
	{
        CVOutMIDINote(0, 60);
        CVOutMIDINote(1, 60);

        nd.addSample(AudioIn1());

        if(!counter)
        {
            int16_t c = nd.getCloseness();
            if(c == -2048)
            {
            }
            else if(c == 0)
            {
                LedBrightness(0, 0);
                LedOn(2, true);
                LedBrightness(4, 0);
            }
            else if (c < 0)
            {
                int led4 = (abs(c) * 2);
                LedBrightness(0, 0);
                LedOn(2, false);
                LedBrightness(4, led4);
            } else
            {
                int led0 = (c * 2);
                LedBrightness(0, led0);
                LedOn(2, false);
                LedBrightness(4, 0);
            };
        }
        counter++;
		LedBrightness(1, KnobVal(Knob::Main));
	}
};


int main()
{
	Tnr pt;
	pt.Run();
}


