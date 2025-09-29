#ifndef BANDPASS_FILTER_H
#define BANDPASS_FILTER_H

#include <stdint.h>

class BandpassFilter {
private:
    // Fixed-point state variables (Q16.16 format)
    int32_t x1, x2;  // Input delay line
    int32_t y1, y2;  // Output delay line

    // Filter coefficients (Q16.16 format)
    int32_t a0, a1, a2;  // Feedforward coefficients
    int32_t b1, b2;      // Feedback coefficients

    uint32_t sampleRate;
    uint32_t frequency;
    uint16_t resonance;  // 0-4095 (12-bit)

    // Fixed-point math helpers
    static const int32_t FIXED_ONE = 65536; // 1.0 in Q16.16

    // Fast sine/cosine lookup table (quarter wave, 256 entries)
    static const int32_t SIN_TABLE[256];

    // Fixed-point multiplication with proper scaling
    int32_t fixedMul(int32_t a, int32_t b) {
        int64_t result = (int64_t)a * b;
        return (int32_t)(result >> 16);
    }

    // Reciprocal lookup table for common divisors (Q16.16 format)
    // Pre-computed 1/x values to avoid division
    static const int32_t RECIPROCAL_TABLE[256];

    // Fast division using reciprocal multiplication
    int32_t fastDiv(int32_t numerator, int32_t denominator) {
        // For small positive denominators, use lookup table
        if (denominator > 0 && denominator < 256) {
            return fixedMul(numerator, RECIPROCAL_TABLE[denominator]);
        }

        // For larger denominators, use bit shifting approximation
        // Find highest bit position
        int32_t shift = 0;
        int32_t temp = denominator;
        while (temp > 1) {
            temp >>= 1;
            shift++;
        }

        // Approximate 1/denominator as power of 2
        return numerator >> shift;
    }

    // Fast sine approximation using lookup table
    int32_t fixedSin(int32_t angle) {
        // Normalize angle to 0-2π range (angle in Q16.16, 2π ≈ 411775)
        while (angle >= 411775) angle -= 411775;
        while (angle < 0) angle += 411775;

        // Convert to table index (0-1023)
        uint32_t index = (angle * 1024) >> 16;
        if (index >= 1024) index = 1023;

        // Use symmetry to get full wave from quarter table
        if (index < 256) {
            return SIN_TABLE[index];
        } else if (index < 512) {
            return SIN_TABLE[511 - index];
        } else if (index < 768) {
            return -SIN_TABLE[index - 512];
        } else {
            return -SIN_TABLE[1023 - index];
        }
    }

    int32_t fixedCos(int32_t angle) {
        // cos(x) = sin(x + π/2)
        return fixedSin(angle + 102944); // π/2 in Q16.16
    }

    void calculateCoefficients() {
        // Calculate omega = 2π * f / fs using multiplication by reciprocal
        // omega = 2π * f * (1/fs) where 1/fs is pre-computed
        int32_t fs_reciprocal = fastDiv(FIXED_ONE, sampleRate);
        int32_t omega = fixedMul((int32_t)frequency << 16, fixedMul(411775, fs_reciprocal)); // 2π ≈ 411775 in Q16.16

        int32_t cosw = fixedCos(omega);
        int32_t sinw = fixedSin(omega);

        // Calculate Q from 12-bit resonance (0.1 to 30.0 range)
        // Q = 0.1 + (resonance * 29.9) / 4095
        int32_t Q_scaled = 6554 + fixedMul((int32_t)resonance << 4, 478); // Avoid division by pre-scaling

        // alpha = sin(w) / (2 * Q) = sin(w) * (1 / (2 * Q))
        int32_t two_Q_reciprocal = fastDiv(FIXED_ONE, Q_scaled >> 15); // Divide by 2 with shift
        int32_t alpha = fixedMul(sinw, two_Q_reciprocal);

        // Bandpass filter coefficients
        // Avoid division by norm by using reciprocal multiplication
        int32_t norm = FIXED_ONE + alpha;
        int32_t norm_reciprocal = fastDiv(FIXED_ONE, norm >> 16);

        a0 = fixedMul(alpha, norm_reciprocal);
        a1 = 0;
        a2 = -fixedMul(alpha, norm_reciprocal);
        b1 = fixedMul(fixedMul(-2 * cosw, FIXED_ONE), norm_reciprocal);
        b2 = fixedMul(FIXED_ONE - alpha, norm_reciprocal);
    }

public:
    BandpassFilter(uint32_t fs = 44100) :
        x1(0), x2(0), y1(0), y2(0),
        a0(0), a1(0), a2(0), b1(0), b2(0),
        sampleRate(fs), frequency(1000), resonance(2048) {
        calculateCoefficients();
    }

    // Set center frequency in Hz
    void setFrequency(uint32_t freq) {
        if (freq > 10 && freq < sampleRate / 2) {
            frequency = freq;
            calculateCoefficients();
        }
    }

    // Set frequency from MIDI note (0-127, where 69 = A4 = 440Hz)
    void setFrequencyFromMidi(uint8_t midiNote) {
        if (midiNote > 127) return;

        // Calculate frequency using MIDI note formula: f = 440 * 2^((n-69)/12)
        // Use lookup table for 2^(x/12) to avoid floating point
        // This covers the fractional part, then we bit-shift for the integer part

        int32_t noteOffset = (int32_t)midiNote - 69; // Offset from A4

        // Avoid division by using bit operations and lookup table
        // For positive offsets: octaves = noteOffset >> 4 + additional logic
        // For negative offsets: handle separately
        int32_t octaves, semitones;

        if (noteOffset >= 0) {
            // Use bit tricks: divide by 12 ≈ multiply by 5461 then shift right 16
            // This is an approximation: 5461/65536 ≈ 1/12
            octaves = (noteOffset * 5461) >> 16;
            semitones = noteOffset - (octaves * 12);  // octaves * 12 uses multiplication
        } else {
            // For negative numbers, handle carefully
            int32_t absOffset = -noteOffset;
            octaves = -((absOffset * 5461) >> 16);
            semitones = -(absOffset - ((-octaves) * 12));

            if (semitones < 0) {
                semitones += 12;
                octaves--;
            }
        }

        // Base frequency A4 = 440Hz in Q16.16
        int32_t baseFreq = 440 << 16;

        // Apply semitone multiplier using lookup table
        // 2^(n/12) values in Q16.16 format for n = 0 to 11
        static const int32_t SEMITONE_TABLE[12] = {
            65536,  // 2^(0/12) = 1.000000
            69433,  // 2^(1/12) = 1.059463
            73562,  // 2^(2/12) = 1.122462
            77936,  // 2^(3/12) = 1.189207
            82570,  // 2^(4/12) = 1.259921
            87481,  // 2^(5/12) = 1.334840
            92682,  // 2^(6/12) = 1.414214
            98193,  // 2^(7/12) = 1.498307
            104031, // 2^(8/12) = 1.587401
            110218, // 2^(9/12) = 1.681793
            116774, // 2^(10/12) = 1.781797
            123722  // 2^(11/12) = 1.887749
        };

        int32_t freq_fixed = fixedMul(baseFreq, SEMITONE_TABLE[semitones]);

        // Apply octave shift (multiply/divide by powers of 2)
        if (octaves >= 0) {
            freq_fixed <<= octaves;
        } else {
            freq_fixed >>= (-octaves);
        }

        // Convert back to integer Hz
        uint32_t freq = freq_fixed >> 16;

        // Clamp to valid range and set
        if (freq > 10 && freq < sampleRate / 2) {
            frequency = freq;
            calculateCoefficients();
        }
    }

    // Set resonance (0-4095, 12-bit range)
    void setResonance(uint16_t res) {
        if (res <= 4095) {
            resonance = res;
            calculateCoefficients();
        }
    }

    // Process single 12-bit sample (-2048 to +2047 range)
    int16_t process(int16_t input) {
        // Scale signed 12-bit input to Q16.16
        int32_t x0 = (int32_t)input << 16;

        // Biquad difference equation:
        // y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
        int32_t output = fixedMul(a0, x0) +
                        fixedMul(a1, x1) +
                        fixedMul(a2, x2) -
                        fixedMul(b1, y1) -
                        fixedMul(b2, y2);

        // Update delay lines
        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = output;

        // Convert back to signed 12-bit (-2048 to +2047)
        int32_t result = output >> 16;
        if (result < -2048) result = -2048;
        if (result > 2047) result = 2047;

        return (int16_t)result;
    }

    // Reset filter state
    void reset() {
        x1 = x2 = y1 = y2 = 0;
    }
};

// Sine lookup table (quarter wave, 256 entries, Q16.16 format)
const int32_t BandpassFilter::SIN_TABLE[256] = {
    0, 1608, 3216, 4821, 6424, 8022, 9616, 11204,
    12785, 14359, 15924, 17479, 19024, 20557, 22078, 23586,
    25079, 26557, 28020, 29466, 30893, 32302, 33692, 35061,
    36409, 37736, 39040, 40320, 41576, 42806, 44011, 45190,
    46340, 47464, 48559, 49624, 50660, 51665, 52639, 53581,
    54491, 55368, 56212, 57022, 57798, 58538, 59244, 59914,
    60547, 61145, 61705, 62228, 62714, 63162, 63572, 63944,
    64277, 64571, 64827, 65043, 65220, 65358, 65457, 65516,
    65536, 65516, 65457, 65358, 65220, 65043, 64827, 64571,
    64277, 63944, 63572, 63162, 62714, 62228, 61705, 61145,
    60547, 59914, 59244, 58538, 57798, 57022, 56212, 55368,
    54491, 53581, 52639, 51665, 50660, 49624, 48559, 47464,
    46340, 45190, 44011, 42806, 41576, 40320, 39040, 37736,
    36409, 35061, 33692, 32302, 30893, 29466, 28020, 26557,
    25079, 23586, 22078, 20557, 19024, 17479, 15924, 14359,
    12785, 11204, 9616, 8022, 6424, 4821, 3216, 1608,
    0, -1608, -3216, -4821, -6424, -8022, -9616, -11204,
    -12785, -14359, -15924, -17479, -19024, -20557, -22078, -23586,
    -25079, -26557, -28020, -29466, -30893, -32302, -33692, -35061,
    -36409, -37736, -39040, -40320, -41576, -42806, -44011, -45190,
    -46340, -47464, -48559, -49624, -50660, -51665, -52639, -53581,
    -54491, -55368, -56212, -57022, -57798, -58538, -59244, -59914,
    -60547, -61145, -61705, -62228, -62714, -63162, -63572, -63944,
    -64277, -64571, -64827, -65043, -65220, -65358, -65457, -65516,
    -65536, -65516, -65457, -65358, -65220, -65043, -64827, -64571,
    -64277, -63944, -63572, -63162, -62714, -62228, -61705, -61145,
    -60547, -59914, -59244, -58538, -57798, -57022, -56212, -55368,
    -54491, -53581, -52639, -51665, -50660, -49624, -48559, -47464,
    -46340, -45190, -44011, -42806, -41576, -40320, -39040, -37736,
    -36409, -35061, -33692, -32302, -30893, -29466, -28020, -26557,
    -25079, -23586, -22078, -20557, -19024, -17479, -15924, -14359,
    -12785, -11204, -9616, -8022, -6424, -4821, -3216, -1608
};

// Reciprocal lookup table (1/x in Q16.16 format for x = 1 to 255)
const int32_t BandpassFilter::RECIPROCAL_TABLE[256] = {
    0, 65536, 32768, 21845, 16384, 13107, 10922, 9362,
    8192, 7281, 6553, 5957, 5461, 5041, 4681, 4369,
    4096, 3855, 3640, 3449, 3276, 3120, 2978, 2849,
    2730, 2621, 2520, 2427, 2340, 2259, 2184, 2114,
    2048, 1985, 1927, 1872, 1820, 1771, 1724, 1680,
    1638, 1598, 1560, 1524, 1489, 1456, 1424, 1394,
    1365, 1337, 1310, 1285, 1260, 1236, 1214, 1192,
    1170, 1150, 1130, 1111, 1092, 1074, 1057, 1040,
    1024, 1008, 992, 978, 963, 949, 936, 923,
    910, 898, 886, 874, 862, 851, 840, 829,
    819, 809, 799, 789, 780, 771, 762, 753,
    744, 736, 728, 720, 712, 704, 697, 689,
    682, 675, 668, 661, 655, 648, 642, 636,
    630, 624, 618, 612, 607, 601, 596, 590,
    585, 580, 575, 570, 565, 560, 555, 551,
    546, 542, 537, 533, 529, 524, 520, 516,
    512, 508, 504, 500, 496, 492, 489, 485,
    481, 478, 474, 471, 468, 464, 461, 458,
    455, 451, 448, 445, 442, 439, 436, 434,
    431, 428, 425, 422, 420, 417, 414, 412,
    409, 407, 404, 402, 399, 397, 394, 392,
    390, 387, 385, 383, 381, 378, 376, 374,
    372, 370, 368, 366, 364, 362, 360, 358,
    356, 354, 352, 350, 348, 346, 344, 343,
    341, 339, 337, 336, 334, 332, 330, 329,
    327, 325, 324, 322, 320, 319, 317, 316,
    314, 312, 311, 309, 308, 306, 305, 303,
    302, 300, 299, 297, 296, 295, 293, 292,
    290, 289, 288, 286, 285, 284, 282, 281,
    280, 278, 277, 276, 275, 273, 272, 271,
    270, 268, 267, 266, 265, 264, 262, 261,
    260, 259, 258, 257, 256, 255, 254, 252
};

#endif
