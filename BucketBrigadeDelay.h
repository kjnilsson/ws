#include <cstdint>
#include <cstring>

class BucketBrigadeDelay {
private:
    int8_t* buffer;  // Changed to signed
    uint16_t writePos;
    uint16_t bufferSize;

    // Clock simulation (48kHz / 12kHz = clock every 4 samples)
    uint8_t clockCounter;
    const uint8_t clockDivider = 4;

    // Filter state
    int8_t filterState;  // Changed to signed
    uint8_t filterCoeff;
    int8_t clipThreshold;  // Changed to signed

    // Feedback control (0-255)
    uint8_t feedbackAmount;

    // Delay control with sub-sample interpolation
    uint32_t targetDelaySamples;
    uint32_t currentDelaySamples;
    int32_t delaySlewRate;

    // Convert 12-bit signed sample to 8-bit signed for storage
    inline int8_t pack12to8(int16_t sample12) {
        // Sample is -2048 to +2047, shift down to -128 to +127
        return static_cast<int8_t>((sample12 >> 4) & 0xFF);
    }

    // Convert 8-bit signed back to 12-bit signed
    inline int16_t unpack8to12(int8_t sample8) {
        // Shift up from -128..+127 to -2048..+2032
        return static_cast<int16_t>(sample8) << 4;
    }

    // Simple one-pole lowpass filter (signed)
    inline int8_t lowpass8(int8_t input) {
        int16_t diff = input - filterState;
        int16_t delta = (diff * filterCoeff) >> 8;
        filterState += delta;
        return filterState;
    }

    // Fast soft clipping (signed input/output)
    inline int8_t softClip(int8_t input) {
        int16_t x = input;

        // Scale up: x * 1.5 = x + x/2
        x = x + (x >> 1);

        // Piecewise soft clipping
        if (x > clipThreshold) {
            int16_t excess = x - clipThreshold;
            x = clipThreshold + (excess >> 2);
        } else if (x < -clipThreshold) {
            int16_t excess = x + clipThreshold;
            x = -clipThreshold + (excess >> 2);
        }

        // Hard limit (symmetric)
        if (x > 120) x = 120;
        if (x < -120) x = -120;

        // Scale back: x * 0.668 ≈ x * 2/3
        x = (x * 171) >> 8;

        return (int8_t)x;
    }

    // Linear interpolation between buffer positions
    inline int8_t interpolateRead(uint32_t delaySamplesFP) {
        // Convert 48kHz samples to 12kHz stages
        uint32_t delayStagesFP = delaySamplesFP >> 2;

        // Extract integer and fractional parts
        uint32_t delayStages = delayStagesFP >> 16;
        uint16_t frac = delayStagesFP & 0xFFFF;

        // Clamp to buffer bounds
        if (delayStages >= (uint32_t)(bufferSize - 1)) {
            delayStages = bufferSize - 2;
            frac = 0;
        }

        // Calculate read positions (circular buffer)
        int32_t readPos1 = (int32_t)writePos - (int32_t)delayStages;
        if (readPos1 < 0) readPos1 += bufferSize;

        int32_t readPos2 = readPos1 - 1;
        if (readPos2 < 0) readPos2 += bufferSize;

        // Read two adjacent samples
        int8_t sample1 = buffer[readPos1];
        int8_t sample2 = buffer[readPos2];

        // Linear interpolation (signed)
        int32_t interpolated = ((int32_t)sample1 * (65536 - frac) +
                                (int32_t)sample2 * frac) >> 16;

        return (int8_t)interpolated;
    }

public:
    // Constructor
    BucketBrigadeDelay(uint32_t maxDelaySamples,
                       uint8_t filterAmount = 200,
                       uint8_t clipAmount = 64,
                       uint8_t feedback = 128,
                       uint16_t slewRate = 100)
        : writePos(0), clockCounter(0), filterState(0),
          feedbackAmount(feedback) {

        // Convert 48kHz samples to 12kHz stages
        int32_t bs = maxDelaySamples / clockDivider;

        // Clamp to reasonable limits
        if (bs < 64) bufferSize = 64;
        if (bs > 0xFFFF) bufferSize = 0xFFFF;
        bufferSize = bs;

        // Allocate buffer
        buffer = new int8_t[bufferSize];
        memset(buffer, 0, bufferSize);  // Initialize to zero (center)

        // Set filter coefficient
        filterCoeff = 255 - filterAmount;
        if (filterCoeff < 10) filterCoeff = 10;

        // Set clipping threshold (signed)
        clipThreshold = 100 - (clipAmount >> 2);
        if (clipThreshold < 20) clipThreshold = 20;

        // Initialize delay (in 16.16 fixed point)
        targetDelaySamples = maxDelaySamples << 16;
        currentDelaySamples = maxDelaySamples << 16;

        delaySlewRate = slewRate;
    }

    ~BucketBrigadeDelay() {
        delete[] buffer;
    }

    // Process a single signed 12-bit sample at 48kHz (-2048 to +2047)
    // Returns delayed/processed signed 12-bit sample
    int16_t process(int16_t input12) {
        bool clockTick = false;

        // Increment clock counter
        clockCounter++;
        if (clockCounter >= clockDivider) {
            clockCounter = 0;
            clockTick = true;
        }

        // Smoothly interpolate current delay toward target
        if (currentDelaySamples < targetDelaySamples) {
            currentDelaySamples += delaySlewRate;
            if (currentDelaySamples > targetDelaySamples) {
                currentDelaySamples = targetDelaySamples;
            }
        } else if (currentDelaySamples > targetDelaySamples) {
            currentDelaySamples -= delaySlewRate;
            if (currentDelaySamples < targetDelaySamples) {
                currentDelaySamples = targetDelaySamples;
            }
        }

        // Read delayed signal with interpolation
        int8_t delayedRaw = interpolateRead(currentDelaySamples);
        /* int8_t delayedClipped = softClip(delayedRaw); */
        int8_t delayedClipped = delayedRaw;
        int8_t delayedFiltered = lowpass8(delayedClipped);
        int16_t delayed12 = unpack8to12(delayedFiltered);

        // Mix input with feedback (all signed arithmetic)
        int32_t feedbackSignal = (delayed12 * (int16_t)feedbackAmount) >> 8;
        int32_t mixedInput = input12 + feedbackSignal;

        //headroom
        mixedInput = (mixedInput * 171) >> 8;

        // Clamp to signed 12-bit range (-2048 to +2047)
        if (mixedInput > 2047) mixedInput = 2047;
        if (mixedInput < -2048) mixedInput = -2048;

        // Only update BBD buffer on clock ticks
        if (clockTick) {
            buffer[writePos] = pack12to8((int16_t)mixedInput);

            writePos++;
            if (writePos >= bufferSize) {
                writePos = 0;
            }
        }

        return delayed12;
    }

    // Set delay time in 48kHz samples
    void setDelaySamples(uint32_t samples) {
        uint32_t maxSamples = (bufferSize * clockDivider);
        if (samples > maxSamples) {
            samples = maxSamples;
        }
        if (samples < clockDivider) {
            samples = clockDivider;
        }
        targetDelaySamples = samples << 16;
    }

    // Set delay time in milliseconds
    void setDelayMs(uint16_t ms) {
        setDelaySamples(ms * 48);
    }

    // Set interpolation speed
    void setSlewRate(uint16_t rate) {
        delaySlewRate = rate;
    }

    // Set feedback amount (0-255)
    void setFeedback(uint8_t feedback) {
        feedbackAmount = feedback;
    }

    // Set filter amount (0 = bright, 255 = very dark)
    void setFilterAmount(uint8_t amount) {
        filterCoeff = 255 - amount;
        if (filterCoeff < 10) filterCoeff = 10;
    }

    // Set clipping/saturation amount (0 = clean, 255 = heavy)
    void setClipAmount(uint8_t amount) {
        clipThreshold = 100 - (amount >> 2);
        if (clipThreshold < 20) clipThreshold = 20;
    }

    // Clear buffer and reset state
    void clear() {
        memset(buffer, 0, bufferSize);
        writePos = 0;
        filterState = 0;
        clockCounter = 0;
    }

    // Get maximum delay in 48kHz samples
    uint32_t getMaxDelaySamples() const {
        return bufferSize * clockDivider;
    }

    // Get current delay in 48kHz samples
    uint32_t getCurrentDelaySamples() const {
        return currentDelaySamples >> 16;
    }

    // Get target delay in 48kHz samples
    uint32_t getTargetDelaySamples() const {
        return targetDelaySamples >> 16;
    }

    // Get current delay in milliseconds
    uint16_t getCurrentDelayMs() const {
        return (currentDelaySamples >> 16) / 48;
    }
};
