#include <cstdint>
#include "ComputerCard.h"
#include "hihat.h"

//                    step: 1 2 3 4 5 6 7 8

// Bass drum patterns (bit 7 set = always hit step 1)
// Sorted by density: sparse -> dense
static constexpr uint8_t bassPatterns[32] = {
    0b10000000, 0b10001000, 0b10000011, 0b10001010,
    0b10010100, 0b10100100, 0b11000010, 0b11100000,
    0b10001110, 0b10011001, 0b10100101, 0b10101100,
    0b10111000, 0b11001001, 0b11010010, 0b11100010,
    0b10001111, 0b10011110, 0b10101110, 0b10111001,
    0b11001011, 0b11010101, 0b11011100, 0b11101001,
    0b11110010, 0b10101111, 0b10111110, 0b11011101,
    0b11101101, 0b11110110, 0b10111111, 0b11111011,
};

// Snare drum patterns (bit 7 unset, bit 4 often set = backbeat)
// Sorted by density: sparse -> dense
static constexpr uint8_t snarePatterns[32] = {
    0b00001000, 0b00000001, 0b00000101, 0b00010010,
    0b00101000, 0b00110000, 0b00001101, 0b00010101,
    0b00011001, 0b00110001, 0b00111000, 0b01000011,
    0b01010010, 0b01100100, 0b01110000, 0b00011101,
    0b00110011, 0b00111001, 0b01001011, 0b01010011,
    0b01010110, 0b01011100, 0b01101100, 0b01110100,
    0b00011111, 0b00111101, 0b01011011, 0b01011110,
    0b01110110, 0b01111100, 0b01011111, 0b01111101,
};

class Dr8 : public ComputerCard
{
    private:
        int step = 0;
        int prevStep = -1;
        uint32_t clockCounter = 0;
        uint16_t bassLed = 0;
        uint16_t snareLed = 0;
        uint16_t bassTrig = 0;
        uint16_t snareTrig = 0;
        int32_t bassEnv = 0;
        int32_t snareEnv = 0;
        uint32_t hihatPos = HIHAT_LEN; // start past end = silent
        uint32_t noiseSeed = 1;

        int16_t noise()
        {
            noiseSeed = 1664525 * noiseSeed + 1013904223;
            return static_cast<int16_t>(noiseSeed >> 20) - 2048;
        }

        // Internal clock: 100 BPM, 8 steps per bar (8th notes)
        // 48000 * 60 / (100 * 2) = 14400 samples per step
        static constexpr uint32_t DEFAULT_STEP_SAMPLES = 14400;
        static constexpr uint16_t TRIG_SAMPLES = 240;  // ~5ms trigger pulse
        static constexpr int32_t KICK_ENV_MAX = 7200; // ~150ms decay
        static constexpr int32_t SNARE_ENV_MAX = 3840; // ~80ms decay
        static constexpr uint16_t LED_FLASH_BRIGHT = 4095;
        static constexpr uint16_t LED_DECAY = 8;

    public:
        Dr8() {}

        virtual void ProcessSample() override
        {
            // Advance step: external clock if connected, else internal 100 BPM
            if (Connected(Input::Pulse1))
            {
                if (PulseIn1RisingEdge())
                    step = (step + 1) & 7;
            }
            else
            {
                if (++clockCounter >= DEFAULT_STEP_SAMPLES)
                {
                    clockCounter = 0;
                    step = (step + 1) & 7;
                }
            }

            // X selects bass pattern, Y selects snare pattern
            int bassIdx = KnobVal(Knob::X) * 31 / 4095;
            int snareIdx = KnobVal(Knob::Y) * 31 / 4095;

            uint8_t bassPat = bassPatterns[bassIdx];
            uint8_t snarePat = snarePatterns[snareIdx];

            // Bit 7 = step 0, bit 0 = step 7
            bool bassHit = (bassPat >> (7 - step)) & 1;
            bool snareHit = (snarePat >> (7 - step)) & 1;

            // Snare muted when bass drum plays
            if (bassHit)
                snareHit = false;

            // Fire short trigger pulses on step change
            if (step != prevStep)
            {
                if (bassHit) { bassTrig = TRIG_SAMPLES; bassLed = LED_FLASH_BRIGHT; bassEnv = KICK_ENV_MAX; }
                if (snareHit) { snareTrig = TRIG_SAMPLES; snareLed = LED_FLASH_BRIGHT; snareEnv = SNARE_ENV_MAX; }
                if (!bassHit && !snareHit) hihatPos = 0;
                prevStep = step;
            }
            PulseOut1(bassTrig > 0);
            PulseOut2(snareTrig > 0);
            if (bassTrig > 0) bassTrig--;
            if (snareTrig > 0) snareTrig--;

            // CV envelopes: ramp down from 2047 to 0
            CVOut1(static_cast<int16_t>(bassEnv * 2047 / KICK_ENV_MAX));
            CVOut2(static_cast<int16_t>(snareEnv * 2047 / SNARE_ENV_MAX));
            if (bassEnv > 0) bassEnv--;
            if (snareEnv > 0) snareEnv--;

            // Hi-hat sample on audio out 1
            if (hihatPos < HIHAT_LEN)
                AudioOut1(hihatSample[hihatPos++]);
            else
                AudioOut1(0);

            // White noise on audio out 2
            AudioOut2(noise());

            LedBrightness(0, bassLed);
            LedBrightness(1, snareLed);
            if (bassLed > LED_DECAY) bassLed -= LED_DECAY; else bassLed = 0;
            if (snareLed > LED_DECAY) snareLed -= LED_DECAY; else snareLed = 0;

            // LEDs 2-4: step position binary
            LedOn(2, (step >> 2) & 1);
            LedOn(3, (step >> 1) & 1);
            LedOn(4, step & 1);
        }
};

int main()
{
    Dr8 card;
    card.Run();
}
