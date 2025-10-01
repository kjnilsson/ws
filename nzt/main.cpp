#include <cstdint>
#include "ComputerCard.h"

constexpr int16_t BIT_12_MIN = -2048;
constexpr int16_t BIT_12_MAX = 2047;
///
class NoiseTools : public ComputerCard
{
    private:

        uint16_t gateOut1Count = 0;
        uint32_t rnd12Seed = 1;
        int16_t sampleHoldValue = 0;
        uint16_t Rnd12() noexcept
        {
            rnd12Seed = 1664525 * rnd12Seed + 1013904223;
            return rnd12Seed >> 20;
        }

        uint16_t Rnd() noexcept
        {
            static uint32_t lcgSeed = 1;
            lcgSeed = 1664525 * lcgSeed + 1013904223;
            return lcgSeed >> 16;
        }

        inline uint16_t LogScale(uint16_t input) noexcept
        {
            if (input == 0) return 0;

            // 65 points - subdivides each original segment into 4 parts
            static constexpr uint16_t LOG_POINTS[65] = {
                0, 16, 32, 48, 64, 80, 96, 112,
                128, 160, 192, 224, 256, 320, 384, 448,
                512, 576, 640, 704, 768, 832, 896, 960,
                1024, 1152, 1280, 1725, 2242, 2587, 2587, 2932,
                2932, 3967, 4484, 5001, 5519, 6209, 6381, 6726,
                7416, 8278, 9485, 11727, 13107, 15004, 17591, 20523,
                22937, 24834, 27249, 30870, 33975, 36562, 41218, 46047,
                51048, 54498, 57602, 61223, 62948, 63810, 64845, 64845,
                65018
            };
            uint16_t index = input >> 6;  // Upper 6 bits (0-63)
            uint16_t fraction = input & 0x3F;  // Lower 6 bits for interpolation
            if (index >= 64) return 65535;

            uint32_t base = LOG_POINTS[index];
            uint32_t next = LOG_POINTS[index + 1];
            return base + (((next - base) * fraction) >> 6);
        }

        inline int16_t Clamp12Bit(int32_t value) const {
            if(value < BIT_12_MIN)
                return  BIT_12_MIN;
            if(value > BIT_12_MAX)
                return BIT_12_MAX;
            return static_cast<int16_t>(value);
        }

        int16_t RingMod(int16_t carrier, int16_t modulator)
        {
            int32_t x = static_cast<int32_t>(carrier) * static_cast<int32_t>(modulator);
            return Clamp12Bit(x >> 11);
        }

        inline int16_t apply_gain(int16_t signal, uint16_t gain) {
            // Multiply
            int32_t result = static_cast<int32_t>(signal) *
                static_cast<int32_t>(gain * 3);

            // Divide by 4096 (2^12) instead of 4095 using bit shift
            result = result >> 12;

            return static_cast<int16_t>(result);
        }

    public:
        NoiseTools()
        {
        }

        virtual void ProcessSample() override
        {
            if (PulseIn1RisingEdge())
            {
                uint16_t knobXVal = 0;
                if(Connected(Input::CV2))
                    knobXVal = CVIn2() + 2047;
                else
                    knobXVal = KnobVal(Knob::X);

                // reset the seed
                rnd12Seed = knobXVal >> 5;
            }

            uint16_t main = KnobVal(Knob::Main);

            if(Connected(Input::CV1))
            {
                // apply cv and cv gain
                int32_t afterGain = main + apply_gain(CVIn1(), KnobVal(Knob::Y));
                if(afterGain < 0)
                    main = 0;
                else if(afterGain > 4095)
                    main = 4095;
                else
                    main = afterGain;
            }

            const uint16_t mainScaled = LogScale(main);
            const uint16_t mainScaledInv = LogScale(4095 - main);
            const uint16_t which = Rnd();

            int16_t noise = 0;
            if(Connected(Input::Audio2))
                noise = AudioIn2();
            else
                noise = Rnd12() - 2048;

            // if there is audio in ringmod the noise with it
            if(Connected(Input::Audio1))
                noise = RingMod(noise, AudioIn1());


            if(SwitchVal() == Switch::Up)
                noise = (noise >> 10) << 10;

            AudioOut1(0);
            AudioOut2(0);

            if(which < mainScaled)
                AudioOut1(noise);

            if (which < mainScaledInv)
                AudioOut2(noise);

            /* CVOut1(KnobVal(Knob::Y) - 2048); */
            // static value
            CVOut1(BIT_12_MIN);

            if (PulseIn2RisingEdge())
                sampleHoldValue = noise;

            CVOut2(sampleHoldValue);

            LedBrightness(0, main);

            // regular gate thing
            if(gateOut1Count < 512)
            {
                PulseOut1(true);
                LedOn(1, true);
            } else
            {
                PulseOut1(false);
                LedOn(1, false);
            }

            gateOut1Count++;
        }
};


int main()
{
	NoiseTools nzt;
    nzt.EnableNormalisationProbe();
	nzt.Run();

}


