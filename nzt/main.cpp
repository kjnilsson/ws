#include "ComputerCard.h"

///
class NoiseTools : public ComputerCard
{
    private:

        uint16_t gate_out_1_c = 0;
        uint32_t rnd12_seed = 1;
        uint16_t rnd12() noexcept
        {
            rnd12_seed = 1664525 * rnd12_seed + 1013904223;
            return rnd12_seed >> 20;
        }

        uint16_t rnd() noexcept
        {
            static uint32_t lcg_seed = 1;
            lcg_seed = 1664525 * lcg_seed + 1013904223;
            return lcg_seed >> 16;
        }
        /* inline int16_t lerp12(int16_t a, int16_t b, uint8_t fract) */
        /* { */
        /*     return a + (((b - a) * fract) >> 8); */
        /* } */
        inline uint16_t log_scale(uint16_t input) noexcept
        {
            if (input == 0) return 0;

            // Lookup table for 12-bit to 16-bit logarithmic mapping
            static constexpr uint16_t log_points[17] = {
                0, 64, 128, 256, 512, 768, 1024, 1536,
                2048, 3072, 4608, 6912, 13824, 27648, 41472, 55296, 65535
            };

            // Map input to table index
            uint16_t index = input >> 8;  // Use upper 8 bits
            uint16_t fraction = input & 0xFF;  // Lower 8 bits for interpolation

            if (index >= 16) return 65535;

            // Linear interpolation between table points
            uint32_t base = log_points[index];
            uint32_t next = log_points[index + 1];

            return base + (((next - base) * fraction) >> 8);
        }

    public:
        NoiseTools()
        {
        }

        virtual void ProcessSample() override
        {
            if (PulseIn1RisingEdge())
                // reset the seed
                rnd12_seed = KnobVal(Knob::X) >> 5;

            uint16_t main = KnobVal(Knob::Main);
            if(Connected(Input::CV1))
            {
                if(SwitchVal() == Switch::Up)
                {
                    main = CVIn1();
                    //manual tuning here, could be device specific
                    main = (main << 2) - 256;
                }
                else
                    main = CVIn1() + 2047;
            }

            const uint16_t mainVal = log_scale(main);
            const uint16_t mainVal2 = log_scale(4096 - main);
            const uint16_t which = rnd();
            const int16_t noise = rnd12() - 2048;

            AudioOut1(0);
            AudioOut2(0);

            if(which < mainVal)
                AudioOut1(noise);

            if (which < mainVal2)
                AudioOut2(noise);

            CVOut1(KnobVal(Knob::Y) - 2048);


            LedBrightness(0, main);

            if(gate_out_1_c < 512)
            {
                PulseOut1(true);
                LedOn(1, true);
            } else
            {
                PulseOut1(false);
                LedOn(1, false);
            }
            gate_out_1_c++;

        }
};


int main()
{
	NoiseTools nzt;
    nzt.EnableNormalisationProbe();
	nzt.Run();
}


