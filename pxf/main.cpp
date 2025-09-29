#include "ComputerCard.h"

/// Probabilistic Cross Fade
class ProbXFade : public ComputerCard
{
public:

    uint64_t u = UniqueCardID();
    uint32_t led0count = 0;

    /* uint8_t shift = 0; */
    int16_t previousOutput = 0;
    int shift = 0;
    int bits = 12;

    inline int16_t abs(int16_t x) {
        int16_t mask = x >> 15;  // Sign bit replicated to all bits
        return (x ^ mask) - mask;
    }

	uint16_t rnd()
	{
		static uint32_t lcg_seed = 1;
		lcg_seed = 1664525 * lcg_seed + 1013904223;
		return lcg_seed >> 16;
	}

    /* int16_t noise = (rnd() & 0xFFF) - 2048; */

    inline int16_t bitRed1(int16_t sample, int shift)
    {
        return (sample >> shift) << shift;
    }

    int16_t bitRed2(int16_t sample, int shift) {
        int smoothingShift = 4;
        // Clamp and quantize
        if (sample < -2048) sample = -2048;
        if (sample > 2047) sample = 2047;

        int16_t quantized = ((sample + (1 << (shift - 1))) >> shift) << shift;

        // Fast smoothing using bit shifting
        int16_t diff = quantized - previousOutput;
        previousOutput += (diff >> smoothingShift);

        return previousOutput;
    }

    inline int16_t lerp12(int16_t a, int16_t b, uint8_t fract)
    {
        return a + (((b - a) * fract) >> 8);
    }

    virtual void ProcessSample()
   {
        if (PulseIn1RisingEdge())
            LedOn(0, true);
        else if (PulseIn1FallingEdge())
            LedOn(0, false);

		LedBrightness(2, CVIn1() + 2048);

        uint16_t knobY = KnobVal(Knob::Y) * 12;
        //shift goes from 0 - 11
        shift = knobY >> 12;

        LedOn(5, shift == 0);
        LedOn(4, shift == 11);

        uint8_t frac = (knobY >> 4) % 0xFF;

        uint16_t out1 = lerp12(bitRed1(AudioIn1(), shift),
                               bitRed1(AudioIn1(), shift + 1), frac);
		AudioOut1(out1);
        uint16_t out2 = lerp12(bitRed2(AudioIn2(), shift),
                               bitRed2(AudioIn2(), shift + 1), frac);
		AudioOut2(out2);
		/* AudioOut2( bitReduce(AudioIn2())); */
        /* int rounding = 1 << (shift - 1); */
		/* AudioOut1(((AudioIn1() >> shift) << shift) - 2047); */


		CVOut1(CVIn1());
		CVOut2(CVIn2());

		/* PulseOut1(PulseIn1()); */
		/* PulseOut2(PulseIn2()); */

		/* // Get switch position and set LEDs 0, 2, 4 accordingly */
		/* int s = SwitchVal(); */
		/* LedOn(4, s == Switch::Down); */
		/* LedOn(2, s == Switch::Middle); */
		/* LedOn(0, s == Switch::Up); */

		/* // Set LED 1, 3, 5 brightness to knob values */
		/* LedBrightness(1, KnobVal(Knob::Main)); */
		/* LedBrightness(3, KnobVal(Knob::X)); */
		/* LedBrightness(5, KnobVal(Knob::Y)); */
	}
};


int main()
{
	ProbXFade pxf;
	pxf.Run();
}


