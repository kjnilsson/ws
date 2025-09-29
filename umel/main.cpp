#include "ComputerCard.h"

class Delay8
{

};
///
class UMel : public ComputerCard
{
    private:

        uint64_t u = UniqueCardID();
        uint32_t led1Count = 0;
        uint32_t led4Count = 0;
        uint32_t samplesPerStep = 12000; // 4 steps per second
        uint32_t count = 0;

        uint8_t firstStep = 0;
        uint16_t lastStep = 63;
        uint8_t curStep = 0;
        uint8_t notes[256];
        uint8_t notes3[256];

        inline int16_t lerp12(int16_t a, int16_t b, uint8_t fract)
        {
            return a + (((b - a) * fract) >> 8);
        }

        inline uint8_t getNext(int i, int numBits=4)
        {

            uint8_t value = 0;
            // Extract 3 bits starting at position i, wrapping around
            for (int j = 0; j < numBits; ++j) {
                int bitPos = (i + j) % 64;  // Wrap around at bit 64
                uint64_t bit = (u >> bitPos) & 1;
                value |= (bit << j);
            }

            return value;
        }

        bool isSet(uint8_t i)
        {
            int bitPos = i % 64;
            return (u >> bitPos) & 1;
        }

    public:
        UMel()
        {
            int n = 0;
            uint8_t note = 60;
            uint8_t note3 = 60;
            uint8_t dir = 64;
            //skip unset notes at start of sequence
            uint8_t i2 = 0;
            while(i2 < 64 && !isSet(i2))
                i2++;

            // pre calculate 256 steps of the 4 bit melody
            for (int i = 0; i < 256 - i2; i++)
            {
                notes[i] = 0; //default to a rest
                notes3[i] = 0; //default to a rest
                if(isSet(i + i2))
                {
                    uint8_t interval = getNext(n % 64, 4);
                    if(isSet(dir))
                        note = note + interval;
                    else
                        note = note - interval;

                    uint8_t interval3 = getNext(n % 64, 3);
                    if(isSet(dir))
                        note3 = note3 + interval3;
                    else
                        note3 = note3 - interval3;
                    // wrap around
                    note %= 0x80;
                    note3 %= 0x80;

                    notes[i] = note;
                    notes3[i] = note3;
                    dir--;
                    dir %= 64;
                    n++;
                }
            }
        }

        virtual void ProcessSample() override
        {
            samplesPerStep = (4064 << 4) - (KnobVal(Knob::Y) << 4);
            samplesPerStep += 6000;
            samplesPerStep = (samplesPerStep >> 2) << 2;

            if((!Connected(Input::Pulse1) && count++ > samplesPerStep) ||
                    (Connected(Input::Pulse1) && PulseInRisingEdge(0)))
            {
                //evaluate offset knob
                if(Connected(Input::CV1))
                {
                    firstStep = (CVIn1() + 2024) >> 5;
                }
                else
                {
                    firstStep = KnobVal(Knob::Main) >> 5;
                }
                //shift down to 4 bits, 0-15
                uint32_t knobX = KnobVal(Knob::X) >> 8;
                /* auto seqLen = (knobX >> 12) + 2; */
                auto seqLen = (knobX + 1) * 4;
                lastStep = firstStep + seqLen - 1;
                if(lastStep > 255)
                    lastStep = 255;

                LedOn(5, firstStep == 0);
                LedOn(3, curStep == 0);

                //flash led 1
                LedOn(1, true);
                led1Count = 256;
                if(curStep == firstStep)
                {
                    //flash led 5 to indicate start of sequence
                    LedOn(4, true);
                    led4Count = 256;
                }

                uint8_t n;
                if(SwitchVal() == Switch::Middle)
                    n = notes[curStep] &= 0x7F;
                else
                    n = notes3[curStep] &= 0x7F;

                if(n)
                {
                    CVOutMIDINote(0, n);
                    CVOutMIDINote(1, n);
                }

                LedOn(0, n > 0);
                PulseOut1(n > 0);

                curStep++;

                if(curStep < firstStep || curStep > lastStep)
                    curStep = firstStep;

                count = 0;
            }

            if(led1Count == 0)
                LedOn(1, false);

            if(led4Count == 0)
                LedOn(4, false);

            led1Count--;
            led4Count--;
        }
};


int main()
{
	UMel um;
    um.EnableNormalisationProbe();
	um.Run();
}


