#include "ComputerCard.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
///
class Seq6 : public ComputerCard
{
    struct Stage {
        uint8_t steps = 1; //1-6
        uint8_t note = 60;
        bool editable = false;
        bool stepsEditable = false;
    };

    class Gate {

        private:
            uint32_t count = 0;
            uint16_t pre_count = 0;
            uint8_t pulse = 0;
            Seq6* parent;  // ← Reference to parent object
        public:
            Gate(Seq6* p, uint8_t pulse) : count(0), pulse(pulse), parent(p) { }

            void Activate(uint32_t num_samples)
            {
                this->count = num_samples;
            }

            bool IsActive()
            {
                return count != 0;
            }

            void Reset()
            {
                count = 0;
                pre_count = 48;
                parent->PulseOut(pulse, false);
            }

            void Tick()
            {
                if(pre_count == 0 && IsActive())
                {
                    count--;
                    parent->PulseOut(pulse, true);
                }
                else
                {
                    if(pre_count > 0)
                        pre_count--;

                    parent->PulseOut(pulse, false);
                }
            }
    };


    private:

        Stage stages[6];
        Gate gate1 = Gate(this, 0);
        uint16_t step_len_samples = 48000 / 2;
        uint16_t step_len_c = 0;
        uint8_t current_stage = 0;
        uint8_t next_step = 0;
        Switch last_switch_val = Switch::Up;
        int16_t audioOut2Value = 0;
        bool reset = true;

        uint32_t lcg_seed = getRandomSeed();
        uint16_t rnd12() noexcept
        {
            static uint32_t rnd12_seed = static_cast<uint32_t>(UniqueCardID());
            rnd12_seed = 1664525 * rnd12_seed + 1013904223;
            return rnd12_seed >> 20;
        }

        uint32_t getRandomSeed() {
            adc_set_temp_sensor_enabled(true);
            adc_select_input(4);  // Temp sensor = ADC channel 4

            uint32_t random = 0;
            for (int i = 0; i < 8; i++) {
                random = (random << 4) | (adc_read() & 0xF);
            }

            return random;
        }

        uint16_t rnd() noexcept
        {
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
        Seq6()
        {
            /* lcg_seed = KnobVal(Knob::Main); */
            //initialise to random note values
            for(int i = 0; i < 6; i++)
            {
                stages[i].note = 48 + rnd() % 36;
                stages[i].steps = 1 + (rnd() % 5);
            }
        }

        virtual void ProcessSample() override
        {

            // noise and s&h
            AudioOut1(rnd12() - 2048);
            AudioOut2(audioOut2Value);

            for(int i = 0; i < 6; i++)
                LedOn(i, false);


            auto switchVal = SwitchVal();
            switch (switchVal)
            {
                case Switch::Up:
                {
                    // handle reset
                    if(PulseIn2RisingEdge())
                    {
                        reset = true;
                    }

                    Stage stage = stages[current_stage];
                    // always set the note value
                    CVOutMIDINote(0, stage.note);
                    CVOutMIDINote(1, stage.note);

                    gate1.Tick();

                    if(gate1.IsActive())
                        LedOn(current_stage, true);

                    if (PulseIn1RisingEdge() ||
                            (!Connected(Input::Pulse1) &&
                             step_len_c >= step_len_samples))
                    {
                        step_len_samples = step_len_c;
                        step_len_c = 0;
                        //adjust step_len_samples based on incoming pulses
                        if(reset)
                        {
                            current_stage = 0;
                            next_step = 0;
                            gate1.Reset();
                            auto gate_len =
                                (stages[current_stage].steps * step_len_samples) >> 1;
                            gate1.Activate(gate_len);
                            audioOut2Value = rnd12() - 2048;
                            reset = false;
                        }
                        else if(next_step >= stage.steps - 1)
                        {
                            //enough steps, increment stage
                            next_step = 0;

                            /* current_stage++; */
                            if(++current_stage == 6)
                                current_stage = 0;
                            // 50% step length
                            auto gate_len =
                                (stages[current_stage].steps * step_len_samples) >> 1;
                            gate1.Activate(gate_len);
                            // s&h
                            audioOut2Value = rnd12() - 2048;
                        }
                        else
                        {
                            next_step++;
                        }

                    }
                    else
                    {
                        step_len_c++;
                    }

                    //playback
                    break;
                };
                case Switch::Middle:
                {
                    gate1.Activate(10);
                    gate1.Tick();
                    current_stage = (KnobVal(Knob::Y) * 6) >> 12;
                    LedOn(current_stage, true);
                    Stage stage = stages[current_stage];
                    CVOutMIDINote(0, stage.note);
                    CVOutMIDINote(1, stage.note);
                    break;
                };
                case Switch::Down:
                {
                    for(int i = 0; i < 6; i++)
                        LedOff(i);

                    Stage& stage = stages[current_stage];
                    //it is the first time after a switch, set ediable = false
                    if(switchVal != last_switch_val)
                    {
                        stage.editable = false;
                        stage.stepsEditable = false;
                    }

                    auto knobNote = 32 + ((KnobVal(Knob::Main) * 72) >> 12);

                    if(!stage.editable && stage.note == knobNote)
                        stage.editable = true;

                    //implement "catch" approach
                    if(stage.editable)
                        stage.note = knobNote;

                    auto stepsValue = ((KnobVal(Knob::X) * 6) >> 12) + 1;

                    if(!stage.stepsEditable && stage.steps == stepsValue)
                        stage.stepsEditable = true;

                    if(stage.stepsEditable)
                        stage.steps = stepsValue;

                    for(int i = 0; i < stage.steps; i++)
                        LedBrightness(i, 512);

                    /* LedOn(stage.steps - 1, true); */
                    if(stage.editable)
                        LedOn(0);

                    if(stage.stepsEditable)
                        LedOn(stage.steps - 1);

                    CVOutMIDINote(0, stage.note);
                    CVOutMIDINote(1, stage.note);
                    break;
                };
                default:
                {}

            };

            last_switch_val = switchVal;
        }
};


int main()
{
	Seq6 seq;
    seq.EnableNormalisationProbe();
	seq.Run();
}


