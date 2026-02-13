#include "ComputerCard.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "BucketBrigadeDelay.h"
///
class Seq6 : public ComputerCard
{
    static constexpr uint8_t  NUM_STAGES       = 6;
    static constexpr uint16_t DEFAULT_STEP_LEN = 48000 / 2;
    static constexpr uint16_t GATE_PRE_COUNT   = 48;
    static constexpr uint16_t GATE2_PULSE_LEN  = 96;
    static constexpr uint8_t  MIN_NOTE         = 48;
    static constexpr uint8_t  NOTE_RANGE       = 36;
    static constexpr uint8_t  EDIT_NOTE_OFFSET = 32;
    static constexpr uint8_t  EDIT_NOTE_RANGE  = 72;
    static constexpr uint32_t BBD_MAX_DELAY    = 65535 * 4;
    static constexpr uint16_t MIN_STEP_LEN     = 2880;   // ~60ms at 48kHz
    static constexpr uint16_t MAX_STEP_LEN     = 48000;  // 1s

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
        public:
            Gate() : count(0) {}

            void Activate(uint32_t num_samples) { count = num_samples; }
            bool IsActive() { return count != 0; }

            void Reset() {
                count = 0;
                pre_count = GATE_PRE_COUNT;
            }

            bool Tick() {
                if (pre_count == 0 && IsActive()) {
                    count--;
                    return true;
                }
                if (pre_count > 0)
                    pre_count--;
                return false;
            }
    };


    private:

        Stage stages[NUM_STAGES];
        Gate gate1;
        Gate gate2;
        uint16_t step_len_samples = DEFAULT_STEP_LEN;
        uint16_t step_len_counter = 0;
        uint8_t current_stage = 0;
        uint8_t steps_completed = 0;
        Switch last_switch_val = Switch::Up;
        int16_t sample_and_hold = 0;
        bool reset = true;
        static BucketBrigadeDelay bbdDelay;

        uint32_t lcg_seed = getRandomSeed();

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

        uint16_t rnd_12bit() noexcept
        {
            return rnd() >> 4;
        }

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

        // 25-50% zone on main knob: increasing probability of ±octave shift
        void maybeOctaveShift(uint8_t& note)
        {
            auto knob = KnobVal(Knob::Main);
            if (knob > 1024 && knob <= 2048) {
                uint32_t prob = ((knob - 1024) * 65535) / 1024;
                if (rnd() < prob) {
                    if (rnd() & 1)
                        note = (note + 12 <= 127) ? note + 12 : note - 12;
                    else
                        note = (note >= 12) ? note - 12 : note + 12;
                }
            }
        }

        void applyReset()
        {
            current_stage = 0;
            steps_completed = 0;
            step_len_counter = 0;
            gate1.Reset();
            auto gate_len =
                (stages[current_stage].steps * step_len_samples) >> 1;
            gate1.Activate(gate_len);
            gate2.Activate(GATE2_PULSE_LEN);
            sample_and_hold = rnd_12bit() - 2048;
            reset = false;
        }

        void processAudio()
        {
            if (Connected(Input::Audio1) && !Connected(Input::Audio2))
            {
                bbdDelay.setDelaySamples(step_len_samples);
                bbdDelay.setFeedback(160);
                AudioOut1(bbdDelay.process(AudioIn1()));
            }
            else if (Connected(Input::Audio2))
            {
                auto delaySamples = step_len_samples << 1;
                if (Connected(Input::Audio1))
                    delaySamples = delaySamples + step_len_samples;

                bbdDelay.setFeedback(128);
                bbdDelay.setDelaySamples(delaySamples);
                AudioOut1(bbdDelay.process(AudioIn2()));
            }
            else
            {
                // noise and s&h
                AudioOut1(rnd_12bit() - 2048);
            }

            AudioOut2(sample_and_hold);
        }

        void processPlayback()
        {
            // X knob controls tempo when no pulse input connected
            if (!Connected(Input::Pulse1)) {
                auto knobX = KnobVal(Knob::X);
                step_len_samples = MAX_STEP_LEN - (((uint32_t)knobX * (MAX_STEP_LEN - MIN_STEP_LEN)) >> 12);
            }

            if (PulseIn2RisingEdge()) {
                if (Connected(Input::Pulse1)) {
                    // external clock: defer reset to next clock edge
                    reset = true;
                } else {
                    // internal clock: reset immediately
                    applyReset();
                }
            }

            Stage stage = stages[current_stage];
            CVOutMIDINote(0, stage.note);
            CVOutMIDINote(1, stage.note);

            PulseOut(0, gate1.Tick());
            PulseOut(1, gate2.Tick());

            // dim LED to indicate main knob performance zone
            // 0: forward, 1: octave shift, 2: reverse, 3: random
            auto knobMain = KnobVal(Knob::Main);
            uint8_t zone = (knobMain > 3072) ? 3 : (knobMain > 2048) ? 2 : (knobMain > 1024) ? 1 : 0;
            LedBrightness(zone, 256);

            if (gate1.IsActive())
                LedOn(current_stage, true);

            if (PulseIn1RisingEdge() ||
                    (!Connected(Input::Pulse1) &&
                     step_len_counter >= step_len_samples))
            {
                if (Connected(Input::Pulse1))
                    step_len_samples = step_len_counter;
                step_len_counter = 0;

                if (reset)
                {
                    applyReset();
                }
                else if (steps_completed >= stage.steps - 1)
                {
                    // enough steps, advance to next stage
                    steps_completed = 0;

                    // performance knob: 4 zones
                    // 0-25% forward | 25-50% forward+octave shift | 50-75% reverse | 75-100% random
                    auto knob = KnobVal(Knob::Main);

                    if (knob > 3072) {
                        // random stage
                        current_stage = rnd() % NUM_STAGES;
                    } else if (knob > 2048) {
                        // reverse
                        current_stage = (current_stage == 0) ? NUM_STAGES - 1 : current_stage - 1;
                    } else {
                        // forward
                        if (++current_stage == NUM_STAGES)
                            current_stage = 0;
                    }

                    maybeOctaveShift(stages[current_stage].note);

                    // Y knob > 50%: retrigger gate on every step
                    bool retrigger = KnobVal(Knob::Y) > 2048;
                    auto gate_len = retrigger
                        ? step_len_samples >> 1
                        : (stages[current_stage].steps * step_len_samples) >> 1;
                    gate1.Activate(gate_len);
                    gate2.Activate(GATE2_PULSE_LEN);
                    // s&h
                    sample_and_hold = rnd_12bit() - 2048;
                }
                else
                {
                    steps_completed++;
                    // retrigger: re-fire gate on intermediate steps
                    if (KnobVal(Knob::Y) > 2048) {
                        maybeOctaveShift(stages[current_stage].note);
                        gate1.Reset();
                        gate1.Activate(step_len_samples >> 1);
                        gate2.Activate(GATE2_PULSE_LEN);
                    }
                }
            }
            else
            {
                step_len_counter++;
            }
        }

        void processManual()
        {
            gate1.Activate(10);
            gate2.Activate(1);
            PulseOut(0, gate1.Tick());
            PulseOut(1, gate2.Tick());
            current_stage = (KnobVal(Knob::Y) * NUM_STAGES) >> 12;
            LedOn(current_stage, true);
            Stage stage = stages[current_stage];
            CVOutMIDINote(0, stage.note);
            CVOutMIDINote(1, stage.note);
        }

        // "Catch" editing: knob must first match the current value before
        // changes are applied, preventing jumps when switching stages.
        void processEdit(Switch switchVal)
        {
            for (int i = 0; i < NUM_STAGES; i++)
                LedOff(i);

            Stage& stage = stages[current_stage];

            // first frame after switching: disable editing until knobs catch
            if (switchVal != last_switch_val)
            {
                stage.editable = false;
                stage.stepsEditable = false;
            }

            auto knobNote = EDIT_NOTE_OFFSET + ((KnobVal(Knob::Main) * EDIT_NOTE_RANGE) >> 12);

            if (!stage.editable && stage.note == knobNote)
                stage.editable = true;

            if (stage.editable)
                stage.note = knobNote;

            auto stepsValue = ((KnobVal(Knob::X) * NUM_STAGES) >> 12) + 1;

            if (!stage.stepsEditable && stage.steps == stepsValue)
                stage.stepsEditable = true;

            if (stage.stepsEditable)
                stage.steps = stepsValue;

            for (int i = 0; i < stage.steps; i++)
                LedBrightness(i, 512);

            if (stage.editable)
                LedOn(0);

            if (stage.stepsEditable)
                LedOn(stage.steps - 1);

            CVOutMIDINote(0, stage.note);
            CVOutMIDINote(1, stage.note);
        }

    public:
        Seq6()
        {
            for (int i = 0; i < NUM_STAGES; i++)
            {
                stages[i].note = MIN_NOTE + rnd() % NOTE_RANGE;
                stages[i].steps = 1 + (rnd() % 5);
            }

            bbdDelay.setDelaySamples(12000);
        }

        virtual void ProcessSample() override
        {
            processAudio();

            for (int i = 0; i < NUM_STAGES; i++)
                LedOn(i, false);

            auto switchVal = SwitchVal();
            switch (switchVal)
            {
                case Switch::Up:     processPlayback();      break;
                case Switch::Middle: processManual();         break;
                case Switch::Down:   processEdit(switchVal);  break;
                default: break;
            }

            last_switch_val = switchVal;
        }
};

BucketBrigadeDelay Seq6::bbdDelay(BBD_MAX_DELAY, 200, 0, 128, 20000);

int main()
{
	Seq6 seq;
    seq.EnableNormalisationProbe();
	seq.Run();
}
