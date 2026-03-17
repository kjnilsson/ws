#include "ComputerCard.h"
#include "../Tuner.h"

class Tnr : public ComputerCard {
    PitchDetector pd1, pd2;
    uint8_t counter;

    // ledFlat=4, ledCenter=2, ledSharp=0  (0 = above C, 4 = below C)
    void applyLeds(TunerLedValues v, uint8_t ledFlat, uint8_t ledCenter, uint8_t ledSharp) {
        LedBrightness(ledFlat,   v.flat);
        LedBrightness(ledCenter, v.center);
        LedBrightness(ledSharp,  v.sharp);
    }

public:
    Tnr() : counter(0) {}

    virtual void ProcessSample() override {
        // Both CV outputs emit middle C
        CVOutMIDINote(0, 60);
        CVOutMIDINote(1, 60);

        pd1.addSample(AudioIn1());
        pd2.addSample(AudioIn2());

        // Update display at ~187 Hz (every 256 samples)
        if (!counter) {
            applyLeds(tunerLedValues(periodToCloseness(pd1.period())), 4, 2, 0);
            applyLeds(tunerLedValues(periodToCloseness(pd2.period())), 5, 3, 1);
        }
        counter++;
    }
};

int main() {
    Tnr tnr;
    tnr.Run();
}
