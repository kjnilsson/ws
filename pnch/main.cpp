// pnch — flash scratchpad sampler
//
// A durable audio scratchpad stored in flash.  Record patchwork audio into
// arbitrary positions, then loop-play any region.  Survives power cycles.
// 16-bit lossless audio at 24 kHz (ISR runs at 48 kHz, every other sample).
//
// SWITCH UP — Playback:
//   Main knob = playback start offset within flash
//   X knob    = loop length (short → full region)
//   Audio Out 1 = playback, Audio Out 2 = dry monitor
//
// SWITCH MIDDLE — Stopped / position cursor:
//   Main knob = set recording start position (cursor)
//   No audio output.  LEDs show cursor position.
//
// SWITCH DOWN — Record (sound-on-sound):
//   Records Audio In 1 at 24 kHz starting from the cursor position.
//   Advances linearly through flash until released (ignores loop length).
//   Y knob = mix balance: CCW = keep existing, CW = replace with input.
//   Audio Out 1 = input monitor, Audio Out 2 = input monitor.
//
// ERASE: hold switch down at power-on to wipe all recorded data.
//
// Buffer duration (2 bytes/sample at 24 kHz):
//   2 MB flash  → ~38 s
//  16 MB flash  → ~5.7 min
//
// LEDs: show position as a bar across the flash region.

#include "ComputerCard.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// ---- Flash layout ----

static constexpr uint32_t FLASH_CODE_RESERVE   = 256u * 1024u;
static constexpr uint32_t FLASH_REGION_SIZE    = PICO_FLASH_SIZE_BYTES - FLASH_CODE_RESERVE;
static constexpr uint32_t FLASH_REGION_SECTORS = FLASH_REGION_SIZE / FLASH_SECTOR_SIZE;
static constexpr uint32_t FLASH_REGION_OFFSET  = FLASH_CODE_RESERVE;

// 16-bit samples: 2048 samples per 4 KB sector
static constexpr int SAMPLES_PER_SECTOR = FLASH_SECTOR_SIZE / 2;  // 2048
static constexpr int BYTES_PER_SECTOR   = SAMPLES_PER_SECTOR * 2; // 4096

static constexpr uint32_t TOTAL_SAMPLES    = FLASH_REGION_SECTORS * (uint32_t)SAMPLES_PER_SECTOR;
static constexpr uint32_t MIN_LOOP_SAMPLES = (uint32_t)SAMPLES_PER_SECTOR * 2;

static constexpr int XFADE_LEN = 256;  // at 24 kHz, ~10 ms

// Double-buffered sector staging for recording.
static uint8_t sectorBuf[2][FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
static volatile int  sectorSampleIdx;  // sample index within current buffer (0..2047)
static volatile int  writeBank;
static volatile int  flushBank;        // -1 = none
static volatile int  writeSectorIdx;

// Pre-read buffers for sound-on-sound
static uint8_t preRead[2][FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

static volatile bool recording;
static volatile bool recPending;
static volatile int  recMix;

// Read a sample from flash given an absolute sample index (24 kHz domain).
static inline int16_t readFlashSample(uint32_t sampleIdx)
{
    uint32_t sectorIdx   = sampleIdx / SAMPLES_PER_SECTOR;
    uint32_t sampleInSec = sampleIdx % SAMPLES_PER_SECTOR;
    uint32_t flashAddr   = FLASH_REGION_OFFSET + sectorIdx * FLASH_SECTOR_SIZE
                         + sampleInSec * 2;
    const int16_t* p     = (const int16_t*)(XIP_BASE + flashAddr);
    return *p;
}

class Pnch : public ComputerCard
{
public:
    Pnch()
        : cursorSample(0), playPos(0), playOffset(0), playLoopLen(TOTAL_SAMPLES),
          targetOffset(0), targetLoopLen(TOTAL_SAMPLES),
          lastPlaySample(0), lastRecInput(0),
          phase(0), wasDown(false), wasUp(false), knobUpdateCounter(0) {}

    static void audioEntry()
    {
        multicore_lockout_victim_init();
        s_instance->Run();
    }

    static void readSector(int sectorIdx, uint8_t* dst)
    {
        uint32_t off = FLASH_REGION_OFFSET + (uint32_t)sectorIdx * FLASH_SECTOR_SIZE;
        memcpy(dst, (const uint8_t*)(XIP_BASE + off), FLASH_SECTOR_SIZE);
    }

    void flashCore()
    {
        // Hold switch down at boot to erase all recorded data.
        sleep_ms(500);
        bool eraseConfirmed = true;
        for (int check = 0; check < 5; check++) {
            if (SwitchVal() != Switch::Down) { eraseConfirmed = false; break; }
            sleep_ms(50);
        }
        if (eraseConfirmed) {
            for (int i = 0; i < 6; i++) LedOn(i, true);
            for (uint32_t s = 0; s < FLASH_REGION_SECTORS; s++) {
                uint32_t off = FLASH_REGION_OFFSET + s * FLASH_SECTOR_SIZE;
                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(off, FLASH_SECTOR_SIZE);
                restore_interrupts(ints);
                int ledIdx = (int)(s * 6 / FLASH_REGION_SECTORS);
                for (int i = 0; i < 6; i++) LedOn(i, i <= ledIdx);
            }
            for (int i = 0; i < 6; i++) LedOn(i, false);
            while (SwitchVal() == Switch::Down) sleep_ms(10);
        }

        while (true)
        {
            if (recPending) {
                int sec = writeSectorIdx;
                int sec1 = (sec + 1) % (int)FLASH_REGION_SECTORS;

                readSector(sec,  preRead[0]);
                readSector(sec1, preRead[1]);

                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(FLASH_REGION_OFFSET + (uint32_t)sec  * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
                flash_range_erase(FLASH_REGION_OFFSET + (uint32_t)sec1 * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
                restore_interrupts(ints);

                recPending = false;
            }

            if (flushBank >= 0) {
                int fb = flushBank;
                uint32_t off = FLASH_REGION_OFFSET
                             + (uint32_t)writeSectorIdx * FLASH_SECTOR_SIZE;

                uint32_t ints = save_and_disable_interrupts();
                flash_range_program(off, sectorBuf[fb], FLASH_SECTOR_SIZE);
                restore_interrupts(ints);

                int nextSec = writeSectorIdx + 1;
                if (nextSec >= (int)FLASH_REGION_SECTORS) nextSec = 0;
                writeSectorIdx = nextSec;
                flushBank = -1;

                if (recording) {
                    int aheadSec = (nextSec + 1) % (int)FLASH_REGION_SECTORS;
                    readSector(aheadSec, preRead[fb]);

                    uint32_t eraseOff = FLASH_REGION_OFFSET
                                      + (uint32_t)nextSec * FLASH_SECTOR_SIZE;
                    ints = save_and_disable_interrupts();
                    flash_range_erase(eraseOff, FLASH_SECTOR_SIZE);
                    restore_interrupts(ints);
                }
            }
        }
    }

    virtual void ProcessSample() override
    {
        int16_t input = AudioIn1();
        Switch sw = SwitchVal();

        // ISR runs at 48 kHz; we process one 24 kHz sample every other call.
        bool tick = (phase == 0);
        phase ^= 1;

        // =============================================================
        // SWITCH DOWN — Record (sound-on-sound, 16-bit @ 24 kHz)
        // =============================================================
        if (sw == Switch::Down) {
            if (!wasDown) {
                writeSectorIdx = (int)(cursorSample / SAMPLES_PER_SECTOR);
                sectorSampleIdx = 0;
                writeBank      = 0;
                flushBank      = -1;
                recording      = true;
                recPending     = true;
                recMix         = KnobVal(Knob::Y);
                phase          = 0;
                tick           = true;
                wasDown = true;
            }

            if (++knobUpdateCounter >= 1024) {
                knobUpdateCounter = 0;
                recMix = KnobVal(Knob::Y);
            }

            if (tick && !recPending) {
                int mix = recMix;
                int16_t* pre = (int16_t*)(preRead[writeBank]);
                int16_t existing = pre[sectorSampleIdx];

                int32_t mixed = ((int32_t)existing * (4095 - mix)
                               + (int32_t)input * mix) >> 12;
                if (mixed >  2047) mixed =  2047;
                if (mixed < -2048) mixed = -2048;

                int16_t* dst = (int16_t*)(sectorBuf[writeBank]);
                dst[sectorSampleIdx] = (int16_t)mixed;

                if (++sectorSampleIdx >= SAMPLES_PER_SECTOR) {
                    if (flushBank < 0) {
                        flushBank      = writeBank;
                        writeBank      = writeBank ^ 1;
                        sectorSampleIdx = 0;
                    } else {
                        sectorSampleIdx = 0;  // core 0 busy — small gap
                    }
                }
            }

            AudioOut1(input);
            AudioOut2(input);

            int ledIdx = (writeSectorIdx * 5) / (int)FLASH_REGION_SECTORS;
            for (int i = 0; i < 5; i++) LedOn(i, i == ledIdx);
            LedBrightness(5, (uint16_t)recMix);

            wasUp = false;
            return;
        }

        if (wasDown) {
            recording = false;
            cursorSample = (uint32_t)writeSectorIdx * SAMPLES_PER_SECTOR;
            wasDown = false;
        }

        // =============================================================
        // SWITCH UP — Playback
        // =============================================================
        if (sw == Switch::Up) {
            if (!wasUp) {
                playOffset  = knobToSampleOffset(KnobVal(Knob::Main));
                playLoopLen = knobToLoopLen(KnobVal(Knob::X));
                targetOffset  = playOffset;
                targetLoopLen = playLoopLen;
                playPos = 0;
                phase   = 0;
                tick    = true;
                wasUp = true;
            }

            if (++knobUpdateCounter >= 1024) {
                knobUpdateCounter = 0;
                targetOffset  = knobToSampleOffset(KnobVal(Knob::Main));
                targetLoopLen = knobToLoopLen(KnobVal(Knob::X));
            }

            if (tick) {
                uint32_t pos = playPos++;

                if (pos >= playLoopLen) {
                    playOffset  = targetOffset;
                    playLoopLen = targetLoopLen;
                    playPos = 0;
                    pos = 0;
                }

                uint32_t absSample = (playOffset + pos) % TOTAL_SAMPLES;
                int16_t smp = readFlashSample(absSample);

                // Crossfade at loop boundary
                if (playLoopLen > (uint32_t)(XFADE_LEN * 2) && pos >= playLoopLen - XFADE_LEN) {
                    uint32_t xOff = pos - (playLoopLen - XFADE_LEN);
                    uint32_t headSample = (playOffset + xOff) % TOTAL_SAMPLES;
                    int16_t smp2 = readFlashSample(headSample);
                    int fade = (int)(xOff * 256 / XFADE_LEN);
                    smp = (int16_t)(smp + (((smp2 - smp) * fade) >> 8));
                }

                lastPlaySample = smp;
            }

            AudioOut1(lastPlaySample);
            AudioOut2(input);

            showRegionLEDs(playOffset, playLoopLen);
            return;
        }

        // =============================================================
        // SWITCH MIDDLE — Stopped, position cursor
        // =============================================================
        wasUp = false;
        cursorSample  = knobToSampleOffset(KnobVal(Knob::Main));
        targetLoopLen = knobToLoopLen(KnobVal(Knob::X));
        playLoopLen   = targetLoopLen;

        AudioOut1(0);
        AudioOut2(0);

        int ledIdx = (int)((uint64_t)cursorSample * 6 / TOTAL_SAMPLES);
        if (ledIdx > 5) ledIdx = 5;
        for (int i = 0; i < 6; i++) LedOn(i, i == ledIdx);
    }

private:
    uint32_t knobToSampleOffset(int knob)
    {
        uint32_t sample = ((uint32_t)knob * (TOTAL_SAMPLES - SAMPLES_PER_SECTOR)) >> 12;
        sample = (sample / SAMPLES_PER_SECTOR) * SAMPLES_PER_SECTOR;
        return sample;
    }

    uint32_t knobToLoopLen(int knob)
    {
        uint32_t len = MIN_LOOP_SAMPLES
                     + ((uint32_t)knob * (TOTAL_SAMPLES - MIN_LOOP_SAMPLES)) / 4096u;
        len = (len / SAMPLES_PER_SECTOR) * SAMPLES_PER_SECTOR;
        if (len < MIN_LOOP_SAMPLES) len = MIN_LOOP_SAMPLES;
        return len;
    }

    void showRegionLEDs(uint32_t offset, uint32_t len)
    {
        for (int i = 0; i < 6; i++) {
            uint32_t ledStart = (uint32_t)((uint64_t)i * TOTAL_SAMPLES / 6);
            uint32_t ledEnd   = (uint32_t)((uint64_t)(i + 1) * TOTAL_SAMPLES / 6);
            bool lit = false;
            uint32_t loopEnd = offset + len;
            if (loopEnd <= TOTAL_SAMPLES) {
                lit = (ledEnd > offset && ledStart < loopEnd);
            } else {
                lit = (ledEnd > offset) || (ledStart < (loopEnd % TOTAL_SAMPLES));
            }
            LedOn(i, lit);
        }
    }

    uint32_t cursorSample;
    uint32_t playPos;
    uint32_t playOffset;
    uint32_t playLoopLen;
    uint32_t targetOffset;
    uint32_t targetLoopLen;
    int16_t  lastPlaySample;  // held for sample-and-hold on odd ISR calls
    int16_t  lastRecInput;
    int      phase;           // 0 or 1, toggles each ISR call for 24 kHz decimation
    bool     wasDown;
    bool     wasUp;
    int      knobUpdateCounter;

public:
    static Pnch* s_instance;
};

Pnch* Pnch::s_instance = nullptr;

int main()
{
    set_sys_clock_khz(192000, true);
    Pnch p;
    Pnch::s_instance = &p;
    p.EnableNormalisationProbe();
    multicore_launch_core1(Pnch::audioEntry);
    p.flashCore();
}
