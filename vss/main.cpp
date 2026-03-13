// VSS - Yamaha VSS-30 inspired 8-bit sampler
// for Music Thing Workshop System Computer
//
// RECORD  (hold Z switch DOWN):
//   Samples Audio In 1 into an 8-bit mono buffer (~2 sec at 48 kHz).
//   Stops when the buffer is full or the switch is released.
//
// PLAY  (USB MIDI, 4-voice polyphonic):
//   Connect a USB MIDI keyboard or controller (device mode).
//   Base pitch = MIDI note 60 (C4).  Playback loops with a fixed
//   loop point at 50 % of the recorded length (sustain-loop style:
//   the first pass plays the whole sample, then it loops the second half).
//
// X KNOB:  Selects an ADSR envelope preset.
//   Left  (CCW) = short / plucky
//   Right (CW)  = slow / sweepy
//
// Y KNOB:  Delay time (Audio Out 2 only).
//   Left  (CCW) = ~50 ms (slapback)
//   Right (CW)  = ~750 ms (long echo, ~5 repeats, darkening)
//
// OUTPUTS:
//   Audio Out 1 = dry mix
//   Audio Out 2 = reverb wet only
//
// LEDs:
//   0-3  = voice 1-4 active
//   4    = recording in progress
//   5    = sample ready

#include "ComputerCard.h"
#include "MuLawCodec.h"
#include "Delay.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include <math.h>
#include <string.h>


static MuLawCodec codec;
static Delay delay;

// ===========================================================
// ADSR PRESETS  –  Edit these freely to taste!
//
//   attack_ms / decay_ms / release_ms  in milliseconds
//   sustain   0 = silent  …  255 = full level
// ===========================================================
struct ADSRPreset {
    const char* name;
    uint32_t attack_ms;
    uint32_t decay_ms;
    uint8_t  sustain;       // 0-255
    uint32_t release_ms;
};

static const ADSRPreset PRESETS[] = {
//   Name             Att    Dec   Sus   Rel
    { "Pluck",           5,    80,   0,    50 },
    { "Clavinet",        2,    50,   0,    30 },
    { "Marimba",         2,   200,   0,   100 },
    { "Piano",           5,   500,  50,   300 },
    { "Harpsichord",     2,   900,   0,   120 },
    { "Organ",           5,     0, 255,    30 },
    { "Strings",       150,   300, 210,   700 },
    { "Pad",           500,   800, 225,  1100 },
    { "Choir",         350,   500, 230,  1000 },
    { "Slow Sweep",   1200,  2500, 210,  2500 },
};

static const int NUM_PRESETS = (int)(sizeof(PRESETS) / sizeof(PRESETS[0]));

// ===========================================================
// Sample buffer  –  8-bit unsigned, ~2 seconds at 48 kHz
// ===========================================================
static const uint32_t SAMPLE_RATE = 48000;
// Record at half the output rate (24 kHz) for a longer buffer.
// Playback step is halved accordingly so pitch stays correct.
static const int      RECORD_RATE = SAMPLE_RATE / 2;          // 24 kHz
static const int      BUFFER_SIZE = RECORD_RATE * 6;          // 144 000 bytes ≈ 141 KB, 6 s

// Loop crossfade length in samples (~5 ms at 48 kHz).
// When pos enters the last XFADE_LEN samples before the loop end, we
// simultaneously read from the corresponding position at the loop start and
// linearly blend between them.  After the wrap we resume from
// loopStart + XFADE_LEN, so the first post-wrap output sample is adjacent to
// the last crossfade output sample — no discontinuity, no click.
static const int XFADE_LEN = 1024;

// Flash storage layout (end of 2 MB flash):
//   last sector           = metadata  (magic, sampleLen, loopStartPt)
//   preceding 36 sectors  = sample data  (36 × 4096 = 147456 bytes ≥ BUFFER_SIZE)
static const uint32_t FLASH_DATA_SECTORS = ((uint32_t)BUFFER_SIZE + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;  // 36
static const uint32_t FLASH_DATA_OFFSET  = PICO_FLASH_SIZE_BYTES - (FLASH_DATA_SECTORS + 1u) * FLASH_SECTOR_SIZE;
static const uint32_t FLASH_META_OFFSET  = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static const uint32_t FLASH_MAGIC        = 0x56535302u;  // 'V','S','S', version 2 (+ XOR field)

static uint8_t sampleBuffer[BUFFER_SIZE] __attribute__((aligned(4)));
static volatile int  sampleLen  = 0;
static volatile bool hasSample  = false;

static const int BASE_NOTE = 60;   // MIDI C4 = 1 : 1 playback speed

// ===========================================================
// ADSR  –  Q8 fixed-point envelope
//
// envAccum  0 .. ENV_TOP  (= 65535 × 256 = 16 776 960)
//   level = envAccum >> 8        (0 .. 65535, 16-bit)
//   audio = (sample * level) >> 16   (12-bit output)
//
// Rates are in Q8 units / sample (256 = advance 1 level unit / sample).
// This supports times from ~0.02 ms up to ~5 seconds accurately.
// ===========================================================
static const uint32_t ENV_TOP = 65535u * 256u;   // max envAccum value
static const uint32_t ENV_MAX = 65535u;           // max level (= ENV_TOP >> 8)

// Rate to sweep from 0 to ENV_MAX in `ms` milliseconds
static uint32_t adsrRate(uint32_t ms)
{
    if (ms == 0) return ENV_TOP;          // instant
    uint32_t samples = ms * SAMPLE_RATE / 1000u;
    uint32_t rate    = ENV_TOP / samples;
    return rate < 1u ? 1u : rate;
}

// Rate to decay over a partial swing (= ENV_MAX - sustainLevel) in `ms` ms
static uint32_t decayRateFn(uint32_t ms, uint32_t swing)
{
    if (swing == 0) return ENV_TOP;       // nothing to decay  → instant
    if (ms    == 0) return swing * 256u;  // instant
    uint32_t samples = ms * SAMPLE_RATE / 1000u;
    uint32_t rate    = (swing * 256u) / samples;
    return rate < 1u ? 1u : rate;
}

// ===========================================================
// Polyphonic voice
// ===========================================================
static const int NUM_VOICES = 6;

enum VoiceState : uint8_t { VS_IDLE, VS_ATTACK, VS_DECAY, VS_SUSTAIN, VS_RELEASE };

struct Voice {
    volatile bool        active;
    volatile uint8_t     note;
    volatile uint32_t    phase;        // Q8 playback position (phase >> 8 = sample index)
    volatile uint32_t    step;         // Q8 playback step per sample (256 = 1 sample/sample)
    volatile uint32_t    envAccum;     // Q8 envelope accumulator  (0 .. ENV_TOP)
    volatile uint32_t    attackRate;
    volatile uint32_t    decayRate;
    volatile uint32_t    sustainAccum; // sustainLevel × 256
    volatile uint32_t    releaseRate;
    volatile VoiceState  state;
    volatile uint32_t    age;          // voice-stealing: larger = older

    Voice()
        : active(false), note(0), phase(0), step(256u),
          envAccum(0), attackRate(1u), decayRate(1u),
          sustainAccum(0), releaseRate(1u),
          state(VS_IDLE), age(0) {}
};

static Voice   voices[NUM_VOICES];
static volatile uint32_t voiceAgeCtr = 0;

// ===========================================================
// MIDI message parser
// ===========================================================
struct MIDIMsg {
    enum Cmd { Unknown = 0, NoteOn = 0x90, NoteOff = 0x80 };
    Cmd     cmd;
    uint8_t note, vel;

    explicit MIDIMsg(uint8_t* p) : cmd(Unknown), note(0), vel(0) {
        switch (p[0] & 0xF0) {
        case 0x90: cmd = NoteOn;  note = p[1]; vel = p[2]; break;
        case 0x80: cmd = NoteOff; note = p[1]; vel = p[2]; break;
        default: break;
        }
    }
};

// ===========================================================
// VSS card
// ===========================================================
class VSS : public ComputerCard
{
public:
    VSS() : recording(false), writeHead(0), recordDecimate(0), presetIdx(0),
            prevSwitchDown(true), prevSwitchUp(false),
            loopStartPt(0), isUSBMIDIHost(false), savedToFlash(false),
            pendingDelayCapture(false), delayKnobY(0),
            pendingCVNoteOn(false), pendingCVNote(0), gateState(false),
            flashPending(false)
    {
        loadFromFlash();                      // restore last sample before audio starts
    }

    // ---- Flash load/save -----------------------------------------------

    void loadFromFlash()
    {
        const uint8_t* meta = (const uint8_t*)(XIP_BASE + FLASH_META_OFFSET);

        uint32_t magic; memcpy(&magic, meta, 4);
        if (magic != FLASH_MAGIC) return;

        int32_t len, lsp;
        memcpy(&len, meta + 4, 4);
        memcpy(&lsp, meta + 8, 4);
        if (len <= 0 || len > BUFFER_SIZE) return;

        memcpy(sampleBuffer, (const uint8_t*)(XIP_BASE + FLASH_DATA_OFFSET), (size_t)len);

        uint8_t storedXor = 0;
        memcpy(&storedXor, meta + 12, 1);
        uint8_t computedXor = 0;
        for (int32_t i = 0; i < len; i++) computedXor ^= sampleBuffer[i];
        if (computedXor != storedXor) return;   // data corrupted

        sampleLen    = len;
        loopStartPt  = lsp;
        hasSample    = true;
        savedToFlash = true;
    }

    void saveToFlash()
    {
        int len = sampleLen;

        // Pause core 1 FIRST so we own the LEDs cleanly for all diagnostics.
        // Core 1 called multicore_lockout_victim_init() and will park within ~21 µs.
        multicore_lockout_start_blocking();

        // 5 rapid blinks = saveToFlash entered (ProcessSample paused, clean visuals).
        for (int b = 0; b < 5; b++) {
            for (int i = 0; i < 6; i++) LedOn(i, true);  sleep_ms(200);
            for (int i = 0; i < 6; i++) LedOn(i, false); sleep_ms(200);
        }

        if (len <= 0 || !hasSample) {
            // 3 slow blinks = entered but nothing to save (len/hasSample invalid)
            for (int b = 0; b < 3; b++) {
                LedOn(0, true); sleep_ms(500); LedOn(0, false); sleep_ms(500);
            }
            multicore_lockout_end_blocking();
            flashPending = false;
            return;
        }

        // Prepare pages before disabling interrupts (memcpy/memset are safe here).
        static uint8_t metaPage[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
        memset(metaPage, 0, FLASH_PAGE_SIZE);
        uint32_t magic = FLASH_MAGIC;
        int32_t  iLen  = (int32_t)len;
        int32_t  iLsp  = (int32_t)loopStartPt;
        uint8_t  xorChk = 0;
        for (int i = 0; i < len; i++) xorChk ^= sampleBuffer[i];
        memcpy(metaPage,      &magic,  4);
        memcpy(metaPage + 4,  &iLen,   4);
        memcpy(metaPage + 8,  &iLsp,   4);
        memcpy(metaPage + 12, &xorChk, 1);

        static uint8_t lastPage[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
        uint32_t fullBytes = ((uint32_t)len / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
        uint32_t remaining = (uint32_t)len - fullBytes;
        if (remaining > 0) {
            memset(lastPage, 0, FLASH_PAGE_SIZE);
            memcpy(lastPage, sampleBuffer + fullBytes, remaining);
        }
        sleep_ms(500);
        // All 6 LEDs = about to write flash
        for (int i = 0; i < 6; i++) LedOn(i, true);

        uint32_t ints = save_and_disable_interrupts();

        flash_range_erase(FLASH_DATA_OFFSET, (FLASH_DATA_SECTORS + 1u) * FLASH_SECTOR_SIZE);
        LedOn(5, false); // 5 LEDs = erase done, programming data

        if (fullBytes > 0)
            flash_range_program(FLASH_DATA_OFFSET, sampleBuffer, fullBytes);
        if (remaining > 0)
            flash_range_program(FLASH_DATA_OFFSET + fullBytes, lastPage, FLASH_PAGE_SIZE);
        LedOn(4, false); // 4 LEDs = data written, programming metadata

        flash_range_program(FLASH_META_OFFSET, metaPage, FLASH_PAGE_SIZE);
        LedOn(3, false); // 3 LEDs = metadata written

        restore_interrupts(ints);
        // Core 1 still paused — verify and display result while we own the LEDs.

        const uint8_t* fmeta = (const uint8_t*)(XIP_BASE + FLASH_META_OFFSET);
        const uint8_t* fdata = (const uint8_t*)(XIP_BASE + FLASH_DATA_OFFSET);

        // Verify magic
        uint32_t flash_magic_r = 0;
        memcpy(&flash_magic_r, fmeta, 4);
        bool magicOk = (flash_magic_r == FLASH_MAGIC);

        // Verify len stored in meta
        int32_t flash_len_r = 0;
        memcpy(&flash_len_r, fmeta + 4, 4);
        bool lenOk = (flash_len_r == (int32_t)len);

        // Verify loopStartPt stored in meta
        int32_t flash_lsp_r = 0;
        memcpy(&flash_lsp_r, fmeta + 8, 4);
        bool lspOk = (flash_lsp_r == (int32_t)loopStartPt);

        // Full-buffer XOR checksum: catches any data corruption, no false positives
        // from silence (0xFF matches erased flash) unlike spot checks.
        uint8_t sram_xor = 0, flash_xor = 0;
        for (int i = 0; i < len; i++) {
            sram_xor  ^= sampleBuffer[i];
            flash_xor ^= fdata[i];
        }
        bool xorOk = (sram_xor == flash_xor);

        // 4 blinks (ProcessSample still paused):
        //   LED 5 = magic   LED 4 = len   LED 3 = loopStart   LED 2 = full XOR
        //   LEDs 1,0 off (spare)
        for (int b = 0; b < 4; b++) {
            LedOn(5, magicOk); LedOn(4, lenOk); LedOn(3, lspOk); LedOn(2, xorOk);
            LedOn(1, false); LedOn(0, false);
            sleep_ms(600);
            for (int i = 0; i < 6; i++) LedOn(i, false);
            sleep_ms(200);
        }

        bool ok = magicOk && lenOk && lspOk && xorOk;
        multicore_lockout_end_blocking();
        flashPending = false;
        savedToFlash = ok;
    }

    // Core 1 entry: audio processing.  Must call victim_init so core 0 can
    // pause us safely during flash writes.
    // Uses s_instance instead of ThisPtr() because thisptr isn't set until
    // Run() itself is entered — calling ThisPtr() before Run() returns null.
    static void audioEntry()
    {
        multicore_lockout_victim_init();
        s_instance->Run();   // Run() sets ComputerCard::thisptr as its first act
    }

    // ---- MIDI core (core 0 main thread) ----------------------------------------

    void MIDICore()
    {
        // Wait for USB power state to settle, then detect host vs device
        sleep_us(150000);
        USBPowerState_t pwr = USBPowerState();
        isUSBMIDIHost = (pwr == DFP);   // DFP = keyboard plugged in → we are host
                                         // UFP / Unsupported → plugged into laptop → device

        if (isUSBMIDIHost)
            tuh_init(TUH_OPT_RHPORT);
        else
            tud_init(TUD_OPT_RHPORT);

        uint8_t buf[64];
        while (true) {
            // Core 0 may request a flash write; park here (in RAM) until done.
            // ProcessSample() (on core 1) sets this flag when the user requests save.
            // We do the actual flash write here on core 0's main thread, where
            // multicore_lockout_start_blocking() is safe to call.
            if (flashPending)
                saveToFlash();

            if (isUSBMIDIHost) {
                tuh_task();
                // MIDI data arrives via tuh_midi_rx_cb callback below
            } else {
                tud_task();
                while (tud_midi_available()) {
                    uint32_t n = tud_midi_stream_read(buf, sizeof(buf));
                    uint8_t* p = buf;
                    while (n > 0) {
                        MIDIMsg m(p);
                        handleMIDI(m);
                        do { ++p; --n; } while (n > 0 && !(*p & 0x80));
                    }
                }
            }
        }
    }

    // Called from tuh_midi_rx_cb (free function below) when in host mode
    static void handleRx(uint8_t dev_addr, uint32_t num_packets)
    {
        if (midi_dev_addr == 0 || dev_addr != midi_dev_addr || num_packets == 0)
            return;

        uint8_t cable;
        uint8_t buf[64];
        while (true) {
            uint32_t n = tuh_midi_stream_read(dev_addr, &cable, buf, sizeof(buf));
            if (n == 0) return;
            uint8_t* p = buf;
            while (n > 0) {
                MIDIMsg m(p);
                static_cast<VSS*>(ThisPtr())->handleMIDI(m);
                do { ++p; --n; } while (n > 0 && !(*p & 0x80));
            }
        }
    }

    void handleMIDI(const MIDIMsg& m)
    {
        if (m.cmd == MIDIMsg::NoteOn && m.vel > 0)
            noteOn(m.note);
        else if (m.cmd == MIDIMsg::NoteOff ||
                 (m.cmd == MIDIMsg::NoteOn && m.vel == 0))
            noteOff(m.note);
    }

    void noteOn(uint8_t note)
    {
        int len = sampleLen;
        if (len <= 0) return;

        const ADSRPreset& p = PRESETS[presetIdx];

        // Q8 playback step: base is 128 (= 256/2) because recording is at
        // half the output rate, so we advance half a sample per output tick.
        float semis = (float)((int)note - BASE_NOTE);
        uint32_t step = (uint32_t)(128.0f * powf(2.0f, semis / 12.0f));
        if (step <   4u) step =   4u;    // floor ~  5 octaves below base
        if (step > 4096u) step = 4096u;  // cap   ~  5 octaves above base

        // ADSR parameters
        uint32_t susLev   = (uint32_t)p.sustain * 257u;      // 0 .. 65535
        uint32_t susAccum = susLev * 256u;                    // Q8 sustain target
        uint32_t swing    = ENV_MAX - susLev;                 // decay swing
        uint32_t aRate    = adsrRate(p.attack_ms);
        uint32_t dRate    = decayRateFn(p.decay_ms, swing);
        uint32_t rRate    = adsrRate(p.release_ms);

        // Voice selection priority:
        //   1. Any free (inactive / idle) voice          → envelope starts at 0
        //   2. Same-note voice in release phase           → envelope starts from current level
        //   3. Same-note voice in any other active state  → envelope starts at 0
        //   4. Oldest active voice (voice steal)          → envelope starts from current
        //      level if it happens to be in release, else 0
        int freeSlot      = -1;
        int sameReleasing = -1;
        int sameActive    = -1;
        uint32_t oldestAge = 0;
        int      oldest    = 0;

        for (int i = 0; i < NUM_VOICES; i++) {
            if (!voices[i].active || voices[i].state == VS_IDLE) {
                if (freeSlot < 0) freeSlot = i;
                continue;
            }
            if (voices[i].note == note) {
                if (voices[i].state == VS_RELEASE) sameReleasing = i;
                else                               sameActive    = i;
            }
            if (voices[i].age > oldestAge) { oldestAge = voices[i].age; oldest = i; }
        }

        int  slot            = -1;
        bool startFromCurrent = false;
        if      (freeSlot      >= 0) { slot = freeSlot; }
        else if (sameReleasing >= 0) { slot = sameReleasing; startFromCurrent = true; }
        else if (sameActive    >= 0) { slot = sameActive; }
        else {
            slot = oldest;
            startFromCurrent = (voices[slot].state == VS_RELEASE);
        }

        uint32_t startEnv = startFromCurrent ? voices[slot].envAccum : 0u;

        // Write all parameters first; set `active` last so the audio
        // core sees a fully-initialised voice when it picks it up.
        voices[slot].note         = note;
        voices[slot].phase        = 0;
        voices[slot].step         = step;
        voices[slot].sustainAccum = susAccum;
        voices[slot].attackRate   = aRate;
        voices[slot].decayRate    = dRate;
        voices[slot].releaseRate  = rRate;
        voices[slot].envAccum     = startEnv;
        voices[slot].state        = VS_ATTACK;
        voices[slot].age          = ++voiceAgeCtr;
        voices[slot].active       = true;   // ← set last
        pendingDelayCapture = true;

        // CV/gate: trigger only if this note is the lowest currently held
        uint8_t lowest = 127;
        for (int i = 0; i < NUM_VOICES; i++) {
            if (voices[i].active &&
                voices[i].state != VS_RELEASE && voices[i].state != VS_IDLE &&
                voices[i].note < lowest)
                lowest = voices[i].note;
        }
        if (note <= lowest) {
            pendingCVNote   = note;
            pendingCVNoteOn = true;
        }
    }

    void noteOff(uint8_t note)
    {
        for (int i = 0; i < NUM_VOICES; i++) {
            if (voices[i].active && voices[i].note == note &&
                voices[i].state != VS_RELEASE && voices[i].state != VS_IDLE)
            {
                voices[i].state = VS_RELEASE;
            }
        }

    }

    // Find the nearest upward zero crossing to sampleLen/2 and store it in
    // loopStartPt.  Called once after each recording ends — blocking for a
    // few hundred µs is inaudible because all voices are silent at that point.
    void computeLoopStart()
    {
        int len     = sampleLen;
        int nominal = len / 2;
        int lo = nominal - XFADE_LEN;  if (lo < 1)    lo = 1;
        int hi = nominal + XFADE_LEN;  if (hi >= len) hi = len - 1;

        int best     = nominal;
        int bestDist = XFADE_LEN + 1;

        for (int i = lo; i < hi; i++) {
            if (codec.decodeSample(sampleBuffer[i - 1]) <= 0 &&
                codec.decodeSample(sampleBuffer[i])     >  0)
            {
                int dist = i - nominal;  if (dist < 0) dist = -dist;
                if (dist < bestDist) { bestDist = dist; best = i; }
            }
        }
        loopStartPt = best;
    }

    // ---- Audio core (core 0) ----------------------------------------

    virtual void ProcessSample() override
    {
        // --- Recording (Z switch Down = momentary press) ---
        Switch sw = SwitchVal();
        bool switchDown = (sw == Switch::Down);
        bool switchJustPressed = switchDown && !prevSwitchDown;
        prevSwitchDown = switchDown;

        if (switchDown) {
            if (switchJustPressed && !recording) {
                // New recording: silence all voices and reset buffer
                for (int i = 0; i < NUM_VOICES; i++) {
                    voices[i].active = false;
                    voices[i].state  = VS_IDLE;
                }
                writeHead    = 0;
                sampleLen    = 0;
                hasSample    = false;
                loopStartPt  = 0;
                savedToFlash = false;   // new recording invalidates any previous save
                recording    = true;
            }
            if (writeHead < BUFFER_SIZE) {
                // Decimate to 24 kHz: write one sample every two output ticks
                if (recordDecimate == 0)
                    sampleBuffer[writeHead++] = codec.encodeSample(AudioIn1());
                recordDecimate ^= 1;
            } else if (recording) {
                // Buffer full: finish recording (guard prevents re-entry each tick)
                sampleLen = writeHead;
                hasSample = true;
                recording = false;
                computeLoopStart();
            }
        } else {
            if (recording) {
                // Switch released: finish recording
                sampleLen = writeHead;
                hasSample = (writeHead > 0);
                recording = false;
                if (hasSample) computeLoopStart();
            }
        }

        // --- Save to flash (switch Up, once per new recording) ---
        bool switchUp    = (sw == Switch::Up);
        bool switchJustUp = switchUp && !prevSwitchUp;
        prevSwitchUp = switchUp;
        if (switchJustUp && hasSample && !savedToFlash)
            flashPending = true;   // core 0 USB loop will call saveToFlash()

        // --- Preset selection from X knob ---
        presetIdx = (KnobVal(Knob::X) * NUM_PRESETS) >> 12;
        if (presetIdx >= NUM_PRESETS) presetIdx = NUM_PRESETS - 1;

        // --- Voice processing ---
        int len = sampleLen;
        int32_t mix = 0;

        if (len > 0) {
            int loopStart = loopStartPt > 0 ? loopStartPt : len >> 1;
            int loopLen   = len - loopStart;

            // Runtime crossfade: is the loop long enough to support it?
            // Need loopLen > 2*XFADE_LEN so the crossfade head and tail don't overlap.
            bool canXfade     = (loopLen > XFADE_LEN * 2);
            int  xfZoneStart  = len - XFADE_LEN;   // pos where crossfade begins
            // After wrap we jump here, past the region we just faded in
            int  wrapTarget   = canXfade ? loopStart + XFADE_LEN : loopStart;
            int  wrapLoopLen  = len - wrapTarget;

            for (int i = 0; i < NUM_VOICES; i++) {
                if (!voices[i].active) continue;

                // Advance playback position (Q8)
                voices[i].phase += voices[i].step;
                int pos = (int)(voices[i].phase >> 8);

                // Loop wrap: land at wrapTarget so we're past the faded-in head
                if (pos >= len) {
                    int over = pos - len;
                    pos = (wrapLoopLen > 0) ? wrapTarget + (over % wrapLoopLen) : wrapTarget;
                    voices[i].phase = (uint32_t)pos << 8;
                }

                // 8-bit unsigned → 12-bit signed sample, with runtime loop crossfade.
                // When pos is in [xfZoneStart, len): simultaneously read from pos (tail,
                // fading out) and loopStart+xOff (head, fading in).  After the wrap we
                // resume at loopStart+XFADE_LEN, adjacent to the last faded-in sample,
                // so there is no discontinuity.
                int16_t smp;
                if (canXfade && pos >= xfZoneStart) {
                    int xOff = pos - xfZoneStart;           // 0 .. XFADE_LEN-1
                    int t    = (xOff * 256) / XFADE_LEN;   // 0..255 linear
                    // Smoothstep: 3t² - 2t³  (avoids the level dip of a linear crossfade)
                    int t2   = (t * t) >> 8;                // t² normalised to 0..255
                    int t3   = (t2 * t) >> 8;               // t³ normalised to 0..255
                    int ts   = 3 * t2 - 2 * t3;            // smoothstep, 0..256
                    int s1   = codec.decodeSample(sampleBuffer[pos]);
                    int s2   = codec.decodeSample(sampleBuffer[loopStart + xOff]);
                    smp = (int16_t)(s1 + (((s2 - s1) * ts) >> 8));
                } else {
                    smp = codec.decodeSample(sampleBuffer[pos]);
                }

                // ADSR envelope update
                switch (voices[i].state) {
                case VS_ATTACK:
                    voices[i].envAccum += voices[i].attackRate;
                    if (voices[i].envAccum >= ENV_TOP) {
                        voices[i].envAccum = ENV_TOP;
                        voices[i].state    = VS_DECAY;
                    }
                    break;

                case VS_DECAY:
                    if (voices[i].envAccum > voices[i].sustainAccum &&
                        voices[i].envAccum - voices[i].sustainAccum > voices[i].decayRate)
                    {
                        voices[i].envAccum -= voices[i].decayRate;
                    } else {
                        voices[i].envAccum = voices[i].sustainAccum;
                        voices[i].state    = VS_SUSTAIN;
                    }
                    break;

                case VS_SUSTAIN:
                    voices[i].envAccum = voices[i].sustainAccum;
                    break;

                case VS_RELEASE:
                    if (voices[i].envAccum > voices[i].releaseRate) {
                        voices[i].envAccum -= voices[i].releaseRate;
                    } else {
                        voices[i].envAccum = 0;
                        voices[i].active   = false;
                        voices[i].state    = VS_IDLE;
                    }
                    break;

                default:
                    break;
                }

                // Apply envelope:  level = envAccum >> 8  (0-65535)
                // Output = (12-bit sample × 16-bit level) >> 16  = 12-bit
                uint32_t level = voices[i].envAccum >> 8;
                mix += ((int32_t)smp * (int32_t)level) >> 16;
            }

            // Divide by 8 (next power of 2 above 6 voices) to stay in 12-bit range
            mix >>= 3;
        }

        // Hard-clip to ±2047
        if (mix >  2047) mix =  2047;
        if (mix < -2048) mix = -2048;

        AudioOut1((int16_t)mix);

        // CV Out 1 / Gate Out 1:
        //   - CV updates only on note-on, set to the lowest held note at that moment
        //   - Gate goes high on that same note-on, stays high until all voices
        //     have fully finished their release envelopes, then goes low and waits
        //     for the next note-on.  Note-offs never change CV or cause retriggering.
        if (pendingCVNoteOn) {
            CVOut1MIDINote(pendingCVNote);
            if (!gateState) { PulseOut1(true);  gateState = true; }
            pendingCVNoteOn = false;
        } else if (gateState) {
            bool anyActive = false;
            for (int i = 0; i < NUM_VOICES; i++)
                if (voices[i].active) { anyActive = true; break; }
            if (!anyActive) { PulseOut1(false); gateState = false; }
        }

        // Audio Out 2: delay wet only.  Y knob sampled on each note-on.
        if (pendingDelayCapture) {
            delayKnobY          = KnobVal(Knob::Y);
            pendingDelayCapture = false;
        }
        AudioOut2(delay.process((int16_t)mix, delayKnobY));

        // --- LEDs ---
        // All 6 LEDs = voice activity.  During recording all voices are silenced
        // so we light all LEDs to show recording is in progress.
        static uint32_t sampleCount = 0;
        ++sampleCount;

        if (recording) {
            for (int i = 0; i < 6; i++) LedOn(i, true);
        } else {
            for (int i = 0; i < NUM_VOICES; i++) LedOn(i, voices[i].active);
            if (flashPending) LedOn(5, (sampleCount >> 12) & 1u);  // blink while save pending
        }
    }

private:
    volatile bool flashPending;   // set by core1 ProcessSample; serviced by core0 MIDICore

    bool         recording;
    bool         prevSwitchDown;
    bool         prevSwitchUp;
    bool         savedToFlash;
    bool         isUSBMIDIHost;
    int          writeHead;
    int          recordDecimate;  // toggles 0/1 to halve the recording sample rate
    volatile int  presetIdx;
    volatile int  loopStartPt;     // upward zero-crossing near 50% of buffer
    volatile bool    pendingDelayCapture;
    int              delayKnobY;
    volatile bool    pendingCVNoteOn;
    volatile uint8_t pendingCVNote;
    bool             gateState;        // last value written to PulseOut1

public:
    static uint8_t midi_dev_addr;
    static uint8_t device_connected;
    static VSS*    s_instance;   // set by main() before multicore_launch_core1
};

uint8_t VSS::midi_dev_addr    = 0;
uint8_t VSS::device_connected = 0;
VSS*    VSS::s_instance       = nullptr;

// ---- TinyUSB MIDI host callbacks (required by usb_midi_host driver) --------

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                       uint8_t num_cables_rx, uint16_t num_cables_tx)
{
    (void)in_ep; (void)out_ep; (void)num_cables_rx; (void)num_cables_tx;
    if (VSS::midi_dev_addr == 0) {
        VSS::midi_dev_addr    = dev_addr;
        VSS::device_connected = 1;
    }
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)instance;
    if (dev_addr == VSS::midi_dev_addr) {
        VSS::midi_dev_addr    = 0;
        VSS::device_connected = 0;
    }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    VSS::handleRx(dev_addr, num_packets);
}

void tuh_midi_tx_cb(uint8_t dev_addr) { (void)dev_addr; }

// ----------------------------------------------------------------------------

int main()
{
    set_sys_clock_khz(200000, true);   // 200 MHz – comfortable headroom for 6-voice processing
    VSS vss;
    VSS::s_instance = &vss;   // audioEntry() needs this before Run() sets thisptr
    // Audio on core 1 (lockout victim); USB + flash save on core 0 (main thread).
    // This matches the reverb card architecture: flash ops happen from the non-ISR
    // core so multicore_lockout_start_blocking() is safe to call.
    multicore_launch_core1(VSS::audioEntry);
    vss.MIDICore();   // USB loop runs here on core 0; never returns
}
