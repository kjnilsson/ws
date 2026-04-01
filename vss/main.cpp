// VSS - Yamaha VSS-30 inspired 8-bit sampler
// for Music Thing Workshop System Computer
//
// RECORD  (hold Z switch DOWN):
//   Samples Audio In 1 into an 8-bit mono buffer (~6 sec at 24 kHz).
//   Stops when the buffer is full or the switch is released.
//
// PLAY  (USB MIDI, 6-voice polyphonic):
//   Connect a USB MIDI keyboard or controller (device mode).
//   Base pitch = MIDI note 60 (C4).  Playback loops with a fixed
//   loop point at 2/3 of the recorded length (sustain-loop style:
//   the first pass plays the whole sample, then it loops the last third).
//
// MAIN KNOB:  Selects an ADSR envelope preset.
//   Left  (CCW) = short / plucky
//   Right (CW)  = slow / sweepy
//
// X KNOB:  Selects sample bank (0-5).
//
// Y KNOB:  Delay time (Audio Out 2 only).
//   Left  (CCW) = ~50 ms (slapback)
//   Right (CW)  = ~750 ms (long echo, ~5 repeats, darkening)
//
// OUTPUTS:
//   Audio Out 1 = dry mix
//   Audio Out 2 = delay wet only
//
// LEDs:
//   0-5  = voice activity / bank indicator

#include "ComputerCard.h"
#include "MuLawCodec.h"
#include "../Tuner.h"
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
    uint32_t attack_ms;
    uint32_t decay_ms;
    uint8_t  sustain;       // 0-255
    uint32_t release_ms;
};

static const ADSRPreset PRESETS[] = {
//              Att    Dec   Sus   Rel
    /*Pluck*/  {   5,    80,   0,    50 },
    /*Clav*/   {   2,    50,   0,    30 },
    /*Marimba*/{   2,   200,   0,   100 },
    /*Piano*/  {   5,   500,  50,   300 },
    /*Harpsi*/ {   2,   900,   0,   120 },
    /*Organ*/  {   5,     0, 255,    30 },
    /*Strings*/{  150,  300, 210,   700 },
    /*Pad*/    {  500,  800, 225,  1500 },
    /*Choir*/  {  350,  500, 230,  1800 },
    /*Sweep*/  { 1200, 2500, 210,  4000 },
    /*Rhubarb*/{ 800,  1000, 245,  7000 },
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

// Loop crossfade length in samples (~170 ms at base pitch).
// When pos enters the last XFADE_LEN samples before the loop end, we
// simultaneously read from the corresponding position at the loop start and
// linearly blend between them.  After the wrap we resume from
// loopStart + XFADE_LEN, so the first post-wrap output sample is adjacent to
// the last crossfade output sample — no discontinuity, no click.
static const int XFADE_LEN = 4096;  // must be a power of 2
static_assert((XFADE_LEN & (XFADE_LEN - 1)) == 0, "XFADE_LEN must be a power of 2");

// Flash storage layout (end of 2 MB flash), 6 banks stacked from the end:
//   each bank = 36 data sectors + 1 meta sector = 37 sectors = 148 KB
//   bank 0 is closest to end of flash (same location as the original single-bank layout)
static const uint32_t FLASH_DATA_SECTORS = ((uint32_t)BUFFER_SIZE + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;  // 36
static const uint32_t FLASH_BANK_SECTORS = FLASH_DATA_SECTORS + 1u;   // 37 per bank
static const int      FLASH_NUM_BANKS    = 6;
static const uint32_t FLASH_MAGIC        = 0x56535302u;  // 'V','S','S', version 2 (+ XOR field)
static const uint32_t FLASH_CONFIG_MAGIC = 0x56535343u;  // 'V','S','S','C'

static uint32_t flashDataOffset(int bank) {
    return PICO_FLASH_SIZE_BYTES - (uint32_t)(bank + 1) * FLASH_BANK_SECTORS * FLASH_SECTOR_SIZE;
}
static uint32_t flashMetaOffset(int bank) {
    return flashDataOffset(bank) + FLASH_DATA_SECTORS * FLASH_SECTOR_SIZE;
}
// Config sector: one sector immediately before (lower address than) all sample banks.
// Format: [0-3] magic  [4] midiChannel (0=omni,1-16)  [5-255] reserved (0xFF)
static uint32_t flashConfigOffset() {
    return PICO_FLASH_SIZE_BYTES
           - (uint32_t)(FLASH_NUM_BANKS * FLASH_BANK_SECTORS + 1u) * FLASH_SECTOR_SIZE;
}

static uint8_t sampleBuffer[BUFFER_SIZE] __attribute__((aligned(4)));
static volatile int  sampleLen  = 0;

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

// Exponential release shift: each sample envAccum -= envAccum >> shift.
// τ (time constant) = 2^shift samples.  We pick shift so that τ ≈ ms/5,
// meaning the envelope reaches ~1 % (-40 dB) in roughly `ms` milliseconds.
static uint8_t releaseShiftFn(uint32_t ms)
{
    if (ms == 0) return 0;                      // instant: drain in one sample
    uint32_t tau = ms * SAMPLE_RATE / 5000u;    // τ in samples
    if (tau < 1u) tau = 1u;
    uint8_t shift = 0;
    while (tau > 1u && shift < 20u) { tau >>= 1; ++shift; }
    return shift;
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
    volatile uint8_t     releaseShift; // exponential release: envAccum -= envAccum >> releaseShift
    volatile VoiceState  state;
    volatile uint32_t    age;          // voice-stealing: larger = older

    Voice()
        : active(false), note(0), phase(0), step(256u),
          envAccum(0), attackRate(1u), decayRate(1u),
          sustainAccum(0), releaseShift(8u),
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

    uint8_t channel;  // 1-16

    explicit MIDIMsg(uint8_t* p) : cmd(Unknown), note(0), vel(0),
                                   channel((p[0] & 0x0F) + 1) {
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
            pendingCVNoteOn(false), pendingCVNote(0), gateState(false), cvGateNote(0),
            flashPending(false), bankIdx(0), pendingBank(0),
            arpIdx(-1), arpGateTimer(0), arpIntTimer(0),
            midiChannel(0), inConfigMode(false), configMidiChan(0),
            presetFlashTimer(0), tunerCounter(0)
    {
    }

    // ---- Flash load/save -----------------------------------------------

    void loadFromFlash(int bank)
    {
        // Clear sample state first — if the load fails the bank is treated as empty.
        sampleLen    = 0;
        savedToFlash = false;

        const uint8_t* meta = (const uint8_t*)(XIP_BASE + flashMetaOffset(bank));

        uint32_t magic; memcpy(&magic, meta, 4);
        if (magic != FLASH_MAGIC) return;

        int32_t len, lsp;
        memcpy(&len, meta + 4, 4);
        memcpy(&lsp, meta + 8, 4);
        if (len <= 0 || len > BUFFER_SIZE) return;

        memcpy(sampleBuffer, (const uint8_t*)(XIP_BASE + flashDataOffset(bank)), (size_t)len);

        uint8_t storedXor = 0;
        memcpy(&storedXor, meta + 12, 1);
        uint8_t computedXor = 0;
        for (int32_t i = 0; i < len; i++) computedXor ^= sampleBuffer[i];
        if (computedXor != storedXor) return;   // data corrupted

        sampleLen    = len;
        loopStartPt  = lsp;
        savedToFlash = true;
    }

    // ---- Config load/save/mode -----------------------------------------

    void loadConfig()
    {
        const uint8_t* p = (const uint8_t*)(XIP_BASE + flashConfigOffset());
        uint32_t magic; memcpy(&magic, p, 4);
        if (magic != FLASH_CONFIG_MAGIC) { midiChannel = 0; return; }
        midiChannel = p[4];
        if (midiChannel > 16) midiChannel = 0;
    }

    void saveConfig()
    {
        static uint8_t cfgPage[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
        memset(cfgPage, 0xFF, FLASH_PAGE_SIZE);
        uint32_t magic = FLASH_CONFIG_MAGIC;
        memcpy(cfgPage, &magic, 4);
        cfgPage[4] = midiChannel;

        multicore_lockout_start_blocking();
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(flashConfigOffset(), FLASH_SECTOR_SIZE);
        flash_range_program(flashConfigOffset(), cfgPage, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        multicore_lockout_end_blocking();
    }

    // Called from MIDICore() when the Z switch is held at boot.
    // Core 1 is already running; inConfigMode tells ProcessSample to
    // display the channel in binary on LEDs 0-4 (LED 5 = slow blink).
    // Exits when the switch is released, then saves config to flash.
    void runConfigMode()
    {
        inConfigMode = true;
        while (SwitchVal() == Switch::Down) {
            // Map Main knob 0-4095 to channel 0-16 (0 = omni)
            configMidiChan = (uint8_t)((KnobVal(Knob::Main) * 17) >> 12);
            sleep_ms(10);
        }
        midiChannel    = configMidiChan;
        inConfigMode   = false;
        saveConfig();
    }

    void saveToFlash()
    {
        // Pause core 1 first so we read consistently and own the LEDs.
        multicore_lockout_start_blocking();
        int len  = sampleLen;
        int bank = pendingBank;

        // 5 rapid blinks = save entered.
        for (int b = 0; b < 5; b++) {
            for (int i = 0; i < 6; i++) LedOn(i, true);  sleep_ms(100);
            for (int i = 0; i < 6; i++) LedOn(i, false); sleep_ms(100);
        }

        if (len <= 0) {
            for (int b = 0; b < 3; b++) {
                LedOn(0, true); sleep_ms(250); LedOn(0, false); sleep_ms(250);
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
        sleep_ms(300);
        for (int i = 0; i < 6; i++) LedOn(i, true);  // all 6 = about to write

        uint32_t dataOff = flashDataOffset(bank);
        uint32_t metaOff = flashMetaOffset(bank);

        uint32_t ints = save_and_disable_interrupts();

        flash_range_erase(dataOff, FLASH_BANK_SECTORS * FLASH_SECTOR_SIZE);
        LedOn(5, false); // 5 LEDs = erase done, programming data

        if (fullBytes > 0)
            flash_range_program(dataOff, sampleBuffer, fullBytes);
        if (remaining > 0)
            flash_range_program(dataOff + fullBytes, lastPage, FLASH_PAGE_SIZE);
        LedOn(4, false); // 4 LEDs = data written, programming metadata

        flash_range_program(metaOff, metaPage, FLASH_PAGE_SIZE);
        LedOn(3, false); // 3 LEDs = metadata written

        restore_interrupts(ints);

        const uint8_t* fmeta = (const uint8_t*)(XIP_BASE + metaOff);
        const uint8_t* fdata = (const uint8_t*)(XIP_BASE + dataOff);

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
            sleep_ms(300);
            for (int i = 0; i < 6; i++) LedOn(i, false);
            sleep_ms(100);
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
        // Wait for USB power state to settle AND for the knob IIR filter to converge.
        // ComputerCard's filter starts at 0 on boot; it takes ~150 ms at 12 kHz updates
        // to reach the true ADC value.  Reading the X knob too early would select the
        // wrong bank (e.g. knob at bank 1 reads as bank 0 after only 10 ms).
        sleep_us(150000);   // USB settle + knob IIR convergence (~150 ms for IIR to converge from 0)

        // Config mode: hold Z switch down while powering on.
        if (SwitchVal() == Switch::Down)
            runConfigMode();

        // Load persisted config (MIDI channel filter etc.)
        loadConfig();

        // Now read the (fully converged) X knob and load the correct bank.
        multicore_lockout_start_blocking();
        {
            int loadBank = (KnobVal(Knob::X) * FLASH_NUM_BANKS) >> 12;
            if (loadBank >= FLASH_NUM_BANKS) loadBank = FLASH_NUM_BANKS - 1;
            loadFromFlash(loadBank);
            bankIdx = loadBank;   // sync initial value so ProcessSample doesn't reset savedToFlash
        }
        multicore_lockout_end_blocking();
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
                    parseMIDI(buf, n);
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
            static_cast<VSS*>(ThisPtr())->parseMIDI(buf, n);
        }
    }

    void parseMIDI(uint8_t* buf, uint32_t n)
    {
        uint8_t* p = buf;
        while (n >= 3) {
            MIDIMsg m(p);
            handleMIDI(m);
            p += 3; n -= 3;
            while (n > 0 && !(*p & 0x80)) { ++p; --n; }
        }
    }

    void handleMIDI(const MIDIMsg& m)
    {
        // midiChannel 0 = omni (accept all); otherwise filter by channel
        if (midiChannel != 0 && m.channel != midiChannel) return;

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
        uint8_t  rShift   = releaseShiftFn(p.release_ms);

        // Voice selection priority:
        //   1. Same-note voice (any state)       → envelope starts from current level
        //   2. Any free (inactive / idle) voice  → envelope starts at 0
        //   3. Oldest active voice (voice steal) → envelope starts from current level
        int freeSlot = -1;
        int sameNote = -1;
        uint32_t oldestAge = UINT32_MAX;
        int      oldest    = 0;

        for (int i = 0; i < NUM_VOICES; i++) {
            if (!voices[i].active || voices[i].state == VS_IDLE) {
                if (freeSlot < 0) freeSlot = i;
                continue;
            }
            if (voices[i].note == note && sameNote < 0) sameNote = i;
            if (voices[i].age < oldestAge) { oldestAge = voices[i].age; oldest = i; }
        }

        int slot;
        if      (sameNote >= 0) slot = sameNote;
        else if (freeSlot >= 0) slot = freeSlot;
        else                    slot = oldest;

        uint32_t startEnv = (slot != freeSlot) ? voices[slot].envAccum : 0u;

        // Write all parameters first; set `active` last so the audio
        // core sees a fully-initialised voice when it picks it up.
        // For voice re-trigger (stealing an active slot), clear active first
        // so core 1 skips this voice while we update its fields.
        voices[slot].active       = false;
        __dmb();
        voices[slot].note         = note;
        voices[slot].phase        = 0;
        voices[slot].step         = step;
        voices[slot].sustainAccum = susAccum;
        voices[slot].attackRate   = aRate;
        voices[slot].decayRate    = dRate;
        voices[slot].releaseShift = rShift;
        voices[slot].envAccum     = startEnv;
        voices[slot].state        = VS_ATTACK;
        voices[slot].age          = ++voiceAgeCtr;
        __dmb();
        voices[slot].active       = true;
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
        int nominal = (len * 2) / 3;
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
        bool tunerMode = Connected(Input::Audio2);
        if (tunerMode) tunerPd.addSample(AudioIn1());

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
                recording = false;
                computeLoopStart();
            }
        } else {
            if (recording) {
                // Switch released: finish recording
                sampleLen = writeHead;
                recording = false;
                if (sampleLen > 0) computeLoopStart();
            }
        }

        // --- Save to flash (switch Up, once per new recording) ---
        bool switchUp    = (sw == Switch::Up);
        bool switchJustUp = switchUp && !prevSwitchUp;
        prevSwitchUp = switchUp;
        // --- Preset selection (Main knob) and bank selection (X knob) ---
        {
            int newPreset = (KnobVal(Knob::Main) * NUM_PRESETS) >> 12;
            if (newPreset >= NUM_PRESETS) newPreset = NUM_PRESETS - 1;
            if (newPreset != presetIdx) {
                presetIdx       = newPreset;
                presetFlashTimer = PRESET_FLASH_LEN;
            }
        }
        {
            int newBank = (KnobVal(Knob::X) * FLASH_NUM_BANKS) >> 12;
            if (newBank >= FLASH_NUM_BANKS) newBank = FLASH_NUM_BANKS - 1;
            if (newBank != bankIdx) {
                bankIdx = newBank;
            }
        }

        // --- Save to flash ---
        if (switchJustUp && sampleLen > 0 && !savedToFlash) {
            pendingBank  = bankIdx;   // lock in the bank now, not when saveToFlash runs
            flashPending = true;
        }

        // --- Voice processing ---
        int len = sampleLen;
        int32_t mix = 0;

        if (len > 0) {
            int loopStart = loopStartPt > 0 ? loopStartPt : (len * 2) / 3;
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
                    int t    = (xOff << 8) >> 12;          // 0..255 linear  (= xOff*256/4096)
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

                case VS_RELEASE: {
                    uint32_t drop = voices[i].envAccum >> voices[i].releaseShift;
                    if (drop == 0) drop = 1;   // ensure termination at very low levels
                    if (voices[i].envAccum > drop) {
                        voices[i].envAccum -= drop;
                    } else {
                        voices[i].envAccum = 0;
                        voices[i].active   = false;
                        voices[i].state    = VS_IDLE;
                    }
                    break;
                }

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

        // Audio Out 1: passthrough of Audio In 1 in tuner mode; dry voice mix otherwise.
        AudioOut1(tunerMode ? AudioIn1() : (int16_t)mix);

        // CV Out 1 / Gate Out 1:
        //   In tuner mode: CV Out 1 = middle C reference (gate unused).
        //   Otherwise: triggers on the first note-on that is the lowest playing note,
        //   but only when the gate is currently low. Holds the gate high until that
        //   specific note's voice finishes completely, then waits for the next trigger.
        if (tunerMode) {
            CVOut1MIDINote(60);
        } else if (pendingCVNoteOn) {
            if (!gateState) {
                cvGateNote = pendingCVNote;
                CVOut1MIDINote(cvGateNote > 11 ? cvGateNote - 12 : cvGateNote);
                PulseOut1(true);
                gateState = true;
            }
            pendingCVNoteOn = false;
        } else if (gateState) {
            bool noteStillPlaying = false;
            for (int i = 0; i < NUM_VOICES; i++)
                if (voices[i].active && voices[i].note == cvGateNote
                    && voices[i].state != VS_RELEASE)
                    { noteStillPlaying = true; break; }
            if (!noteStillPlaying) { PulseOut1(false); gateState = false; }
        }

        // Audio Out 2: delay wet only (unchanged by tuner mode).
        if (pendingDelayCapture) {
            delayKnobY          = KnobVal(Knob::Y);
            pendingDelayCapture = false;
        }
        AudioOut2(delay.process((int16_t)mix, delayKnobY));

        // --- Arpeggiator (CV Out 2 / Gate Out 2) ---
        // Collect held notes (active, non-releasing) into a sorted list
        uint8_t arpNotes[NUM_VOICES];
        int     arpCount = 0;
        for (int i = 0; i < NUM_VOICES; i++) {
            if (!voices[i].active ||
                voices[i].state == VS_RELEASE ||
                voices[i].state == VS_IDLE) continue;
            uint8_t n = voices[i].note;
            int j = arpCount;
            while (j > 0 && arpNotes[j-1] > n) { arpNotes[j] = arpNotes[j-1]; --j; }
            arpNotes[j] = n;
            ++arpCount;
        }

        if (arpCount == 0) {
            arpIdx = -1;
        } else if (arpIdx >= arpCount) {
            arpIdx %= arpCount;
        }

        // Clock: external rising edge on Pulse In 2 if connected, else delay-synced
        bool arpTick = false;
        if (Connected(Input::Pulse2)) {
            arpTick    = PulseIn2RisingEdge();
            arpIntTimer = 0;
        } else {
            int period = 2400 + ((int32_t)KnobVal(Knob::Y) * 33600) / 4096;
            if (++arpIntTimer >= period) { arpIntTimer = 0; arpTick = true; }
        }

        if (arpTick && arpCount > 0) {
            arpIdx = (arpIdx + 1) % arpCount;
            // 50 % chance of octave-down shift (skipped for notes below C1)
            static uint32_t arpRand = 0xdeadbeef;
            arpRand ^= arpRand << 13;
            arpRand ^= arpRand >> 17;
            arpRand ^= arpRand << 5;
            uint8_t arpNote = arpNotes[arpIdx];
            if ((arpRand & 1u) && arpNote >= 24u) arpNote -= 12u;
            CVOut2MIDINote(arpNote);
            PulseOut2(true);
            arpGateTimer = ARP_GATE_LEN;
        }
        if (arpGateTimer > 0 && --arpGateTimer == 0) PulseOut2(false);
        if (arpCount == 0)                           PulseOut2(false);

        // --- LEDs ---
        // All 6 LEDs = voice activity.  During recording all voices are silenced
        // so we light all LEDs to show recording is in progress.
        static uint32_t sampleCount = 0;
        ++sampleCount;

        if (inConfigMode) {
            // LEDs 0-4: MIDI channel in binary (0 = omni = all off, 1-16 = channel)
            // LED 5: slow blink to indicate config mode
            uint8_t ch = configMidiChan;
            for (int i = 0; i < 5; i++) LedOn(i, (ch >> i) & 1u);
            LedOn(5, (sampleCount >> 15) & 1u);
        } else if (recording) {
            for (int i = 0; i < 6; i++) LedOn(i, true);
        } else {
            for (int i = 0; i < NUM_VOICES; i++) {
                if (tunerMode && (i == 0 || i == 2 || i == 4)) continue;
                if (i == 5 && presetFlashTimer > 0) {
                    LedOn(5, true);
                    --presetFlashTimer;
                } else if (i == bankIdx && !voices[i].active) {
                    // Selected bank LED: dim when idle, blink while save pending
                    if (flashPending)
                        LedOn(i, (sampleCount >> 12) & 1u);
                    else
                        LedBrightness(i, 2048);
                } else {
                    LedBrightness(i, voices[i].active ? (uint16_t)(voices[i].envAccum >> 12) : 0);
                }
            }
            if (tunerMode && !tunerCounter) {
                // LEDs 0/2/4: tuner display (flat=4, center=2, sharp=0)
                TunerLedValues v = tunerLedValues(periodToCloseness(tunerPd.period()));
                LedBrightness(4, v.flat);
                LedBrightness(2, v.center);
                LedBrightness(0, v.sharp);
            }
            tunerCounter++;
        }
    }

private:
    volatile bool flashPending;   // set by core1 ProcessSample; serviced by core0 MIDICore
    volatile int  bankIdx;        // active sample bank 0-5, set by X knob in ProcessSample
    volatile int  pendingBank;    // bank locked in at the moment flashPending is set

    bool         recording;
    bool         prevSwitchDown;
    bool         prevSwitchUp;
    volatile bool savedToFlash;
    bool         isUSBMIDIHost;
    int          writeHead;
    int          recordDecimate;
    volatile int  presetIdx;
    int           presetFlashTimer;
    static constexpr int PRESET_FLASH_LEN = 4800;  // 100 ms at 48 kHz
    volatile int  loopStartPt;
    volatile bool    pendingDelayCapture;
    int              delayKnobY;
    volatile bool    pendingCVNoteOn;
    volatile uint8_t pendingCVNote;
    bool             gateState;
    uint8_t          cvGateNote;    // note currently committed to CV Out 1 / Gate Out 1

    // Arpeggiator
    int  arpIdx;        // current note index (-1 = reset)
    int  arpGateTimer;  // samples until Gate Out 2 goes low
    int  arpIntTimer;   // internal delay-synced tick counter
    static constexpr int ARP_GATE_LEN = 2400;  // 50 ms gate pulse

    // Config
    uint8_t          midiChannel;     // 0 = omni, 1-16 = specific channel
    volatile bool    inConfigMode;    // tells ProcessSample to show config LEDs
    volatile uint8_t configMidiChan;  // channel being dialled in during config mode

    // Tuner (active when Audio In 2 is connected)
    PitchDetector tunerPd;
    uint8_t       tunerCounter;      // wraps every 256 samples for display update

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
    set_sys_clock_khz(192000, true);   // 192 MHz = 48 kHz × 4000 — clean audio clock multiple
    VSS vss;
    VSS::s_instance = &vss;   // audioEntry() needs this before Run() sets thisptr
    vss.EnableNormalisationProbe();   // must be before Run() (called inside audioEntry)
    // Audio on core 1 (lockout victim); USB + flash save on core 0 (main thread).
    // This matches the reverb card architecture: flash ops happen from the non-ISR
    // core so multicore_lockout_start_blocking() is safe to call.
    multicore_launch_core1(VSS::audioEntry);
    vss.MIDICore();   // USB loop runs here on core 0; never returns
}
