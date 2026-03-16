# VSS — µ-law Sampler

A 6-voice polyphonic sampler inspired by the Yamaha VSS-30, for the Music Thing Workshop System Computer. Record any audio source, then play it back chromatically via USB MIDI with looping, envelope shaping, and tape-style echo.

Audio is recorded and stored at 24 kHz using µ-law compression (8 bits/sample), giving up to 6 seconds of mono sample time. The sample survives power-off — it is saved to flash and restored automatically on boot.

## Sample banks

VSS stores up to 6 independent samples in flash (banks 0–5). The X knob selects the active bank, shown as a dimly lit LED (LEDs 0–5 = banks 0–5).

- **Loading**: the bank selected by the X knob is loaded on power-on. The playing sample does not change mid-session when the knob moves.
- **Saving**: flip Z switch up after recording to save to the current bank. Banks are independent — saving to one does not affect the others.

## Recording

Hold the **Z switch down**. All six LEDs light to confirm recording is in progress. Release the switch (or wait for the buffer to fill) to stop. The sample is ready to play immediately.

Recording resets the sample — any previously held notes stop.

## Saving to flash

Flip the **Z switch up** after recording. The module saves the sample to flash and verifies the write. Six LEDs count down as each stage completes, then the result is shown as four blinks:

| LED | Meaning |
|-----|---------|
| 5   | Magic header written correctly |
| 4   | Sample length stored correctly |
| 3   | Loop point stored correctly |
| 2   | Data checksum matches |

All four LEDs lit = save verified. The sample will be restored on the next power-on.

LED 5 blinks slowly while a save is in progress.

## Playback

Connect a USB MIDI keyboard or controller. The module auto-detects whether it is the host (keyboard plugged directly into the Computer's USB port) or a device (Computer plugged into a laptop or sequencer).

- **Base pitch**: MIDI note 60 (C4) plays the sample at its recorded pitch.
- **Polyphony**: up to 6 simultaneous voices.
- **Loop**: after the first full pass, playback loops the second half of the sample with a short crossfade at the loop point to avoid clicks.
- **CV Out 1 / Gate Out 1**: tracks the lowest currently held note for use with an analogue voice.

## Controls

| Control | Function |
|---------|----------|
| Main knob | ADSR envelope preset (see table below) |
| X knob    | Sample bank select (0–5, loaded on power-on) |
| Y knob    | Delay time, sampled on each note-on — slapback (~50 ms) to long echo (~750 ms) |
| Z switch down | Record |
| Z switch up   | Save to current bank |

## Outputs

| Output | Signal |
|--------|--------|
| Audio Out 1 | Dry polyphonic mix |
| Audio Out 2 | Tape-style echo (wet only) |
| CV Out 1    | Lowest held MIDI note as 1V/oct |
| Gate Out 1  | High while any voice is active |

## Envelope presets (Main knob, left to right)

| Position | Name | Character |
|----------|------|-----------|
| 1 | Pluck | Very short, no sustain |
| 2 | Clavinet | Tight attack, fast decay |
| 3 | Marimba | Soft attack, medium decay |
| 4 | Piano | Natural piano decay |
| 5 | Harpsichord | Instant attack, long decay |
| 6 | Organ | Instant on, full sustain |
| 7 | Strings | Slow attack, smooth sustain |
| 8 | Pad | Very slow attack, long release |
| 9 | Choir | Slow swell, breathy sustain |
| 10 | Slow Sweep | Very slow attack and release |

## Delay (Audio Out 2)

The echo on Audio Out 2 is a tape-style mono delay with low-pass filtered feedback, giving progressively darker repeats (~5 audible echoes). The Y knob sets the delay time at the moment each note is struck (~50 ms to ~750 ms) — notes played with different Y positions will have different echo times.

Mix Audio Out 1 (dry) and Audio Out 2 (wet) externally to taste.

## LEDs

| LED | Meaning |
|-----|---------|
| 0–5 | Voice activity (one per voice) |
| 0–5 all lit | Recording in progress |
| One LED dimly lit | Selected sample bank (X knob position) |
| Dimly lit LED blinking | Save pending |
