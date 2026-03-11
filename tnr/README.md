# Dual C-Note Tuner (TNR)

A dual-channel tuner that listens to audio inputs and shows how close each signal is to any C note. Useful for tuning oscillators to a C before starting a patch, or for keeping two oscillators in tune with each other.

## How It Works

Each channel detects the fundamental pitch of the incoming audio using zero-crossing period measurement. It then octave-reduces the frequency and compares it to the nearest C note, displaying the deviation across three LEDs.

The detection range is 20 Hz – 4 kHz. It takes about eight cycles of the input signal to lock on, so low notes take slightly longer to register than high ones.

## Display

Two sets of three LEDs, one per channel:

| LED | Channel 1 | Channel 2 |
|-----|-----------|-----------|
| 0   | Sharp (above C) | Sharp (above C) |
| 1   | — | Sharp (above C) |
| 2   | In tune | — |
| 3   | — | In tune |
| 4   | Flat (below C) | — |
| 5   | — | Flat (below C) |

- **Center LED (2 or 3)**: fades from off (6 semitones away) to fully lit (exactly on a C).
- **Side LEDs (0/4 or 1/5)**: fully lit when 6 or more semitones away, fade to off as the pitch approaches C.
- **All LEDs off**: no signal detected.

When perfectly in tune, the center LED is fully lit and both side LEDs are dark.

## Connections

- **Audio In 1**: signal for tuner channel 1
- **Audio In 2**: signal for tuner channel 2
- **CV Out 1**: middle C (C4, ~261.6 Hz) — use this as a tuning reference pitch
- **CV Out 2**: middle C (C4, ~261.6 Hz) — use this as a tuning reference pitch

## Patch Ideas

### Tuning an oscillator to C

Patch an oscillator's audio output into **Audio In 1**. Patch **CV Out 1** into the oscillator's CV input as a starting point. Tune the oscillator's coarse and fine controls until the center LED (2) is fully lit and LEDs 0 and 4 are dark.

### Tuning two oscillators to each other on C

Patch the first oscillator into **Audio In 1** and the second into **Audio In 2**. Tune each until both center LEDs (2 and 3) are lit. The oscillators will now be in tune with each other and both on a C.
