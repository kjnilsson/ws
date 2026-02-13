### Six!

A six stage sequencer. Each stage can have a note and multiple steps, up to six.

## Switch == Up (Playback)

Plays back each stage in order, one step per pulse received on Pulse In 1.
Pulse In 2 resets the sequence — on the next clock edge when externally clocked,
or immediately when using the internal clock.

CV Out 1 and 2 output v/oct pitch. Gate 1 outputs a gate, Gate 2 fires a short
trigger pulse on each stage change.

### Knob X — Tempo

When no external clock is connected (Pulse In 1), the X knob controls the
internal tempo from ~1 step/sec (fully left) to ~16 steps/sec (fully right).
When an external clock is connected, the X knob has no effect.

### Knob Main — Performance

The main knob is divided into four zones:

- **0-25%** — Forward playback
- **25-50%** — Forward + octave shift (probability increases across the range)
- **50-75%** — Reverse playback
- **75-100%** — Random stage selection

LEDs 0-3 dim to indicate the active zone.

### Knob Y — Retrigger

When Y is above 50%, the gate re-fires on every step within a stage instead of
holding for the full stage duration. When combined with octave shift, each
retrigger also rolls for an octave shift.

## Switch == Middle (Manual)

Use knob Y to scrub through the stages. The LEDs indicate the selected stage.

## Switch == Down (Edit)

From manual mode, hold the switch down to edit the selected stage. Use the main
knob to edit the note and knob X to modify the number of steps. The current
step count is shown by the LEDs at 50% brightness.

Notes and steps use a "catch" mechanism — you must move the knob to match the
current value before edits take effect, preventing jumps when switching stages.

## Audio

- **Audio In 1 only** — routed through a BBD delay synced to step length
- **Audio In 2** — BBD delay with longer delay time (doubled, or tripled if Audio In 1 is also connected)
- **Nothing connected** — Audio Out 1 outputs noise, Audio Out 2 outputs sample & hold
