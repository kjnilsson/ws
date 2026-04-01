# soz — flash scratchpad sampler

A durable audio scratchpad for the Music Thing Workshop System Computer. Record patchwork audio into arbitrary positions in flash, then loop-play any region. Recordings survive power cycles.

16-bit lossless audio at 24 kHz. Audio In 1 is always passed through to Audio Out 2.

## Buffer duration

| Flash size | Duration |
|------------|----------|
| 2 MB       | ~38 s    |
| 16 MB      | ~5.7 min |

## Controls

### Switch Up — Playback

- **Main knob** — playback start offset within flash
- **X knob** — loop length (short to full region)
- Audio Out 1 = playback

### Switch Middle — Stopped / cursor

- **Main knob** — set recording start position (cursor)
- LEDs show cursor position

### Switch Down — Record (sound-on-sound)

- Records Audio In 1 starting from the cursor position
- Advances linearly through flash until released (ignores loop length)
- **Y knob** — mix balance: fully CCW = keep existing audio, fully CW = replace with input
- Audio Out 1 = input monitor
- Audio Out 2 = existing flash content at current position

### Erase

Hold switch down at power-on to wipe all recorded data. LEDs show progress.

## Build

```
make soz       # 2 MB flash (standard)
make soz16     # 16 MB flash
```

## Technical notes

- Dual-core: core 0 handles flash erase/program, core 1 runs the 48 kHz audio ISR
- Entire binary runs from RAM (`copy_to_ram`) so flash operations don't stall code execution
- Double-buffered sector staging prevents recording gaps
- Sector fill time (~85 ms) comfortably exceeds flash erase+program cycle (~55 ms)
- Sound-on-sound pre-reads existing flash content before erasing for mix
- Loop playback crossfades at boundaries to avoid clicks
