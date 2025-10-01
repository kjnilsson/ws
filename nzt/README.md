# Grain Noise and Tools (NZT)

This is a card that provides "grain" noise similar to that found in some machines
inside the Elektron Digitone II.


## Controls

* **Main**: controls the "density" of the noise
* **X**: Controls the noise seed when **Pulse In 1** is connected and **CV In 2** isn't. (see GateIn 1)
* **Y**: Controls the gain applied to the value of **CV In 1** (if connected) that is added to the
value of the main knob.
* **Audio Out 1**: the grain noise output
* **Audio Out 2**: the "opposite" grain noise output, if **Audio Out 1** is dense this is sparse and so on.
* **CV In 1**: CV in for modulating the noise density.
* **CV In 2**: CV in for modulating the noise seed. **Knob X** is not used when there is
a connection here.
* **Pulse In 1**: resets the seed of the noise generator. Connect the square wave of an oscillator to create a noise oscillator (as long as density is high) similar to what can be found in a Nord Modular G1.
* **CV Out 1**: outputs a static ~-6V - useful for making a selfrunning slope more suitable for
input into the computers CV ins.
* **CV Out 2**: S&H output, updated every time there is a trigger into **Pulse In 2**.
* **Audio In 1**: This signal is used as a ring modulator of the noise. Put a sine wave
here to mellow the noise sound somewhat.
* **Audio In 2**: The internal noise source is replaced with the audio from this input if connected.
* **Pulse Out 2**: Outputs a short pulse every 1.366 seconds or so. Useful for patching into a pulseinput for drum sound design.


## Patch Ideas

### 1. Different Waves

A different take on making seaside sounds.

* Audio Out 1 -> Mixer 1 with pan fully CCW
* Audio Out 2 -> Mixer 2 with pan fully CW
* CV Out 1 -> Slope 1 Input
* Slope 1 Out -> CV In 1

Set slope 1 to triangle mode, loop with a slow time.
Set Computer's Main knob in the middle and turn Y a third of the way up.

Enjoy some waves. Maybe a touch of reverb would not go amiss.

**Bonus**: patch a CV melody into one of the oscillators and path the square wave into
the computer's pulse 1 input. When each wave hit's the "crest" you'll hear a
distant melody (if Main and Y are at the right positions, adjust to taste, also
adjust X for different timbres).

### 2. Snare Generator

**Basic**:

* Audio Out 1 -> Mixer 3
* Pulse Out 1 -> Slope 2 In
* Slope 2 Out -> CV In 1

Set Main knob to fully CCW and Y to 50%. Set Slope 2 to decay mode with no loop.
Adjust Slope 2 time and computer knob Y until you get a reasonably
snappy noise snare type sound. The noise will probably never decay quite fully,
this is normal - you probably want to sample and edit these anyway or patch
via the ringmod to get a VCA over it.
For more fun patch the computer audio out via the distortion (obvs).

**Next**:

Get the oscillators involved.
Set Osc 1 tune to 50% and connect square out into computer pulse 1 in. Now you
should get a bit more "body" to the sound. Try a bit of FM from Osc 2.
Maybe tweak X and Y a bit.
Connect Osc 2 sine out to computer audio input 1 for a slightly more mellow
sound.

### 3. Bag O' Pipes

This uses the noise oscillator and we need a CV/Gate source for this. Like a
keyboard.

Audio Out 1 -> Mixer 3
CV Source -> Osc 2 pitch
Osc 2 square -> Pulse In 1
Gate Source -> Slope 1 In
Slope 1 Out -> CV In 1

Set Slope 1 to decay mode with a slow time. Set Main knob fully CCW and Y to
~66%. Play your CV/Gate source, enjoy the nasal tones turning into increasingly
sparse noise as you release a note.

**Extra**:

Split your gate so you can also patch it to the computer's Pulse In 2 jack then
patch CV Out 2 into CV In 2. Each note now should have a slightly different timbre.

Reverb is recommended (isn't it always?).
