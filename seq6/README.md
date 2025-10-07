### Six!

A six stage sequencer. Each stage can have a note and multiple steps, up to six.

## Switch == Up

This is playback mode. Currently it will play back each stage in order taking
one step per pulse received on Pulse In 1. Pulse In 2 will reset the sequence to
the first step on the next pulse.

CV Out 1 and 2 will output the v/oct pitch. Gate 1 will output a gate lasting
50% of the total steps time.


Knobs currently have no function when in playback mode.


## Switch == Middle

This is manual mode. Here you can use knob Y to scrub through the stages.
The leds will indicate the stage currently selected.

When wanting to edit a note scrub to the stage in question and hold the
switch down. Here you can use the main knob to edit the note and knob X to modify
the number of steps. The current number of steps will be shown by the leds (50% brightness).

The steps and notes will have to be "caught", i.e. you have to move the knob to
the current value before anything will be edited. When finished editing go back
to manual or playback mode.
