## Welcome
This is a simple C program designed to run on a PIC16F684. The program
uses the chip's PWM output to generate a low-frequency voltage wave.
As one might expect, the PWM output must be heavily filtered before
the output voltage wave's "true form" can be observed.

One can program the output frequency of the wave by applying an analog
voltage between 0 and 5V to pin 9 of the chip. This may be done with
a simple linear potentiometer or whatever other means are convenient.

The specific waveshape of the output can be configured by applying
a combination of 0 and / or 5V signals to pins 6 and 7 of the chip.
Sine, triangle, and square waveshapes are available.

## Implementation notes
# external clock
this program requires a 20MHz external clock to work as designed.
# programming using ICSP
put a diode between MCLR and VDD (cathode to MCLR) to prevent negative
voltage transients from screwing up the circuit being programmed.

