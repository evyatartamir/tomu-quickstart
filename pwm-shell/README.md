# OpticSpy Transmitter

This program demonstrates the usage of Timer PWM and interrupts to control the Tomu's green and red LEDs.
A shell interface over USB serial (ACM) is implemented, based on the ACM and Opticspy demos.
The interface allows setting the Tomu's LEDs to various brightness values and blink/fade patterns.

The changes to the PWM duty cycle for each LED are done individually in its concomitant timer's OF interrupt.
Millisecond timing is estimated based on the LED Timer's frequency (derived from clock, prescale, top value settings).
This timing method can still achieve 99% accuracy, which seems acceptable for a LED indicator viewed by humans.
It is possible to use another timer or the System Tick interrupt to count milliseconds more accurately and handle both LEDs together.
