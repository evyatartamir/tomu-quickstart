# PWM Shell LED indicator

This program demonstrates the usage of Timer PWM and interrupts to control the Tomu's green and red LEDs.
A shell interface over USB serial (CDC-ACM) is implemented, based on the ACM and Opticspy demos.
The interface allows setting the Tomu's LEDs to various brightness values and blink/fade patterns.
Commands can be scripted, so the Tomu may be used as a notification light (in the spirit of "blink(1)").

The general case for LED operation is a cyclic fade ramp, from Low to ramp up, high, ramp down: __/---\__

The PWM duty cycle values for each LED are updated individually in its concomitant timer's OF interrupt.
Millisecond timing is estimated based on the LED Timer's frequency (derived from clock, prescale, top value settings).
This timing method can achieve > 99% accuracy, which seems acceptable for a LED indicator viewed by humans.
An alternative implementation can use an additional timer or the System Tick interrupt to count milliseconds and update the LEDs.

Usage: <COMMAND>[PARAMETERS]
Command parameters are case-insensitive.
Commands are parsed when the return key is sent ("Enter").
Several commands may be sent in a single line of input, separated by spaces.

Supported commands:

H : Print help message

T : Advance both LEDs to their next test mode (for demo purposes)

G,R : Configure Green/Red LED settings
Syntax: <G,R><T,N,X,L,U,H,D,P>
    T : Set LED configuration to the next test mode (for demo purposes).
    N : Min brightness, 0-100, duty cycle percentage.
    X : Max brightness, 0-100, duty cycle percentage.
    L : Low (Min) value duration in millisconds, positive integer.
    U : Ramp up duration in millisconds, positive integer.
    H : High (Max) value duration in millisconds, positive integer.
    D : Ramp down duration in millisconds, positive integer.
    P : Current phase, letter [L,U,H,D] or number [0,1,2,3].

D : Disable/Enable debug printouts
Syntax: <D><0,1>

E : Disable/Enable Serial Echo for the shell
Syntax: <E><0,1>

P : Print current configuration of the Green/Red LED
Syntax: <P><G,R>

S : Sync both LED timers. Stops, reset, set phase, then start counting at the same time.
The first phase letter is for the Green LED, second for Red LED.
Syntax: <S><[L,U,H,D][L,U,H,D]>
