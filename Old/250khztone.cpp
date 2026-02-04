/* ATtiny85 1kHz Test Tone Generator
   Continuous square wave on pin 4 (PB4/speaker) - FIXED for compile
Key Setup Steps
Enables the 64MHz PLL (Phase-Locked Loop) on Timer1 for faster timing than the default 16MHz system clock, then waits for it to lock.
​

Disables interrupts (TIMSK=0), stops Timer0 (TCCR0A/B=0, affecting millis()/delay()), and turns off ADC to minimize interference and power use.

Configures Timer1 for Fast PWM with OCR1C as TOP (period limit), no prescaler (runs at full 64MHz), and toggle-on-match output on OC1B (pin 4/PB4).

apparently this put sa 250KHz tone on PB4...

*/

#include <Arduino.h>

const int speaker = 4;

void setup() {
  // Enable 64 MHz PLL for Timer1
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));  // Wait lock

  TIMSK = 0;  // Disable interrupts
  TCCR0A = 0; TCCR0B = 0;  // Stop Timer0
  ADCSRA &= ~(1<<ADEN);    // Disable ADC

  // Timer1 Fast PWM: OCR1C=TOP, toggle OC1B (PB4) for square wave
  GTCCR = (1<<COM1B0) | (1<<PWM1B);  // Toggle B on match UP, PWM mode
  TCCR1  = (3<<CS10);                // Fast PWM OCR1C=TOP, no prescale (64MHz)
  OCR1C  = 64;                       // TOP ~1kHz (64MHz / 2 / 64 = 500kHz toggle rate → 1kHz)
  OCR1B  = 32;                       // Match every half cycle (50% duty)

  pinMode(speaker, OUTPUT);
}

void loop() {
  // Empty - continuous tone
}

