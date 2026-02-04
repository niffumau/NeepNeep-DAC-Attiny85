/* TinyTone on PB4 (Pin 3) for ATtiny85 PCB - Plays Scale
   Software ISR toggle + Timer1 CTC timing
*/

#include <Arduino.h>
#define PIN_SPEAKER PB4

// Notes (divisors for ~8MHz/64 → audible range)
const int Note_C  = 479;  // Adjusted for 8MHz/64 CTC
const int Note_D  = 508;
const int Note_E  = 538;
const int Note_F  = 570;
const int Note_G  = 604;
const int Note_A  = 640;
const int Note_AS = 678;
const int Note_B  = 719;

volatile boolean tone_on = false;

ISR(TIM1_COMPA_vect) {
  if (tone_on) PINB ^= (1 << PIN_SPEAKER);  // Toggle PB4
}

void TinyTone(unsigned int divisor, unsigned long duration) {
  // Setup Timer1 CTC for note freq: 8MHz/64 / (2*divisor) Hz
  TCCR1 = 0;                           // Stop timer
  TCNT1 = 0;
  OCR1A = divisor;                     // Half-period ticks
  TIMSK |= (1<<OCIE1A);                // ISR
  
  tone_on = true;
  sei();
  delay(duration);                     // Note length
  tone_on = false;
  TIMSK &= ~(1<<OCIE1A);               // Stop ISR
  PORTB &= ~(1 << PIN_SPEAKER);        // Off
}

void playTune(void) {
  TinyTone(Note_C,  400);
  delay(50);
  TinyTone(Note_D,  400);
  delay(50);
  TinyTone(Note_E,  400);
  delay(50);
  TinyTone(Note_F,  400);
  delay(50);
  TinyTone(Note_G,  400);
  delay(50);
  TinyTone(Note_A,  400);
  delay(50);
  TinyTone(Note_AS, 400);
  delay(50);
  TinyTone(Note_B,  400);
  delay(50);
  TinyTone(Note_C*2/1, 800);         // High C
}

void setup() {
  DDRB |= (1 << PIN_SPEAKER);         // PB4 output
  ADCSRA &= ~(1<<ADEN);
  TCCR1 = (1<<CTC1) | (6<<CS10);      // CTC /64 prescale (global)
}

void loop() {
  playTune();
  delay(2000);
}
