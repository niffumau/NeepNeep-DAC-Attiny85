////// Includes //////////
#include "Arduino.h"

#include <avr/wdt.h>  // For Watchdog Timer
#include <util/delay.h>  // _delay_us()
//#include <EEPROM.h>

#include "functions.h"

/***************************************************
 *  
 ***************************************************
 * 
 */
void warning_alarm(uint8_t _count) {

    // Play the first tone so we know its an error
    playTestTone_ms_freq(500, 1000);
    _delay_ms(100);

    // play the nubmer of tones in count
    for (uint8_t i=0; i < _count; i++) {
        playTestTone_ms_freq(50, 440);
        playTestTone_ms_freq(50, 1000);
        _delay_ms(100);

    }
  _delay_ms(4000);

  // Proper stop sequence
  TCCR1 = 0;  // Stop Timer1 clock first
  GTCCR = 0;  // Disable PWM1B mode
  OCR1B = 0;  // Duty to zero
  PLLCSR &= ~(1<<PCKE);  // Crucial: Disable PLL clock source (resets to system clk)
  PINB |= (1<<4);  // Force pin high
  pinMode(PIN_SPEAKER, INPUT);

}

/***************************************************
 *  
 ***************************************************
 * 
 */

uint16_t getSeed() {
  ADMUX = _BV(MUX3) | _BV(MUX2); // Temperature sensor
  ADCSRA = _BV(ADEN) | _BV(ADSC) | 0x07; // Enable + prescaler

  while (ADCSRA & _BV(ADSC)); // wait
  return ADC;
}

/***************************************************
 *  
 ***************************************************
 * 
 */
uint16_t jitterSeed() {
  uint16_t _seed = 0;
  for (uint8_t i = 0; i < 16; i++) {
    TCNT0 = 0;
    while (!(TIFR & _BV(TOV0))); // wait overflow
    _seed ^= TCNT0 << i;
  }
  return _seed;
}

//uint16_t seed;


/*******************************************************************************************************************************
 *  Play test Tones  
 *******************************************************************************************************************************
 * 
 */


void playTestTone() {
  pinMode(PIN_SPEAKER, OUTPUT);
  
  // 1kHz PWM on PB4: 64MHz PLL /512 = ~125kHz → OCR1C=125 (1kHz)
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));
  
  GTCCR = (1<<COM1B0) | (1<<PWM1B);
  TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512 prescale
  OCR1C = 125;  // TOP=1kHz
  OCR1B = 62;   // 50% duty
  
  // 100ms delay (calibrated @64MHz PLL)
  for(volatile uint16_t i=0; i<8333; i++) { _delay_us(12); }
  
  // Stop
  TCCR1 = 0;                    // Stop Timer1 clock first
  GTCCR = 0;                    // Disable PWM1B mode
  OCR1B = 0;                    // Duty to zero
  //PLLCSR &= ~(1<<PCKE);         // Crucial: Disable PLL clock source (resets to system clk)
  PINB |= (1<<4);               // Force pin high
  pinMode(PIN_SPEAKER, INPUT);       // Avoid click
}

/***************************************************
 *  
 ***************************************************
 * 
 */
// Parameterized version: playTestTone_ms(500);  // 500ms
void playTestTone_ms(uint16_t ms) {
  pinMode(PIN_SPEAKER, OUTPUT);
  
  // 1kHz PWM setup (unchanged)
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));
  
  GTCCR = (1<<COM1B0) | (1<<PWM1B);
  TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512
  OCR1C = 125;  // 1kHz TOP
  OCR1B = 62;   // 50% duty
  
  // Play for exact ms (1kHz cycles * ms)
  uint32_t cycles = (uint32_t)ms * 1000;  // ms → total cycles @1kHz
  for(volatile uint32_t i = 0; i < cycles; i++) {
    _delay_us(1);  // 1ms per 1000 loops (precise)
  }
  
  // Stop (unchanged)
  TCCR1 = 0;
  GTCCR = 0;
  OCR1B = 0;
  PINB |= (1<<4);
  pinMode(PIN_SPEAKER, INPUT);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
// Full control: playTestTone_ms_freq(100, 1000);  // 100ms @1kHz
void playTestTone_ms_freq(uint16_t _ms, uint16_t _freq_hz) {
    pinMode(PIN_SPEAKER, OUTPUT);

    PLLCSR = (1<<PCKE) | (1<<PLLE);
    while(!(PLLCSR & (1<<PLOCK)));

    uint16_t top = 64000 / _freq_hz;
    if (top > 255) top = 255;

    GTCCR = (1<<COM1B0) | (1<<PWM1B);
    TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512
    OCR1C = top;
    OCR1B = top / 2;

    // FIXED: cycles = ms * 1000 (microseconds total)
    uint32_t total_us = (uint32_t)_ms * 1000;
    for(volatile uint32_t i = 0; i < total_us; i++) {
    _delay_us(1);  // Simple µs counter
    }

    // Stop
    TCCR1 = 0; GTCCR = 0; OCR1B = 0;
    PINB |= (1<<4);
    pinMode(PIN_SPEAKER, INPUT);


  // apparently the way to clear it
  // In stop section, replace with:
    TCCR1 = 0;       // Stop timer clock FIRST (safe even with TOP=0)
    GTCCR = 0;       // Disable PWM modes
    OCR1C = 0;       // NOW safe: clear TOP
    OCR1B = 0;       // Clear duty
    PLLCSR &= ~(1<<PCKE);  // System clock
    PINB |= (1<<4);
    pinMode(PIN_SPEAKER, INPUT);



 
}




