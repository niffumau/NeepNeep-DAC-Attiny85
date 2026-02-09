/**
 * @file
 * @brief ATTiny13A/85 utility functions: Error alarms, entropy seeds, test tones (1kHz PWM via Timer1 PLL64x).
 * 
 * Bit-banged speaker on PB4. Precise cleanup prevents clicks/pops. Frequency-variable tones for debugging.
 */

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

/**
 * @brief Error indicator: 1 warning beep + _count short beeps (audible error code).
 * 
 * Used by flash driver, timeouts, ID checks. Leaves speaker as INPUT (no drain).
 * @param _count Number of short 440Hz beeps (e.g., 4=flash error, 5=ID fail).
 */
void warning_alarm(uint8_t _count) {

    // Play the first tone so we know its an error
    playTestTone_ms_freq(50, 1000);
    _delay_ms(100);

    // play the nubmer of tones in count
    for (uint8_t i=0; i < _count; i++) {
        playTestTone_ms_freq(30, 440);
       // playTestTone_ms_freq(50, 1000);
        _delay_ms(30);

    }
  _delay_ms(500);

  /*// Proper stop sequence
  TCCR1 = 0;  // Stop Timer1 clock first
  GTCCR = 0;  // Disable PWM1B mode
  OCR1B = 0;  // Duty to zero
  PLLCSR &= ~(1<<PCKE);  // Crucial: Disable PLL clock source (resets to system clk)
  PINB |= (1<<4);  // Force pin high*/
  pinMode(PIN_SPEAKER, INPUT);

}

/***************************************************
 *  
 ***************************************************
 * 
 */

/**
 * @brief Generates entropy seed from internal temperature sensor (ADMUX=14).
 * 
 * 
 * @note I didn't get this to work
 * @return 10-bit ADC reading (biased toward unique temperature).
 */
uint16_t getSeed() {
  ADMUX = _BV(MUX3) | _BV(MUX2);          ///< Temperature sensor
  ADCSRA = _BV(ADEN) | _BV(ADSC) | 0x07;  ///> Enable + prescaler

  while (ADCSRA & _BV(ADSC)); // wait
  return ADC;
}

/***************************************************
 *  
 ***************************************************
 * 
 */

/**
 * @brief High-entropy seed via Timer0 overflow jitter (16x XOR accumulation).
 * 
 * @note I didn't get this to work
 * @return 16-bit random-ish value from counter timing noise.
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

/**
 * @brief Fixed 100ms 1kHz test tone (PLL64x → Timer1 PWM /512 prescale).
 * 
 * Calibrated software delay. Safe shutdown prevents clicks.
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

/**
 * @brief Duration-variable 1kHz test tone.
 * @param ms Play duration in milliseconds (software-timed @1kHz cycles).
 */
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

/**
 * @brief Full control test tone: arbitrary duration + frequency on ATTiny speaker pin.
 * 
 * Generates precise musical tones using Timer1 PWM (64MHz PLL /512 prescale) on PB4.
 * Frequency calculated as 64kHz / TOP (clamped 1-250Hz range). Blocking call with safe cleanup.
 *
 * @param _ms Duration in milliseconds (software timed via _delay_us(1) loops).
 * @param _freq_hz Target audible frequency (Hz). 
 *                  - Min: ~250Hz (TOP=255) 
 *                  - Max: ~25kHz (TOP=2, ultrasonic)
 *                  - Musical range: 261-4186Hz (middle A to high C)
 *
 * @pre PIN_SPEAKER defined (typically PB4). No other Timer1/PLL usage during call.
 *
 * @note PWM math: 64MHz PLL ÷ 512 prescale = 125kHz base → TOP=125 = 1kHz tone.
 * @note Contains **duplicate cleanup code**—second block has improved sequence + comments.
 * @warning Direct register manipulation. Do **not** call from ISR. Blocking (~_ms ms).
 *
 * @todo Remove first "Stop" block (lines 25-29)—keep only improved sequence (lines 34-40).
 */
void playTestTone_ms_freq(uint16_t _ms, uint16_t _freq_hz) {
    pinMode(PIN_SPEAKER, OUTPUT);

    // Phase 1: PLL + PWM startup (~3µs lock time)
    PLLCSR = (1<<PCKE) | (1<<PLLE);
    while(!(PLLCSR & (1<<PLOCK)));

    // Phase 2: Calculate TOP for target frequency (64kHz base / TOP)
    uint16_t top = 64000 / _freq_hz;
    if (top > 255) top = 255;           ///< Safety: prevent div0/overflow

    // Phase 3: Timer1 Fast PWM Mode 15 (TOP=OCR1C)
    GTCCR = (1<<COM1B0) | (1<<PWM1B);           // PWM B, toggle OC1B on match
    TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512 prescale (64MHz→125kHz)
    OCR1C = top;                                // Frequency TOP
    OCR1B = top / 2;                             // 50% duty cycle

    // FIXED: cycles = ms * 1000 (microseconds total)
    // Phase 4: Blocking play duration (precise µs timing)
    uint32_t total_us = (uint32_t)_ms * 1000;
    for(volatile uint32_t i = 0; i < total_us; i++) {
      _delay_us(1);  // Calibrated 1µs per iteration
    }

    // Stop
    TCCR1 = 0; GTCCR = 0; OCR1B = 0;
    PINB |= (1<<4);
    pinMode(PIN_SPEAKER, INPUT);

    // Phase 5: CRITICAL SAFE SHUTDOWN SEQUENCE (prevents clicks/pops)
  // apparently the way to clear it
  // In stop section, replace with:
    TCCR1 = 0;       ///> Stop timer clock FIRST (safe even with TOP=0)
    GTCCR = 0;       ///> Disable PWM modes
    OCR1C = 0;       ///> NOW safe: clear TOP
    OCR1B = 0;       ///> Clear duty
    PLLCSR &= ~(1<<PCKE);  ///> Disable PLL (back to system clock: 9.6/1.2MHz)
    PINB |= (1<<4);         ///> Force PB4 HIGH (silence)  ****** I think i shouldn't do this, it should be PB4 not referenced as just 4???
    pinMode(PIN_SPEAKER, INPUT);   ///>  High-Z input (zero power draw)

 
}




