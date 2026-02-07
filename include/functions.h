#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "main.h"


//extern uint16_t 
extern const int speaker;


uint16_t getSeed();
uint16_t jitterSeed();

/*******************************************************************************************************************************
 *  Play test Tones  
 *******************************************************************************************************************************
 * 
 */
void playTestTone();
// Parameterized version: playTestTone_ms(500);  // 500ms
void playTestTone_ms(uint16_t ms) ;

// Full control: playTestTone_ms_freq(100, 1000);  // 100ms @1kHz
void playTestTone_ms_freq(uint16_t ms, uint16_t freq_hz);

#endif