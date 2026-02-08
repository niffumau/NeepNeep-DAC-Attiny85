#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "main.h"


//extern uint16_t 
//extern const int speaker;


uint16_t getSeed();
uint16_t jitterSeed();

void warning_alarm(uint8_t _count);

/*****************************************************************************************
 * @brief Plays warning alarm
 * 
 * Detailed Notes
 * 
 ****************************************************************************************/

void playTestTone();
// Parameterized version: playTestTone_ms(500);  // 500ms
void playTestTone_ms(uint16_t ms) ;

/************************************************************************************//**
 * @brief Return the time difference between two times
 * 
 * Full control: playTestTone_ms_freq(100, 1000);  // 100ms @1kHz
 * @param _ms the time to play the tone for
 * @param _freq_hz the frequency in hz
 */
void playTestTone_ms_freq(uint16_t ms, uint16_t freq_hz);



//void play_random_sample();


#endif