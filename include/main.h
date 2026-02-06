#ifndef MAIN_H
#define MAIN_H


// includes
#include <Arduino.h>
#include <avr/io.h>


// Debug Parameters
//#define DEBUG_FIXED_8S    // this is to debug the timer is intervals of 8 seconds

//#define DEBUG_TONE  1          // debug directly with a tone
//#define DEBUG_FIXED_WAV 1


// Pin Definitions
#define PIN_SPEAKER   PB4

// Winbond DataFlash commands
#define PAGEPROG      0x02
#define READSTATUS    0x05
#define READDATA      0x03
#define WRITEENABLE   0x06
#define CHIPERASE     0x60
#define READID        0x90
#define POWERDOWN     0xB9
#define RELEASEPD     0xAB






#endif