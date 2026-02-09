#ifndef MAIN_H
#define MAIN_H


// includes
#include <Arduino.h>
#include <avr/io.h>


// Debug Parameters
//#define DEBUG_FIXED_8S    // this is to debug the timer is intervals of 8 seconds
#define DEBUG_NO_DELAY

//#define DEBUG_TONE  1          // debug directly with a tone

// Set a fixed one to play      // 17 is the freedom one.
//#define DEBUG_FIXED_WAV 17
//#define DEBUG_FORCE_SIZES

    // Random delay 10min-4hr (75-1800 cycles of 8s)
    // 4 hours is 1800
    // 2 hours is 900
    // 1 hour is 450
    // 10 min is 75
    // 45 is 6 min
    // 37 is about 5 min
    // 8 is about 1 min...
#define TIME_MAX 450
#define TIME_MIN 45

// Max samples
// @8kHz, 8000 samples per second 
// 32000=4s
// 40000=5s
#define MAX_SAFE_SAMPLES 40000



// Pin Definitions
#define PIN_SPEAKER   PB4

#define PIN_MOSI PB0
#define PIN_MISO PB1
#define PIN_SCK PB2
#define PIN_CS PB3

// Winbond DataFlash commands
#define PAGEPROG      0x02
#define READSTATUS    0x05
#define READDATA      0x03
#define WRITEENABLE   0x06
#define CHIPERASE     0x60
#define READID        0x90
#define POWERDOWN     0xB9
#define RELEASEPD     0xAB

#define MAX_SIZES 64   // adjust to your maximum expected number (e.g. 10 samples)


// variables export
extern volatile boolean StayAwake;


#endif