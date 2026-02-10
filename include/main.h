/**
 * @file main.h
 * @brief Project‑wide configuration and shared declarations for the ATtiny85 project.
 *
 * This header defines:
 * - Compile‑time options that control debugging behaviour, fixed timing, and
 *   fixed‑tone selection.
 * - Random‑sleep and delay parameters (min/max watchdog cycles and sample limits).
 * - GPIO pin assignments for the speaker and SPI‑connected DataFlash (Winbond‑style).
 * - Common command codes for interacting with the external flash chip.
 * - Shared global variables visible to other modules (e.g., `StayAwake`).
 *
 * @note
 *   - Several macros such as `WIPE_ATTINY`, `DEBUG_FIXED_8S`, `DEBUG_TONE`, and
 *     `DEBUG_FIXED_WAV` are intended for debugging and development and should
 *     normally be disabled in production.
 *   - `TIME_MIN` and `TIME_MAX` define the number of 8‑second watchdog intervals
 *     between “wakeup + tone” events, corresponding to roughly 1 minute to 1 hour.
 *   - `MAX_SAFE_SAMPLES` and `MAX_SIZES` constrain audio‑data handling in flash
 *     and playback routines.
 *
 * @see main.cpp, functions.h, sleep.h, flash.h
 */


#ifndef MAIN_H
#define MAIN_H

// includes
#include <Arduino.h>
#include <avr/io.h>


// Debug Parameters
//#define WIPE_ATTINY            /// used to entirely wipe the ATTINY
//#define DEBUG_FIXED_8S    // this is to debug the timer is intervals of 8 seconds
//#define DEBUG_NO_DELAY

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
#define TIME_MIN 8

// Max samples
// @8kHz, 8000 samples per second 
// 32000=4s
// 40000=5s
#define MAX_SAFE_SAMPLES 40000      ///< The maximum samples that we will allow per play
#define MAX_SIZES           64      ///< The maximum number of samples that can be loaded onto the flash

/**
 * @defgroup pin_macros Pin Configuration Macros
 * @brief ATtiny GPIO pin assignments used in the project.
 * @{
 */
#define PIN_SPEAKER     PB4     ///< Speaker Pin
#define PIN_MOSI        PB0     ///< Flash MOSI Pin
#define PIN_MISO        PB1     ///< Flash MISO Pin
#define PIN_SCK         PB2     ///< Flash Serial Clock Pin
#define PIN_CS          PB3     ///< Flash Chip Select Pin
/** @} */

/**
 * @defgroup Winbond DataFlash Commands
 * @{
 */
#define PAGEPROG      0x02
#define READSTATUS    0x05
#define READDATA      0x03
#define WRITEENABLE   0x06
#define CHIPERASE     0x60
#define READID        0x90
#define POWERDOWN     0xB9
#define RELEASEPD     0xAB
/** @} */



// variables export
extern volatile boolean StayAwake;


#endif