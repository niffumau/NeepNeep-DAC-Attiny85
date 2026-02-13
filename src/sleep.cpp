#include "Arduino.h"

#include <avr/sleep.h>
#include <avr/wdt.h>  // For Watchdog Timer
#include <util/delay.h>  // _delay_us()

#include "sleep.h"

/**
 * @brief Executes randomized power-down sleep cycles using 8-second WDT interrupts on AVR.
 *
 * Puts the ATTiny13A (or similar) into power-down mode for a random duration between TIME_MIN
 * and TIME_MAX cycles (each cycle = 8 seconds via WDT interrupt). Resets WDT each iteration to
 * prevent unintended reset. Final wdt_enable(WDTO_8S) arms full-system reset timeout.
 * Uses #DEBUG_NO_DELAY to disable entirely, #DEBUG_FIXED_8S for single 8s cycle testing.
 *
 * @pre Requires:
 *      - `sleep.h` with `#define TIME_MIN` and `#define TIME_MAX` (uint16_t bounds)
 *      - WDT already configured/initialized in setup() (as per comments)
 *      - `srand()` called earlier for rand() quality
 *
 * @note Each sleep cycle:
 *      1. Configures WDT for 8s interrupt (WDIE + WDP3/WDP0 = 8.0s)
 *      2. Enters SLEEP_MODE_PWR_DOWN (deepest sleep, no peripherals)
 *      3. WDT interrupt auto-wakes (ISR must call wdt_reset())
 *      4. Repeats for random cycles
 * @note Total sleep: 8s × (random[TIME_MIN..TIME_MAX]) ≈ 8s to several minutes
 * @note DEBUG_FIXED_8S forces exactly 1 cycle (8s total)—ideal for testing
 *
 * @warning 
 *      - No bounds checking on TIME_MIN/MAX—ensure TIME_MIN <= TIME_MAX
 *      - Requires external WDT ISR: `ISR(WDT_vect) { wdt_reset(); }`
 *      - Final wdt_enable(WDTO_8S) starts reset countdown—caller must reset/rearm
 *      - #DEBUG_NO_DELAY completely skips (for active debugging)
 * @todo Possibly merge some of the things that both chips have in common, i can see that the wdt_enable(WDTO_8S) appears
 * to be the same.
 */
void sleep_function(void) {

  #if !defined(DEBUG_NO_DELAY)
    uint16_t time_max = TIME_MAX; 
    uint16_t time_min = TIME_MIN;
    uint16_t cycles = (rand() % (time_max - time_min + 1)) + time_min;

    #if defined(DEBUG_FIXED_8S) 
    cycles = 1;
    #endif

    
  #ifdef __AVR_ATtiny85__

    for(uint16_t i = 0; i < cycles; i++) {
      wdt_reset();
      // these should already be taken care of from the setup
      WDTCR = (1<<WDCE) | (1<<WDE);               ///< Enable config
      WDTCR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);  ///< 8s
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);        
      sleep_enable();
      sleep_cpu();
      sleep_disable();
      wdt_reset();
    }    
    wdt_enable(WDTO_8S);
    #endif

  #elif defined(__AVR_ATtiny412__)
    // STuff for the ATtiny412
    for(uint16_t i = 0; i < cycles; i++) {
      wdt_disable();  // Stop any prior WDT [web:112]
      wdt_enable(WDTO_8S);  // 8s interrupt (WDIE auto-enabled) [web:109][web:112]
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sleep_cpu();
      sleep_disable();
      wdt_reset();  // Reset counter
    }
    // Final reset timeout
    wdt_disable();
    wdt_enable(WDTO_8S);  // Arms reset (no interrupt) 
    
  
  #else
    #error "Unsupported board! Define ATTINY85 or ATTINY412."
  #endif

}

