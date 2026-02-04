#ifndef FUNCTIONS_SLEEP_H
#define FUNCTIONS_SLEEP_H


#ifdef __AVR_ATtiny13__
  #define SLEEP_FOREVER  128
  #define SLEEP_016MS    ~(1<<WDP3)|~(1<<WDP2)|~(1<<WDP1)|~(1<<WDP0)
  #define SLEEP_125MS    (1<<WDP1) | (1<<WDP0)
  #define SLEEP_250MS    (1<<WDP2)
  #define SLEEP_500MS    (1<<WDP2) | (1<<WDP0)
  #define SLEEP_1SEC     (1<<WDP2) | (1<<WDP1)
  #define SLEEP_2SEC     (1<<WDP2) | (1<<WDP1) | (1<<WDP0)
  #define SLEEP_4SEC     (1<<WDP3)
  #define SLEEP_8SEC     (1<<WDP3) | (1<<WDP0)
#endif /* ifdef __AVR_ATtiny13__ */

extern uint16_t RANDOM_SLEEP_MIN;
extern uint16_t RANDOM_SLEEP_MAX;

void updateWatchDog(uint8_t c);
void system_sleep(byte b);

#endif