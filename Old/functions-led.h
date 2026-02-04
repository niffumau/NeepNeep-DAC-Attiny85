#ifndef FUNCTIONS_LED_H
#define FUNCTIONS_LED_H


void led_setup(void);
void led_toggle(uint8_t _PIN);
void led_on(int _PIN);
void led_off(int _PIN);
void led_blink(int _PIN,int _number) ;
void led_status(int _first,int _second);


#endif