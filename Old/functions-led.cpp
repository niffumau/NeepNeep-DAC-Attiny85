
#include <Arduino.h>
#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

#include "main.h"
#include "functions-led.h"


/*******************************************************************************************************************************
 *  LED Functions
 *******************************************************************************************************************************/

/***************************************************
 *  led_setup
 ***************************************************
 * 
 */
void led_setup(void){
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  //DDRB |= (1 << LED_PIN);	// set as output
  //DDRB |= (1 << LED_PIN2);	// set as output
  PORTB = 0b00000000;	// Set all of port b to 0
  //digitalWrite(LED_PIN, LOW); 
  //digitalWrite(LED_PIN2, LOW); 

}
/***************************************************
 *  led_toggle
 ***************************************************
 * 
 */
void led_toggle(uint8_t _PIN){
  //PORTB ^= _BV(LED_PIN2);  // toggle LED pin		//_BV returns the byte value of some shit
  PORTB ^= _BV(_PIN);  // toggle LED pin		//_BV returns the byte value of some shit
  //digitalWrite(LED_PIN2, HIGH); 

}
/***************************************************
 *  led_on
 ***************************************************
 * 
 */
void led_on(int _PIN){
	digitalWrite(_PIN, HIGH); 
	//DDRB |= (1 << _PIN);		// these result in the same shit

}

/***************************************************
 *  led_off
 ***************************************************
 * 
 */
void led_off(int _PIN){
  	digitalWrite(_PIN, LOW); 
	//DDRB &= ~(0 << _PIN);
}
void led_blink(int _PIN,int _number) {
	for (uint8_t i=0;i < _number;i++ ){ 
		//PORTB ^= _BV(_PIN);
		digitalWrite(_PIN, HIGH);
		_delay_ms(BLINK_INTERVAL_ON);
		digitalWrite(_PIN, LOW);
		_delay_ms(BLINK_INTERVAL_OFF);
	}
	_delay_ms(BLINK_INTERVAL_END);
}

void led_status(int _first,int _second) {
	led_blink(LED_RED,_first);
	led_blink(LED_GREEN,_second);
	_delay_ms(BLINK_INTERVAL_STATUS_END);
}