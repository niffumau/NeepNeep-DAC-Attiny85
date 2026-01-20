////// Includes //////////
#include "Arduino.h"

//#include <avr/io.h>
//#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

//#include <avr/interrupt.h>

//#include <avr/power.h>
//#include <avr/sleep.h>


#include "main.h"
#include "functions.h"
#include "functions-led.h"
#include "functions-sleep.h"




/*******************************************************************************************************************************
 *  Delay Routine
 *******************************************************************************************************************************/

void _mydelay(uint8_t _delay) {
	for (uint8_t i=0;i < _delay;i++ ){ 
		_delay_ms(40);
		//delay(40);
	}

}
void _mydelay_new(void) {

}


/*******************************************************************************************************************************
 *  Random Number Generator that probably doesn't work cunt
 *******************************************************************************************************************************/

/***************************************************
 * Random Number Generator 
 ***************************************************
 * 
 */

static uint16_t random_number = 0;
static uint16_t lfsr16_next(uint16_t n) {
    return (n >> 0x01U) ^ (-(n & 0x01U) & 0xB400U);    
}

void random_init(void) {		// fuck the seed?
	pinMode(PIN_RANDOM, INPUT);
	uint8_t Rand1 = analogRead((analog_pin_t) PIN_RANDOM);
	uint8_t Rand2 = analogRead((analog_pin_t) PIN_RANDOM);
	//random_number = Rand1 + (Rand1<<8);
	random_number = Rand1 + (Rand2<<8);
}

uint16_t _random( uint16_t _min, uint16_t _max) {
	random_init();	// Re-initialize the random number seed every time
	uint16_t _return=0;
guessagain:;
	//remainder = dividend % divisor;
	uint16_t _modulus = (_max - _min);
	_return =   lfsr16_next(random_number);
	_return =   _return % _modulus;
	_return += _min;

	if (_return < _min) {
		//led_blink(LED_RED,4);
		goto guessagain;
	}
	if (_return > _max){
		//led_blink(LED_RED,8);
		goto guessagain;
	}

	//led_status(3,_return);			// debug for when it doesn't look random anymore

	return _return;
}


/***************************************************
 * check_random 
 ***************************************************
 * Check that the random number is between our limits
 */
void check_random(uint16_t _return) {

	if (_return < RANDOM_SLEEP_MIN*8) {
		//led_blink(LED_RED,4);
		led_status(4,2);
	}
	if (_return > RANDOM_SLEEP_MAX*8 + 2){
		//led_blink(LED_GREEN,4);
		led_status(4,4);
	}
}






