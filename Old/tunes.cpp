////// Includes //////////
#include "Arduino.h"

#include "main.h"
#include "tunes.h"
#include "functions.h"
#include "functions-led.h"
//#include "functions-sleep.h"


const uint8_t divisors[] = {142,134,127,120,113,106,100,95,89,84,79,75,71,67,63,59,56,53,50,47,44,42,39,37,35,33,31,29,27,26,24,23,22,20,19,18};




const uint8_t tune_nokia_new[] PROGMEM = {
  (uint8_t) 13,
  NOTE_6E,2, NOTE_6D,2, NOTE_5FS,4, NOTE_5GS,2,
  NOTE_6CS,2, NOTE_5B,2, NOTE_5D,4, NOTE_5E,2,
  NOTE_5B,2, NOTE_5A,2, NOTE_5CS,4, NOTE_5E,4, NOTE_5A,4
};


const uint8_t tune_sms_new[] PROGMEM = {
  (uint8_t) 8,
  NOTE_5A,2, NOTE_5A,2, NOTE_5A,2,
  NOTE_5A,6, NOTE_5A,6, 
  NOTE_5A,2, NOTE_5A,2, NOTE_5A,2
};

const uint8_t tune_iphone_new[] PROGMEM = {
  (uint8_t) 26,
	NOTE_4G,4, NOTE_4G,2, NOTE_4AS,2, NOTE_5C,2,
	NOTE_5C,1, NOTE_4AS,1, NOTE_4G,2, 
	NOTE_5C,2, NOTE_4G,2, 
	NOTE_5C,2, NOTE_4A,2, NOTE_5C,2, NOTE_4F,8,

  NOTE_4G,4, NOTE_4G,2, NOTE_4AS,2, NOTE_5C,2,
	NOTE_5C,1, NOTE_4AS,1, NOTE_4G,2, 
	NOTE_5C,2, NOTE_4G,2, 
	NOTE_5C,2, NOTE_4A,2, NOTE_5C,2, NOTE_4F,8,
};


/////////////// testing stuff //////////////////////////////
const notes_t tune_newd[] PROGMEM = {
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,3},
  {NOTE_5A,3},
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,1}
};

const tune_t tune_new PROGMEM = {
  .size = 8,
  .notes = { {NOTE_5A,1}, {NOTE_5A,1}, {NOTE_5A,1}, {NOTE_5A,1},}
};

const uint8_t tune_nokia_rep[] PROGMEM = {
  (uint8_t) 13,
  NOTE_6E,1, NOTE_6D,1, NOTE_5FS,2, NOTE_5GS,1,
  NOTE_6CS,1, NOTE_5B,1, NOTE_5D,2, NOTE_5E,1,
  NOTE_5B,1, NOTE_5A,1, NOTE_5CS,2, NOTE_5E,2, NOTE_5A,2

};



/*
const tune_t tune_new2 = {
  .size = 8,
  .notes = tune_newd
};*/


////////////////////////////////////////////////////////////////////




#ifndef IS_BUZZER		
/***************************************************
 *  play_note
 ***************************************************
 *   I am not really sure how this is going to pan out
 *   I'll start by trying to work out what the frequency of the note is
 *   https://pages.mtu.edu/~suits/NoteFreqCalcs.html
 */
	// 9.6MHz internal oscilator... lol, no, i think this is 1.2MHz for the ATTiny13a

void play_note(uint8_t _note, uint8_t _duration) {
	uint8_t _prescaler;
	uint8_t divisor;
	
	/*
	if (_note < 0)  {
		led_status(4,2);
	}
	if (_note > 83)  {
		led_status(4,3);
	}*/

	if (_note <  24) {		// Work out the prescaler
		_prescaler = N_256;
		divisor = divisors[_note];
	} else if (_note < 60 ) {
		_prescaler = N_64;
		divisor = divisors[_note-24];
	} else {
		_prescaler = N_8;
		divisor = divisors[_note-60];

	}

	TCCR0B = (TCCR0B & ~((1<<CS02)|(1<<CS01)|(1<<CS00))) | _prescaler;

	OCR0A = divisor;		// set count
	OCR0B = divisor/2;		// set count for duty, so duty = OCR0B/OCR0A
	TCCR0A |= _BV(COM0B0);

	_mydelay(_duration);
	TCCR0B &= ~((1<<CS02)|(1<<CS01)|(1<<CS00)); // stop the timer
	digitalWrite(BUZZER_PIN, LOW); // set the output to low
	_mydelay(_duration);

}



/***************************************************
 *  playtune_scale
 ***************************************************
 *  
 *   
 */
void playtune_scale(void) {

	for (uint8_t octave=3; octave <= 5; octave++) {
		led_status(1,octave);
		for	(uint8_t i=0; i < 12; i++) {
			uint8_t note = octave*12+i;
			play_note(note,2);
		}
		_mydelay(4);
	}
}

#endif

/***************************************************
 *  playtune_melody
 ***************************************************
 *  This should play a tune 
 *   
 */

void playtune_melody(const notes_t *melody,uint8_t _size) {
  //uint8_t _length = pgm_read_byte(&melody[0]);
	//led_status(4,_length);  return;   // debug

	for	(int i=0; i <_size; i++) {
		play_note(pgm_read_byte(&melody[i].note),pgm_read_byte(&melody[i].duration));	
	}
}

/*void playtune_melody_new(const notes_t *melody,uint8_t _size) {
    playtune_melody_new(&melody);
}*/

void playtune_melody_new(const uint8_t *melody) {
	uint8_t _length = pgm_read_byte(&melody[0]);
	//led_status(4,_length);  return;   // debug  
	for	(int i=0; i <_length; i++) {
		play_note(pgm_read_byte(&melody[1+2*i]),pgm_read_byte(&melody[2+2*i]));	
	}
}

