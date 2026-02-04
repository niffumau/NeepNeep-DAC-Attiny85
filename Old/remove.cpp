
/*
uint16_t _random_NEW( uint16_t _min, uint16_t _max) {
	pinMode(PIN_RANDOM, INPUT);
	uint8_t Rand1 = analogRead((analog_pin_t) PIN_RANDOM);
	uint8_t Rand2 = analogRead((analog_pin_t) PIN_RANDOM);
	uint16_t _return =  Rand1 + (Rand2<<8);

guessagain:
	//remainder = dividend % divisor;
	uint16_t _modulus = (_max - _min);
	_return =   lfsr16_next(random_number);
	_return =   _return % _modulus;
	_return += _min;

	if (_return < _min) {
		led_blink(LED_RED,4);
		goto guessagain;
	}
	if (_return > _max){
		led_blink(LED_RED,8);
		goto guessagain;
	}
	return _return;
}
*/


/***************************************************
 *  play_frequency
 ***************************************************
 *   Play a particular frequency for a duration
 * 
 * 
 * 	
	*
	*  now based on the prescaler, we calculate the count...
	* 
	* so, lets say 440hz.  at 440hz, 
	* Processor clock: 9.6MHz
	* prescaler of N_64
	* so clock becomes 150kHz.  
	* 
	*
 *   
 *   
 */
/*
void play_frequency(uint16_t freq_hz, int _duration) {

	uint8_t _prescaler,prescaler_number;
	uint8_t divisor;

	// Math routines
	// 1200000L converts to (long)1200000


	if (freq_hz <  64) {		// Work out the prescaler
		_prescaler = N_256;
		//prescaler_number = 256;		// maybe i have to calculate the divisor here?
		divisor = 1200000/(256 * freq_hz );
	} else if (freq_hz < 500 ) {
		_prescaler = N_64;
		//prescaler_number = 64;
		divisor = 1200000/(64 * freq_hz );
	} else {
		_prescaler = N_8;
		//prescaler_number = 8;
		divisor = 1200000/(8 * freq_hz );
	}

	
	TCCR0B = (TCCR0B & ~((1<<CS02)|(1<<CS01)|(1<<CS00))) | _prescaler;
	OCR0A = divisor;		// set count
	OCR0B = divisor/2;		// set count for duty, so duty = OCR0B/OCR0A
	TCCR0A |= _BV(COM0B0);

	_mydelay(_duration);
	
	TCCR0B &= ~((1<<CS02)|(1<<CS01)|(1<<CS00)); // stop the timer
	_mydelay(_duration);	

}
*/
/*
static void _tonenew(uint8_t _divisor, uint8_t _prescaler,uint16_t _delay)
{
	// So, TCCR0B is TCCR0B AND...
	// Setting the prescaler...
	// so we take TCCR0B as it currently stands, I think the ~ makes it zero the prescaler bits first
	// then we OR it with what prescaler bits we want
	// so set prescaler then set the count...

	TCCR0B = (TCCR0B & ~((1<<CS02)|(1<<CS01)|(1<<CS00))) | _prescaler;

	OCR0A = _divisor;		// set count
	OCR0B = _divisor/2;		// set count for duty, so duty = OCR0B/OCR0A
	TCCR0A |= _BV(COM0B0);

	_mydelay(_delay);
	TCCR0B &= ~((1<<CS02)|(1<<CS01)|(1<<CS00)); // stop the timer
	_mydelay(_delay);
}*/


/*
 * I think that in future, the following should be reduced into maybe an array of notes and duration.. so
 * NOTE_C5,1,NOTE_G3,2
 * 
*/

/*
void playtune_happybirthday(void){
	_tonenew(95, N_8,1);		//G 5
	_tonenew(95, N_8,1);		//G 5
	_tonenew(84, N_8,2);		//A 5
	_tonenew(95, N_8,2);		//G 5
	_tonenew(71, N_8,1);		//C 6
	_tonenew(75, N_8,2);		//B 5
	_mydelay(4);
	_tonenew(95, N_8,1);		//G 5
	_tonenew(95, N_8,1);		//G 5
	_tonenew(84, N_8,2);		//A 5
	_tonenew(95, N_8,2);		//G 5
	_tonenew(63, N_8,2);		//D 6
	_tonenew(71, N_8,1);		//C 6
	_mydelay(4);
	_tonenew(95, N_8,1);		//G 5
	_tonenew(95, N_8,1);		//G 5
	_tonenew(47, N_8,1);		//G 6
	_tonenew(56, N_8,1);		//E 6
	_tonenew(71, N_8,1);		//C 6
	_tonenew(75, N_8,1);		//B 5
	_tonenew(84, N_8,1);		//A 5
	_mydelay(2);
	_tonenew(53, N_8,1);		//F	6
	_tonenew(53, N_8,1);		//F 6
	_tonenew(56, N_8,1);		//E 6
	_tonenew(71, N_8,1);		//C 6
	_tonenew(63, N_8,2);		//D 6
	_tonenew(71, N_8,1);		//C 6
}
*/
/*
void playtune_nokia(void){
	// E D f# g# 
	// c# B D E
	// B A c# E A
	_tonenew(56, N_8,1);		//E 6
	_tonenew(63, N_8,1);		//D 6
	_tonenew(100, N_8,2);		//FS 5
	_tonenew( 89, N_8,1);		//GS 5

	_tonenew( 67, N_8,1);		//CS 6
	_tonenew( 75, N_8,1);		//B  5
	_tonenew(127, N_8,2);		//D	 5
	_tonenew(113, N_8,1);		//E	 5

	_tonenew( 75, N_8,1);		//B  5
	_tonenew( 84, N_8,1);		//A  5
	_tonenew(134, N_8,2);		//CS 5
	_tonenew(113, N_8,2);		//E	 5
	_tonenew( 84, N_8,2);		//A  5

}
*/




//{ // octave 0
	//.note_C = {142, N_256}, // 16.35 Hz
	//.note_CS = {134, N_256}, // 17.32 Hz
/*
notes_t tune_happybirthday[25] = {
	{.note = NOTE_5G, .duration = 1},
	{.note = NOTE_5G, .duration = 1},
	{.note = NOTE_5A, .duration = 1},
	{.note = NOTE_5G, .duration = 1},
	{.note = NOTE_6C, .duration = 1},
	{.note = NOTE_5B, .duration = 1}
};*/


/**/

/*
uint8_t tune_scale_notes[7] = {NOTE_4C,NOTE_4D,NOTE_4E,NOTE_4F,NOTE_4G,NOTE_4A,NOTE_4B};

uint8_t tune_scale_notes_duration[14] = {NOTE_4C,1,NOTE_4D,1,NOTE_4E,1,NOTE_4F,1,NOTE_4G,1,NOTE_4A,NOTE_4B};

const notes_t tune_scale_0[] PROGMEM = {
  {NOTE_0C,1},
  {NOTE_0CS,1},
  {NOTE_0D,1},
  {NOTE_0DS,1},
  {NOTE_0E,1},
  {NOTE_0F,1},
  {NOTE_0FS,1},
  {NOTE_0G,1},
  {NOTE_0GS,1},
  {NOTE_0A,1},
  {NOTE_0AS,1},
  {NOTE_0B,1}
};

const notes_t tune_scale_3[12] PROGMEM = {
  {NOTE_3C,2},
  {NOTE_3CS,2},
  {NOTE_3D,2},
  {NOTE_3DS,2},
  {NOTE_3E,2},
  {NOTE_3F,2},
  {NOTE_3FS,2},
  {NOTE_3G,2},
  {NOTE_3GS,2},
  {NOTE_3A,2},
  {NOTE_3AS,2},
  {NOTE_3B,2}
};

const notes_t tune_scale_4[12] PROGMEM = {
  {NOTE_4C,2},
  {NOTE_4CS,2},
  {NOTE_4D,2},
  {NOTE_4DS,2},
  {NOTE_4E,2},
  {NOTE_4F,2},
  {NOTE_4FS,2},
  {NOTE_4G,2},
  {NOTE_4GS,2},
  {NOTE_4A,2},
  {NOTE_4AS,2},
  {NOTE_4B,2}
};
*/

/*
const notes_t tune_nokia[] PROGMEM = {
  {NOTE_6E,1},
  {NOTE_6D,1},
  {NOTE_5FS,2},
  {NOTE_5GS,1},

  {NOTE_6CS,1},
  {NOTE_5B,1},
  {NOTE_5D,2},
  {NOTE_5E,1},

  {NOTE_5B,1},
  {NOTE_5A,1},
  {NOTE_5CS,2},
  {NOTE_5E,2},
  {NOTE_5A,2}

};

const notes_t tune_sms[] PROGMEM = {
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,3},
  {NOTE_5A,3},
  {NOTE_5A,1},
  {NOTE_5A,1},
  {NOTE_5A,1}
};


const notes_t tune_test[] PROGMEM = {
  {1,1},
  {2,1},
  {3,2},
  {4,2},{5,2},{6,2},{7,2},{8,2},{9,2}
};

const notes_t tune_happybirthday[] PROGMEM = {
	{NOTE_5G,1},
	{NOTE_5G,1},
	{NOTE_5A,1},
	{NOTE_5G,1},
	{NOTE_6C,1},
	{NOTE_5B,4},

	{NOTE_5G,1},
	{NOTE_5G,1},
	{NOTE_5A,1},
	{NOTE_5G,1},
	{NOTE_6D,1},
	{NOTE_6C,4},

	{NOTE_5G,1},
	{NOTE_5G,1},
	{NOTE_6G,1},
	{NOTE_6E,1},
	{NOTE_6C,1},
	{NOTE_5B,1},
	{NOTE_5A,1},

	{NOTE_6F,1},
	{NOTE_6F,1},
	{NOTE_6E,1},
	{NOTE_6C,1},
	{NOTE_6D,1},
	{NOTE_6C,1}
	
};
*/