/* Four Sample Player

   David Johnson-Davies - www.technoblogy.com - 21st January 2020
   ATtiny85 @ 8 MHz (internal oscillator; BOD disabled)
      
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>  // For Watchdog Timer
#include <stdlib.h>   // For rand() / srand()
#include <util/delay.h>  // _delay_us()

#include <EEPROM.h>

#include "main.h"
#include "functions.h"


// ATtiny85 pins used for dataflash
//const int sck = 2, miso = 1, mosi = 0, cs = 3;
//const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
const int speaker = PIN_SPEAKER;


// Audio player **********************************************


volatile boolean StayAwake = true;
volatile int Play;
volatile uint32_t Count;

uint32_t Samples[MAX_SIZES];
uint8_t Num_Samples = 0;

//uint32_t Sizes[5] = { 0, 2486, 5380, 10291, 1415837 };

/*uint32_t Sizes[5] = { 0,        // Chunk 1: first 2s
                      4000,    // Chunk 2: seconds 3-4  
                      8000,    // Chunk 3: seconds 5-6
                      16000,    // Chunk 4: seconds 7-8
                      32000 };      // End (unused)*/


/* samples

Samples play at 8kHz, so 8,000 per second, so the first one 2486/8000

Sample	Samples	Duration
1	2486	0.31s 
​2	2894	0.36s 
​3	4911	0.61s 
​4	1,405,546	175.69s 
​
Processing come_get_some_x.wav: start byte 0, length 11732 bytes
Processing cough_x.wav: start byte 11732, length 8594 bytes
Processing dentist_drill.wav: start byte 20326, length 17376 bytes
Processing disconnect_11.wav: start byte 37702, length 7424 bytes
All samples concatenated into output.bin (total size: 176 bytes)


*/
//uint32_t Sizes[8] = { 0,11732,19156,49464,71274,116518,140082,190308 };
//uint32_t Sizes[10] = { 0,11732,19156,25646,33200,63508,85318,130562,154126,204352 };
//uint32_t Sizes[11] = { 0,11732,23464,30888,37378,44932,74186,95996,141240,164804,215030 };
//uint32_t Sizes[9] = { 0,11732,19156,25646,33200,62454,84264,107828,158054 };


/*******************************************************************************************************************************
 *  Flash Functions
 *******************************************************************************************************************************/

class DF {
  public:
    boolean Setup();
    void BeginRead(uint32_t addr);
    void BeginWrite(void);
    uint8_t ReadByte(void);
    void WriteByte(uint8_t data);
    void EndRead(void);
    void EndWrite(void);
    void PowerDown(boolean);
  private:
    unsigned long addr;
    uint8_t Read(void);
    void Write(uint8_t);
    void Busy(void);
    void WriteEnable(void);
};

boolean DF::Setup () {
  uint8_t manID, devID;
  pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); 
  pinMode(sck, OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  digitalWrite(sck, LOW); digitalWrite(mosi, HIGH);
  delay(1);
  digitalWrite(cs, LOW);
  delay(100);
  Write(READID); Write(0);Write(0);Write(0);
  manID = Read();
  (void)manID;
  devID = Read();
  digitalWrite(cs, HIGH);
  return (devID == 0x15); // Found correct device
}

void DF::Write (uint8_t data) {
  uint8_t bit = 0x80;
  while (bit) {
    if (data & bit) PORTB = PORTB | (1<<mosi);
    else PORTB = PORTB & ~(1<<mosi);
    PINB = 1<<sck;                        // sck high
    bit = bit>>1;
    PINB = 1<<sck;                        // sck low
  }
}

void DF::Busy () {
  digitalWrite(cs, 0);
  Write(READSTATUS);
  while ((Read() & 1) != 0);
  digitalWrite(cs, 1);
}

void DF::WriteEnable () {
  digitalWrite(cs, 0);
  Write(WRITEENABLE);
  digitalWrite(cs, 1);
}

void DF::PowerDown (boolean on) {
  digitalWrite(cs, 0);
  if (on) Write(POWERDOWN); else Write(RELEASEPD);
  digitalWrite(cs, 1);
}

void DF::BeginRead (uint32_t start) {
  addr = start;
  digitalWrite(cs, 0);
  Write(READDATA);
  Write(addr>>16);
  Write(addr>>8);
  Write(addr);
}

uint8_t DF::Read () {
  uint8_t data = 0;
  uint8_t bit = 0x80;
  while (bit) {
    PINB = 1<<sck;                        // sck high
    if (PINB & 1<<miso) data = data | bit;
    PINB = 1<<sck;                        // sck low
    bit = bit>>1;
  }
  return data;
}

void DF::EndRead(void) {
  digitalWrite(cs, 1);
}

void DF::BeginWrite () {
  addr = 0;
  // Erase DataFlash
  WriteEnable();
  digitalWrite(cs, 0);
  Write(CHIPERASE);
  digitalWrite(cs, 1);
  Busy();
}

uint8_t DF::ReadByte () {
  return Read();
}

void DF::WriteByte (uint8_t data) {
  // New page
  if ((addr & 0xFF) == 0) {
    digitalWrite(cs, 1);
    Busy();
    WriteEnable();
    digitalWrite(cs, 0);
    Write(PAGEPROG);
    Write(addr>>16);
    Write(addr>>8);
    Write(0);
  }
  Write(data);
  addr++;
}

void DF::EndWrite (void) {
  digitalWrite(cs, 1);
  Busy();
}

DF DataFlash;



/***************************************************
 *  
 ***************************************************
 * 
 */
void load_sizes_from_flash(void) {
  DataFlash.BeginRead(0);
  Num_Samples = 0;

  for (uint8_t i = 0; i < MAX_SIZES; i++) {
    uint32_t val = 0;
    
    // Read 4 bytes in REVERSE order for little-endian → AVR
    uint8_t bytes[4];
    for (uint8_t b = 0; b < 4; b++) {
      bytes[b] = DataFlash.ReadByte();
    }
    
    // Little-endian: LSB first, so reverse for AVR uint32_t
    val  = bytes[0];        // LSB
    val |= ((uint32_t)bytes[1]) <<  8;
    val |= ((uint32_t)bytes[2]) << 16;
    val |= ((uint32_t)bytes[3]) << 24;  // MSB
    
    


    // Stop at first null terminator (after offsets, before the two 0x00000000)
    if (val == 0 && i > 0) {
      //Num_Samples = i;
      break;
    }
    Samples[i] = val;


    Num_Samples = i;

    /*playTestTone_ms_freq(500, 440);  // 100ms @ 1kHz
    _delay_us(400);*/
  }
  
  DataFlash.EndRead();

 #ifdef DEBUG_FORCE_SIZES
  // === Then: overwrite with your debug values ===
  uint32_t debug_vals[] = {
    196, 27338, 63606, 75060, 92802, 115046, 138256, 146316, 158442,
    172248, 189816, 208394, 220126, 227096, 242004, 254264, 259750,
    345004, 354518, 366666, 383824, 401550, 416186, 425624, 435236,
    448960, 466180, 477884, 486934, 507158, 527144, 543352, 557400,
    595286, 612084, 626184, 647766, 660112, 668866, 682380, 697734,
    710468, 722408, 749754, 761374, 772730, 786400
  };

  Num_Samples = sizeof(debug_vals) / sizeof(debug_vals[0]);
  Num_Samples = 46;
  for (uint8_t i = 0; i < num_sizes; i++) {
    Samples[i] = debug_vals[i];
  }
  #endif

  //Num_Samples = 46;

  // now the debug to beep the number of times for the number of samples we have
  /*for(uint8_t i = 0; i < Num_Samples; i++) {
    
    playTestTone_ms_freq(100, 440);  // 100ms @ 1kHz
    _delay_ms(100);
  }*/


}



/*******************************************************************************************************************************
 *  ISR Routines
 *******************************************************************************************************************************/

/***************************************************
 *  ISR (TIMER0_COMPA_vect)
 ***************************************************
 * 
 */
ISR (TIMER0_COMPA_vect) {
  char sample = DataFlash.ReadByte();
  OCR1B = sample;
  // This is suppose to kinda normalise it but i'm not convinced it was a good idea.
  /*int8_t signed_sample = (int8_t)DataFlash.ReadByte() - 128;  // -128 to +127
  OCR1B = (uint8_t)(signed_sample + 128);  // 0 to 255, centered 128 avg*/

  if (--Count == 0) {
    DataFlash.EndRead();
    TIMSK = 0;
    StayAwake = false;
  }
}

/***************************************************
 *  Watchdog ISR
 ***************************************************
 * Not sure why this is required???
 */
// Watchdog ISR - just wake up, no action needed
/*ISR(WDT_vect) { }*/

volatile uint32_t wdt_jitter_seed = 0;

ISR(WDT_vect) {
  //wdt_jitter_seed = TCNT0 ^ TCNT1;  // Capture timer state on WDT wake
}

void setup_random_seed() {
  // Enable WDT for 16ms, let it fire once
  wdt_reset();
  WDTCR = (1<<WDCE) | (1<<WDE);
  WDTCR = (1<<WDIE) | (1<<WDP0);  // 16ms interrupt
  
  _delay_ms(20);  // Wait for WDT ISR (uses constant!)
  
  wdt_disable();
  
  uint32_t seed = wdt_jitter_seed ^ TCNT0 ^ PINB;
  //randomSeed(seed);
  srand(seed);
}



//volatile uint32_t wdt_jitter_seed = 0;
/*
static uint32_t rand_state = 12345;  // Arbitrary initial, changes per boot due to timing
uint8_t get_next_sample(uint8_t max_samples) {
  rand_state ^= rand_state << 13;
  rand_state ^= rand_state >> 17;
  rand_state ^= rand_state << 5;
  return ((rand_state % max_samples) + 1);
}

static uint32_t prng_state = 0xACE1;  // Boot-varies from fuse/regs*/

/*******************************************************************************************************************************
 *  Play Random Sample
 ********************************************************************************************************************************
 * Plays a random sample
 */
void play_random_sample() {
  /*// SETUP: SPI + speaker for playback
  pinMode(sck, OUTPUT);    digitalWrite(sck, LOW);
  pinMode(mosi, OUTPUT);   digitalWrite(mosi, HIGH);
  pinMode(miso, INPUT);
  pinMode(cs, OUTPUT);     digitalWrite(cs, HIGH);
  pinMode(speaker, OUTPUT);*/
  pinMode(speaker, OUTPUT);
  DataFlash.Setup();
  DataFlash.PowerDown(false);
  StayAwake = true;

/*
TCNT1 = 0;  // Safe: Timer1 PWM running, just reads counter
_delay_us(23);  // Tiny wait for jitter (calibrated safe)
uint32_t seed = TCNT1 ^ PINB ^ 0xDEADBEEF;  // XOR SPI + constant
randomSeed(seed);*/

  // Pick random sample (1-4)
  //Play = rand() % 4 + 1;
  //Play = 1;   // force play to 1

  //uint8_t num_samples = sizeof(Sizes) / sizeof(Sizes[0]) - 1;  // 4 for your 5-element array
  //Play = rand() % Num_Samples + 1;  // Picks 1-4 uniformly

// this wone worked but apparenlty it is broken for low numbers?
  Play = rand() % Num_Samples + 1;  // Picks 1-4 uniformly
  
  //Play = ((uint16_t)rand() >> 8) % num_sizes + 1; // apparnelty this fixes the problem with random?

  // this did not work
  /*uint8_t num_samples = num_sizes;
  Play = get_next_sample(num_samples);*/

/*
prng_state *= 1103515245;
prng_state += 12345;
prng_state ^= (prng_state >> 16);
uint8_t num_samples = num_sizes;
Play = ((prng_state ^ PINB) % num_samples) + 1;*/

  

  #if defined(DEBUG_FIXED_WAV)
  Play = DEBUG_FIXED_WAV;
  #endif

  StayAwake = true;
  Count = Samples[Play] - Samples[Play-1];

  DataFlash.BeginRead(Samples[Play-1]);
  //TIMSK = 1<<OCIE0A;              // Enable compare match
  TIMSK |= _BV(OCIE0A);  // Enable (OR, don't overwrite other bits)

  // ADD: Timeout safety (ISR not firing? Escape after ~1s)
  uint16_t timeout = 8000;  // ~1s @8kHz
  while (StayAwake && timeout--) {
    _delay_ms(1);  // Yield CPU
  }
  if (timeout == 0) {
    TIMSK = 0;  // Force stop
    // Optional: blink LED or skip sleep
    playTestTone_ms_freq(50, 440);
  }


  //while (StayAwake);              // Wait for playback end
  //DataFlash.EndRead();  // ADD: Always end (safety) // dont' need this as its part of the function..

  DataFlash.PowerDown(true);

  //playTestTone();

  // Sleep ~10s using 2x WDT cycles (8s + 2s)
  // Setup pins as inputs to avoid drain during sleep
  pinMode(mosi, INPUT_PULLUP);
  pinMode(miso, INPUT_PULLUP);
  pinMode(sck, INPUT_PULLUP);
  pinMode(speaker, INPUT);        // Avoid click

  // unsure about this
  //GIFR = 1<<PCIF;                         // Clear flag
  //GIMSK = 1<<PCIE;                        // Enable interrupts

}



/*******************************************************************************************************************************
 *  Setup
 *******************************************************************************************************************************/
void setup() {

  // Start Timer0 ASAP for entropy
/*  TCCR0B = _BV(CS01);   // clk/8
  _delay_ms(5);         // let it run a little*/


  //srand(getSeed());
  //srand(jitterSeed()); 

/*
  // fuck knows
  uint16_t seed = 0;
  for (int i = 0; i < 32; i++) {
    delay(1);
    seed = (seed << 1) ^ TCNT0;
  }
  srand(seed);
  */




/*
  uint32_t seed;
  EEPROM.get(0, seed);

  seed ^= TCNT0;
  seed ^= TCNT1;
  seed ^= PINB;
  seed ^= (uint32_t)micros() << 16;

  EEPROM.put(0, seed);
  srand(seed);
  */


  DataFlash.Setup();
  DataFlash.PowerDown(false);
  load_sizes_from_flash();

 
  


  PLLCSR = 1<<PCKE | 1<<PLLE;       // Enable 64 MHz PLL and use as source for Timer1

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                        // Timer interrupts OFF
  TCCR1 = 1<<CS10;                  // 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;     // PWM B, clear on match
  OCR1B = 128;                      // 50% duty at start

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;                // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;      // 1/8 prescale
  OCR0A = 124;                      // Divide by 1000

  ADCSRA = ADCSRA & ~(1<<ADEN);     // Disable ADC to save power

/*
  // ************* The original Seed **************************
  // Seed random with ADC noise (pin 5/PB2 floating or unconnected)
  ADCSRA |= (1<<ADEN);              // Temp enable ADC
  //randomSeed(analogRead(PB2));        // Read PB2 (pin 5) PB1 PB2 PB3 PB4 PB5
  srand(analogRead(PB2));           // although i think that it shoudl be srand
  ADCSRA = ADCSRA & ~(1<<ADEN);     // Disable ADC*/


/*  
  // An EEPROM Version
  uint16_t seed;
  eeprom_read_block(&seed, 0, 2);
  seed ^= TCNT0;
  seed ^= ADC;
  eeprom_write_block(&seed, 0, 2);
  srand(seed);*/

  // based on a temprature sensor?
  srand(getSeed());




  //randomSeed(get_random_seed());
  //setup_random_seed();
  // Replace your ADC code with this:
  /*uint32_t seed = TCNT0 ^ TCNT1 ^ PINB ^ GPIOR0;
  randomSeed(seed);*/
  //setup_random_seed();

  // it said to add this to the thing, not sure if it helps???
  TCCR0B |= 2<<CS00;              


  #if defined(DEBUG_TONE)
  playTestTone_ms_freq(50, 440);
  #endif

/*
  // ADC seed - precise bits only
  ADMUX = _BV(REFS0) | 0b11110;
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADCSRA |= _BV(ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  uint8_t adc_noise = ADC;
  ADCSRA &= ~_BV(ADEN);
  randomSeed(adc_noise ^ TCNT1 ^ PINB ^ GPIOR0);*/

  /*uint32_t seed = TCNT0 ^ TCNT1 ^ PINB ^ GPIOR0;
  randomSeed(seed);*/

  //setup_random_seed();
/*  _delay_ms(1);  // Let noise/timers settle
  uint32_t seed = TCNT0 ^ TCNT1 ^ PINB ^ GPIOR0 ^ (ADC >> 4);  // Include ADC mux bits
  randomSeed(seed);*/

/*
  uint32_t seed;
  EEPROM.get(0, seed);

  // Mix real entropy
  seed ^= TCNT0;
  seed ^= (TCNT0 << 8);
  seed ^= PINB;

  // Advance seed so EEPROM never repeats
  seed = seed * 1103515245UL + 12345UL;

  EEPROM.put(0, seed);
  srand(seed);
  randomSeed(seed);
  rand();   // throw away the first random number after seeding (AVR quirk apparently)
*/

  // now the debug to beep the number of times for the number of samples we have
  /*for(uint8_t i = 0; i < Num_Samples; i++) {
    
    playTestTone_ms_freq(500, 440);  // 100ms @ 1kHz
    _delay_us(400);
  }*/






}


/*******************************************************************************************************************************
 *  Main Loop
 *******************************************************************************************************************************/
void loop() {
 

  //playTestTone_ms(10);   // 100ms (original)
  //playTestTone_ms_freq(50, 440);   // 250ms @ A4 (440Hz)


  while (true) {


    //playTestTone_ms(100);
    //playTestTone_ms_freq(100, 1000);  // 100ms @ 1kHz
    //playTestTone_ms_freq(50, 440);   // 250ms @ A4 (440Hz)
    //playTestTone();
    play_random_sample();
  
    


/*
    // Watchdog for first 8s sleep
    wdt_reset();
    WDTCR = (1<<WDCE) | (1<<WDE);   // Unlock
    WDTCR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);  // 8.0s interrupt
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    wdt_disable();

    // Second WDT for ~2s sleep
    wdt_reset();
    WDTCR = (1<<WDCE) | (1<<WDE);
    WDTCR = (1<<WDIE) | (1<<WDP2);  // 2.0s interrupt
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    wdt_disable();*/

  // Random delay 10min-4hr (75-1800 cycles of 8s)
  // 4 hours is 1800
  // 2 hours is 900
  // 1 hour is 450
  uint16_t time_max = 450; 
  uint16_t time_min = 75;

  uint16_t cycles = (rand() % (time_max - time_min + 1)) + time_min;

  #if defined(DEBUG_FIXED_8S) 
  cycles = 1;
  #endif

  for(uint16_t i = 0; i < cycles; i++) {
    // Your 8s WDT sleep
    wdt_reset();
    WDTCR = (1<<WDCE) | (1<<WDE);
    WDTCR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);  // 8s
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    wdt_disable();
  }    

    // Ready for next iteration
  }
}
