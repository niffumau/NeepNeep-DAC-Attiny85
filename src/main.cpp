/* 
 * Plays random samples from Flash

 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>  // For Watchdog Timer
#include <stdlib.h>   // For rand() / srand()
#include <util/delay.h>  // _delay_us()

//#include <EEPROM.h>

#include "main.h"
#include "functions.h"


// ATtiny85 pins used for dataflash
//const int sck = 2, miso = 1, mosi = 0, cs = 3;
//const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
//const int speaker = PIN_SPEAKER;


// Audio player **********************************************


volatile boolean StayAwake = true;
volatile int Play;
volatile uint32_t Count;

uint32_t Samples[MAX_SIZES];
uint8_t Num_Samples = 0;


/*******************************************************************************************************************************
 *  Flash Functions
 *******************************************************************************************************************************/

/***************************************************
 *  
 ***************************************************
 * 
 */
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
    void Busy(void);
  private:
    unsigned long addr;
    uint8_t Read(void);
    void Write(uint8_t);
    //void Busy(void);
    void WriteEnable(void);
};

/***************************************************
 *  
 ***************************************************
 * 
 */
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

/***************************************************
 *  DF:Write
 ***************************************************
 * 
 */

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


/***************************************************
 *  
 ***************************************************
 * 
 */
/*
void DF::Busy () {
  digitalWrite(cs, 0);
  Write(READSTATUS);
  while ((Read() & 1) != 0);
  digitalWrite(cs, 1);
}*/
// apparenlty the one above coudl be problmenatic
/*void DF::Busy() {
  digitalWrite(cs, LOW);
  Write(READSTATUS);
  uint8_t cnt=100; while((Read()&1) && cnt--);
  digitalWrite(cs, HIGH);
}*/
/*void DF::Busy() {
  digitalWrite(cs, LOW);
  Write(READSTATUS);
  uint8_t cnt=100;
  while((Read()&1) && cnt--) {}  // Escape if stuck
  digitalWrite(cs, HIGH);
}*/

void DF::Busy() {
  uint8_t cnt=50;
  digitalWrite(cs, LOW);
  Write(READSTATUS);
  while((Read() & 1) && cnt--);  // Exit if stuck
  digitalWrite(cs, HIGH);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::WriteEnable () {
  digitalWrite(cs, 0);
  Write(WRITEENABLE);
  digitalWrite(cs, 1);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::PowerDown (boolean on) {
  digitalWrite(cs, 0);
  if (on) Write(POWERDOWN); else Write(RELEASEPD);
  digitalWrite(cs, 1);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::BeginRead (uint32_t start) {
  addr = start;
  digitalWrite(cs, 0);
  Write(READDATA);
  Write(addr>>16);
  Write(addr>>8);
  Write(addr);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
/*uint8_t DF::Read () {
  uint8_t data = 0;
  uint8_t bit = 0x80;
  while (bit) {
    PINB = 1<<sck;                        // sck high
    if (PINB & 1<<miso) data = data | bit;
    PINB = 1<<sck;                        // sck low
    bit = bit>>1;
  }
  return data;
}*/
/*uint8_t DF::Read() {
  uint8_t data = 0;
  uint8_t bit = 0x80;
  while(bit) {
    PINB = 1<<sck;             // SCK ↑
    _delay_us(0.5);              // ✅ Hold/setup time
    if(PINB & 1<<miso) data |= bit;  // Sample stable rising edge
    PINB = 1<<sck;             // SCK ↓
    bit >>= 1;
  }
  return data;
}*/
uint8_t DF::Read() {
  uint8_t data = 0;
  uint8_t bit = 0x80;
  while(bit) {
    PINB = 1<<sck;                    // SCK ↑
    asm volatile("nop\nnop\nnop");    // 3 cycles delay (~0.4µs @8MHz)
    if(PINB & 1<<miso) data |= bit;   // Sample stable
    PINB = 1<<sck;                    // SCK ↓
    bit >>= 1;
  }
  return data;
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::EndRead(void) {
  digitalWrite(cs, 1);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::BeginWrite () {
  addr = 0;
  // Erase DataFlash
  WriteEnable();
  digitalWrite(cs, 0);
  Write(CHIPERASE);
  digitalWrite(cs, 1);
  Busy();
}

/***************************************************
 *  
 ***************************************************
 * 
 */
uint8_t DF::ReadByte () {
  return Read();
}

/***************************************************
 *  
 ***************************************************
 * 
 */
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

/***************************************************
 *  
 ***************************************************
 * 
 */
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
  //int8_t signed_sample = (int8_t)DataFlash.ReadByte() - 128;  // -128 to +127
  //OCR1B = (uint8_t)(signed_sample + 128);  // 0 to 255, centered 128 avg
  wdt_reset();

  if (--Count == 0) {
    DataFlash.EndRead();
    TIMSK = 0;
    StayAwake = false;
  }
  if (Count > MAX_SAFE_SAMPLES) {
    while (1) {
    playTestTone_ms_freq(100, 1000);
    playTestTone_ms_freq(100, 800);
    playTestTone_ms_freq(100,500);
    }
  }
}


/***************************************************
 *  Watchdog ISR
 ***************************************************
 * Not sure why this is required???
 */
// Watchdog ISR - just wake up, no action needed
ISR(WDT_vect) { 
   while (1) {
    playTestTone_ms_freq(100, 1000);
    playTestTone_ms_freq(100,500);
    }
}

/*******************************************************************************************************************************
 *  Play Random Sample
 ********************************************************************************************************************************
 * Plays a random sample
 */
void play_random_sample() {


  pinMode(PIN_SPEAKER, OUTPUT);

  

  wdt_enable(WDTO_120MS);  // Short: forces reset if ISR stalls >120ms
  wdt_reset();

  DataFlash.Setup();
  DataFlash.PowerDown(false);
  StayAwake = true;

// this wone worked but apparenlty it is broken for low numbers?
  Play = random() % Num_Samples + 1;  // Picks 1-4 uniformly
  
  //Play = ((uint16_t)rand() >> 8) % num_sizes + 1; // apparnelty this fixes the problem with random?

  #if defined(DEBUG_FIXED_WAV)
  Play = DEBUG_FIXED_WAV;
  #endif

  StayAwake = true;
  Count = Samples[Play] - Samples[Play-1];

  if(Count > MAX_SAFE_SAMPLES) {
    Count = MAX_SAFE_SAMPLES;  // Truncate long samples
    playTestTone_ms_freq(20, 200);  // Brief "warning" beep
  }


  PLLCSR = 1<<PCKE | 1<<PLLE;       // Enable 64 MHz PLL and use as source for Timer1

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                        // Timer interrupts OFF
  TCCR1 = 1<<CS10;                  // 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;     // PWM B, clear on match
  OCR1B = 128;                      // 50% duty at start
  OCR1C = 255;                      // 250kHz carrier, which is the default

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
//  TCCR0A = 3<<WGM00;                // Fast PWM
//  TCCR0B = 1<<WGM02 | 2<<CS00;      // 1/8 prescale
//  OCR0A = 124;                      // Divide by 1000

  pinMode(PIN_SPEAKER, OUTPUT);

  DataFlash.BeginRead(Samples[Play-1]);

 // DataFlash.Busy();  // Ensure ready before ISR

  TIMSK = 1<<OCIE0A;              // Enable compare match
  //TIMSK |= _BV(OCIE0A);  // Enable (OR, don't overwrite other bits)

  //wdt_reset();
  // ADD: Timeout safety (ISR not firing? Escape after ~1s)
  uint16_t timeout = 8000;  // ~1s @8kHz
  while (StayAwake && timeout--) {
    //wdt_reset();    // for every ms ///added this becuase I wasnt sure ///////////////////////////////////maybe its getting stuck here?
    _delay_ms(1);  // Yield CPU
  }

  // apparently i shoudl add these?
  DataFlash.EndRead();  // Ensure CS high
  //wdt_disable();  // After EndRead() //////////////////////////////////////////////////////////////////////////////////////////////
  TIMSK = 0;  // Kill ISR

  
  if (timeout == 0) {
    //TIMSK = 0;  // Force stop /// this was already done above...
    // Optional: blink LED or skip sleep, we don't have an LED
    playTestTone_ms_freq(50, 440);
  }
  
  //while (StayAwake);              // Wait for playback end
  //DataFlash.EndRead();  // ADD: Always end (safety) // dont' need this as its part of the function..

  DataFlash.PowerDown(true);

  //playTestTone();

  // Sleep ~10s using 2x WDT cycles (8s + 2s)
  // Setup pins as inputs to avoid drain during sleep
  pinMode(PIN_MOSI, INPUT_PULLUP);
  pinMode(PIN_MISO, INPUT_PULLUP);
  pinMode(PIN_SCK, INPUT_PULLUP);
  pinMode(PIN_SPEAKER, INPUT);        // Avoid click

  // unsure about this
  //GIFR = 1<<PCIF;                         // Clear flag
  //GIMSK = 1<<PCIE;                        // Enable interrupts

}


/*******************************************************************************************************************************
 *  Setup
 *******************************************************************************************************************************/
void setup() {

  // I think that this breaks it for now?
  //wdt_enable(WDTO_120MS);  // Short: forces reset if ISR stalls >120ms
  //wdt_reset();

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
  randomSeed(analogRead(PB2));        // Read PB2 (pin 5) PB1 PB2 PB3 PB4 PB5
  //srand(analogRead(PB2));           // although i think that it shoudl be srand
  ADCSRA = ADCSRA & ~(1<<ADEN);     // Disable ADC*/

  // it said to add this to the thing, not sure if it helps???
  TCCR0B |= 2<<CS00;              


  #if defined(DEBUG_TONE)
  playTestTone_ms_freq(50, 440);
  #endif

  warning_alarm(2);


  _delay_ms(1);  // Let noise/timers settle
  uint32_t seed = TCNT0 ^ TCNT1 ^ PINB ^ GPIOR0 ^ (ADC >> 4);  // Include ADC mux bits
  randomSeed(seed);


}


/*******************************************************************************************************************************
 *  Main Loop
 *******************************************************************************************************************************/
void loop() {

  while (true) {

    #if defined(DEBUG_TONE)
    playTestTone_ms_freq(50, 440);
    #else
    play_random_sample();
    #endif


    #if !defined(DEBUG_NO_DELAY)
    uint16_t time_max = TIME_MAX; 
    uint16_t time_min = TIME_MIN;
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
    #endif
    // Ready for next iteration
  }
}
