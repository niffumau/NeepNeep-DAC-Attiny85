/* 
 * Plays random samples from Flash

 */

/**
 * @file
 * @brief ATTiny13A/85 audio player: Plays random raw audio samples from Winbond W25Q SPI DataFlash.
 * 
 * Loads sample offsets from flash (little-endian uint32_t array), selects random sample, streams via
 * 8kHz ISR (Timer0) → 250kHz PWM carrier (Timer1 PLL64x). Sleeps via 8s WDT between plays.
 * Supports 16/32Mbit flashes (ID 0x15/0x16). Power-optimized with PWR_DOWN and pin tri-stating.
 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>  // For Watchdog Timer
#include <stdlib.h>   // For rand() / srand()
#include <util/delay.h>  // _delay_us()


#include "main.h"
#include "sleep.h"
#include "functions.h"



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

/**
 * @brief Winbond W25Qxx SPI DataFlash driver class for ATTiny (bit-banged SPI).
 * 
 * Supports READID (0x9F), READDATA (0x03), PAGEPROG (0x02), CHIPERASE (0xC7), power/sleep modes.
 * Bit-banged on PB0-3 (MOSI/SCK/MISO/CS). Handles Busy polling, 24-bit addressing.
 * Designed for raw 8-bit mono samples @8kHz.
 */
class DF {
  public:
    /**
     * @brief Initializes SPI pins and verifies flash ID (Winbond 0xEF + 16/32Mbit 0x15/0x16).
     * @return true if valid chip detected, false otherwise (triggers warning_alarm).
     */  
    boolean Setup();

    /**
     * @brief JEDEC READID (0x9F) variant—primary init method.
     * @return true if manID=0xEF && devID=0x15/16.
     */
    boolean Setup_READID();

    /**
     * @brief Alternative 0x9F JEDEC ID read (memType check)—less reliable on retries.
     * @return true if manID=0xEF && memType=0x40 && devID=0x15/16.
     */
    boolean Setup_9F();

    /**
     * @brief Starts sequential read at 24-bit address (wakes chip first).
     * @param addr 24-bit flash address (0-16/32Mbit).
     */    
    void BeginRead(uint32_t addr);

    /**
     * @brief Starts page-program write session + CHIPERASE.
     */    
    void BeginWrite(void);

    /**
     * @brief Reads 1 SPI byte during active read.
     * @return Byte from MISO (assembly-nop timed).
     */
    uint8_t ReadByte(void);

    /**
     * @brief Writes 1 SPI byte during active write (auto page boundary handling).
     * @param data Byte to write (addr++ internally).
     */
    void WriteByte(uint8_t data);

    /**
     * @brief Ends read session (CS high).
     */
    void EndRead(void);

    /**
     * @brief Ends write + polls BUSY (status bit 0).
     */
    void EndWrite(void);

    /**
     * @brief Power-down (deep sleep, <1µA) or wake (3µs tRES1).
     * @param on true=powerdown, false=wake.
     */
    void PowerDown(boolean);

    /**
     * @brief Enters deep sleep (alias for PowerDown(true)).
     */
    void Sleep (void);

    /**
     * @brief Wakes from deep sleep (alias for PowerDown(false)).
     */
    void Wake (void);

    /**
     * @brief Polls BUSY status (retries 50x, alarms on timeout).
     */
    void Busy(void);
    //uint8_t safeReadByte();

  private:
    unsigned long addr;   ///< Current 24-bit address.

    /**
     * @brief Bit-banged SPI byte transmit (MSB first, PBx direct).
     * @param data Byte to MOSI.
     */
    uint8_t Read(void);

    /**
     * @brief Issues Write Enable (0x06) latch.
     */
    void Write(uint8_t);

    //void Busy(void);
    void WriteEnable(void);
};

/************************************************************************************//**
 * @brief Sets up the flash chip thing
 * 
 * 
 */
/*boolean DF::Setup_OLD () {
  uint8_t manID, devID;   // Manufacturer ID and Device ID
  boolean _return = true;
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);    
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  digitalWrite(PIN_SCK, LOW); digitalWrite(PIN_MOSI, HIGH);
  delay(1);
  digitalWrite(PIN_CS, LOW);
  delay(100);
  Write(READID); Write(0);Write(0);Write(0);
  manID = Read();
  (void)manID;        // This tells the compiler to implicity ignore this value
  //Read();  // Dummy byte 1
  //Read();  // Dummy byte 2  

  if (manID != 0xEF) {
    _return = false;
    warning_alarm(4);
  }

  devID = Read();
  //  W25Q16 → 0x15
  //  W25Q32 → 0x16
  digitalWrite(PIN_CS, HIGH);
  
  //if (devID != 0x15) while(1) warning_alarm(5);
  if (devID != 0x15 && devID != 0x16) {
    _return = false;
    warning_alarm(5);
  }


  //return (devID != 0x15 && devID != 0x16); // Found correct device Returns True if the deviceID is 0x15
  return _return;
}*/

boolean DF::Setup() {
  return Setup_READID();
  //return Setup_9F();
}

boolean DF::Setup_READID() {
  uint8_t manID, devID;
  boolean _return = true;
  pinMode(PIN_CS, OUTPUT); 
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, HIGH);
  //pinMode(PIN_MISO, INPUT);  // Or INPUT_PULLUP if noisy
  pinMode(PIN_MISO, INPUT_PULLUP);
  _delay_ms(10);  // Power stable delay
  
  digitalWrite(PIN_CS, LOW);
  _delay_us(10);  // CS setup time
  
  Write(READID);    // 0x9F JEDEC ID
  Write(0); Write(0); Write(0);  // 3 dummy addr bytes
  
  manID = Read();   // EFh (Winbond)
  devID = Read();   // 15h (16Mbit) or 16h (32Mbit)
  
  digitalWrite(PIN_CS, HIGH);
  
  
  if (manID != 0xEF) {    // this is often wrong after the first time.. so ignore it?
    //warning_alarm(4);
    _return = false;
  }

  if (devID != 0x15 && devID != 0x16) {
    //warning_alarm(2);
    _return = false;
  }
  
  return _return;
}



// This is another way of doing it but it doens't work the 2nd or 3rd time around
boolean DF::Setup_9F() {
  uint8_t manID, memType, devID;
  boolean _return = true;
  pinMode(PIN_CS, OUTPUT); 
  _delay_us(10);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, HIGH);
  //pinMode(PIN_MISO, INPUT);  // Or INPUT_PULLUP if noisy
  pinMode(PIN_MISO, INPUT_PULLUP);
  _delay_ms(10);  // Power stable delay


  digitalWrite(PIN_CS, LOW);
  _delay_us(10);  // CS setup time

  Write(0x9F);    // 0x9F JEDEC ID

  manID   = Read();   // should be 0xEF
  memType = Read();   // should be 0x40
  devID   = Read();   // 15h (16Mbit) or 16h (32Mbit) should be 0x16 for W25Q32JV
  
  _delay_us(10);
  digitalWrite(PIN_CS, HIGH);
  
  
  // often manID is not returned as 0xEF for some reason???
  if (manID != 0xEF) {  // often the first byte, this one is returned a
    //warning_alarm(4);
    _return = false;
  }
  if (memType != 0x40) {
    //warning_alarm(10);
    _return = false;
  }
  if (devID != 0x15 && devID != 0x16) {
    warning_alarm(5);
    _return = false;
  }
  
  return _return;
}



/***************************************************
 *  DF:Write
 ***************************************************
 * 
 */

void DF::Write (uint8_t data) {
  uint8_t bit = 0x80;
  while (bit) {
    if (data & bit) PORTB = PORTB | (1<<PIN_MOSI);
    else PORTB = PORTB & ~(1<<PIN_MOSI);
    PINB = 1<<PIN_SCK;                        // sck high
    bit = bit>>1;
    PINB = 1<<PIN_SCK;                        // sck low
  }
}


/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::Busy() {
  uint8_t cnt=50;
  digitalWrite(PIN_CS, LOW);
  Write(READSTATUS);
  while((Read() & 1) && cnt--);  // Exit if stuck
  if (cnt==0) {
    while(1) warning_alarm(6);
  }
  digitalWrite(PIN_CS, HIGH);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::WriteEnable () {
  digitalWrite(PIN_CS, 0);
  Write(WRITEENABLE);
  digitalWrite(PIN_CS, 1);
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::PowerDown (boolean on) {
  digitalWrite(PIN_CS, LOW);
  if (on) {   // Power Down the chip
    Write(POWERDOWN);
    digitalWrite(PIN_CS, HIGH);
  } else {   // Wake up the chip
    Write(RELEASEPD);
    digitalWrite(PIN_CS, HIGH);
    _delay_us(5);   // tRES1 AFTER CS high
  }
  
  //if (!on) {
    
  //}
}

/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::Sleep (void) {
  digitalWrite(PIN_CS, LOW);  
  Write(POWERDOWN);
  digitalWrite(PIN_CS, HIGH);
}  
/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::Wake (void) {
  digitalWrite(PIN_CS, LOW);
  Write(RELEASEPD);
  digitalWrite(PIN_CS, HIGH);
  _delay_us(5);   // tRES1 AFTER CS high
  
}  



/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::BeginRead (uint32_t start) {
  Wake();     // make sure it is awake first...

  addr = start;
  digitalWrite(PIN_CS, 0);
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
uint8_t DF::Read() {
  uint8_t data = 0;
  uint8_t bit = 0x80;
  while(bit) {
    PINB = 1<<PIN_SCK;                    // SCK ↑
    asm volatile("nop\nnop\nnop");    // 3 cycles delay (~0.4µs @8MHz)
    if(PINB & 1<<PIN_MISO) data |= bit;   // Sample stable
    PINB = 1<<PIN_SCK;                    // SCK ↓
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
  digitalWrite(PIN_CS, 1);
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
  digitalWrite(PIN_CS, 0);
  Write(CHIPERASE);
  digitalWrite(PIN_CS, 1);
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

/*
uint8_t DF::safeReadByte() {
  uint8_t cnt=100;
  uint8_t data = ReadByte();
  while((data = 0xFF || data == 0x00) && cnt--)
  {
    _delay_us(10);
    data = ReadByte();;
  }
  return data;
}*/


/***************************************************
 *  
 ***************************************************
 * 
 */
void DF::WriteByte (uint8_t data) {
  // New page
  if ((addr & 0xFF) == 0) {
    digitalWrite(PIN_CS, 1);
    Busy();
    WriteEnable();
    digitalWrite(PIN_CS, 0);
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
  digitalWrite(PIN_CS, 1);
  Busy();
}

// Global instance
DF DataFlash;



/***************************************************
 *  
 ***************************************************
 * 
 */

/**
 * @brief Loads sample offsets table from flash addr 0 (null-terminated uint32_t little-endian).
 * 
 * Reverse-byte swaps for AVR big-endian. DEBUG_FORCE_SIZES overrides with hardcoded table.
 * Beeps Num_Samples on load (commented).
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

/**
 * @brief 8kHz sample streamer ISR (Timer0 COMP A).
 * 
 * Reads flash byte → OCR1B PWM duty. Ends playback at Count=0 (Sleeps flash, disables ISR).
 * Safety: Infinite alarm if Count > MAX_SAFE_SAMPLES.
 */
ISR (TIMER0_COMPA_vect) {

  // Check that we are not over the number of maximum safe samples
  // it should NOT end up here
  if (Count > MAX_SAFE_SAMPLES) {
    while (1) {
      warning_alarm(8);
    }
  }

  // Maybe check that data read is ok ? 


  char sample = DataFlash.ReadByte();   // Read the sample
  OCR1B = sample;                       // Put the sample in the duty cycle thing
  // This is suppose to kinda normalise it but i'm not convinced it was a good idea.
  //int8_t signed_sample = (int8_t)DataFlash.ReadByte() - 128;  // -128 to +127
  //OCR1B = (uint8_t)(signed_sample + 128);  // 0 to 255, centered 128 avg
  //wdt_reset();

  // Apparently one of these two shoudl work, possibly the second one breaks the code? //
  if (Count == 0) {
    DataFlash.EndRead();
    DataFlash.Sleep();  // DataFlash.PowerDown(true);
  
    TIMSK &= ~(1<<OCIE0A);
    StayAwake = false;
    return;
  }
  Count--;

  /*if (--Count == 0) {
    DataFlash.EndRead();
    TIMSK = 0;
    StayAwake = false;
  }*/


}


/***************************************************
 *  Watchdog ISR
 ***************************************************
 * Watchdog ISR - just wake up, no action needed
 */

volatile bool wdt_alarm = false;

/**
 * @brief WDT wake ISR (8s)—just resets (handled by sleep_function).
 */
ISR(WDT_vect) { 
  /*while(1) {
    warning_alarm(4);
  }*/
}

/*******************************************************************************************************************************
 *  Play Random Sample
 ********************************************************************************************************************************
 * Plays a random sample
 */

/**
 * @brief Core playback: Random sample → 250kHz PWM (Timer1 PLL64x) modulated @8kHz ISR.
 * 
 * Truncates >MAX_SAFE_SAMPLES. 1s timeout safety (alarms on stall). Tri-states pins post-play.
 * DEBUG_FIXED_WAV forces specific sample.
 */
void play_random_sample() {
  pinMode(PIN_SPEAKER, OUTPUT);

  /*while (!DataFlash.Setup()) {
    //playTestTone_ms_freq(20, 440);
    _delay_us(20);

  }*/
  DataFlash.Setup();

  DataFlash.Wake(); // DataFlash.PowerDown(false);

  /// Create Random Number ///
  // this wone worked but apparenlty it is broken for low numbers?
  Play = random() % Num_Samples + 1;  // Picks 1-4 uniformly
  //Play = ((uint16_t)rand() >> 8) % num_sizes + 1; // apparnelty this fixes the problem with random?

  #if defined(DEBUG_FIXED_WAV)      // Force a fixed wave file play
  Play = DEBUG_FIXED_WAV;
  #endif

  StayAwake = true;
  Count = Samples[Play] - Samples[Play-1];

  if(Count > MAX_SAFE_SAMPLES) {      // Check if the sample is longer than the maximum number of samples?
    Count = MAX_SAFE_SAMPLES;         // If so, trunkate teh sample to the maximum length
    playTestTone_ms_freq(20, 440);    // Brief "warning" beep
  }

  PLLCSR = 1<<PCKE | 1<<PLLE;       // Enable 64 MHz PLL and use as source for Timer1

  // Set up Timer/Counter1 for PWM output
  TIMSK &= ~(1<<OCIE0A);            // Timer Interrupts Off
  TCCR1 = 1<<CS10;                  // 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;     // PWM B, clear on match
  OCR1B = 128;                      // 50% duty at start
  OCR1C = 255;                      // 250kHz carrier, which is the default

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;                // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;      // 1/8 prescale
  OCR0A = 124;                      // Divide by 1000

  pinMode(PIN_SPEAKER, OUTPUT);

  

  DataFlash.BeginRead(Samples[Play-1]);

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
  //wdt_disable();  // After EndRead() //////////////////////////////////////////////////////////////////////////////////////////////
  TIMSK &= ~(1<<OCIE0A);

  
  if (timeout == 0) {
    //TIMSK = 0;  // Force stop /// this was already done above...
    // Optional: blink LED or skip sleep, we don't have an LED
    //playTestTone_ms_freq(50, 440);
    while (1) warning_alarm(7);
  }
  



  // Sleep ~10s using 2x WDT cycles (8s + 2s)
  // Setup pins as inputs to avoid drain during sleep
  pinMode(PIN_MOSI, INPUT_PULLUP);
  pinMode(PIN_MISO, INPUT_PULLUP);
  pinMode(PIN_SCK, INPUT_PULLUP);
  pinMode(PIN_SPEAKER, INPUT);        // Avoid click

  //20260208 enabled the below two lines just incase they work
  // unsure about this
  GIFR = 1<<PCIF;                         // Clear flag
  GIMSK = 1<<PCIE;                        // Enable interrupts

}

#if defined(WIPE_ATTINY)
/** @brief Permanent dead sleep mode: Tri-states pins, PWR_DOWN forever (power-cycle wake). */
void setup() {
  // Set SPI pins high-Z input (no drive)
  pinMode(PIN_MOSI, INPUT);    // PB0 - no pullup
  pinMode(PIN_MISO, INPUT);    // PB1 - no pullup  
  pinMode(PIN_SCK, INPUT);     // PB2 - no pullup
  pinMode(PIN_CS, INPUT);      // PB3? - no pullup
  
  // Disable all modules
  ADCSRA = 0;  // ADC off
  PRR = 0xFF;  // Power-down everything possible
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Sleep forever, wake=power cycle
}

void loop() {}  // Never reached

#else

/*******************************************************************************************************************************
 *  Setup
 *******************************************************************************************************************************/

/**
 * @brief One-time init: Flash setup, load offsets, timers/PWM/PLL/WDT, random seed.
 */
void setup() {
  DataFlash.Setup();
  DataFlash.Wake(); // DataFlash.PowerDown(false);
  load_sizes_from_flash();
  //DataFlash.PowerDown(true);

  PLLCSR = 1<<PCKE | 1<<PLLE;       // Enable 64 MHz PLL and use as source for Timer1

  // Set up Timer/Counter1 for PWM output
  //TIMSK = 0;                        // Timer interrupts OFF
  TIMSK &= ~(1<<OCIE0A);
  TCCR1 = 1<<CS10;                  // 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;     // PWM B, clear on match
  OCR1B = 128;                      // 50% duty at start

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;                // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;      // 1/8 prescale
  OCR0A = 124;                      // Divide by 1000

  ADCSRA = ADCSRA & ~(1<<ADEN);     // Disable ADC to save power

  // it said to add this to the thing, not sure if it helps???
  TCCR0B |= 2<<CS00;              


  #if defined(DEBUG_TONE)
  playTestTone_ms_freq(50, 440);
  #endif

  //warning_alarm(2);


  _delay_ms(1);  // Let noise/timers settle
  uint32_t seed = TCNT0 ^ TCNT1 ^ PINB ^ GPIOR0 ^ (ADC >> 4);  // Include ADC mux bits
  randomSeed(seed);

  wdt_disable();  // Clean slate
  WDTCR = (1<<WDCE)|(1<<WDE);
  WDTCR = (1<<WDIE)|(1<<WDE)|(1<<WDP3)|(1<<WDP0);  // 8s interrupt+reset
  // No more changes needed
  wdt_enable(WDTO_8S);  

}


/*******************************************************************************************************************************
 *  Main Loop
 *******************************************************************************************************************************/

/**
 * @brief Infinite loop: play_random_sample() → sleep_function() → repeat.
 * 
 * WDT reset each iteration. DEBUG_TONE replaces with test tone.
 */
void loop() {
  wdt_reset();

  // Debug thing to play a sound every 8 times in the loop
/*  if (count_loop <= 0){
    count_loop=8;
    playTestTone_ms_freq(50, 440);
  } else --count_loop;*/


  //while (true) {

    #if defined(DEBUG_TONE)
    playTestTone_ms_freq(50, 440);
    #else
    play_random_sample();
    #endif




    sleep_function();
    
   
    // Ready for next iteration
  //}
}
#endif
