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


// Winbond DataFlash commands
#define PAGEPROG      0x02
#define READSTATUS    0x05
#define READDATA      0x03
#define WRITEENABLE   0x06
#define CHIPERASE     0x60
#define READID        0x90
#define POWERDOWN     0xB9
#define RELEASEPD     0xAB

// ATtiny85 pins used for dataflash
//const int sck = 2, miso = 1, mosi = 0, cs = 3;
//const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
const int sck = PB2, miso = PB1, mosi = PB0, cs = PB3;
const int speaker = PB4;


// Audio player **********************************************


volatile boolean StayAwake = true;
volatile int Play;
volatile uint32_t Count;
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
uint32_t Sizes[10] = { 0,11732,19156,25646,33200,63508,85318,130562,154126,204352 };

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




ISR (TIMER0_COMPA_vect) {
  /*static uint8_t dbg=0;
  PINB ^= 1<<PB3;  // Toggle PB3 (pin2) as scope probe*/

  char sample = DataFlash.ReadByte();
  OCR1B = sample;
  Count--;
  if (Count == 0) {
    DataFlash.EndRead();
    TIMSK = 0;
    StayAwake = false;
  }
}

// Watchdog ISR - just wake up, no action needed
ISR(WDT_vect) { }

// Add before setup() - include <util/delay.h> at top
void playTestTone() {
  pinMode(speaker, OUTPUT);
  
  // 1kHz PWM on PB4: 64MHz PLL /512 = ~125kHz → OCR1C=125 (1kHz)
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));
  
  GTCCR = (1<<COM1B0) | (1<<PWM1B);
  TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512 prescale
  OCR1C = 125;  // TOP=1kHz
  OCR1B = 62;   // 50% duty
  
  // 100ms delay (calibrated @64MHz PLL)
  for(volatile uint16_t i=0; i<8333; i++) { _delay_us(12); }
  
  // Stop
  TCCR1 = 0;
  GTCCR = 0;
  OCR1B = 0;
  PINB |= (1<<4);
  pinMode(speaker, INPUT);        // Avoid click
}

// Parameterized version: playTestTone_ms(500);  // 500ms
void playTestTone_ms(uint16_t ms) {
  pinMode(speaker, OUTPUT);
  
  // 1kHz PWM setup (unchanged)
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));
  
  GTCCR = (1<<COM1B0) | (1<<PWM1B);
  TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512
  OCR1C = 125;  // 1kHz TOP
  OCR1B = 62;   // 50% duty
  
  // Play for exact ms (1kHz cycles * ms)
  uint32_t cycles = (uint32_t)ms * 1000;  // ms → total cycles @1kHz
  for(volatile uint32_t i = 0; i < cycles; i++) {
    _delay_us(1);  // 1ms per 1000 loops (precise)
  }
  
  // Stop (unchanged)
  TCCR1 = 0;
  GTCCR = 0;
  OCR1B = 0;
  PINB |= (1<<4);
  pinMode(speaker, INPUT);
}

// Full control: playTestTone_ms_freq(100, 1000);  // 100ms @1kHz
void playTestTone_ms_freq(uint16_t ms, uint16_t freq_hz) {
  pinMode(speaker, OUTPUT);
  
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while(!(PLLCSR & (1<<PLOCK)));
  
  uint16_t top = 64000 / freq_hz;
  if (top > 255) top = 255;
  
  GTCCR = (1<<COM1B0) | (1<<PWM1B);
  TCCR1 = (1<<CS13) | (1<<CS11) | (1<<CS10);  // /512
  OCR1C = top;
  OCR1B = top / 2;
  
  // FIXED: cycles = ms * 1000 (microseconds total)
  uint32_t total_us = (uint32_t)ms * 1000;
  for(volatile uint32_t i = 0; i < total_us; i++) {
    _delay_us(1);  // Simple µs counter
  }
  
  // Stop
  TCCR1 = 0; GTCCR = 0; OCR1B = 0;
  PINB |= (1<<4);
  pinMode(speaker, INPUT);
}
/***************************************************************/
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

  // Pick random sample (1-4)
  //Play = rand() % 4 + 1;
  //Play = 1;   // force play to 1
  uint8_t num_samples = sizeof(Sizes) / sizeof(Sizes[0]) - 1;  // 4 for your 5-element array
  Play = rand() % num_samples + 1;  // Picks 1-4 uniformly


  StayAwake = true;
  Count = Sizes[Play] - Sizes[Play-1];

  DataFlash.BeginRead(Sizes[Play-1]);
  TIMSK = 1<<OCIE0A;              // Enable compare match

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

void setup() {



  // Enable 64 MHz PLL and use as source for Timer1
  PLLCSR = 1<<PCKE | 1<<PLLE;

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

  // Seed random with ADC noise (pin 5/PB2 floating or unconnected)
  ADCSRA |= (1<<ADEN);              // Temp enable ADC
  randomSeed(analogRead(2));        // Read PB2 (pin 5)
  ADCSRA = ADCSRA & ~(1<<ADEN);     // Disable ADC

  TCCR0B |= 2<<CS00;              // it said to add this to the thing
}



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
  uint16_t cycles = (rand() % (1800 - 75 + 1)) + 75;
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
