/* Four Sample Player

   David Johnson-Davies - www.technoblogy.com - 21st January 2020
   ATtiny85 @ 8 MHz (internal oscillator; BOD disabled)
      
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

#include <Arduino.h>
/*#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <avr/interrupt.h>*/


#include <avr/sleep.h>

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
const int sck = 2, miso = 1, mosi = 0, cs = 3;

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
  while (Read() & 1 != 0);
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

ISR(TIM1_COMPA_vect) {
  PINB ^= (1 << 4);  // Toggle PB4 (pin 4) - fast & glitch-free
}

void setup() {
  // 64MHz PLL for Timer1
  PLLCSR = (1<<PCKE) | (1<<PLLE);
  while (!(PLLCSR & (1<<PLOCK)));

  // Timer1 CTC: OCR1A=32k-1 (64MHz/1024 / 2 / 32k = ~1kHz toggle)
  TCCR1 = (1<<CTC1) | (5<<CS10);  // CTC mode, /1024 prescale
  OCR1A = 31250 - 1;              // ~0.5ms half-period (1kHz full)

  TIMSK |= (1<<OCIE1A);           // Enable compare interrupt

  ADCSRA &= ~(1<<ADEN);           // Disable ADC
  pinMode(speaker, OUTPUT);
  digitalWrite(speaker, LOW);     // Start low

  sei();  // Global interrupts
}

void loop() {
  // Empty - ISR handles tone
}