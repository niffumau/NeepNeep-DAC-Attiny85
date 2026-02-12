# NeepNeep DAC  ATtiny85

[[_TOC_]]


# Future Thoughts

The price of the ATtiny85 is like $2.50+, to bring the cost down i should look at moving to another chip. some of the contenderse are:
  - ATtiny412

## ATtiny412

things to consider/remmber/note:
  - Uses UPDI to repogram


| Parameter | ATtiny85 | ATtiny412 |
| --------- | -------- | --------- |
| Flash     | 8 KB     | 4 KB      |
| SRAM      | 512 B    | 256 B     |
| EEPROM    | 512 B    | 128 B     |


Peripherals and I/O
ATtiny85
  * Two 8‑bit timers, basic 4‑channel 10‑bit ADC, hardware SPI + TWI (I²C).
  * Very “oldest‑generation AVR‑Tiny”‑style; peripherals vary more between old‑series parts.

ATtiny412
  * Two 16‑bit timers, 6‑channel 10‑bit ADC, USART → hardware UART, USI‑style SPI, and I²C in the same 8‑pin SOIC.
  * Implements USART with TX/RX pins (unlike ATtiny85), which is especially handy for debugging or extra serial I/O.


# General

Test update 

This is based on the ATtiny85 and the W25Q32 flash chip.

Just so I remember the markup for this readme...
This is <mark>highlighted</mark> like in Dokuwiki.
Or use <code>gray highlight</code> for code-like emphasis.

This was an example I strated from:
https://www.hackster.io/news/building-a-simple-audio-player-with-an-attiny85-a6b6bf635c21

This is the page: that it came from apparently:
http://www.technoblogy.com/show?2XJD

I have tried to convert this all to USI but it conflicts with my ISR routine I think... it didn't go well.

# Hardware
Essentiall this is a ATtiny85 with a flash chip.  The PCB design is 
  - https://github.com/niffumau/NeepNeep-KiCAD

I am a little concerned that i'm programming the ATtiny85 with 5V rail, apparenlty can program it with a 3.3v programmer which is probaly what i shoudl be looking into.  Yes, i'm pretty sure that its not really tollerant of 5V so ive moved to using a USBASP that is 3.3v.

ATtiny85:
  - 8 KB flash

W25Q32:
  - I am using the  W25Q32JVSS variant  
  - Capacity: 32 Mbit / 4 MByte, organized in 16,384 pages of 256 bytes each.
  - Voltage: 2.7–3.6V operation, <1 µA power-down current.
  - Interface: SPI (standard, Dual/Quad I/O), up to 133 MHz clock (66 MB/s read).
  - Erase: Uniform 4 KB sectors, 32/64 KB blocks; >100K program/erase cycles, 20+ year retention.
  - Temp Range: -40°C to +85°C (fits Perth's climate for outdoor IoT/DIY).

Speaker is driven by a MMBT3904 which grounds the -ve side of the speaker (via the C of the transistor) with the postiive lead going to the 3.3v rail (2032 watch battery)

The ATtiny85 I use
  - ATTINY85-20SU (LCSC C31540447) $2.50 - Current chip $3.40 on JLCPCB
  - ATtiny412 (LCSC C1338345) $1.14 - Possible Repacement $1.58 on JLCPCB

The Flash chip
  - W25Q32JVSSIQ (LCSC C179173)  $1.40  - Current chip  #2.14 on JLCPCB
  - W25Q80 (LCSC C14086) $0.93     - Possibly a cheaper replacement $1.29 on JLCPCB

The speaker i generally use is:
  - YS-SBZ9032C03R16 (LCSC C409828) $0.34

The transistor thing TRANS NPN 40V 200mA SOT-23
  - MMBT3904(RANGE:100-300) (LCSC C20526)

Should probably change out the transistor for a mosfet as i think it drives the speaker harder, this one should be a drop in replacement:
  -  BSS138  

I removed the diodes and it seemed to program fine but i did have some issues programming the flash on the one I removed the diodes from, not entirely sure but maybe this will be a problem in the future with the boards that don't have the diodes.

![NeepNeep-DAC-V2](NeepNeep-DAC-V2.png)


The pins i use for the Flash chip are the standard pins for programming the ATTiny along with the chip select pin:
```c
  #define PIN_MOSI PB0
  #define PIN_MISO PB1
  #define PIN_SCK PB2
  #define PIN_CS PB3
```

the pin for the speaker i use is:
```c
  #define PIN_SPEAKER   PB4
```

# Pinouts

## Attiny85 
| Pin # | Function   | Programming Header | W25Q32 | Other   |
|:-----:|:-----------|-------------------:|-------:|:-------:|
|   1   | PB5/Reset  | Reset (5)          |        |         |
|   2   | PB3        |                    | CS (1) |         |
|   3   | PB4        |                    |        | Speaker |
|   4   | Ground     |                    |        | Ground  |
|   5   | AREF/PB0   | MOSI (4)           | DI (5) |         |
|   6   | PB1        | MISO (1)           | DO (2) |         |
|   7   | PB2        | SCK (3)            | CLK (6)|         |
|   8   | VCC (3.3V) |                    |        | 3.3V    |



## Programming Header
| Header | Name  | Function | Pin on Attiny |
|:------:|:-----:|----------|--------------:|
| G      | GND   | Ground   |       GND (6) |
| R      | RST   | Reset    | PB5/Reset (5) |
| O      | MO    | MOSI     |       PB0 (4) |
| C      | CLK   | Clock    |   PB2/SCK (3) |
| V      | V     |  3.3V    |      3.3V (2) |
| I      | MISO  | MISO     |       PB1 (6) |

The header is a 1x06_P1.27mm header, I use a clamp that has 1.27mm pitch pogo pins on it that i can just clip on.


## The Winbond flash chip W25Q32

| W25Q32 | AtTiny  | Other      |
|-------:|--------:|:----------:|
|  CS (1)|  PB3 (2)| 10k to VCC |
|  DO (2)|  PB1 (6)|            |
| IO2 (3)| N/A     | VCC        |
| GND (4)| N/A     | GND        |
|  DI (5)| PB0 (5) | 10k to VCC |
| CLK (6)| PB2 (7) | 10k to VCC |
| IO3 (7)|         | VCC        |
| VCC (8)|         | VCC        |


# Audio File Binary

The format of the binary file on the flash is:
| Sample Offsets |
| Null Character |
| Wave file 1    |
| Wave file 2    |
| ...            |



## Sample Offsets

At the begining of the file, it has the offsets of the beginning of each audio file and then the last offset is the end of the last file.

The offsets are 32bit integers stored in the binary file in little-endian format (the least significant byte comes first).  Lets say the first offset is at 192 (decimal).  <code>192d</code> is <code>0xC0</code> in hex, so while the 32bit integer in hex is:
```
00 00 00 C2
```
it will be stored in the binary file as:
```
C2 00 00 00
```

<code>load_sizes_from_flash();</code> loads the offsets from the flash starting at address 0x00.  It will keep reading 32bit integers untill it gets a null for the least significant byte.


So in this case:
```
192 27334 63602 75056 92798 115042 138252 146312 158438 172244 189812 208390 220122 227092 242000 254260 259746 269260 281408 298566 316292 330928 340366 349978 363702 380922 392626 401676 421900 441886 458094 472142 510028 526826 540926 562508 574854 583608 597122 612476 625210 637150 664496 676116 687472 701142
```
687472 is the beginning of the last audio file and 701142 is the end of that file.

In the binary file it will look like this:
```
C0 00 00 00 C6 6A 00 00 72 F8 00 00 30 25 01 00 7E 6A 01 00 62 C1 01 00 0C 1C 02 00 88 3B 02 00 E6 6A 02 00 D4 A0 02 00 74 E5 02 00 06 2E 03 00 DA 5B 03 00 14 77 03 00 50 B1 03 00 34 E1 03 00 A2 F6 03 00 CC 1B 04 00 40 4B 04 00 46 8E 04 00 84 D3 04 00 B0 0C 05 00 8E 31 05 00 1A 57 05 00 B6 8C 05 00 FA CF 05 00 B2 FD 05 00 0C 21 06 00 0C 70 06 00 1E BE 06 00 6E FD 06 00 4E 34 07 00 4C C8 07 00 EA 09 08 00 FE 40 08 00 4C 95 08 00 86 C5 08 00 B8 E7 08 00 82 1C 09 00 7C 58 09 00 3A 8A 09 00 DE B8 09 00 B0 23 0A 00 14 51 0A 00 70 7D 0A 00 D6 B2 0A 00 00 00 00 00 00 00 00 00 7F 7F 80 7D 7A 7A 7F 77 77 77 77 76 76 76 70 6D 70 70 6A 67 71 6E 73 74 82 89 8E 91 A1 A7 A4 AD
```
so first sample is the 192, the second is <code>C6 6A 00 00</code> which as the 32bit integer ass about is <code>00 00 61 C6</code> which in decimal is <code>27334</code>


Created a script to do this for me:
```bash
$ ./wav_to_bin.sh
Generated output.bin (701142 bytes)
Offsets (46 entries + 2 nulls): 192 27334 63602 75056 92798 115042 138252 146312 158438 172244 189812 208390 220122 227092 242000 254260 259746 269260 281408 298566 316292 330928 340366 349978 363702 380922 392626 401676 421900 441886 458094 472142 510028 526826 540926 562508 574854 583608 597122 612476 625210 637150 664496 676116 687472 701142
```

This script will just take all the wav files in the directory, strip the 44byte header off, shove them in output.bin and then put the offsets at the front of the file.

## Sound Files

Audio files should be saved as a WAV file:
  - 8 kHz
  - 8-Bit
  - Mono

The WAV file format contains a 44-byte header followed by the raw data so i need to skip the first 44 bytes.



​

## Quick Audacity Steps

Meh, i ended up using duke nukem sounds but this is a way to apparently make the sounds sound ok on the speaker:
1.  Import WAV → Resample to exactly 8 kHz mono (matches your 8 kHz playback).
2.  Effect > Filter Curve EQ:
3.  High-pass: +20 Hz, steep rolloff to cut <1 kHz rumble.
4.  Peak boost: +10–15 dB at 2.7 kHz (Q=1.5 width).
5.  Normalize to -1 dB peak.
6.  Export as raw 8-bit unsigned PCM → Concatenate to flash.

For the filter, you can just import this filter under Effect > Filter Curve EQ neepneep-piezo-eq.txt:
```
FilterCurve:f0="20" f1="50" f2="100" f3="2600" f4="2700" f5="2800" f6="4000" f7="8000" FilterLength="8191" InterpolateLin="0" InterpolationMethod="B-spline" v0="-24" v1="-20" v2="-8" v3="8" v4="12" v5="8" v6="2" v7="0"
```


# Installation/Programming

This comes in two parts:
1.  Programming the ATTiny
2.  Programming the Flash Chip

## Programming the ATtiny85

I use VSCode. and the USBASP programmer.

1.  For new ATtiny85's, you will most likely have to set the fuses.
    In VSCode, the platformio button.  Go under attiny85 > Platform > Setfuses

2.  Upload the code.
    In VSCode, the platformio button.  Go under attiny85 > General > Upload








## Programming the Flash chip

There are a number of ways you can program the flash chip.

I use a flash chip programmer (CH321a) to program the chip by using a clip thing that clips over the chip while it is already on the board.  On the neepneep board i ground the ATtiny85's reset pin using a wire to Ijumper on the programming interface pin thing between R and G (they are next to eachother.) otherwise the ATtiny85 interferes with programming the flash chip.

Sometimes i have found that its been really difficult to reprogram the flash chip and maybe the cause was the ATtiny85, even though i had held the reset on that down so i created a define in main.h you can set so you program the ATtiny85 with essentially an empty program:
```c
#define WIPE_ATTINY            /// used to entirely wipe the ATTINY
```



