## General

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

## Hardware
Essentiall this is a ATtiny85 with a flash chip.  The PCB design is https://gitlab.niffum.net/kicad/neepneep-dac-kicad.

I am a little concerned that i'm programming the ATtiny85 with 5V rail, apparenlty can program it with a 3.3v programmer which is probaly what i shoudl be looking into.

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




## Audio File

setup audio as 8.000 kHz, 8-bit, Mono
 
The WAV file format contains a 44-byte header followed by the raw data so i need to skip the first 44 bytes.

At the begining of the file, it has the offsets of the beginning of each audio file and then the last offset is the end of the last file.

So in this case:
```
192 27334 63602 75056 92798 115042 138252 146312 158438 172244 189812 208390 220122 227092 242000 254260 259746 269260 281408 298566 316292 330928 340366 349978 363702 380922 392626 401676 421900 441886 458094 472142 510028 526826 540926 562508 574854 583608 597122 612476 625210 637150 664496 676116 687472 701142
```
687472 is the beginning of the last audio file and 701142 is the end of that file.

<code>load_sizes_from_flash();</code> loads it from the flash.  Basically reads uint32_t values which represent the offsets untill get gets nulls..  I can't really remember how i did it but looking at the code looks like the when its reading the offsets, if the first byte it reads is a null then it stopps.. 

It is little-endian so you read the bytes ass about.  An example, if you store the value 192decimal as an offset, 192 converted to hex is <code>C0</code> but its 32 bits so it will be 00 00 00 C0 but it will be stored in the binary file as <code>C0 00 00 00</codde>

so the header of the above offsets are:
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



​

### Quick Audacity Steps

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


## Installation/Programming

I use VSCode. and the USBASP programmer.

1.  For new ATtiny13a's, you will most likely have to set the fuses.
    In VSCode, the platformio button.  Go under attiny85 > Platform > Setfuses

2.  Upload the code.
    In VSCode, the platformio button.  Go under attiny85 > General > Upload

## Programming the Flash chip

I use a flash chip programmer to program the chip ch341 and i use a chip to clip on the chip.  On the neepneep board i ground the ATtiny85's reset pin using a wire to jumper on the programming interface pin thing between R and G (they are next to eachother.)



