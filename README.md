## General

This is based on the ATtiny85 and the W25Q32 flash chip.



This was an example
https://www.hackster.io/news/building-a-simple-audio-player-with-an-attiny85-a6b6bf635c21

This is the page: that it came from apparently:
http://www.technoblogy.com/show?2XJD


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
 
Length is C5C2 apparently which is 50626

The WAV file format contains a 44-byte header followed by the raw data so i need to skip the first 44 bytes.

So my length is 0xC5C2 - 44 
so 50626 - 44 = 50582


Maybe create a script that will take a directory of wav files, and dump them in to the binary file (without the 44 byte header)
and spit out the start location and length of the sounds that have been put in there.

new tone 3A2E4 length

uint32_t Sizes[5] = { 0,        // Chunk 1: first 2s
                      16000,    // Chunk 2: seconds 3-4  
                      32000,    // Chunk 3: seconds 5-6
                      48000,    // Chunk 4: seconds 7-8
                      0 };      // End (unused)


                      assumes standard 44-byte PCM headers as per your Audacity workflow; test with hexdump -C output.bin | head for clean 8-bit samples post-header

I created a script for this, I will have to include it.


es, absolutely use Audacity to filter/EQ your samples—even with the series capacitor, this delivers the biggest volume boost (8–15 dB perceived) by concentrating energy in your YS-SBZ piezo's 2.5–3 kHz resonance band.
​

Quick Audacity Steps
Import WAV → Resample to exactly 8 kHz mono (matches your 8 kHz playback).

Effect > Filter Curve EQ:

High-pass: +20 Hz, steep rolloff to cut <1 kHz rumble.

Peak boost: +10–15 dB at 2.7 kHz (Q=1.5 width).

Normalize to -1 dB peak.

Export as raw 8-bit unsigned PCM → Concatenate to flash.

Expected Gains
Pre-EQ: Broad voice (peaks ~500 Hz–2 kHz) → weak piezo drive.

Post-EQ: High-mid focused → max SPL resonance → "much louder" like tone tests.

Cap + EQ + DC center code = optimal without voltage changes. Test one sample first!


## Installation/Programming

I use VSCode. and the USBASP programmer.

1.  For new ATtiny13a's, you will most likely have to set the fuses.
    In VSCode, the platformio button.  Go under attiny85 > Platform > Setfuses

2.  Upload the code.
    In VSCode, the platformio button.  Go under attiny85 > General > Upload

## Programming the Flash chip

I use a flash chip programmer to program the chip ch341 and i use a chip to clip on the chip.  On the neepneep board i ground the ATtiny85's reset pin using a wire to jumper on the programming interface pin thing between R and G (they are next to eachother.)
