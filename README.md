## General

This is the one I am actually using
just update to find where its saving it lol

This is the one I am actually using
just update to find where its saving it lol


## Hardware
Essentiall this is a ATtiny85 with a flash chip.  The PCB design is https://gitlab.niffum.net/kicad/neepneep-dac-kicad.



This was an example
https://www.hackster.io/news/building-a-simple-audio-player-with-an-attiny85-a6b6bf635c21

This is the page: that it came from apparently:
http://www.technoblogy.com/show?2XJD


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



## Installation/Programming

I use VSCode. and the USBASP programmer.

1.  For new ATtiny13a's, you will most likely have to set the fuses.
    In VSCode, the platformio button.  Go under attiny85 > Platform > Setfuses

2.  Upload the code.
    In VSCode, the platformio button.  Go under attiny85 > General > Upload

## Programming the Flash chip

I use a flash chip programmer to program the chip ch341 and i use a chip to clip on the chip.  On the neepneep board i ground the ATtiny85's reset pin using a wire to jumper on the programming interface pin thing between R and G (they are next to eachother.)
