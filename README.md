This is the one I am actually using
just update to find where its saving it lol

This is the one I am actually using
just update to find where its saving it lol

This was an example
https://www.hackster.io/news/building-a-simple-audio-player-with-an-attiny85-a6b6bf635c21

This is the page: that it came from apparently:
http://www.technoblogy.com/show?2XJD

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
