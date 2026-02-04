#ifndef TUNES_H
#define TUNES_H

#include "main.h"

/*
typedef struct s_notes {
	uint8_t	note; 
	uint8_t duration;
} notes_t;
*/


typedef struct s_tune {
    uint8_t size;
    notes_t notes[];
} tune_t;

//extern const notes_t tune_new[];
extern const uint8_t tune_nokia_rep[];


extern const uint8_t tune_nokia_new[];
extern const uint8_t tune_sms_new[];
extern const uint8_t tune_iphone_new[];


void play_note(uint8_t _note, uint8_t _duration);
void playtune_scale(void);
void playtune_melody(const notes_t *melody,uint8_t _size);
//void playtune_melody_new(const notes_t *melody,uint8_t _size);
void playtune_melody_new(const uint8_t *melody);


#endif