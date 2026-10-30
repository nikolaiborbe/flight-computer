#ifndef __UTILITIES_BUZZER_H__
#define __UTILITIES_BUZZER_H__

#include "main.h"
#include "tim.h"
#include <math.h>

#define MUSIC_REST (-128)
#define MUSIC_END (-127)

// Octave 3
#define MUSIC_C3 (-21)
#define MUSIC_Db3 (-20)
#define MUSIC_D3 (-19)
#define MUSIC_Eb3 (-18)
#define MUSIC_E3 (-17)
#define MUSIC_F3 (-16)
#define MUSIC_Gb3 (-15)
#define MUSIC_G3 (-14)
#define MUSIC_Ab3 (-13)
#define MUSIC_A3 (-12)
#define MUSIC_Bb3 (-11)
#define MUSIC_B3 (-10)

// Octave 4
#define MUSIC_C4 (-9)
#define MUSIC_Db4 (-8)
#define MUSIC_D4 (-7)
#define MUSIC_Eb4 (-6)
#define MUSIC_E4 (-5)
#define MUSIC_F4 (-4)
#define MUSIC_Gb4 (-3)
#define MUSIC_G4 (-2)
#define MUSIC_Ab4 (-1)
#define MUSIC_A4 (0)
#define MUSIC_Bb4 (1)
#define MUSIC_B4 (2)

// Octave 5
#define MUSIC_C5 (3)
#define MUSIC_Db5 (4)
#define MUSIC_D5 (5)
#define MUSIC_Eb5 (6)
#define MUSIC_E5 (7)
#define MUSIC_F5 (8)
#define MUSIC_Gb5 (9)
#define MUSIC_G5 (10)
#define MUSIC_Ab5 (11)
#define MUSIC_A5 (12)
#define MUSIC_Bb5 (13)
#define MUSIC_B5 (14)

// Octave 6
#define MUSIC_C6 (15)
#define MUSIC_Db6 (16)
#define MUSIC_D6 (17)
#define MUSIC_Eb6 (18)
#define MUSIC_E6 (19)
#define MUSIC_F6 (20)
#define MUSIC_Gb6 (21)
#define MUSIC_G6 (22)
#define MUSIC_Ab6 (23)
#define MUSIC_A6 (24)
#define MUSIC_Bb6 (25)
#define MUSIC_B6 (26)

typedef struct {
    int8_t pitch;      /* pitches are given as semitones away from A=440Hz and -128 is rest */
    uint8_t velocity;  /* velocity are given in values between 0 and 100 where 100 is loudest */
    uint16_t duration; /* duration are given in ms */
} note_t;

#ifdef __cplusplus
extern "C" {
#endif

void play_song(TIM_HandleTypeDef* htim, uint8_t tim_channel);
void play_note(
    TIM_HandleTypeDef* htim, uint8_t tim_channel, uint16_t note_duration, int8_t note_pitch,
    uint8_t note_velocity
);

void play_success_sound();
void play_bad_apple();
void play_failure_sound();

#ifdef __cplusplus
}
#endif

#endif /* __UTILITIES_BUZZER_H__ */
