#pragma once
#include <stdint.h>
#include <stdbool.h>
// Board 2.0 audio: PIO PWM on PIN_AUDIO_PWM (GPIO45) at about 44.1 kHz, ~10.7 bits. For now only a
// test tone generator (bring-up of the RC filter / J3 / SOUNDIN path); the SCC / MSX-Audio synthesis
// will feed the same DMA stream later. On board 1.0 GPIO45 is the LED: never call audio_init there.
bool audio_init(void);                 // claims a PIO1 state machine (after led_disable on a 1.0-style boot)
bool audio_ready(void);
void audio_tone(uint32_t hz, uint32_t level);   // sine at ~hz (quantised to 44100/2048 steps), level 0..100 %; hz 0 = silence
void audio_off(void);
uint32_t audio_rate_hz(void);          // actual PWM period rate
