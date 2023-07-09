// io.h
#pragma once

#include <stdbool.h>

void initIo();

// RGB LED
void rgb_off();
void rgb_red();
void rgb_green();
void rgb_blue();
void rgb_yellow();

// Builtin LED
void builtinLed_set(bool value);

// Relay L1
void relayL1_set(bool);

// Relay L2
void relayL2_set(bool);

// Relay L3
void relayL3_set(bool);

// Fan
void fanOnOff_set(bool);
float fan_readSpeed(); // RPM
void fan_setSpeed(float speed); // 0.0 to 1.0

// Buzzer
void buzzer_on();
void buzzer_off();
void buzzer_beepLong();
void buzzer_beepShort();

// Temperature sensors
long tempSense1_read(); // milli degrees C
