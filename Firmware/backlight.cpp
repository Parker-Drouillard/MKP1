//backlight.cpp

#include "backlight.h"
#include "macros.h"
#include <avr/eeprom.h>
#include <Arduino.h>
#include "eeprom.h"
#include "pins.h"
#include "fastio.h"
#include "Timer.h"


void force_bl_on(bool) {}
void backlight_update() {}
void backlight_init() {}
void backlight_save() {}
void backlight_wake(const uint8_t) {}

