#ifndef PINS_H
#define PINS_H

#include "boards.h"

#define LARGE_FLASH true

/*****************************************************************
* Rambo Pin Assignments 1.3
******************************************************************/

#if MOTHERBOARD == BOARD_RAMBO_MINI_1_0 //200 - orig 102
#include "pins_Rambo_1_0.h"
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_0

#if MOTHERBOARD == BOARD_RAMBO_MINI_1_3 //203 - orig 302
#include "pins_Rambo_1_3.h"
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_3

#if MOTHERBOARD == BOARD_EINSY_1_0a //310 - new
#include "pins_Einsy_1_0.h"
#endif //MOTHERBOARD == BOARD_EINSY_1_0a

#if MOTHERBOARD == BOARD_RAMBO //400
#include "pins_RAMBO.h"
#endif //MOTHERBOARD == BOARD_RAMBO

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, TEST_PIN0, TEST_PIN1, TEST_PIN2, TEST_PIN3, TEST_PIN4, TEST_PIN5, SCK_PIN,\
                         X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN,  \
                        HEATER_BED_PIN,\
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(PROBE_PIN0), analogInputToDigitalPin(PROBE_PIN1), analogInputToDigitalPin(PROBE_PIN2), analogInputToDigitalPin(PROBE_PIN3), }

#endif //__PINS_H
