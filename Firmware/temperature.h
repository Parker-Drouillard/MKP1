/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperature_h
#define temperature_h 

#include "Marlin.h"
#include "planner.h"
#ifdef PID_ADD_EXTRUSION_RATE
  #include "stepper.h"
#endif

#include "config.h"


#ifdef SYSTEM_TIMER_2

#define ENABLE_TEMPERATURE_INTERRUPT()  TIMSK2 |= (1<<OCIE2B)
#define DISABLE_TEMPERATURE_INTERRUPT() TIMSK2 &= ~(1<<OCIE2B)

#else //SYSTEM_TIMER_2

#define ENABLE_TEMPERATURE_INTERRUPT()  TIMSK0 |= (1<<OCIE0B)
#define DISABLE_TEMPERATURE_INTERRUPT() TIMSK0 &= ~(1<<OCIE0B)

#endif //SYSTEM_TIMER_2


// public functions
void tp_init();  //initialize the heating

extern uint16_t current_probetemps_raw[NUMTEMPPROBES];
extern uint16_t current_probetemps_raw_fast[NUMTEMPPROBES];
extern float current_temperature_probes[NUMTEMPPROBES];


#ifdef BABYSTEPPING
  extern volatile int babystepsTodo[3];
#endif


inline void babystepsTodoZadd(int n)
{
    if (n != 0) {
        CRITICAL_SECTION_START
        babystepsTodo[Z_AXIS] += n;
        CRITICAL_SECTION_END
    }
}

inline void babystepsTodoZsubtract(int n)
{
    if (n != 0) {
        CRITICAL_SECTION_START
        babystepsTodo[Z_AXIS] -= n;
        CRITICAL_SECTION_END
    }
}


#endif
