#include "mcodes.h"




/**
  * M900: Set and/or Get advance K factor
  *
  *  K<factor>                  Set advance K factor
  */
inline void gcode_M900() {
    #ifdef LIN_ADVANCE
  float newK = code_seen('K') ? code_value_float() : -2;
#ifdef LA_NOCOMPAT
  if (newK >= 0 && newK < LA_K_MAX) {
    extruder_advance_K = newK;
  } else {
    SERIAL_ECHOLNPGM("K out of allowed range!");
  }
#else
  if (newK == 0) {
    extruder_advance_K = 0;
    la10c_reset();
  } else {
    newK = la10c_value(newK);
    if (newK < 0) {
      SERIAL_ECHOLNPGM("K out of allowed range!");
    } else {
      extruder_advance_K = newK;
    }
  }
#endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Advance K=");
  SERIAL_ECHOLN(extruder_advance_K);
  #endif // LIN_ADVANCE
}



