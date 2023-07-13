//All MCode routines

#include "Marlin.h"
#include "mcodes.cpp"


extern bool gcode_M45(bool onlyZ, int8_t verbosity_level);  //   M45    Calibrate XYZ

extern void gcode_M114();                                   //  M114    Get current position
extern void M600_load_filament();                           //  M600    Load filament for any config
extern static void gcode_M600(bool automatic, float x_position, float y_position, float z_shift, float e_shift, float /*e_shift_late*/);
extern inline void gcode_M900(void);                        //  M900    Set and/or Get advance K factor


