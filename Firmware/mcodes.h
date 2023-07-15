#ifndef mcodes_h
#define mcodes_h
//All MCode routines

#include "Marlin.h"
#include "mcodes.cpp"



void adjust_bed_reset(); 
void home_xy();

extern bool gcode_M45(bool onlyZ, int8_t verbosity_level);  //   M45    Calibrate XYZ

extern void gcode_M114();  //  M114    Get current position
extern void M117(char *starpos); //Display message !! High Priority
extern void M600_load_filament(); //  M600    Load filament for any config
extern static void gcode_M600(); //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
extern static void gcode_M600(bool automatic, float x_position, float y_position, float z_shift, float e_shift, float /*e_shift_late*/);
extern void gcode_M701(); // Load Filament
extern inline void gcode_M900(void);                        //  M900    Set and/or Get advance K factor


#endif //mcodes_h