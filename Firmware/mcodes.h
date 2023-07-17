#ifndef MCODES_H
#define MCODES_H
//All MCode routines

#include "Marlin.h"
#include "mcodes.cpp"



void adjust_bed_reset(); 
void home_xy();

extern unsigned long gcode_M1(char *starpos);
extern void gcode_M17(); //	### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
extern void gcode_M20(); //	  ### M20 - SD Card file list <a href="https://reprap.org/wiki/G-code#M20:_List_SD_card">M20: List SD card</a>
extern bool gcode_M45(bool onlyZ, int8_t verbosity_level);  //   M45    Calibrate XYZ
extern void gcode_M114();  //  M114    Get current position
extern void M117(char *starpos); //Display message !! High Priority
void M600_check_state(float nozzle_temp);
void M600_load_filament_movements();
void M600_load_filament(); //  M600    Load filament for any config
void M600_wait_for_user(float HotendTempBckp);
extern void gcode_M600(bool automatic, float x_position, float y_position, float z_shift, float e_shift, float /*e_shift_late*/);
extern void gcode_M600(); //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
extern void gcode_M701(); // Load Filament
extern inline void gcode_M900(void);                        //  M900    Set and/or Get advance K factor


#endif //mcodes_h