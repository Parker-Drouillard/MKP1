#ifndef gcodes_h
#define gcodes_h

#include "Marlin.h"
#include "gcodes.cpp"

extern static void gcode_G1(); //Plan move
extern static void gcode_G2(bool stopped); //Clockwise arc move
extern static void gcode_G3(bool stopped); //CounterClockwise Arc move
extern static unsigned long gcode_G4(unsigned long codenum); //Dwell
extern static void gcode_G10(); //Retract
extern static void gcode_G11(); //Retract recover
extern static void gcode_G80(); // Mesh based Z probe
extern static void gcode_G28(bool home_x_axis, long home_x_value, bool home_y_axis, long home_y_value, bool home_z_axis, long home_z_value, bool without_mbl);
extern static void gcode_G28(bool home_x_axis, bool home_y_axis, bool home_z_axis);
extern static void gcode_G28(); //Home all Axes one at a time
extern static void gcode_G29(void); //Probe bed - Not used in favor of MBL. See G81
extern static void gcode_G30(void); //Single Probe
extern static void gcode_G75(void); //Print temperature interpolation
extern static void gcode_G76(void); //PINDA probe temperature compensation calibration
extern static void gcode_G81(); //Mesh bed levelling status
extern static void gcode_G92(); // G92 - Set current position to coordinates given

#endif //gcodes_h