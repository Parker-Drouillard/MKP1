#include "Marlin.h"
#include "gcodes.cpp"



extern static void gcode_G28(bool home_x_axis, long home_x_value, bool home_y_axis, long home_y_value, bool home_z_axis, long home_z_value, bool without_mbl);
extern static void gcode_G28(bool home_x_axis, bool home_y_axis, bool home_z_axis);
extern static void gcode_G29(void); //Probe bed - Not used in favor of MBL. See G81
extern static void gcode_G76(void); //PINDA probe temperature compensation calibration
extern static void gcode_G92(); // G92 - Set current position to coordinates given
