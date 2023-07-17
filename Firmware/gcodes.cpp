#include "Marlin.h"

bool retracted_swap[EXTRUDERS]={false
#if EXTRUDERS > 1
  , false
#if EXTRUDERS > 2
  , false
#endif
#endif
};

//                         ## ##    ## ##             ## ##      ##    
//                        ##   ##  ##   ##           ##   ##    ###    
//                        ##       ##   ##           ##          ##    
//                        ##  ###  ##   ##           ##  ###     ##    
//                        ##   ##  ##   ##           ##   ##     ##    
//                        ##   ##  ##   ##           ##   ##     ##    
//                         ## ##    ## ##             ## ##     ####  

/*!
---------------------------------------------------------------------------------
# G Codes
### G0, G1 - Coordinated movement X Y Z E <a href="https://reprap.org/wiki/G-code#G0_.26_G1:_Move">G0 & G1: Move</a> 
In Prusa Firmware G0 and G1 are the same.
#### Usage

G0 [ X | Y | Z | E | F | S ]
G1 [ X | Y | Z | E | F | S ]

#### Parameters
    - `X` - The position to move to on the X axis
    - `Y` - The position to move to on the Y axis
    - `Z` - The position to move to on the Z axis
    - `E` - The amount to extrude between the starting point and ending point
    - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
*/
static void gcode_G1(){
  if(Stopped == false) {
  #ifdef FILAMENT_RUNOUT_SUPPORT
  #if EXTRUDERS > 1
    bool filCheck = true;
    if(active_extruder == 0){
        if(READ(FILAMENT_RUNOUT_SENSOR)){
          filCheck = false;
        }
    } else if (active_extruder == 1){
        if(READ(FILAMENT_RUNOUT2_SENSOR)){
          filCheck = false;
        } 
    }
#endif //Extruders > 1
    if (filCheck && PRINTER_ACTIVE) {
      int feedmultiplyBckp=feedmultiply;
      float target[4];
      float lastpos[4];
      target[X_AXIS]=current_position[X_AXIS];
      target[Y_AXIS]=current_position[Y_AXIS];
      target[Z_AXIS]=current_position[Z_AXIS];
      target[E_AXIS]=current_position[E_AXIS];
      lastpos[X_AXIS]=current_position[X_AXIS];
      lastpos[Y_AXIS]=current_position[Y_AXIS];
      lastpos[Z_AXIS]=current_position[Z_AXIS];
      lastpos[E_AXIS]=current_position[E_AXIS];
      //retract by E
                  
      target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;          
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 400, active_extruder);
      target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 300, active_extruder);
      target[X_AXIS]= FILAMENTCHANGE_XPOS ;           
      target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder);
      target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder);

      //finish moves
      st_synchronize();
      //disable extruder steppers so filament can be removed
      disable_e0();
      disable_e1();
      disable_e2();
      _delay(100);
                  
      //LCD_ALERTMESSAGEPGM(_T(MSG_FILAMENTCHANGE));
      uint8_t cnt=0;
      int counterBeep = 0;
      lcd_wait_interact();
      while(!lcd_clicked()){
        cnt++;
        manage_heater();
        manage_inactivity(true);
        if(cnt==0) {
#if BEEPER > 0   
          if (counterBeep== 500){
            counterBeep = 0;              
          }              
          SET_OUTPUT(BEEPER);
          if (counterBeep== 0){
            if(eSoundMode!=e_SOUND_MODE_SILENT) {
              WRITE(BEEPER,HIGH);
            }
          }
          if (counterBeep== 20){
            WRITE(BEEPER,LOW);
          }
          counterBeep++;
#endif
          }
        }  
        WRITE(BEEPER,LOW); 
        target[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder);    
        target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder);                  
        lcd_change_fil_state = 0;
        lcd_loading_filament();
        while ((lcd_change_fil_state == 0)||(lcd_change_fil_state != 1)){
          lcd_change_fil_state = 0;
          lcd_alright();
          switch(lcd_change_fil_state){                        
            case 2:
              target[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
              plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder);    
              target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
              plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder);                    
              lcd_loading_filament();
            break;
            case 3:
              target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
              plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder); 
              lcd_loading_color();
            break;                  
            default:
            lcd_change_success();
            break;
          }               
        }
                    
        target[E_AXIS]+= 5;
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder);         
        target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT;
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 400, active_extruder);
                    
        //current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
        //plan_set_e_position(current_position[E_AXIS]);
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder); //should do nothing
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder); //move xy back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], 200, active_extruder); //move z back
                    
        target[E_AXIS]= target[E_AXIS] - FILAMENTCHANGE_FIRSTRETRACT;    
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], 5, active_extruder); //final untretract
        plan_set_e_position(lastpos[E_AXIS]);
        feedmultiply=feedmultiplyBckp;
        char cmd[9];

        sprintf_P(cmd, PSTR("M220 S%i"), feedmultiplyBckp);
        enquecommand(cmd);
    }
#endif

    get_coordinates(); // For X Y Z E F

    // When recovering from a previous print move, restore the originally
    // calculated target position on the first USB/SD command. This accounts
    // properly for relative moves
    if ((saved_target[0] != SAVED_TARGET_UNSET) &&
        ((CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_SDCARD) ||
            (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR))) {
        memcpy(destination, saved_target, sizeof(destination));
        saved_target[0] = SAVED_TARGET_UNSET;
    }

    if (total_filament_used > ((current_position[E_AXIS] - destination[E_AXIS]) * 100)) { //protection against total_filament_used overflow
        total_filament_used = total_filament_used + ((destination[E_AXIS] - current_position[E_AXIS]) * 100);
    }
#ifdef FWRETRACT
    if(cs.autoretract_enabled) {
      if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
        float echange=destination[E_AXIS]-current_position[E_AXIS];
        if((echange<-MIN_RETRACT && !retracted[active_extruder]) || (echange>MIN_RETRACT && retracted[active_extruder])) { //move appears to be an attempt to retract or recover
            current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
            plan_set_e_position(current_position[E_AXIS]); //AND from the planner
            retract(!retracted[active_extruder]);
            return;
        }
      }
    }
#endif //FWRETRACT
    prepare_move();
  }
}

 /*!
  ### G2, G3 - Controlled Arc Move <a href="https://reprap.org/wiki/G-code#G2_.26_G3:_Controlled_Arc_Move">G2 & G3: Controlled Arc Move</a>
  These commands don't propperly work with MBL enabled. The compensation only happens at the end of the move, so avoid long arcs.
  #### Usage

  G2 [ X | Y | I | E | F ] (Clockwise Arc)
  G3 [ X | Y | I | E | F ] (Counter-Clockwise Arc)

  #### Parameters
  - `X` - The position to move to on the X axis
  - `Y` - The position to move to on the Y axis
  - `I` - The point in X space from the current X position to maintain a constant distance from
  - `J` - The point in Y space from the current Y position to maintain a constant distance from
  - `E` - The amount to extrude between the starting point and ending point
  - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
*/
static void gcode_G2(bool stopped) {
  if(stopped == false) {
    get_arc_coordinates();
    prepare_arc_move(true);
  }
}

// See documentation for G2
static void gcode_G3(bool stopped) {
  if(stopped == false) {
    get_arc_coordinates();
    prepare_arc_move(false);
  }
}


/*!
### G4 - Dwell <a href="https://reprap.org/wiki/G-code#G4:_Dwell">G4: Dwell</a>
Pause the machine for a period of time.
#### Usage
G4 [ P | S ]
#### Parameters
- `P` - Time to wait, in milliseconds
- `S` - Time to wait, in seconds
*/
static unsigned long gcode_G4(unsigned long codenum){
  if(code_seen('P')) {codenum = code_value();} // milliseconds to wait
  if(code_seen('S')) {codenum = code_value() * 1000; }// seconds to wait
  if(codenum != 0) { LCD_MESSAGERPGM(_n("Sleep...")); }////MSG_DWELL
  st_synchronize();
  codenum += _millis();  // keep track of when we started waiting
  unsigned long previous_millis_cmd = _millis();
  while(_millis() < codenum) {
    manage_heater();
    manage_inactivity();
    lcd_update(0);
  }

  return previous_millis_cmd;
}


/*!
### G10 - Retract <a href="https://reprap.org/wiki/G-code#G10:_Retract">G10: Retract</a>
Retracts filament according to settings of `M207`
*/
static void gcode_G10(){
#if EXTRUDERS > 1
  retracted_swap[active_extruder]=(code_seen('S') && code_value_long() == 1); // checks for swap retract argument
  retract(true,retracted_swap[active_extruder]);
#else
  retract(true);
#endif
}


/*!
### G11 - Retract recover <a href="https://reprap.org/wiki/G-code#G11:_Unretract">G11: Unretract</a>
Unretracts/recovers filament according to settings of `M208`
*/
static void gcode_G11(){
#if EXTRUDERS > 1
  retract(false,retracted_swap[active_extruder]);
#else
  retract(false);
#endif 
}


//  ## ##   ## ##     ## ##   
// ##   ##  ##  ##   ##   ##  
// ##           ##   ##   ##  
// ##  ###     ##     ## ##   
// ##   ##    ##     ##   ##  
// ##   ##   #   ##   #   ##  
//  ## ##   ######    ## ##   


 /*!
  ### G28 - Home all Axes one at a time <a href="https://reprap.org/wiki/G-code#G28:_Move_to_Origin_.28Home.29">G28: Move to Origin (Home)</a>
  Using `G28` without any parameters will perfom homing of all axes AND mesh bed leveling, while `G28 W` will just home all axes (no mesh bed leveling).
  #### Usage
	G28 [ X | Y | Z | W | C ] 
	#### Parameters
  - `X` - Flag to go back to the X axis origin
  - `Y` - Flag to go back to the Y axis origin
  - `Z` - Flag to go back to the Z axis origin
  - `W` - Suppress mesh bed leveling if `X`, `Y` or `Z` are not provided
  - `C` - Calibrate X and Y origin (home) - Only on MK3/s
	*/
static void gcode_G28(bool home_x_axis, long home_x_value, bool home_y_axis, long home_y_value, bool home_z_axis, long home_z_value, bool without_mbl) {
	st_synchronize();
	// Flag for the display update routine and to disable the print cancelation during homing.
	homing_flag = true;

	// Which axes should be homed?
	bool home_x = home_x_axis;
	bool home_y = home_y_axis;
	bool home_z = home_z_axis;

	// Either all X,Y,Z codes are present, or none of them.
	bool home_all_axes = home_x == home_y && home_x == home_z;
	if (home_all_axes) {
		// No X/Y/Z code provided means to home all axes.
		home_x = home_y = home_z = true;
  }

	//if we are homing all axes, first move z higher to protect heatbed/steel sheet
	if (home_all_axes) {
        raise_z_above(MESH_HOME_Z_SEARCH);
		st_synchronize();
	}
#ifdef ENABLE_AUTO_BED_LEVELING
  plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING
            
  // Reset world2machine_rotation_and_skew and world2machine_shift, therefore
  // the planner will not perform any adjustments in the XY plane. 
  // Wait for the motors to stop and update the current position with the absolute values.
  world2machine_revert_to_uncorrected();

  // For mesh bed leveling deactivate the matrix temporarily.
  // It is necessary to disable the bed leveling for the X and Y homing moves, so that the move is performed
  // in a single axis only.
  // In case of re-homing the X or Y axes only, the mesh bed leveling is restored after G28.
#ifdef MESH_BED_LEVELING
  uint8_t mbl_was_active = mbl.active;
  mbl.active = 0;
  current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
#endif

  // Reset baby stepping to zero, if the babystepping has already been loaded before. The babystepsTodo value will be
  // consumed during the first movements following this statement.
  if (home_z) {
    babystep_undo();
  }

  saved_feedrate = feedrate;
  int l_feedmultiply = feedmultiply;
  feedmultiply = 100;
  previous_millis_cmd = _millis();
  enable_endstops(true);
  memcpy(destination, current_position, sizeof(destination));
  feedrate = 0.0;

#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
  if(home_z) {
    homeaxis(Z_AXIS);
  }
#endif
#ifdef QUICK_HOME
  // In the quick mode, if both x and y are to be homed, a diagonal move will be performed initially.
  if(home_x && home_y) {  //first diagonal move
    current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;
    int x_axis_home_dir = home_dir(X_AXIS);
    plan_set_position_curposXYZE();
    destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
    feedrate = homing_feedrate[X_AXIS];
    if(homing_feedrate[Y_AXIS]<feedrate) {
      feedrate = homing_feedrate[Y_AXIS];
    }
    if (max_length(X_AXIS) > max_length(Y_AXIS)) {
      feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
    } else {
      feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
    }
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();

    axis_is_at_home(X_AXIS);
    axis_is_at_home(Y_AXIS);
    plan_set_position_curposXYZE();
    destination[X_AXIS] = current_position[X_AXIS];
    destination[Y_AXIS] = current_position[Y_AXIS];
    plan_buffer_line_destinationXYZE(feedrate/60);
    feedrate = 0.0;
    st_synchronize();
    endstops_hit_on_purpose();

    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];
    current_position[Z_AXIS] = destination[Z_AXIS];
  }
#endif /* QUICK_HOME */

  if(home_x) { homeaxis(X_AXIS); }
  if(home_y) { homeaxis(Y_AXIS); }

  if(home_x_axis && home_x_value != 0) {
    current_position[X_AXIS]=home_x_value+cs.add_homing[X_AXIS];
  }
  if(home_y_axis && home_y_value != 0) {
    current_position[Y_AXIS]=home_y_value+cs.add_homing[Y_AXIS];
  }

#if Z_HOME_DIR < 0                      // If homing towards BED do Z last
#ifndef Z_SAFE_HOMING
  if(home_z) {
#if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
    raise_z_above(Z_RAISE_BEFORE_HOMING);
    st_synchronize();
#endif // defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
#if (defined(MESH_BED_LEVELING))  // If Mesh bed leveling, move X&Y to safe position for home
    raise_z_above(MESH_HOME_Z_SEARCH);
    st_synchronize();
    if (!axis_known_position[X_AXIS]) {homeaxis(X_AXIS);}
    if (!axis_known_position[Y_AXIS]) {homeaxis(Y_AXIS);}
    // 1st mesh bed leveling measurement point, corrected.
    world2machine_initialize();
    world2machine(pgm_read_float(bed_ref_points_4), pgm_read_float(bed_ref_points_4+1), destination[X_AXIS], destination[Y_AXIS]);
    world2machine_reset();
    if (destination[Y_AXIS] < Y_MIN_POS){
      destination[Y_AXIS] = Y_MIN_POS;
    }
    feedrate = homing_feedrate[X_AXIS] / 20;
    enable_endstops(false);
#ifdef DEBUG_BUILD
    SERIAL_ECHOLNPGM("plan_set_position()");
    MYSERIAL.println(current_position[X_AXIS]);MYSERIAL.println(current_position[Y_AXIS]);
    MYSERIAL.println(current_position[Z_AXIS]);MYSERIAL.println(current_position[E_AXIS]);
#endif //DEBUG_BUILD
    plan_set_position_curposXYZE();
#ifdef DEBUG_BUILD
    SERIAL_ECHOLNPGM("plan_buffer_line()");
    MYSERIAL.println(destination[X_AXIS]);MYSERIAL.println(destination[Y_AXIS]);
    MYSERIAL.println(destination[Z_AXIS]);MYSERIAL.println(destination[E_AXIS]);
    MYSERIAL.println(feedrate);MYSERIAL.println(active_extruder);
#endif //DEBUG_BUILD
    plan_buffer_line_destinationXYZE(feedrate);
    st_synchronize();
    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];
    enable_endstops(true);
    endstops_hit_on_purpose();
    homeaxis(Z_AXIS);
#else // MESH_BED_LEVELING
    homeaxis(Z_AXIS);
#endif // MESH_BED_LEVELING
  }
#else // defined(Z_SAFE_HOMING): Z Safe mode activated.
  if(home_all_axes) {
    destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
    destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
    destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
    feedrate = XY_TRAVEL_SPEED/60;
    current_position[Z_AXIS] = 0;

    plan_set_position_curposXYZE();
    plan_buffer_line_destinationXYZE(feedrate);
    st_synchronize();
    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];

    homeaxis(Z_AXIS);
  }
  // Let's see if X and Y are homed and probe is inside bed area.
  if(home_z) {
    if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
    && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
    && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
    && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
    && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {

      current_position[Z_AXIS] = 0;
      plan_set_position_curposXYZE();
      destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
      feedrate = max_feedrate[Z_AXIS];
      plan_buffer_line_destinationXYZE(feedrate);
      st_synchronize();

      homeaxis(Z_AXIS);
    } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
      LCD_MESSAGERPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNRPGM(MSG_POSITION_UNKNOWN);
    } else {
      LCD_MESSAGERPGM(MSG_ZPROBE_OUT);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNRPGM(MSG_ZPROBE_OUT);
    }
  }
#endif // Z_SAFE_HOMING
#endif // Z_HOME_DIR < 0

  if(home_z_axis && home_z_value != 0) {
    current_position[Z_AXIS]=home_z_value+cs.add_homing[Z_AXIS];
  }
#ifdef ENABLE_AUTO_BED_LEVELING
  if(home_z) {
    current_position[Z_AXIS] += cs.zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
  }
#endif
      
  // Set the planner and stepper routine positions.
  // At this point the mesh bed leveling and world2machine corrections are disabled and current_position
  // contains the machine coordinates.
  plan_set_position_curposXYZE();

#ifdef ENDSTOPS_ONLY_FOR_HOMING
  enable_endstops(false);
#endif

  feedrate = saved_feedrate;
  feedmultiply = l_feedmultiply;
  previous_millis_cmd = _millis();
  endstops_hit_on_purpose();
#ifndef MESH_BED_LEVELING
  //-// Oct 2019 :: this part of code is (from) now probably un-compilable
  // If MESH_BED_LEVELING is not active, then it is the original Prusa i3.
  // Offer the user to load the baby step value, which has been adjusted at the previous print session.
  if(card.sdprinting && eeprom_read_word((uint16_t *)EEPROM_BABYSTEP_Z)) {
    lcd_adjust_z();
  }
#endif

  // Load the machine correction matrix
  world2machine_initialize();
  // and correct the current_position XY axes to match the transformed coordinate system.
  world2machine_update_current();

#if (defined(MESH_BED_LEVELING))
	if (home_x_axis || home_y_axis || without_mbl || home_z_axis) {
    if (! home_z && mbl_was_active) {
      // Re-enable the mesh bed leveling if only the X and Y axes were re-homed.
      mbl.active = true;
      // and re-adjust the current logical Z axis with the bed leveling offset applicable at the current XY position.
      current_position[Z_AXIS] -= mbl.get_z(st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS));
    }
  }	else {
    st_synchronize();
    homing_flag = false;
  }
#endif
  homing_flag = false;
}

static void gcode_G28(bool home_x_axis, bool home_y_axis, bool home_z_axis) {
  gcode_G28(home_x_axis, 0, home_y_axis, 0, home_z_axis, 0, true);
}

static void gcode_G28(){
  long home_x_value = 0;
  long home_y_value = 0;
  long home_z_value = 0;
  // Which axes should be homed?
  bool home_x = code_seen(axis_codes[X_AXIS]);
  home_x_value = code_value_long();
  bool home_y = code_seen(axis_codes[Y_AXIS]);
  home_y_value = code_value_long();
  bool home_z = code_seen(axis_codes[Z_AXIS]);
  home_z_value = code_value_long();
  bool without_mbl = code_seen('W');
  // calibrate?
  gcode_G28(home_x, home_x_value, home_y, home_y_value, home_z, home_z_value, without_mbl);
  if ((home_x || home_y || without_mbl || home_z) == false) {
      // Push the commands to the front of the message queue in the reverse order!
      // There shall be always enough space reserved for these commands.
      gcode_G80();
  }
}


//  ## ##   ## ##     ## ##   
// ##   ##  ##  ##    #   ##  
// ##           ##   ##   ##  
// ##  ###     ##     ## ###  
// ##   ##    ##          ##  
// ##   ##   #   ##  ##   ##  
//  ## ##   ######    ## ##  

/*!
### G29 - Detailed Z-Probe <a href="https://reprap.org/wiki/G-code#G29:_Detailed_Z-Probe">G29: Detailed Z-Probe</a>
In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.

See `G81`
*/
static void gcode_G29(void){
#if Z_MIN_PIN == -1
#error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature! Z_MIN_PIN must point to a valid hardware pin."
#endif
  // Prevent user from running a G29 without first homing in X and Y
  if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) ) {
    LCD_MESSAGERPGM(MSG_POSITION_UNKNOWN);
    SERIAL_ECHO_START;
    SERIAL_ECHOLNRPGM(MSG_POSITION_UNKNOWN);
    break; // abort G29, since we don't know where we are
  }

  st_synchronize();
  // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
  //vector_3 corrected_position = plan_get_position_mm();
  //corrected_position.debug("position before G29");
  plan_bed_level_matrix.set_to_identity();
  vector_3 uncorrected_position = plan_get_position();
  //uncorrected_position.debug("position durring G29");
  current_position[X_AXIS] = uncorrected_position.x;
  current_position[Y_AXIS] = uncorrected_position.y;
  current_position[Z_AXIS] = uncorrected_position.z;
  plan_set_position_curposXYZE();
  int l_feedmultiply = setup_for_endstop_move();

  feedrate = homing_feedrate[Z_AXIS];
#ifdef AUTO_BED_LEVELING_GRID
    // probe at the points of a lattice grid

    int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
    int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);

    // solve the plane equation ax + by + d = z
    // A is the matrix with rows [x y 1] for all the probed points
    // B is the vector of the Z positions
    // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
    // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

    // "A" matrix of the linear system of equations
    double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
    // "B" vector of Z points
    double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];

    int probePointCounter = 0;
    bool zig = true;

    for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing) {
      int xProbe, xInc;
      if (zig) {
        xProbe = LEFT_PROBE_BED_POSITION;
        //xEnd = RIGHT_PROBE_BED_POSITION;
        xInc = xGridSpacing;
        zig = false;
      } else { // zag
        xProbe = RIGHT_PROBE_BED_POSITION;
        //xEnd = LEFT_PROBE_BED_POSITION;
        xInc = -xGridSpacing;
        zig = true;
      }
      for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++) {
      float z_before;
      if (probePointCounter == 0) {
        // raise before probing
        z_before = Z_RAISE_BEFORE_PROBING;
      } else {
        // raise extruder
        z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
      }
      float measured_z = probe_pt(xProbe, yProbe, z_before);
      eqnBVector[probePointCounter] = measured_z;

      eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
      eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
      eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
      probePointCounter++;
      xProbe += xInc;
    }
  }
  clean_up_after_endstop_move(l_feedmultiply);

  // solve lsq problem
  double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

  SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
  SERIAL_PROTOCOL(plane_equation_coefficients[0]);
  SERIAL_PROTOCOLPGM(" b: ");
  SERIAL_PROTOCOL(plane_equation_coefficients[1]);
  SERIAL_PROTOCOLPGM(" d: ");
  SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);

  set_bed_level_equation_lsq(plane_equation_coefficients);
  free(plane_equation_coefficients);

#else // AUTO_BED_LEVELING_GRID not defined
  // Probe at 3 arbitrary points
  // probe 1
  float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);
  // probe 2
  float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
  // probe 3
  float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

  clean_up_after_endstop_move(l_feedmultiply);
  set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
#endif // AUTO_BED_LEVELING_GRID
  st_synchronize();
  // The following code correct the Z height difference from z-probe position and hotend tip position.
  // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
  // When the bed is uneven, this height must be corrected.
  real_z = float(st_get_position(Z_AXIS))/cs.axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
  x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
  y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
  z_tmp = current_position[Z_AXIS];
  apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
  current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
  plan_set_position_curposXYZE();
}


  /*!
  ### G30 - Single Z Probe <a href="https://reprap.org/wiki/G-code#G30:_Single_Z-Probe">G30: Single Z-Probe</a>
  Sensor must be over the bed.
  The maximum travel distance before an error is triggered is 10mm.
  */
static void gcode_G30(){
  st_synchronize();
  // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
  int l_feedmultiply = setup_for_endstop_move();
  feedrate = homing_feedrate[Z_AXIS];
  find_bed_induction_sensor_point_z(-10.f, 3);
  printf_P(_N("%S X: %.5f Y: %.5f Z: %.5f\n"), _T(MSG_BED), _x, _y, _z);
  clean_up_after_endstop_move(l_feedmultiply);
}

/*!
### G75 - Print temperature interpolation <a href="https://reprap.org/wiki/G-code#G75:_Print_temperature_interpolation">G75: Print temperature interpolation</a>
Show/print PINDA temperature interpolating.
*/
static void gcode_G75(){
  for (int i = 40; i <= 110; i++) {
    printf_P(_N("%d  %.2f"), i, temp_comp_interpolation(i));
  }
}


//  ## ##   ######    ## ###  
// ##   ##  ##   #   ##   ##  
// ##          ##    ##       
// ##  ###    ##     ## ###   
// ##   ##    ##     ##   ##  
// ##   ##    ##     ##   ##  
//  ## ##     ##      ## ## 

/*!
### G76 - PINDA probe temperature calibration <a href="https://reprap.org/wiki/G-code#G76:_PINDA_probe_temperature_calibration">G76: PINDA probe temperature calibration</a>
This G-code is used to calibrate the temperature drift of the PINDA (inductive Sensor).

The PINDAv2 sensor has a built-in thermistor which has the advantage that the calibration can be done once for all materials.

The Original i3 Prusa MK2/s uses PINDAv1 and this calibration improves the temperature drift, but not as good as the PINDAv2.

superPINDA sensor has internal temperature compensation and no thermistor output. There is no point of doing temperature calibration in such case.
If PINDA_THERMISTOR and DETECT_SUPERPINDA is defined during compilation, calibration is skipped with serial message "No PINDA thermistor".
This can be caused also if PINDA thermistor connection is broken or PINDA temperature is lower than PINDA_MINTEMP.

#### Example

```
G76

echo PINDA probe calibration start
echo start temperature: 35.0°
echo ...
echo PINDA temperature -- Z shift (mm): 0.---
```
*/

static void gcode_G76(){
  #ifdef PINDA_THERMISTOR
  if (!has_temperature_compensation()) {
    SERIAL_ECHOLNPGM("No PINDA thermistor");
    break;
  }
  if (calibration_status() >= CALIBRATION_STATUS_XYZ_CALIBRATION) {
    //we need to know accurate position of first calibration point
    //if xyz calibration was not performed yet, interrupt temperature calibration and inform user that xyz cal. is needed
    lcd_show_fullscreen_message_and_wait_P(_i("Please run XYZ calibration first."));
    break;
  }
  if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
    // We don't know where we are! HOME!
    // Push the commands to the front of the message queue in the reverse order!
    // There shall be always enough space reserved for these commands.
    repeatcommand_front(); // repeat G76 with all its parameters
    enquecommand_front_P((PSTR("G28 W0")));
    break;
  }
  lcd_show_fullscreen_message_and_wait_P(_i("Stable ambient temperature 21-26C is needed a rigid stand is required."));////MSG_TEMP_CAL_WARNING c=20 r=4
  bool result = lcd_show_fullscreen_message_yes_no_and_wait_P(_T(MSG_STEEL_SHEET_CHECK), false, false);

  if (result) {
    current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
    plan_buffer_line_curposXYZE(3000 / 60);
    current_position[Z_AXIS] = 50;
    current_position[Y_AXIS] = 180;
    plan_buffer_line_curposXYZE(3000 / 60);
    st_synchronize();
    lcd_show_fullscreen_message_and_wait_P(_T(MSG_REMOVE_STEEL_SHEET));
    current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
    current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
    plan_buffer_line_curposXYZE(3000 / 60);
    st_synchronize();
    gcode_G28(false, false, true);
  }
  if ((current_temperature_pinda > 35)) {
    //waiting for PIDNA probe to cool down
    current_position[Z_AXIS] = 100;
    plan_buffer_line_curposXYZE(3000 / 60);
    if (lcd_wait_for_pinda(35) == false) { //waiting for PINDA probe to cool, if this takes more then time expected, temp. cal. fails
      lcd_temp_cal_show_result(false);
      break;
    }
  }
  lcd_update_enable(true);
  KEEPALIVE_STATE(NOT_BUSY); //no need to print busy messages as we print current temperatures periodicaly
  SERIAL_ECHOLNPGM("PINDA probe calibration start");

  float zero_z;
  int z_shift = 0; //unit: steps
  float start_temp = 5 * (int)(current_temperature_pinda / 5);
  if (start_temp < 35) {start_temp = 35;}
  if (start_temp < current_temperature_pinda) {start_temp += 5;}
  printf_P(_N("start temperature: %.1f\n"), start_temp);
  setTargetBed(70 + (start_temp - 30));

  custom_message_type = CustomMsg::TempCal;
  custom_message_state = 1;
  lcd_setstatuspgm(_T(MSG_TEMP_CALIBRATION));
  current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
  plan_buffer_line_curposXYZE(3000 / 60);
  current_position[X_AXIS] = PINDA_PREHEAT_X;
  current_position[Y_AXIS] = PINDA_PREHEAT_Y;
  plan_buffer_line_curposXYZE(3000 / 60);
  current_position[Z_AXIS] = PINDA_PREHEAT_Z;
  plan_buffer_line_curposXYZE(3000 / 60);
  st_synchronize();

  while (current_temperature_pinda < start_temp) {
    delay_keep_alive(1000);
    serialecho_temperatures();
  }

  eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0); //invalidate temp. calibration in case that in will be aborted during the calibration process

  current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
  plan_buffer_line_curposXYZE(3000 / 60);
  current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
  current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
  plan_buffer_line_curposXYZE(3000 / 60);
  st_synchronize();

  bool find_z_result = find_bed_induction_sensor_point_z(-1.f);
  if (find_z_result == false) {
    lcd_temp_cal_show_result(find_z_result);
    break;
  }
  zero_z = current_position[Z_AXIS];

  printf_P(_N("\nZERO: %.3f\n"), current_position[Z_AXIS]);

  int i = -1; 
  for (; i < 5; i++) {
    float temp = (40 + i * 5);
    printf_P(_N("\nStep: %d/6 (skipped)\nPINDA temperature: %d Z shift (mm):0\n"), i + 2, (40 + i*5));
    if (i >= 0) {EEPROM_save_B(EEPROM_PROBE_TEMP_SHIFT + i * 2, &z_shift);}
    if (start_temp <= temp) {break;}
  }

  for (i++; i < 5; i++) {
    float temp = (40 + i * 5);
    printf_P(_N("\nStep: %d/6\n"), i + 2);
    custom_message_state = i + 2;
    setTargetBed(50 + 10 * (temp - 30) / 5);
    current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
    plan_buffer_line_curposXYZE(3000 / 60);
    current_position[X_AXIS] = PINDA_PREHEAT_X;
    current_position[Y_AXIS] = PINDA_PREHEAT_Y;
    plan_buffer_line_curposXYZE(3000 / 60);
    current_position[Z_AXIS] = PINDA_PREHEAT_Z;
    plan_buffer_line_curposXYZE(3000 / 60);
    st_synchronize();
    while (current_temperature_pinda < temp) {
      delay_keep_alive(1000);
      serialecho_temperatures();
    } 
    current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
    plan_buffer_line_curposXYZE(3000 / 60);
    current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
    current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
    plan_buffer_line_curposXYZE(3000 / 60);
    st_synchronize();
    find_z_result = find_bed_induction_sensor_point_z(-1.f);
    if (find_z_result == false) {
      lcd_temp_cal_show_result(find_z_result);
      break;
    }
    z_shift = (int)((current_position[Z_AXIS] - zero_z)*cs.axis_steps_per_unit[Z_AXIS]);
    printf_P(_N("\nPINDA temperature: %.1f Z shift (mm): %.3f"), current_temperature_pinda, current_position[Z_AXIS] - zero_z);
    EEPROM_save_B(EEPROM_PROBE_TEMP_SHIFT + i * 2, &z_shift);
    }
    lcd_temp_cal_show_result(true);

#else //PINDA_THERMISTOR

		setTargetBed(PINDA_MIN_T);
		float zero_z;
		int z_shift = 0; //unit: steps
		int t_c; // temperature

		if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
			// We don't know where we are! HOME!
			// Push the commands to the front of the message queue in the reverse order!
			// There shall be always enough space reserved for these commands.
			repeatcommand_front(); // repeat G76 with all its parameters
			enquecommand_front_P((PSTR("G28 W0")));
			break;
		}
		puts_P(_N("PINDA probe calibration start"));
		custom_message_type = CustomMsg::TempCal;
		custom_message_state = 1;
		lcd_setstatuspgm(_T(MSG_TEMP_CALIBRATION));
		current_position[X_AXIS] = PINDA_PREHEAT_X;
		current_position[Y_AXIS] = PINDA_PREHEAT_Y;
		current_position[Z_AXIS] = PINDA_PREHEAT_Z;
		plan_buffer_line_curposXYZE(3000 / 60);
		st_synchronize();
		
		while (abs(degBed() - PINDA_MIN_T) > 1) {
			delay_keep_alive(1000);
			serialecho_temperatures();
		}
		
		//enquecommand_P(PSTR("M190 S50"));
		for (int i = 0; i < PINDA_HEAT_T; i++) {
			delay_keep_alive(1000);
			serialecho_temperatures();
		}
		eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0); //invalidate temp. calibration in case that in will be aborted during the calibration process 

		current_position[Z_AXIS] = 5;
		plan_buffer_line_curposXYZE(3000 / 60);

		current_position[X_AXIS] = BED_X0;
		current_position[Y_AXIS] = BED_Y0;
		plan_buffer_line_curposXYZE(3000 / 60);
		st_synchronize();
		
		find_bed_induction_sensor_point_z(-1.f);
		zero_z = current_position[Z_AXIS];

		printf_P(_N("\nZERO: %.3f\n"), current_position[Z_AXIS]);

		for (int i = 0; i<5; i++) {
			printf_P(_N("\nStep: %d/6\n"), i + 2);
			custom_message_state = i + 2;
			t_c = 60 + i * 10;

			setTargetBed(t_c);
			current_position[X_AXIS] = PINDA_PREHEAT_X;
			current_position[Y_AXIS] = PINDA_PREHEAT_Y;
			current_position[Z_AXIS] = PINDA_PREHEAT_Z;
			plan_buffer_line_curposXYZE(3000 / 60);
			st_synchronize();
			while (degBed() < t_c) {
				delay_keep_alive(1000);
				serialecho_temperatures();
			}
			for (int i = 0; i < PINDA_HEAT_T; i++) {
				delay_keep_alive(1000);
				serialecho_temperatures();
			}
			current_position[Z_AXIS] = 5;
			plan_buffer_line_curposXYZE(3000 / 60);
			current_position[X_AXIS] = BED_X0;
			current_position[Y_AXIS] = BED_Y0;
			plan_buffer_line_curposXYZE(3000 / 60);
			st_synchronize();
			find_bed_induction_sensor_point_z(-1.f);
			z_shift = (int)((current_position[Z_AXIS] - zero_z)*cs.axis_steps_per_unit[Z_AXIS]);

			printf_P(_N("\nTemperature: %d  Z shift (mm): %.3f\n"), t_c, current_position[Z_AXIS] - zero_z);

			EEPROM_save_B(EEPROM_PROBE_TEMP_SHIFT + i*2, &z_shift);
			
		
		}
		custom_message_type = CustomMsg::Status;

		eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
		puts_P(_N("Temperature calibration done."));
			disable_x();
			disable_y();
			disable_z();
			disable_e0();
			disable_e1();
			disable_e2();
			setTargetBed(0); //set bed target temperature back to 0
		lcd_show_fullscreen_message_and_wait_P(_T(MSG_TEMP_CALIBRATION_DONE));
		eeprom_update_byte((unsigned char *)EEPROM_TEMP_CAL_ACTIVE, 1);
		lcd_update_enable(true);
		lcd_update(2);		

		
#endif //PINDA_THERMISTOR
}


//  ## ##    ## ##    ## ##   
// ##   ##  ##   ##  ##   ##  
// ##       ##   ##  ##   ##  
// ##  ###   ## ##   ##   ##  
// ##   ##  ##   ##  ##   ##  
// ##   ##   #   ##  ##   ##  
//  ## ##    ## ##    ## ##


/*!
### G80 - Mesh-based Z probe <a href="https://reprap.org/wiki/G-code#G80:_Mesh-based_Z_probe">G80: Mesh-based Z probe</a>
Default 3x3 grid can be changed on MK2.5/s and MK3/s to 7x7 grid.
#### Usage

      G80 [ N | R | V | L | R | F | B ]
  
#### Parameters
  - `N` - Number of mesh points on x axis. Default is 3. Valid values are 3 and 7.
  - `R` - Probe retries. Default 3 max. 10
  - `V` - Verbosity level 1=low, 10=mid, 20=high. It only can be used if the firmware has been compiled with SUPPORT_VERBOSITY active.
  
  Using the following parameters enables additional "manual" bed leveling correction. Valid values are -100 microns to 100 microns.
#### Additional Parameters
  - `L` - Left Bed Level correct value in um.
  - `R` - Right Bed Level correct value in um.
  - `F` - Front Bed Level correct value in um.
  - `B` - Back Bed Level correct value in um.
*/

/*
* Probes a grid and produces a mesh to compensate for variable bed height
* The S0 report the points as below
*  +----> X-axis
*  |
*  |
*  v Y-axis
*/
static void gcode_G80(){
  mesh_bed_leveling_flag = true;
#ifndef PINDA_THERMISTOR
  static bool run = false; // thermistor-less PINDA temperature compensation is running
#endif // ndef PINDA_THERMISTOR

#ifdef SUPPORT_VERBOSITY
  int8_t verbosity_level = 0;
  if (code_seen('V')) {
    // Just 'V' without a number counts as V1.
    char c = strchr_pointer[1];
    verbosity_level = (c == ' ' || c == '\t' || c == 0) ? 1 : code_value_short();
  }
#endif //SUPPORT_VERBOSITY
  // Home if we don't know where we are
  if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
    // Push the commands to the front of the message queue in the reverse order!
    // There shall be always enough space reserved for these commands.
    repeatcommand_front(); // repeat G80 with all its parameters
    enquecommand_front_P((PSTR("G28 W0")));
    break;
  } 
		
  uint8_t nMeasPoints = MESH_MEAS_NUM_X_POINTS;
  if (code_seen('N')) {
    nMeasPoints = code_value_uint8();
    if (nMeasPoints != 7) {
      nMeasPoints = 3;
    }
  } else {
    nMeasPoints = eeprom_read_byte((uint8_t*)EEPROM_MBL_POINTS_NR);
  }

  uint8_t nProbeRetry = 3;
  if (code_seen('R')) {
    nProbeRetry = code_value_uint8();
    if (nProbeRetry > 10) {
      nProbeRetry = 10;
    }
  } else {
    nProbeRetry = eeprom_read_byte((uint8_t*)EEPROM_MBL_PROBE_NR);
  }
  bool magnet_elimination = (eeprom_read_byte((uint8_t*)EEPROM_MBL_MAGNET_ELIMINATION) > 0);
		
#ifndef PINDA_THERMISTOR
  if (run == false && temp_cal_active == true && calibration_status_pinda() == true && target_temperature_bed >= 50) {
    temp_compensation_start();
    run = true;
    repeatcommand_front(); // repeat G80 with all its parameters
    enquecommand_front_P((PSTR("G28 W0")));
    break;
  }
  run = false;
#endif //PINDA_THERMISTOR
  // Save custom message state, set a new custom message state to display: Calibrating point 9.
  CustomMsg custom_message_type_old = custom_message_type;
  unsigned int custom_message_state_old = custom_message_state;
  custom_message_type = CustomMsg::MeshBedLeveling;
  custom_message_state = (nMeasPoints * nMeasPoints) + 10;
  lcd_update(1);

  mbl.reset(); //reset mesh bed leveling

  // Reset baby stepping to zero, if the babystepping has already been loaded before. The babystepsTodo value will be
  // consumed during the first movements following this statement.
  babystep_undo();

  // Cycle through all points and probe them
  // First move up. During this first movement, the babystepping will be reverted.
  current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
  plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 60);
  // The move to the first calibration point.
  current_position[X_AXIS] = BED_X0;
  current_position[Y_AXIS] = BED_Y0;

#ifdef SUPPORT_VERBOSITY
  if (verbosity_level >= 1) {
      bool clamped = world2machine_clamp(current_position[X_AXIS], current_position[Y_AXIS]);
    clamped ? SERIAL_PROTOCOLPGM("First calibration point clamped.\n") : SERIAL_PROTOCOLPGM("No clamping for first calibration point.\n");
  }
#else //SUPPORT_VERBOSITY
    world2machine_clamp(current_position[X_AXIS], current_position[Y_AXIS]);
#endif //SUPPORT_VERBOSITY

  plan_buffer_line_curposXYZE(homing_feedrate[X_AXIS] / 30);
  // Wait until the move is finished.
  st_synchronize();

  uint8_t mesh_point = 0; //index number of calibration point

  int XY_AXIS_FEEDRATE = homing_feedrate[X_AXIS] / 20;
  int Z_LIFT_FEEDRATE = homing_feedrate[Z_AXIS] / 40;
  bool has_z = is_bed_z_jitter_data_valid(); //checks if we have data from Z calibration (offsets of the Z heiths of the 8 calibration points from the first point)
#ifdef SUPPORT_VERBOSITY
  if (verbosity_level >= 1) {
    has_z ? SERIAL_PROTOCOLPGM("Z jitter data from Z cal. valid.\n") : SERIAL_PROTOCOLPGM("Z jitter data from Z cal. not valid.\n");
  }
#endif // SUPPORT_VERBOSITY
  int l_feedmultiply = setup_for_endstop_move(false); //save feedrate and feedmultiply, sets feedmultiply to 100
  while (mesh_point != nMeasPoints * nMeasPoints) {
    // Get coords of a measuring point.
    uint8_t ix = mesh_point % nMeasPoints; // from 0 to MESH_NUM_X_POINTS - 1
    uint8_t iy = mesh_point / nMeasPoints;
    if (iy & 1) ix = (nMeasPoints - 1) - ix; // Zig zag
    if (nMeasPoints == 7) { //if we have 7x7 mesh, compare with Z-calibration for points which are in 3x3 mesh
      has_z = ((ix % 3 == 0) && (iy % 3 == 0)) && is_bed_z_jitter_data_valid(); 
    }
    float z0 = 0.f;
    if (has_z && (mesh_point > 0)) {
      uint16_t z_offset_u = 0;
      if (nMeasPoints == 7) {
        z_offset_u = eeprom_read_word((uint16_t*)(EEPROM_BED_CALIBRATION_Z_JITTER + 2 * ((ix/3) + iy - 1)));
      }	else {
        z_offset_u = eeprom_read_word((uint16_t*)(EEPROM_BED_CALIBRATION_Z_JITTER + 2 * (ix + iy * 3 - 1)));
      }
      z0 = mbl.z_values[0][0] + *reinterpret_cast<int16_t*>(&z_offset_u) * 0.01;
#ifdef SUPPORT_VERBOSITY
      if (verbosity_level >= 1) {
        printf_P(PSTR("Bed leveling, point: %d, calibration Z stored in eeprom: %d, calibration z: %f \n"), mesh_point, z_offset_u, z0);
      }
#endif // SUPPORT_VERBOSITY
    }

    // Move Z up to MESH_HOME_Z_SEARCH.
    if((ix == 0) && (iy == 0)) {
      current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
    } else {
      current_position[Z_AXIS] += 2.f / nMeasPoints; 
    }//use relative movement from Z coordinate where PINDa triggered on previous point. This makes calibration faster.
    float init_z_bckp = current_position[Z_AXIS];
    plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE);
    st_synchronize();

    // Move to XY position of the sensor point.
    current_position[X_AXIS] = BED_X(ix, nMeasPoints);
    current_position[Y_AXIS] = BED_Y(iy, nMeasPoints);

    #ifdef SUPPORT_VERBOSITY
    if (verbosity_level >= 1) {
      bool clamped = world2machine_clamp(current_position[X_AXIS], current_position[Y_AXIS]);
      SERIAL_PROTOCOL(mesh_point);
      clamped ? SERIAL_PROTOCOLPGM(": xy clamped.\n") : SERIAL_PROTOCOLPGM(": no xy clamping\n");
    }
    #else //SUPPORT_VERBOSITY
      world2machine_clamp(current_position[X_AXIS], current_position[Y_AXIS]);
    #endif // SUPPORT_VERBOSITY

    plan_buffer_line_curposXYZE(XY_AXIS_FEEDRATE);
    st_synchronize();

    // Go down until endstop is hit
    const float Z_CALIBRATION_THRESHOLD = 1.f;
    if (!find_bed_induction_sensor_point_z((has_z && mesh_point > 0) ? z0 - Z_CALIBRATION_THRESHOLD : -10.f, nProbeRetry)) { //if we have data from z calibration max allowed difference is 1mm for each point, if we dont have data max difference is 10mm from initial point  
      printf_P(_T(MSG_BED_LEVELING_FAILED_POINT_LOW));
      break;
    }
    if (init_z_bckp - current_position[Z_AXIS] < 0.1f) { //broken cable or initial Z coordinate too low. Go to MESH_HOME_Z_SEARCH and repeat last step (z-probe) again to distinguish between these two cases.
      //printf_P(PSTR("Another attempt! Current Z position: %f\n"), current_position[Z_AXIS]);
      current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
      plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE);
      st_synchronize();

      if (!find_bed_induction_sensor_point_z((has_z && mesh_point > 0) ? z0 - Z_CALIBRATION_THRESHOLD : -10.f, nProbeRetry)) { //if we have data from z calibration max allowed difference is 1mm for each point, if we dont have data max difference is 10mm from initial point  
        printf_P(_T(MSG_BED_LEVELING_FAILED_POINT_LOW));
        break;
      }
      if (MESH_HOME_Z_SEARCH - current_position[Z_AXIS] < 0.1f) {
        printf_P(PSTR("Bed leveling failed. Sensor disconnected or cable broken.\n"));
        break;
      }
    }
    if (has_z && fabs(z0 - current_position[Z_AXIS]) > Z_CALIBRATION_THRESHOLD) { //if we have data from z calibration, max. allowed difference is 1mm for each point
      printf_P(PSTR("Bed leveling failed. Sensor triggered too high.\n"));
      break;
    }
#ifdef SUPPORT_VERBOSITY
    if (verbosity_level >= 10) {
      SERIAL_ECHOPGM("X: ");
      MYSERIAL.print(current_position[X_AXIS], 5);
      SERIAL_ECHOLNPGM("");
      SERIAL_ECHOPGM("Y: ");
      MYSERIAL.print(current_position[Y_AXIS], 5);
      SERIAL_PROTOCOLPGM("\n");
    }
#endif // SUPPORT_VERBOSITY
    float offset_z = 0;

#ifdef PINDA_THERMISTOR
    offset_z = temp_compensation_pinda_thermistor_offset(current_temperature_pinda);
#endif //PINDA_THERMISTOR
    mbl.set_z(ix, iy, current_position[Z_AXIS] - offset_z); //store measured z values z_values[iy][ix] = z - offset_z;

    custom_message_state--;
    mesh_point++;
    lcd_update(1);
  }
  current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
#ifdef SUPPORT_VERBOSITY
  if (verbosity_level >= 20) {
    SERIAL_ECHOLNPGM("Mesh bed leveling while loop finished.");
    SERIAL_ECHOLNPGM("MESH_HOME_Z_SEARCH: ");
    MYSERIAL.print(current_position[Z_AXIS], 5);
  }
#endif // SUPPORT_VERBOSITY
  plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE);
  st_synchronize();
  if (mesh_point != nMeasPoints * nMeasPoints) {
    Sound_MakeSound(e_SOUND_TYPE_StandardAlert);
    bool bState;
    do {                             // repeat until Z-leveling o.k.
      lcd_display_message_fullscreen_P(_i("Some problem encountered, Z-leveling enforced ..."));
      lcd_wait_for_click_delay(0);  // ~ no timeout
      lcd_calibrate_z_end_stop_manual(true); // Z-leveling (X-assembly stay up!!!)
      // ~ Z-homing (can not be used "G28", because X & Y-homing would have been done before (Z-homing))
      bState=enable_z_endstop(false);
      current_position[Z_AXIS] -= 1;
      plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
      st_synchronize();
      enable_z_endstop(true);
      current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
      plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
      st_synchronize();
      enable_z_endstop(bState);
    } while (st_get_position_mm(Z_AXIS) > MESH_HOME_Z_SEARCH); // i.e. Z-leveling not o.k.
    //plan_set_z_position(MESH_HOME_Z_SEARCH); // is not necessary ('do-while' loop always ends at the expected Z-position)

    custom_message_type=CustomMsg::Status; // display / status-line recovery
    lcd_update_enable(true);           // display / status-line recovery
    gcode_G28(true, true, true);       // X & Y & Z-homing (must be after individual Z-homing (problem with spool-holder)!)
    repeatcommand_front();             // re-run (i.e. of "G80")
    break;
  }
  clean_up_after_endstop_move(l_feedmultiply);

#ifndef PINDA_THERMISTOR
  if(temp_cal_active == true && calibration_status_pinda() == true) temp_compensation_apply(); //apply PINDA temperature compensation
#endif //PINDA_THERMISTOR
  babystep_apply(); // Apply Z height correction aka baby stepping before mesh bed leveing gets activated.
  bool eeprom_bed_correction_valid = eeprom_read_byte((unsigned char*)EEPROM_BED_CORRECTION_VALID) == 1;
#ifdef SUPPORT_VERBOSITY
  if (verbosity_level >= 1) {
    eeprom_bed_correction_valid ? SERIAL_PROTOCOLPGM("Bed correction data valid\n") : SERIAL_PROTOCOLPGM("Bed correction data not valid\n");
  }
#endif // SUPPORT_VERBOSITY

  for (uint8_t i = 0; i < 4; ++i) {
    unsigned char codes[4] = { 'L', 'R', 'F', 'B' };
    long correction = 0;
    if (code_seen(codes[i])) {
      correction = code_value_long();
    } else if (eeprom_bed_correction_valid) {
      unsigned char *addr = (i < 2) ?
        ((i == 0) ? (unsigned char*)EEPROM_BED_CORRECTION_LEFT : (unsigned char*)EEPROM_BED_CORRECTION_RIGHT) :
        ((i == 2) ? (unsigned char*)EEPROM_BED_CORRECTION_FRONT : (unsigned char*)EEPROM_BED_CORRECTION_REAR);
      correction = eeprom_read_int8(addr);
    }
    if (correction == 0) {
      continue;
    }
    
    if (labs(correction) > BED_ADJUSTMENT_UM_MAX) {
      SERIAL_ERROR_START;
      SERIAL_ECHOPGM("Excessive bed leveling correction: ");
      SERIAL_ECHO(correction);
      SERIAL_ECHOLNPGM(" microns");
    }	else {
      float offset = float(correction) * 0.001f;
      switch (i) {
      case 0:
        for (uint8_t row = 0; row < nMeasPoints; ++row) {						
          for (uint8_t col = 0; col < nMeasPoints - 1; ++col) {
            mbl.z_values[row][col] += offset * (nMeasPoints - 1 - col) / (nMeasPoints - 1);
          }
        }
        break;
      case 1:
        for (uint8_t row = 0; row < nMeasPoints; ++row) {					
          for (uint8_t col = 1; col < nMeasPoints; ++col) {
            mbl.z_values[row][col] += offset * col / (nMeasPoints - 1);
          }
        }
        break;
      case 2:
        for (uint8_t col = 0; col < nMeasPoints; ++col) {						
          for (uint8_t row = 0; row < nMeasPoints; ++row) {
            mbl.z_values[row][col] += offset * (nMeasPoints - 1 - row) / (nMeasPoints - 1);
          }
        }
        break;
      case 3:
        for (uint8_t col = 0; col < nMeasPoints; ++col) {						
          for (uint8_t row = 1; row < nMeasPoints; ++row) {
            mbl.z_values[row][col] += offset * row / (nMeasPoints - 1);
          }
        }
        break;
      }
    }
  }
  if (nMeasPoints == 3) {
    mbl.upsample_3x3(); //interpolation from 3x3 to 7x7 points using largrangian polynomials while using the same array z_values[iy][ix] for storing (just coppying measured data to new destination and interpolating between them)
  }
  if (nMeasPoints == 7 && magnet_elimination) {
    mbl_interpolation(nMeasPoints);
  }
  mbl.active = 1; //activate mesh bed leveling
  go_home_with_z_lift();
  if ((degHotend(active_extruder) > EXTRUDE_MINTEMP) && eeprom_read_byte((unsigned char *)EEPROM_TEMP_CAL_ACTIVE) && calibration_status_pinda() && (target_temperature_bed >= 50)) {
    current_position[E_AXIS] += default_retraction;
    plan_buffer_line_curposXYZE(400);
  }
  KEEPALIVE_STATE(NOT_BUSY);
  // Restore custom message state
  lcd_setstatuspgm(_T(WELCOME_MSG));
  custom_message_type = custom_message_type_old;
  custom_message_state = custom_message_state_old;
  mesh_bed_leveling_flag = false;
  mesh_bed_run_from_menu = false;
  lcd_update(2);
}


/*!
### G81 - Mesh bed leveling status <a href="https://reprap.org/wiki/G-code#G81:_Mesh_bed_leveling_status">G81: Mesh bed leveling status</a>
Prints mesh bed leveling status and bed profile if activated.
*/
static void gcode_G81(){
  if (!(mbl.active)) {
    SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");
    return;
  }
  SERIAL_PROTOCOLPGM("Num X,Y: ");
  SERIAL_PROTOCOL(MESH_NUM_X_POINTS);
  SERIAL_PROTOCOL(',');
  SERIAL_PROTOCOL(MESH_NUM_Y_POINTS);
  SERIAL_PROTOCOLPGM("\nZ search height: ");
  SERIAL_PROTOCOL(MESH_HOME_Z_SEARCH);
  SERIAL_PROTOCOLLNPGM("\nMeasured points:");
  for (int y = MESH_NUM_Y_POINTS-1; y >= 0; y--) {
    for (int x = 0; x < MESH_NUM_X_POINTS; x++) {
      SERIAL_PROTOCOLPGM("  ");
      SERIAL_PROTOCOL_F(mbl.z_values[y][x], 5);
      }
    SERIAL_PROTOCOLLN();
  }
}



//  ## ##    ## ##   ## ##    
// ##   ##   #   ##  ##  ##   
// ##       ##   ##      ##   
// ##  ###   ## ###     ##    
// ##   ##       ##    ##     
// ##   ##  ##   ##   #   ##  
//  ## ##    ## ##   ###### 

/*!
  ### G92 - Set position <a href="https://reprap.org/wiki/G-code#G92:_Set_Position">G92: Set Position</a>
  
  It is used for setting the current position of each axis. The parameters are always absolute to the origin.
  If a parameter is omitted, that axis will not be affected.
  If `X`, `Y`, or `Z` axis are specified, the move afterwards might stutter because of Mesh Bed Leveling. `E` axis is not affected if the target position is 0 (`G92 E0`).
  A G92 without coordinates will reset all axes to zero on some firmware. This is not the case for Prusa-Firmware!
  
  #### Usage

      G92 [ X | Y | Z | E ]

  #### Parameters
  - `X` - new X axis position
  - `Y` - new Y axis position
  - `Z` - new Z axis position
  - `E` - new extruder position

  */
static void gcode_G92() {
  bool codes[NUM_AXIS];
  float values[NUM_AXIS];

  // Check which axes need to be set
  for(uint8_t i = 0; i < NUM_AXIS; ++i) {
    codes[i] = code_seen(axis_codes[i]);
    if(codes[i]) {
      values[i] = code_value();
    }
  }

  if((codes[E_AXIS] && values[E_AXIS] == 0) &&
  (!codes[X_AXIS] && !codes[Y_AXIS] && !codes[Z_AXIS])) {
    // As a special optimization, when _just_ clearing the E position
    // we schedule a flag asynchronously along with the next block to
    // reset the starting E position instead of stopping the planner
    current_position[E_AXIS] = 0;
    plan_reset_next_e();
  } else {
    // In any other case we're forced to synchronize
    st_synchronize();
    for(uint8_t i = 0; i < 3; ++i) {
      if(codes[i]) {
        current_position[i] = values[i] + cs.add_homing[i];
      }
    }
    if(codes[E_AXIS]) {
      current_position[E_AXIS] = values[E_AXIS];
    }
    // Set all at once
    plan_set_position_curposXYZE();
  }
}


                    //  ## ##      ##     ## ##    ## ##   
                    // ##   ##    ###    ##   ##  ##   ##  
                    // ##          ##    ##   ##  ##   ##  
                    // ##  ###     ##    ##   ##  ##   ##  
                    // ##   ##     ##    ##   ##  ##   ##  
                    // ##   ##     ##    ##   ##  ##   ##  
                    //  ## ##     ####    ## ##    ## ##  