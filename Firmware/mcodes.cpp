#include "mcodes.h"
#include "cmdqueue.h"
#include "sound.h"
#include "la10compat.h"
#include "Marlin.h"
#include "eeprom.h"
#include "planner.h"
#include "temperature.h"
#include "backlight.h"
#include "mesh_bed_leveling.h"


void adjust_bed_reset() {
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_VALID, 1);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_LEFT, 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_RIGHT, 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_FRONT, 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_REAR, 0);
}

/**/
void home_xy() {
  set_destination_to_current();
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  plan_set_position_curposXYZE();
  endstops_hit_on_purpose();
}



//M1
unsigned long gcode_M1(char *starpos){
	char *src = strchr_pointer + 2;
	unsigned long codenum = 0;

	bool hasP = false, hasS = false;
	if (code_seen('P')) {
		codenum = code_value(); // milliseconds to wait
		hasP = codenum > 0;
	}
	if (code_seen('S')) {
		codenum = code_value() * 1000; // seconds to wait
		hasS = codenum > 0;
	}
	starpos = strchr(src, '*');
	if (starpos != NULL) *(starpos) = '\0';
	while (*src == ' ') {++src;}
	if (!hasP && !hasS && *src != '\0') {
		lcd_setstatus(src);
	} else {
		LCD_MESSAGERPGM(_i("Wait for user..."));////MSG_USERWAIT
	}

	lcd_ignore_click();				//call lcd_ignore_click aslo for else ???
	st_synchronize();
	unsigned long previous_millis_cmd = _millis();
	if (codenum > 0){
		codenum += _millis();  // keep track of when we started waiting
		KEEPALIVE_STATE(PAUSED_FOR_USER);
		while(_millis() < codenum && !lcd_clicked()){
			manage_heater();
			manage_inactivity(true);
			lcd_update(0);
		}
		KEEPALIVE_STATE(IN_HANDLER);
		lcd_ignore_click(false);
	} else {
		marlin_wait_for_click();
	}
	if (IS_SD_PRINTING) {
		LCD_MESSAGERPGM(_T(MSG_RESUMING_PRINT));
	} else {
		LCD_MESSAGERPGM(_T(WELCOME_MSG));
	}
	return previous_millis_cmd;
}

// ### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
void gcode_M17(){
	LCD_MESSAGERPGM(_i("No move."));////MSG_NO_MOVE
	enable_x();
	enable_y();
	enable_z();
	enable_e0();
	enable_e1();
	enable_e2();
}


//	  ### M20 - SD Card file list <a href="https://reprap.org/wiki/G-code#M20:_List_SD_card">M20: List SD card</a>
void gcode_M20(){
	SERIAL_PROTOCOLLNRPGM(_N("Begin file list"));////MSG_BEGIN_FILE_LIST
	card.ls();
	SERIAL_PROTOCOLLNRPGM(_N("End file list"));////MSG_END_FILE_LIST
}


/**
  * M45: Calibrate XYZ
  */
//! @brief Calibrate XYZ
//! @param onlyZ if true, calibrate only Z axis
//! @param verbosity_level
//! @retval true Succeeded
//! @retval false Failed
bool gcode_M45(bool onlyZ, int8_t verbosity_level) {
	bool final_result = false;
  FORCE_BL_ON_START;
  // Only Z calibration?
	if (!onlyZ) {
		setTargetBed(0);
		setAllTargetHotends(0);
		adjust_bed_reset(); //reset bed level correction
	}

	// Disable the default update procedure of the display. We will do a modal dialog.
	lcd_update_enable(false);
	// Let the planner use the uncorrected coordinates.
	mbl.reset();
	// Reset world2machine_rotation_and_skew and world2machine_shift, therefore
	// the planner will not perform any adjustments in the XY plane. 
	// Wait for the motors to stop and update the current position with the absolute values.
	world2machine_revert_to_uncorrected();
	// Reset the baby step value applied without moving the axes.
	babystep_reset();
	// Mark all axes as in a need for homing.
	memset(axis_known_position, 0, sizeof(axis_known_position));

	// Home in the XY plane.
	//set_destination_to_current();
	int l_feedmultiply = setup_for_endstop_move();
	lcd_display_message_fullscreen_P(_T(MSG_AUTO_HOME));
	home_xy();

	enable_endstops(false);
	current_position[X_AXIS] += 5;
	current_position[Y_AXIS] += 5;
	plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
	st_synchronize();

	// Let the user move the Z axes up to the end stoppers.
	if (lcd_calibrate_z_end_stop_manual(onlyZ)) {
		lcd_show_fullscreen_message_and_wait_P(_T(MSG_CONFIRM_NOZZLE_CLEAN));
		if(onlyZ){
			lcd_display_message_fullscreen_P(_T(MSG_MEASURE_BED_REFERENCE_HEIGHT_LINE1));
			lcd_set_cursor(0, 3);
			lcd_print(1);
			lcd_puts_P(_T(MSG_MEASURE_BED_REFERENCE_HEIGHT_LINE2));
		} else {
			//lcd_show_fullscreen_message_and_wait_P(_T(MSG_PAPER));
			lcd_display_message_fullscreen_P(_T(MSG_FIND_BED_OFFSET_AND_SKEW_LINE1));
			lcd_set_cursor(0, 2);
			lcd_print(1);
			lcd_puts_P(_T(MSG_FIND_BED_OFFSET_AND_SKEW_LINE2));
		}

		refresh_cmd_timeout();
#ifndef STEEL_SHEET
		if (((degHotend(0) > MAX_HOTEND_TEMP_CALIBRATION) || (degBed() > MAX_BED_TEMP_CALIBRATION)) && (!onlyZ)) {
			lcd_wait_for_cool_down();
		}
#endif //STEEL_SHEET
		if(!onlyZ) {
			KEEPALIVE_STATE(PAUSED_FOR_USER);
#ifdef STEEL_SHEET
			bool result = lcd_show_fullscreen_message_yes_no_and_wait_P(_T(MSG_STEEL_SHEET_CHECK), false, false);
			if(result) lcd_show_fullscreen_message_and_wait_P(_T(MSG_REMOVE_STEEL_SHEET));
#endif //STEEL_SHEET
      lcd_show_fullscreen_message_and_wait_P(_T(MSG_PAPER));
      KEEPALIVE_STATE(IN_HANDLER);
      lcd_display_message_fullscreen_P(_T(MSG_FIND_BED_OFFSET_AND_SKEW_LINE1));
      lcd_set_cursor(0, 2);
      lcd_print(1);
      lcd_puts_P(_T(MSG_FIND_BED_OFFSET_AND_SKEW_LINE2));
    }
			
		bool endstops_enabled  = enable_endstops(false);
    current_position[Z_AXIS] -= 1; //move 1mm down with disabled endstop
    plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
    st_synchronize();
    // Move the print head close to the bed.
    current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
		enable_endstops(true);
		plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
		st_synchronize();
		enable_endstops(endstops_enabled);

		if ((st_get_position_mm(Z_AXIS) <= (MESH_HOME_Z_SEARCH + HOME_Z_SEARCH_THRESHOLD)) &&
    (st_get_position_mm(Z_AXIS) >= (MESH_HOME_Z_SEARCH - HOME_Z_SEARCH_THRESHOLD))) {
			if (onlyZ) {
				clean_up_after_endstop_move(l_feedmultiply);
				// Z only calibration.
				// Load the machine correction matrix
				world2machine_initialize();
				// and correct the current_position to match the transformed coordinate system.
				world2machine_update_current();
				//FIXME
				bool result = sample_mesh_and_store_reference();
				if (result) {
					if (calibration_status() == CALIBRATION_STATUS_Z_CALIBRATION) {
						// Shipped, the nozzle height has been set already. The user can start printing now.
						calibration_status_store(CALIBRATION_STATUS_CALIBRATED);
          }
          final_result = true;
					// babystep_apply();
				}
			} else {
				// Reset the baby step value and the baby step applied flag.
				calibration_status_store(CALIBRATION_STATUS_XYZ_CALIBRATION);
        eeprom_update_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),0);
				// Complete XYZ calibration.
				uint8_t point_too_far_mask = 0;
				BedSkewOffsetDetectionResultType result = find_bed_offset_and_skew(verbosity_level, point_too_far_mask);
				clean_up_after_endstop_move(l_feedmultiply);
				// Print head up.
				current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
				plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
				st_synchronize();
				if (result >= 0) {
#ifdef HEATBED_V2
					sample_z();
#else //HEATBED_V2
					point_too_far_mask = 0;
					// Second half: The fine adjustment.
					// Let the planner use the uncorrected coordinates.
					mbl.reset();
					world2machine_reset();
					// Home in the XY plane.
					int l_feedmultiply = setup_for_endstop_move();
					home_xy();
					result = improve_bed_offset_and_skew(1, verbosity_level, point_too_far_mask);
					clean_up_after_endstop_move(l_feedmultiply);
					// Print head up.
					current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
					plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
					st_synchronize();
					// if (result >= 0) babystep_apply();					
#endif //HEATBED_V2
				}
				lcd_update_enable(true);
				lcd_update(2);

				lcd_bed_calibration_show_result(result, point_too_far_mask);
				if (result >= 0) {
					// Calibration valid, the machine should be able to print. Advise the user to run the V2Calibration.gcode.
					calibration_status_store(CALIBRATION_STATUS_LIVE_ADJUST);
					if (eeprom_read_byte((uint8_t*)EEPROM_WIZARD_ACTIVE) != 1) lcd_show_fullscreen_message_and_wait_P(_T(MSG_BABYSTEP_Z_NOT_SET));
					final_result = true;
				}
			}
		} else {
			lcd_show_fullscreen_message_and_wait_P(PSTR("Calibration failed! Check the axes and run again."));
			final_result = false;
		}
	} 
	lcd_update_enable(true);
	  FORCE_BL_ON_END;
	return final_result;
}


//  ##   ##    ##      ####     ####   
//  ### ###    ##     ##  ##   ##  ##  
//  #######   ###     ## ###   ## ###  
//  ## # ##    ##     ### ##   ### ##  
//  ##   ##    ##     ##  ##   ##  ##  
//  ##   ##    ##     ##  ##   ##  ##  
//  ##   ##  ######    ####     ####


/**
  * M114: Get current position
  *
  */
void gcode_M114() {
	SERIAL_PROTOCOLPGM("X:");
	SERIAL_PROTOCOL(current_position[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(current_position[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(current_position[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOL(current_position[E_AXIS]);

	SERIAL_PROTOCOLRPGM(_n(" Count X: "));////MSG_COUNT_X
	SERIAL_PROTOCOL(float(st_get_position(X_AXIS)) / cs.axis_steps_per_unit[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(float(st_get_position(Y_AXIS)) / cs.axis_steps_per_unit[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(float(st_get_position(Z_AXIS)) / cs.axis_steps_per_unit[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOL(float(st_get_position(E_AXIS)) / cs.axis_steps_per_unit[E_AXIS]);

	SERIAL_PROTOCOLLN("");
}


/*!
  ---------------------------------------------------------------------------------
  ### M117 - Display Message <a href="https://reprap.org/wiki/G-code#M117:_Display_Message">M117: Display Message</a>
  This causes the given message to be shown in the status line on an attached LCD.
  It is processed early as to allow printing messages that contain G, M, N or T.
  
  ---------------------------------------------------------------------------------
  ### Special internal commands
  These are used by internal functions to process certain actions in the right order. Some of these are also usable by the user.
  They are processed early as the commands are complex (strings).
  These are only available on the MK3(S) as these require TMC2130 drivers:
    - CRASH DETECTED
    - CRASH RECOVER
    - CRASH_CANCEL
    - TMC_SET_WAVE
    - TMC_SET_STEP
    - TMC_SET_CHOP
 */
void M117(char *starpos){
	starpos = (strchr(strchr_pointer + 5, '*'));
	if (starpos != NULL) {
		*(starpos) = '\0';
    }
	lcd_setstatus(strchr_pointer + 5);
}



// ##   ##   ## ###   ## ##    ## ##   
//  ## ##   ##   ##  ##   ##  ##   ##  
// # ### #  ##       ##   ##  ##   ##  
// ## # ##  ## ###   ##   ##  ##   ##  
// ##   ##  ##   ##  ##   ##  ##   ##  
// ##   ##  ##   ##  ##   ##  ##   ##  
// ##   ##   ## ##    ## ##    ## ## 

	/*!
### M600 - Initiate Filament change procedure <a href="https://reprap.org/wiki/G-code#M600:_Filament_change_pause">M600: Filament change pause</a>
	Initiates Filament change, it is also used during Filament Runout Sensor process.
If the `M600` is triggered under 25mm it will do a Z-lift of 25mm to prevent a filament blob.
	#### Usage
	
			M600 [ X | Y | Z | E | L | AUTO ]
		
	- `X`    - X position, default 211
	- `Y`    - Y position, default 0
	- `Z`    - relative lift Z, default 2.
	- `E`    - initial retract, default -2
	- `L`    - later retract distance for removal, default -80
	- `AUTO` - Automatically (only with MMU)
	*/

//! @brief Wait for user action
//!
//! Beep, manage nozzle heater and wait for user to start unload filament
//! If times out, active extruder temperature is set to 0.
//!
//! @param HotendTempBckp Temperature to be restored for active extruder, after user resolves MMU problem.
void M600_wait_for_user(float HotendTempBckp) {

	KEEPALIVE_STATE(PAUSED_FOR_USER);

	int counterBeep = 0;
	unsigned long waiting_start_time = _millis();
	uint8_t wait_for_user_state = 0;
	lcd_display_message_fullscreen_P(_T(MSG_PRESS_TO_UNLOAD));
	bool bFirst=true;

	while (!(wait_for_user_state == 0 && lcd_clicked())){
		manage_heater();
		manage_inactivity(true);

		#if BEEPER > 0
		if (counterBeep == 500) {
			counterBeep = 0;
		}
		SET_OUTPUT(BEEPER);
		if (counterBeep == 0) {
			if((eSoundMode==e_SOUND_MODE_BLIND)|| (eSoundMode==e_SOUND_MODE_LOUD)||((eSoundMode==e_SOUND_MODE_ONCE)&&bFirst)) {
				bFirst=false;
				WRITE(BEEPER, HIGH);
			}
		}
		if (counterBeep == 20) {
			WRITE(BEEPER, LOW);
		}
				
		counterBeep++;
		#endif //BEEPER > 0
			
		switch (wait_for_user_state) {
		case 0: //nozzle is hot, waiting for user to press the knob to unload filament
			delay_keep_alive(4);

			if (_millis() > waiting_start_time + (unsigned long)M600_TIMEOUT * 1000) {
				lcd_display_message_fullscreen_P(_i("Press knob to preheat nozzle and continue."));////MSG_PRESS_TO_PREHEAT c=20 r=4
				wait_for_user_state = 1;
				setAllTargetHotends(0);
				st_synchronize();
				disable_e0();
				disable_e1();
				disable_e2();
			}
		break;
		case 1: //nozzle target temperature is set to zero, waiting for user to start nozzle preheat
			delay_keep_alive(4);
	
			if (lcd_clicked()) {
				setTargetHotend(HotendTempBckp, active_extruder);
				lcd_wait_for_heater();

				wait_for_user_state = 2;
			}
		break;
		case 2: //waiting for nozzle to reach target temperature

			if (abs(degTargetHotend(active_extruder) - degHotend(active_extruder)) < 1) {
				lcd_display_message_fullscreen_P(_T(MSG_PRESS_TO_UNLOAD));
				waiting_start_time = _millis();
				wait_for_user_state = 0;
			} else {
				counterBeep = 20; //beeper will be inactive during waiting for nozzle preheat
				lcd_set_cursor(1, 4);
				lcd_print(ftostr3(degHotend(active_extruder)));
			}
		break;
		}
	}
	WRITE(BEEPER, LOW);
}


void M600_load_filament_movements() {
	current_position[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
	plan_buffer_line_curposXYZE(FILAMENTCHANGE_EFEED_FIRST);                
	load_filament_final_feed();
	lcd_loading_filament();
	st_synchronize();
}


/**
  * M600: Load filament for any config
  *
  */
void M600_load_filament() {
	//load filament for single material
	lcd_wait_interact();
	KEEPALIVE_STATE(PAUSED_FOR_USER);
	while(!lcd_clicked()) {
		manage_heater();
		manage_inactivity(true);
#ifdef FILAMENT_SENSOR
		if (fsensor_check_autoload()) {
            Sound_MakeCustom(50,1000,false);
			break;
		}
#endif //FILAMENT_SENSOR
	}
	KEEPALIVE_STATE(IN_HANDLER);

#ifdef FSENSOR_QUALITY
	fsensor_oq_meassure_start(70);
#endif //FSENSOR_QUALITY

	M600_load_filament_movements();

	Sound_MakeCustom(50,1000,false);

#ifdef FSENSOR_QUALITY
	fsensor_oq_meassure_stop();

	if (!fsensor_oq_result()) {
		bool disable = lcd_show_fullscreen_message_yes_no_and_wait_P(_i("Fil. sensor response is poor, disable it?"), false, true);
		lcd_update_enable(true);
		lcd_update(2);
		if (disable) {
			fsensor_disable();
		}
	}
#endif //FSENSOR_QUALITY
	lcd_update_enable(false);
}


//! @brief Wait for user to check the state
//! @par nozzle_temp nozzle temperature to load filament
void M600_check_state(float nozzle_temp){
  lcd_change_fil_state = 0;
	while (lcd_change_fil_state != 1) {
		lcd_change_fil_state = 0;
		KEEPALIVE_STATE(PAUSED_FOR_USER);
		lcd_alright();
		KEEPALIVE_STATE(IN_HANDLER);
		switch(lcd_change_fil_state) {
			// Filament failed to load so load it again
			case 2:
				current_position[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
				plan_buffer_line_curposXYZE(FILAMENTCHANGE_EFEED_FIRST);
				load_filament_final_feed();
				lcd_loading_filament();
				st_synchronize();
			break;
			// Filament loaded properly but color is not clear
			case 3:
				st_synchronize();
				load_filament_final_feed();
				lcd_loading_color();
				st_synchronize();
			break;

			// Everything good
			default:
				lcd_change_success();
			break;
		}
	}
}


void gcode_M600(bool automatic, float x_position, float y_position, float z_shift, float e_shift, float e_shift_late) {
  st_synchronize();
  float lastpos[4];

  //First backup current position and settings
  int feedmultiplyBckp = feedmultiply;
  float HotendTempBckp = degTargetHotend(active_extruder);
  int fanSpeedBckp = fanSpeed;

  lastpos[X_AXIS] = current_position[X_AXIS];
  lastpos[Y_AXIS] = current_position[Y_AXIS];
  lastpos[Z_AXIS] = current_position[Z_AXIS];
  lastpos[E_AXIS] = current_position[E_AXIS];

  //Retract E
  current_position[E_AXIS] += e_shift;
  plan_buffer_line_curposXYZE(FILAMENTCHANGE_RFEED);
  st_synchronize();

  //Lift Z
  current_position[Z_AXIS] += z_shift;
  plan_buffer_line_curposXYZE(FILAMENTCHANGE_ZFEED);
  st_synchronize();

  //Move XY to side
  current_position[X_AXIS] = x_position;
  current_position[Y_AXIS] = y_position;
  plan_buffer_line_curposXYZE(FILAMENTCHANGE_XYFEED);
  st_synchronize();

  //Beep, manage nozzle heater and wait for user to start unload filament
  M600_wait_for_user(HotendTempBckp);
  lcd_change_fil_state = 0;

  // Unload filament
  unload_filament(); //unload filament for single material (used also in M702)
  //finish moves
  st_synchronize();

  KEEPALIVE_STATE(PAUSED_FOR_USER);
  lcd_change_fil_state = lcd_show_fullscreen_message_yes_no_and_wait_P(_i("Was filament unload successful?"), false, true); ////MSG_UNLOAD_SUCCESSFUL c=20 r=2
  if (lcd_change_fil_state == 0) {
    lcd_clear();
    lcd_set_cursor(0, 2);
    lcd_puts_P(_T(MSG_PLEASE_WAIT));
    current_position[X_AXIS] -= 100;
    plan_buffer_line_curposXYZE(FILAMENTCHANGE_XYFEED);
    st_synchronize();
    lcd_show_fullscreen_message_and_wait_P(_i("Please open idler and remove filament manually."));////MSG_CHECK_IDLER c=20 r=4
  }
  M600_load_filament();

  if (!automatic) {
    M600_check_state(HotendTempBckp);
  }
  lcd_update_enable(true);

  //Not let's go back to print
  fanSpeed = fanSpeedBckp;

  //Feed a little of filament to stabilize pressure
  if (!automatic) {
    current_position[E_AXIS] += FILAMENTCHANGE_RECFEED;
    plan_buffer_line_curposXYZE(FILAMENTCHANGE_EXFEED);
  }

  //Move XY back
  plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
  FILAMENTCHANGE_XYFEED, active_extruder);
  st_synchronize();
  //Move Z back
  plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], current_position[E_AXIS],
  FILAMENTCHANGE_ZFEED, active_extruder);
  st_synchronize();

  //Set E position to original
  plan_set_e_position(lastpos[E_AXIS]);

  memcpy(current_position, lastpos, sizeof(lastpos));
  memcpy(destination, current_position, sizeof(current_position));

  //Recover feed rate
  feedmultiply = feedmultiplyBckp;
  char cmd[9];
  sprintf_P(cmd, PSTR("M220 S%i"), feedmultiplyBckp);
  enquecommand(cmd);

#ifdef IR_SENSOR
	//this will set fsensor_watch_autoload to correct value and prevent possible M701 gcode enqueuing when M600 is finished
	fsensor_check_autoload();
#endif //IR_SENSOR

  lcd_setstatuspgm(_T(WELCOME_MSG));
  custom_message_type = CustomMsg::Status;
}

void gcode_M600(){
	st_synchronize();

	float x_position = current_position[X_AXIS];
	float y_position = current_position[Y_AXIS];
	float z_shift = 0; // is it necessary to be a float?
	float e_shift_init = 0;
	float e_shift_late = 0;
	bool automatic = false;

	//Retract extruder
	if(code_seen('E')) {
		e_shift_init = code_value();
	} else {
#ifdef FILAMENTCHANGE_FIRSTRETRACT
		e_shift_init = FILAMENTCHANGE_FIRSTRETRACT ;
#endif
	}

	//currently don't work as we are using the same unload sequence as in M702, needs re-work 
	if (code_seen('L')) {
		e_shift_late = code_value();
	} else {
#ifdef FILAMENTCHANGE_FINALRETRACT
		e_shift_late = FILAMENTCHANGE_FINALRETRACT;
#endif	
	}

	//Lift Z
	if(code_seen('Z')) {
		z_shift = code_value();
	} else {
#ifdef FILAMENTCHANGE_ZADD
		static_assert(Z_MAX_POS < (255 - FILAMENTCHANGE_ZADD), "Z-range too high, change the T type from uint8_t to uint16_t");
		// avoid floating point arithmetics when not necessary - results in shorter code
		float ztmp = ( current_position[Z_AXIS] );
		z_shift = 0;
		if(ztmp < 25){
			z_shift = 25 - ztmp; // make sure to be at least 25mm above the heat bed
		} 
		z_shift + (FILAMENTCHANGE_ZADD); // always move above printout
#else
		zshift = 0;
#endif
	}
	//Move XY to side
	if(code_seen('X')) {
		x_position = code_value();
	} else {
#ifdef FILAMENTCHANGE_XPOS
		x_position = FILAMENTCHANGE_XPOS;
#endif
	}

	if(code_seen('Y')) {
		y_position = code_value();
	} else {
#ifdef FILAMENTCHANGE_YPOS
		y_position = FILAMENTCHANGE_YPOS ;
#endif
	}

	gcode_M600(automatic, x_position, y_position, z_shift, e_shift_init, e_shift_late);
}






// ##   ##  ######    ## ##    ## ##   
//  ## ##   ##   #   ##   ##  ##   ##  
// # ### #     ##    ##   ##  ##   ##  
// ## # ##    ##     ##   ##  ##   ##  
// ##   ##    ##     ##   ##  ##   ##  
// ##   ##    ##     ##   ##  ##   ##  
// ##   ##    ##      ## ##    ## ##  

  /*!
  ### M701 - Load filament <a href="https://reprap.org/wiki/G-code#M701:_Load_filament">M701: Load filament</a>
  */
void gcode_M701() {
	printf_P(PSTR("gcode_M701 begin\n"));

  enable_z();
  custom_message_type = CustomMsg::FilamentLoading;

#ifdef FSENSOR_QUALITY
  fsensor_oq_meassure_start(40);
#endif //FSENSOR_QUALITY

  lcd_setstatuspgm(_T(MSG_LOADING_FILAMENT));
  current_position[E_AXIS] += 40;
  plan_buffer_line_curposXYZE(400 / 60); //fast sequence
  st_synchronize();
  raise_z_above(MIN_Z_FOR_LOAD, false);
  current_position[E_AXIS] += 30;
  plan_buffer_line_curposXYZE(400 / 60); //fast sequence
  load_filament_final_feed(); //slow sequence
  st_synchronize();

  Sound_MakeCustom(50,500,false);

  if (loading_flag) {
    lcd_load_filament_color_check();
  }
  lcd_update_enable(true);
  lcd_update(2);
  lcd_setstatuspgm(_T(WELCOME_MSG));
  disable_z();
  loading_flag = false;
  custom_message_type = CustomMsg::Status;
}



// ##   ##   ## ##    ## ##    ## ##   
//  ## ##    #   ##  ##   ##  ##   ##  
// # ### #  ##   ##  ##   ##  ##   ##  
// ## # ##   ## ###  ##   ##  ##   ##  
// ##   ##       ##  ##   ##  ##   ##  
// ##   ##  ##   ##  ##   ##  ##   ##  
// ##   ##   ## ##    ## ##    ## ##  

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



