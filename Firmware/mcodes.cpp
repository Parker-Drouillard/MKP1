#include "mcodes.h"



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
#ifdef TMC2130
	FORCE_HIGH_POWER_START;
#endif // TMC2130
    
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
#ifdef TMC2130
	if (calibrate_z_auto()) {
#else //TMC2130
	if (lcd_calibrate_z_end_stop_manual(onlyZ)) {
#endif //TMC2130
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
#ifdef TMC2130
		tmc2130_home_enter(Z_AXIS_MASK);
#endif //TMC2130
		plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 40);
		st_synchronize();
#ifdef TMC2130
		tmc2130_home_exit();
#endif //TMC2130
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
#ifdef TMC2130
			tmc2130_home_exit();
#endif
		} else {
			lcd_show_fullscreen_message_and_wait_P(PSTR("Calibration failed! Check the axes and run again."));
			final_result = false;
		}
	} 
	lcd_update_enable(true);
#ifdef TMC2130
	FORCE_HIGH_POWER_END;
#endif // TMC2130
  FORCE_BL_ON_END;
	return final_result;
}




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



/**
  * M600: Load filament for any config
  *
  */
void M600_load_filament() {
	//load filament for single material and SNMM 
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
		if (disable)
			fsensor_disable();
	}
#endif //FSENSOR_QUALITY
	lcd_update_enable(false);
}



static void gcode_M600(bool automatic, float x_position, float y_position, float z_shift, float e_shift, float /*e_shift_late*/) {
  st_synchronize();
  float lastpos[4];

  if (farm_mode) {
      prusa_statistics(22);
  }

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



