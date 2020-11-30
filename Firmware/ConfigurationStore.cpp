//! @file

#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"
#include "Configuration_prusa.h"


M500_conf cs;

//! @brief Write data to EEPROM
//! @param pos destination in EEPROM, 0 is start
//! @param value value to be written
//! @param size size of type pointed by value
//! @param name name of variable written, used only for debug input if DEBUG_EEPROM_WRITE defined
//! @retval true success
//! @retval false failed
#ifdef DEBUG_EEPROM_WRITE
static bool EEPROM_writeData(uint8_t* pos, uint8_t* value, uint8_t size, const char* name)
#else //DEBUG_EEPROM_WRITE
static bool EEPROM_writeData(uint8_t* pos, uint8_t* value, uint8_t size, const char*)
#endif //DEBUG_EEPROM_WRITE
{
#ifdef DEBUG_EEPROM_WRITE
	printf_P(PSTR("EEPROM_WRITE_VAR addr=0x%04x size=0x%02hhx name=%s\n"), pos, size, name);
#endif //DEBUG_EEPROM_WRITE
	while (size--){

        eeprom_update_byte(pos, *value);
        if (eeprom_read_byte(pos) != *value) {
            SERIAL_ECHOLNPGM("EEPROM Error");
            return false;
        }

		pos++;
		value++;
	}
    return true;
}

#ifdef DEBUG_EEPROM_READ
static void EEPROM_readData(uint8_t* pos, uint8_t* value, uint8_t size, const char* name)
#else //DEBUG_EEPROM_READ
static void EEPROM_readData(uint8_t* pos, uint8_t* value, uint8_t size, const char*)
#endif //DEBUG_EEPROM_READ
{
#ifdef DEBUG_EEPROM_READ
	printf_P(PSTR("EEPROM_READ_VAR addr=0x%04x size=0x%02hhx name=%s\n"), pos, size, name);
#endif //DEBUG_EEPROM_READ
    while(size--)
    {
        *value = eeprom_read_byte(pos);
        pos++;
        value++;
    }
}

#define EEPROM_VERSION "V3"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() {
  strcpy(cs.version,"000"); //!< invalidate data first @TODO use erase to save one erase cycle
  
  if (EEPROM_writeData(reinterpret_cast<uint8_t*>(EEPROM_M500_base),reinterpret_cast<uint8_t*>(&cs),sizeof(cs),0), "cs, invalid version") {
      strcpy(cs.version,EEPROM_VERSION); //!< validate data if write succeed
      EEPROM_writeData(reinterpret_cast<uint8_t*>(EEPROM_M500_base->version), reinterpret_cast<uint8_t*>(cs.version), sizeof(cs.version), "cs.version valid");
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings(uint8_t level) {  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	printf_P(PSTR(
		"%SSteps per unit:\n%S  M92 X%.2f Y%.2f Z%.2f E%.2f\n"
		"%SMaximum feedrates (mm/s):\n%S  M203 X%.2f Y%.2f Z%.2f E%.2f\n"
		"%SMaximum acceleration (mm/s2):\n%S  M201 X%lu Y%lu Z%lu E%lu\n"
		"%SAcceleration: S=acceleration, T=retract acceleration\n%S  M204 S%.2f T%.2f\n"
		"%SAdvanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)\n%S  M205 S%.2f T%.2f B%.2f X%.2f Y%.2f Z%.2f E%.2f\n"
		"%SHome offset (mm):\n%S  M206 X%.2f Y%.2f Z%.2f\n"
		),
		echomagic, echomagic, cs.axis_steps_per_unit[X_AXIS], cs.axis_steps_per_unit[Y_AXIS], cs.axis_steps_per_unit[Z_AXIS], cs.axis_steps_per_unit[E_AXIS],
		echomagic, echomagic, max_feedrate[X_AXIS], max_feedrate[Y_AXIS], max_feedrate[Z_AXIS], max_feedrate[E_AXIS],
		echomagic, echomagic, max_acceleration_units_per_sq_second[X_AXIS], max_acceleration_units_per_sq_second[Y_AXIS], max_acceleration_units_per_sq_second[Z_AXIS], max_acceleration_units_per_sq_second[E_AXIS],
		echomagic, echomagic, cs.acceleration, cs.retract_acceleration,
		echomagic, echomagic, cs.minimumfeedrate, cs.mintravelfeedrate, cs.minsegmenttime, cs.max_jerk[X_AXIS], cs.max_jerk[Y_AXIS], cs.max_jerk[Z_AXIS], cs.max_jerk[E_AXIS],
		echomagic, echomagic, cs.add_homing[X_AXIS], cs.add_homing[Y_AXIS], cs.add_homing[Z_AXIS]
	);
	if (level >= 10) {
	}
}
#endif


#ifdef EEPROM_SETTINGS

// static_assert (EXTRUDERS == 1, "ConfigurationStore M500_conf not implemented for more extruders, fix filament_size array size.");
static_assert (NUM_AXIS == 4, "ConfigurationStore M500_conf not implemented for more axis."
        "Fix axis_steps_per_unit max_feedrate_normal max_acceleration_units_per_sq_second_normal max_jerk max_feedrate_silent"
        " max_acceleration_units_per_sq_second_silent array size.");


static_assert (sizeof(M500_conf) == 196, "sizeof(M500_conf) has changed, ensure that EEPROM_VERSION has been incremented, "
        "or if you added members in the end of struct, ensure that historically uninitialized values will be initialized."
        "If this is caused by change to more then 8bit processor, decide whether make this struct packed to save EEPROM,"
        "leave as it is to keep fast code, or reorder struct members to pack more tightly.");

static const M500_conf default_conf PROGMEM =
{
    EEPROM_VERSION,
    DEFAULT_AXIS_STEPS_PER_UNIT,
    DEFAULT_MAX_FEEDRATE,
    DEFAULT_MAX_ACCELERATION,
    DEFAULT_ACCELERATION,
    DEFAULT_RETRACT_ACCELERATION,
    DEFAULT_MINIMUMFEEDRATE,
    DEFAULT_MINTRAVELFEEDRATE,
    DEFAULT_MINSEGMENTTIME,
    {DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK},
    {0,0,0},
    -(Z_PROBE_OFFSET_FROM_EXTRUDER),
    DEFAULT_Kp,
    DEFAULT_Ki*PID_dT,
    DEFAULT_Kd/PID_dT,
    DEFAULT_bedKp,
    DEFAULT_bedKi*PID_dT,
    DEFAULT_bedKd/PID_dT,
    0,
    false,
    RETRACT_LENGTH,
    RETRACT_FEEDRATE,
    RETRACT_ZLIFT,
    RETRACT_RECOVER_LENGTH,
    RETRACT_RECOVER_FEEDRATE,
    false,
    {DEFAULT_NOMINAL_FILAMENT_DIA,
#if EXTRUDERS > 1
    DEFAULT_NOMINAL_FILAMENT_DIA,
#if EXTRUDERS > 2
    DEFAULT_NOMINAL_FILAMENT_DIA,
#endif
#endif
    },
    DEFAULT_MAX_FEEDRATE_SILENT,
    DEFAULT_MAX_ACCELERATION_SILENT,
    {16,16,16,16},
};

//! @brief Read M500 configuration
//! @retval true Succeeded. Stored settings retrieved or default settings retrieved in case EEPROM has been erased.
//! @retval false Failed. Default settings has been retrieved, because of older version or corrupted data.
bool Config_RetrieveSettings() {
	bool previous_settings_retrieved = true;
    char ver[4]=EEPROM_VERSION;
    EEPROM_readData(reinterpret_cast<uint8_t*>(EEPROM_M500_base->version), reinterpret_cast<uint8_t*>(cs.version), sizeof(cs.version), "cs.version"); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << cs.version << "]");
    if (strncmp(ver,cs.version,3) == 0) {  // version number match

        EEPROM_readData(reinterpret_cast<uint8_t*>(EEPROM_M500_base), reinterpret_cast<uint8_t*>(&cs), sizeof(cs), "cs");
        
		if (cs.max_jerk[X_AXIS] > DEFAULT_XJERK) cs.max_jerk[X_AXIS] = DEFAULT_XJERK;
		if (cs.max_jerk[Y_AXIS] > DEFAULT_YJERK) cs.max_jerk[Y_AXIS] = DEFAULT_YJERK;
        calculate_extruder_multipliers();

		//if max_feedrate_silent and max_acceleration_units_per_sq_second_silent were never stored to eeprom, use default values:
        for (uint8_t i = 0; i < (sizeof(cs.max_feedrate_silent)/sizeof(cs.max_feedrate_silent[0])); ++i) {
            const uint32_t erased = 0xffffffff;
            bool initialized = false;

            for(uint8_t j = 0; j < sizeof(float); ++j) {
                if(0xff != reinterpret_cast<uint8_t*>(&(cs.max_feedrate_silent[i]))[j]) initialized = true;
            }
            if (!initialized) memcpy_P(&cs.max_feedrate_silent[i],&default_conf.max_feedrate_silent[i], sizeof(cs.max_feedrate_silent[i]));
            if (erased == cs.max_acceleration_units_per_sq_second_silent[i]) {
                memcpy_P(&cs.max_acceleration_units_per_sq_second_silent[i],&default_conf.max_acceleration_units_per_sq_second_silent[i],sizeof(cs.max_acceleration_units_per_sq_second_silent[i]));
            }
        }
		reset_acceleration_rates();

        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    } else {
        Config_ResetDefault();
		//Return false to inform user that eeprom version was changed and firmware is using default hardcoded settings now.
		//In case that storing to eeprom was not used yet, do not inform user that hardcoded settings are used.
		if (eeprom_read_byte(reinterpret_cast<uint8_t*>(&(EEPROM_M500_base->version[0]))) != 0xFF ||
			eeprom_read_byte(reinterpret_cast<uint8_t*>(&(EEPROM_M500_base->version[1]))) != 0xFF ||
			eeprom_read_byte(reinterpret_cast<uint8_t*>(&(EEPROM_M500_base->version[2]))) != 0xFF) {
			previous_settings_retrieved = false;
		}
    }
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
	return previous_settings_retrieved;
}
#endif

void Config_ResetDefault() {
    memcpy_P(&cs,&default_conf, sizeof(cs));

	// steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
	calculate_extruder_multipliers();
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}
