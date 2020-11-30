/* -*- c++ -*- */
/**
 * @file
 */

/**
 * @mainpage Reprap 3D printer firmware based on Sprinter and grbl.
 *
 * @section intro_sec Introduction
 *
 * This firmware is a mashup between Sprinter and grbl.
 * https://github.com/kliment/Sprinter
 * https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 * http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 *
 * Prusa Research s.r.o. https://www.prusa3d.cz
 *
 * @section copyright_sec Copyright
 *
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @section notes_sec Notes
 *
 * * Do not create static objects in global functions.
 *   Otherwise constructor guard against concurrent calls is generated costing
 *   about 8B RAM and 14B flash.
 *
 *
 */

//-//
#include "Configuration.h"
#include "Marlin.h"
#include "config.h"

#include "macros.h"



#include "printers.h"

#include "menu.h"
#include "ultralcd.h"
#include "backlight.h"

#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "util.h"
#include "Timer.h"

#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "Dcodes.h"
#include "AutoDeplete.h"

#ifndef LA_NOCOMPAT
#include "la10compat.h"
#endif

#ifdef SWSPI
#include "swspi.h"
#endif //SWSPI

#include "spi.h"

#ifdef SWI2C
#include "swi2c.h"
#endif //SWI2C



#ifdef W25X20CL
#include "w25x20cl.h"
#include "optiboot_w25x20cl.h"
#endif //W25X20CL

#ifdef BLINKM
#include "BlinkM.h"
#include "Wire.h"
#endif

#ifdef ULTRALCD
#include "ultralcd.h"
#endif


#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.0.2"


#include "ultralcd.h"
#include "sound.h"

#include "cmdqueue.h"

//Macro for print fan speed
#define FAN_PULSE_WIDTH_LIMIT ((fanSpeed > 100) ? 3 : 4) //time in ms

//filament types 
#define FILAMENT_DEFAULT 0
#define FILAMENT_FLEX 1
#define FILAMENT_PVA 2
#define FILAMENT_UNDEFINED 255

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================

//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif

unsigned long PingTime = _millis();
unsigned long NcTime;

uint8_t mbl_z_probe_nr = 3; //numer of Z measurements for each point in mesh bed leveling calibration

//used for PINDA temp calibration and pause print
#define DEFAULT_RETRACTION    1
#define DEFAULT_RETRACTION_MM 4 //MM

float default_retraction = DEFAULT_RETRACTION;


float homing_feedrate[] = HOMING_FEEDRATE;

//Although this flag and many others like this could be represented with a struct/bitfield for each axis (more readable and efficient code), the implementation
//would not be standard across all platforms. That being said, the code will continue to use bitmasks for independent axis.
//Moreover, according to C/C++ standard, the ordering of bits is platform/compiler dependent and the compiler is allowed to align the bits arbitrarily,
//thus bit operations like shifting and masking may stop working and will be very hard to fix.
uint8_t axis_relative_modes = 0;

int feedmultiply=100; //100->1 200->2
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100
  #if EXTRUDERS > 1
    , 100
    #if EXTRUDERS > 2
      , 100
    #endif
  #endif
};

int bowden_length[4] = {385, 385, 385, 385};

bool is_usb_printing = false;
bool homing_flag = false;

unsigned long kicktime = _millis()+100000;

unsigned int  usb_printing_counter;

int8_t lcd_change_fil_state = 0;

unsigned long pause_time = 0;
unsigned long start_pause_print = _millis();
unsigned long t_fan_rising_edge = _millis();
LongTimer safetyTimer;
static LongTimer crashDetTimer;

//unsigned long load_filament_time;


bool prusa_sd_card_upload = false;

unsigned int status_number = 0;

unsigned long total_filament_used;
unsigned int heating_status;
unsigned int heating_status_counter;
bool loading_flag = false;



char snmm_filaments_used = 0;


bool fan_state[2];
int fan_edge_counter[2];
int fan_speed[2];

char dir_names[3][9];

bool sortAlpha = false;


float extruder_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
    #endif
  #endif
};

float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
//shortcuts for more readable code
#define _x current_position[X_AXIS]
#define _y current_position[Y_AXIS]
#define _z current_position[Z_AXIS]
#define _e current_position[E_AXIS]

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};

// Extruder offset
#if EXTRUDERS > 1
  #define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif

uint8_t active_extruder = 0;
int fanSpeed=0;

#ifdef FWRETRACT
  bool retracted[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };
  bool retracted_swap[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };

  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
#endif

  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif

bool cancel_heatup = false ;

int8_t busy_state = NOT_BUSY;
static long prev_busy_signal_ms = -1;
uint8_t host_keepalive_interval = HOST_KEEPALIVE_INTERVAL;

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

bool no_response = false;
uint8_t important_status;
uint8_t saved_filament_type;

#define SAVED_TARGET_UNSET (X_MIN_POS-1)
float saved_target[NUM_AXIS] = {SAVED_TARGET_UNSET, 0, 0, 0};

// save/restore printing in case that mmu was not responding 
bool mmu_print_saved = false;

// storing estimated time to end of print counted by slicer
uint8_t print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
uint16_t print_time_remaining_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
uint8_t print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
uint16_t print_time_remaining_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
#define MSG_BED_LEVELING_FAILED_TIMEOUT 30

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

// For tracing an arc
static float offset[3] = {0.0, 0.0, 0.0};

// Current feedrate
float feedrate = 1500.0;

// Feedrate for the next move
static float next_feedrate;

// Original feedrate saved during homing moves
static float saved_feedrate;

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
static unsigned long safetytimer_inactive_time = DEFAULT_SAFETYTIMER_TIME_MINS*60*1000ul;

unsigned long starttime=0;
unsigned long stoptime=0;
unsigned long _usb_timer = 0;

static uint8_t tmp_extruder;

bool Stopped=false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool target_direction;

//Insert variables if CHDK is defined
#ifdef CHDK
unsigned long chdkHigh = 0;
boolean chdkActive = false;
#endif

//! @name RAM save/restore printing
//! @{
bool saved_printing = false; //!< Print is paused and saved in RAM
static uint32_t saved_sdpos = 0; //!< SD card position, or line number in case of USB printing
uint8_t saved_printing_type = PRINTING_TYPE_SD;
static float saved_pos[4] = { 0, 0, 0, 0 };
static uint16_t saved_feedrate2 = 0; //!< Default feedrate (truncated from float)
static int saved_feedmultiply2 = 0;
static uint8_t saved_active_extruder = 0;
static float saved_extruder_temperature = 0.0; //!< Active extruder temperature
static bool saved_extruder_relative_mode = false;
static int saved_fanSpeed = 0; //!< Print fan speed
//! @}

static int saved_feedmultiply_mm = 100;

#ifdef AUTO_REPORT_TEMPERATURES
static LongTimer auto_report_temp_timer;
static uint8_t auto_report_temp_period = 0;
#endif //AUTO_REPORT_TEMPERATURES

//===========================================================================
//=============================Routines======================================
//===========================================================================

static void get_arc_coordinates();
static void wait_for_heater(long codenum, uint8_t extruder);
static void temp_compensation_start();
static void temp_compensation_apply();


uint16_t gcode_in_progress = 0;
uint16_t mcode_in_progress = 0;

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

/*FORCE_INLINE*/ void serialprintPGM(const char *str)
{
#if 0
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
#else
	// hmm, same size as the above version, the compiler did a good job optimizing the above
	while( uint8_t ch = pgm_read_byte(str) ){
	  MYSERIAL.write((char)ch);
	  ++str;
	}
#endif
}

#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif //!SDSUPPORT

void setup_killpin() {
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

// Set home pin
void setup_homepin(void) {
#if defined(HOME_PIN) && HOME_PIN > -1
   SET_INPUT(HOME_PIN);
   WRITE(HOME_PIN,HIGH);
#endif
}

void setup_photpin() {
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold() {
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide() {
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif
}


bool fans_check_enabled = true;



void failstats_reset_print() {
	eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_X, 0);
	eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_Y, 0);
	eeprom_update_byte((uint8_t *)EEPROM_FERROR_COUNT, 0);
	eeprom_update_byte((uint8_t *)EEPROM_POWER_COUNT, 0);
	eeprom_update_byte((uint8_t *)EEPROM_MMU_FAIL, 0);
	eeprom_update_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL, 0);
#if defined(FILAMENT_SENSOR) && defined(PAT9125)
    fsensor_softfail = 0;
#endif
}

void softReset() {
    cli();
    wdt_enable(WDTO_15MS);
    while(1);
}


#ifdef MESH_BED_LEVELING
   enum MeshLevelingState { MeshReport, MeshStart, MeshNext, MeshSet };
#endif


// Factory reset function
// This function is used to erase parts or whole EEPROM memory which is used for storing calibration and and so on.
// Level input parameter sets depth of reset
int  er_progress = 0;
static void factory_reset(char level) {	
	lcd_clear();
  switch (level) {      
        // Level 0: Language reset
    case 0:
      Sound_MakeCustom(100,0,false);
			lang_reset();
    break;
         
		//Level 1: Reset statistics
		case 1:
      Sound_MakeCustom(100,0,false);
			eeprom_update_dword((uint32_t *)EEPROM_TOTALTIME, 0);
			eeprom_update_dword((uint32_t *)EEPROM_FILAMENTUSED, 0);

			eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_X, 0);
			eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_Y, 0);
			eeprom_update_byte((uint8_t *)EEPROM_FERROR_COUNT, 0);
			eeprom_update_byte((uint8_t *)EEPROM_POWER_COUNT, 0);

			eeprom_update_word((uint16_t *)EEPROM_CRASH_COUNT_X_TOT, 0);
			eeprom_update_word((uint16_t *)EEPROM_CRASH_COUNT_Y_TOT, 0);
			eeprom_update_word((uint16_t *)EEPROM_FERROR_COUNT_TOT, 0);
			eeprom_update_word((uint16_t *)EEPROM_POWER_COUNT_TOT, 0);

			eeprom_update_word((uint16_t *)EEPROM_MMU_FAIL_TOT, 0);
			eeprom_update_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT, 0);
			eeprom_update_byte((uint8_t *)EEPROM_MMU_FAIL, 0);
			eeprom_update_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL, 0);


			lcd_menu_statistics();
            
    break;

    // Level 2: Prepare for shipping
    case 2:
			//lcd_puts_P(PSTR("Factory RESET"));
      //lcd_puts_at_P(1,2,PSTR("Shipping prep"));
      
      // Force language selection at the next boot up.
			lang_reset();
      // Force the "Follow calibration flow" message at the next boot up.
      calibration_status_store(CALIBRATION_STATUS_Z_CALIBRATION);
			eeprom_write_byte((uint8_t*)EEPROM_WIZARD_ACTIVE, 1); //run wizard
      farm_no = 0;
			farm_mode = false;
			eeprom_update_byte((uint8_t*)EEPROM_FARM_MODE, farm_mode);
      EEPROM_save_B(EEPROM_FARM_NUMBER, &farm_no);

      eeprom_update_dword((uint32_t *)EEPROM_TOTALTIME, 0);
      eeprom_update_dword((uint32_t *)EEPROM_FILAMENTUSED, 0);

			eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_X, 0);
			eeprom_update_byte((uint8_t *)EEPROM_CRASH_COUNT_Y, 0);
			eeprom_update_byte((uint8_t *)EEPROM_FERROR_COUNT, 0);
			eeprom_update_byte((uint8_t *)EEPROM_POWER_COUNT, 0);

      eeprom_update_word((uint16_t *)EEPROM_CRASH_COUNT_X_TOT, 0);
      eeprom_update_word((uint16_t *)EEPROM_CRASH_COUNT_Y_TOT, 0);
      eeprom_update_word((uint16_t *)EEPROM_FERROR_COUNT_TOT, 0);
      eeprom_update_word((uint16_t *)EEPROM_POWER_COUNT_TOT, 0);

			eeprom_update_word((uint16_t *)EEPROM_MMU_FAIL_TOT, 0);
			eeprom_update_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT, 0);
			eeprom_update_byte((uint8_t *)EEPROM_MMU_FAIL, 0);
			eeprom_update_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL, 0);

#ifdef FILAMENT_SENSOR
			fsensor_enable();
            fsensor_autoload_set(true);
#endif //FILAMENT_SENSOR
      Sound_MakeCustom(100,0,false);   
			//_delay_ms(2000);
    break;

    // Level 3: erase everything, whole EEPROM will be set to 0xFF  
		case 3:
			lcd_puts_P(PSTR("Factory RESET"));
			lcd_puts_at_P(1, 2, PSTR("ERASING all data"));

      Sound_MakeCustom(100,0,false);
			er_progress = 0;
			lcd_puts_at_P(3, 3, PSTR("      "));
			lcd_set_cursor(3, 3);
			lcd_print(er_progress);

			// Erase EEPROM
			for (int i = 0; i < 4096; i++) {
				eeprom_update_byte((uint8_t*)i, 0xFF);
				if (i % 41 == 0) {
					er_progress++;
					lcd_puts_at_P(3, 3, PSTR("      "));
					lcd_set_cursor(3, 3);
					lcd_print(er_progress);
					lcd_puts_P(PSTR("%"));
				}
			}
			softReset();
    break;


    default:
    break;
    }  
}

extern "C" {
  FILE _uartout; //= {0}; Global variable is always zero initialized. No need to explicitly state this.
}

int uart_putchar(char c, FILE *) {
	MYSERIAL.write(c);
	return 0;
}


void lcd_splash() {
	lcd_clear(); // clears display and homes screen
	lcd_puts_P(PSTR("\n Pinda tester V1\n Pep Corporation"));
}


void factory_reset()  {
	KEEPALIVE_STATE(PAUSED_FOR_USER);
	if (!READ(BTN_ENC)) {
		_delay_ms(1000);
		if (!READ(BTN_ENC)) {
			lcd_clear();
			lcd_puts_P(PSTR("Factory RESET"));
			SET_OUTPUT(BEEPER);
      if(eSoundMode!=e_SOUND_MODE_SILENT) {
        WRITE(BEEPER, HIGH);
      }
			while (!READ(BTN_ENC)){};

			WRITE(BEEPER, LOW);
			_delay_ms(2000);
			char level = reset_menu();
			factory_reset(level);

			switch (level) {
			case 0: _delay_ms(0); break;
			case 1: _delay_ms(0); break;
			case 2: _delay_ms(0); break;
			case 3: _delay_ms(0); break;
			}

		}
	}
	KEEPALIVE_STATE(IN_HANDLER);
}

void show_fw_version_warnings() {
	if (FW_DEV_VERSION == FW_VERSION_GOLD || FW_DEV_VERSION == FW_VERSION_RC) {return;}
	switch (FW_DEV_VERSION) {
    case(FW_VERSION_ALPHA):   lcd_show_fullscreen_message_and_wait_P(_i("You are using firmware alpha version. This is development version. Using this version is not recommended and may cause printer damage."));   break;////MSG_FW_VERSION_ALPHA c=20 r=8
    case(FW_VERSION_BETA):    lcd_show_fullscreen_message_and_wait_P(_i("You are using firmware beta version. This is development version. Using this version is not recommended and may cause printer damage."));    break;////MSG_FW_VERSION_BETA c=20 r=8
    case(FW_VERSION_DEVEL):
    case(FW_VERSION_DEBUG):
      lcd_update_enable(false);
      lcd_clear();
#if FW_DEV_VERSION == FW_VERSION_DEVEL
      lcd_puts_at_P(0, 0, PSTR("Development build !!"));
#else
      lcd_puts_at_P(0, 0, PSTR("Debbugging build !!!"));
#endif
      lcd_puts_at_P(0, 1, PSTR("May destroy printer!"));
      lcd_puts_at_P(0, 2, PSTR("ver ")); lcd_puts_P(PSTR(FW_VERSION_FULL));
      lcd_puts_at_P(0, 3, PSTR(FW_REPOSITORY));
      lcd_wait_for_click();
    break;
	}
	lcd_update_enable(true);
}

//! @brief try to check if firmware is on right type of printer
static void check_if_fw_is_on_right_printer(){
#ifdef FILAMENT_SENSOR
  if((PRINTER_TYPE == PRINTER_MK3) || (PRINTER_TYPE == PRINTER_MK3S)){
    #ifdef IR_SENSOR
    swi2c_init();
    const uint8_t pat9125_detected = swi2c_readByte_A8(PAT9125_I2C_ADDR,0x00,NULL);
      if (pat9125_detected){
        lcd_show_fullscreen_message_and_wait_P(_i("MK3S firmware detected on MK3 printer"));}////c=20 r=3
    #endif //IR_SENSOR

    #ifdef PAT9125
      //will return 1 only if IR can detect filament in bondtech extruder so this may fail even when we have IR sensor
      const uint8_t ir_detected = !READ(IR_SENSOR_PIN);
      if (ir_detected){
        lcd_show_fullscreen_message_and_wait_P(_i("MK3 firmware detected on MK3S printer"));}////c=20 r=3
    #endif //PAT9125
  }
#endif //FILAMENT_SENSOR
}

uint8_t check_printer_version() {
	uint8_t version_changed = 0;
	uint16_t printer_type = eeprom_read_word((uint16_t*)EEPROM_PRINTER_TYPE);
	uint16_t motherboard = eeprom_read_word((uint16_t*)EEPROM_BOARD_TYPE);

	if (printer_type != PRINTER_TYPE) {
		if (printer_type == 0xffff) {eeprom_write_word((uint16_t*)EEPROM_PRINTER_TYPE, PRINTER_TYPE);
    } else {
      version_changed |= 0b10;
    }
	}
	if (motherboard != MOTHERBOARD) {
		if(motherboard == 0xffff) {
      eeprom_write_word((uint16_t*)EEPROM_BOARD_TYPE, MOTHERBOARD);
    } else {
      version_changed |= 0b01;
    }
	}
	return version_changed;
}

#ifdef BOOTAPP
#include "bootapp.h" //bootloader support
#endif //BOOTAPP

#if (LANG_MODE != 0) //secondary language support

#ifdef W25X20CL
// language update from external flash
#define LANGBOOT_BLOCKSIZE 0x1000u
#define LANGBOOT_RAMBUFFER 0x0800

void update_sec_lang_from_external_flash()
{
	if ((boot_app_magic == BOOT_APP_MAGIC) && (boot_app_flags & BOOT_APP_FLG_USER0))
	{
		uint8_t lang = boot_reserved >> 4;
		uint8_t state = boot_reserved & 0xf;
		lang_table_header_t header;
		uint32_t src_addr;
		if (lang_get_header(lang, &header, &src_addr))
		{
			lcd_puts_at_P(1,3,PSTR("Language update."));
			for (uint8_t i = 0; i < state; i++) fputc('.', lcdout);
			_delay(100);
			boot_reserved = (state + 1) | (lang << 4);
			if ((state * LANGBOOT_BLOCKSIZE) < header.size)
			{
				cli();
				uint16_t size = header.size - state * LANGBOOT_BLOCKSIZE;
				if (size > LANGBOOT_BLOCKSIZE) size = LANGBOOT_BLOCKSIZE;
				w25x20cl_rd_data(src_addr + state * LANGBOOT_BLOCKSIZE, (uint8_t*)LANGBOOT_RAMBUFFER, size);
				if (state == 0)
				{
					//TODO - check header integrity
				}
				bootapp_ram2flash(LANGBOOT_RAMBUFFER, _SEC_LANG_TABLE + state * LANGBOOT_BLOCKSIZE, size);
			}
			else
			{
				//TODO - check sec lang data integrity
				eeprom_update_byte((unsigned char *)EEPROM_LANG, LANG_ID_SEC);
			}
		}
	}
	boot_app_flags &= ~BOOT_APP_FLG_USER0;
}


#ifdef DEBUG_W25X20CL

uint8_t lang_xflash_enum_codes(uint16_t* codes)
{
	lang_table_header_t header;
	uint8_t count = 0;
	uint32_t addr = 0x00000;
	while (1)
	{
		printf_P(_n("LANGTABLE%d:"), count);
		w25x20cl_rd_data(addr, (uint8_t*)&header, sizeof(lang_table_header_t));
		if (header.magic != LANG_MAGIC)
		{
			printf_P(_n("NG!\n"));
			break;
		}
		printf_P(_n("OK\n"));
		printf_P(_n(" _lt_magic        = 0x%08lx %S\n"), header.magic, (header.magic==LANG_MAGIC)?_n("OK"):_n("NA"));
		printf_P(_n(" _lt_size         = 0x%04x (%d)\n"), header.size, header.size);
		printf_P(_n(" _lt_count        = 0x%04x (%d)\n"), header.count, header.count);
		printf_P(_n(" _lt_chsum        = 0x%04x\n"), header.checksum);
		printf_P(_n(" _lt_code         = 0x%04x (%c%c)\n"), header.code, header.code >> 8, header.code & 0xff);
		printf_P(_n(" _lt_sign         = 0x%08lx\n"), header.signature);

		addr += header.size;
		codes[count] = header.code;
		count ++;
	}
	return count;
}

void list_sec_lang_from_external_flash()
{
	uint16_t codes[8];
	uint8_t count = lang_xflash_enum_codes(codes);
	printf_P(_n("XFlash lang count = %hhd\n"), count);
}

#endif //DEBUG_W25X20CL

#endif //W25X20CL

#endif //(LANG_MODE != 0)


static void w25x20cl_err_msg() {
	lcd_clear();
	lcd_puts_P(_n("External SPI flash\nW25X20CL is not res-\nponding. Language\nswitch unavailable."));
}

// "Setup" function is called by the Arduino framework on startup.
// Before startup, the Timers-functions (PWM)/Analog RW and HardwareSerial provided by the Arduino-code 
// are initialized by the main() routine provided by the Arduino framework.
void setup() {
	ultralcd_init();
	spi_init();
	lcd_splash();
  Sound_Init();                                // also guarantee "SET_OUTPUT(BEEPER)"

	selectedSerialPort = eeprom_read_byte((uint8_t *)EEPROM_SECOND_SERIAL_ACTIVE);
	if (selectedSerialPort == 0xFF) {selectedSerialPort = 0;}
	eeprom_update_byte((uint8_t *)EEPROM_SECOND_SERIAL_ACTIVE, selectedSerialPort);
	MYSERIAL.begin(BAUDRATE);
	fdev_setup_stream(uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE); //setup uart out stream
	stdout = uartout;

	const bool w25x20cl_success = true;

	setup_killpin();
	setup_powerhold();

	if ((selectedSerialPort != 0))
		SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;
	printf_P(PSTR(" " FW_VERSION_FULL "\n"));

	//SERIAL_ECHOPAIR("Active sheet before:", static_cast<unsigned long int>(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet))));


	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if (mcu & 1) puts_P(MSG_POWERUP);
	if (mcu & 2) puts_P(MSG_EXTERNAL_RESET);
	if (mcu & 4) puts_P(MSG_BROWNOUT_RESET);
	if (mcu & 8) puts_P(MSG_WATCHDOG_RESET);
	if (mcu & 32) puts_P(MSG_SOFTWARE_RESET);
	MCUSR = 0;


	SERIAL_ECHO_START;
	SERIAL_ECHORPGM(_n(" Free Memory: "));////MSG_FREE_MEMORY
	SERIAL_ECHO(freeMemory());
	SERIAL_ECHORPGM(_n("  PlannerBufferBytes: "));////MSG_PLANNER_BUFFER_BYTES
	SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
	//lcd_update_enable(false); // why do we need this?? - andre
	// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
	
	bool previous_settings_retrieved = false; 
	uint8_t hw_changed = check_printer_version();
	if (!(hw_changed & 0b10)) { //if printer version wasn't changed, check for eeprom version and retrieve settings from eeprom in case that version wasn't changed
		previous_settings_retrieved = Config_RetrieveSettings();
	} else { //printer version was changed so use default settings 
		Config_ResetDefault();
	}

	tp_init();    // Initialize temperature loop

  lcd_splash(); // we need to do this again, because tp_init() kills lcd

	plan_init();  // Initialize planner;

	factory_reset(); //If button is being held on startup at the time this function is called, and held for the required amount of time, intiate factory reset.
	if (eeprom_read_dword((uint32_t*)(EEPROM_TOP - 4)) == 0x0ffffffff &&
  eeprom_read_dword((uint32_t*)(EEPROM_TOP - 8)) == 0x0ffffffff) {
    // Maiden startup. The firmware has been loaded and first started on a virgin RAMBo board,
    // where all the EEPROM entries are set to 0x0ff.
    // Once a firmware boots up, it forces at least a language selection, which changes
    // EEPROM_LANG to number lower than 0x0ff.
    // 1) Set a high power mode.
    eeprom_update_byte((uint8_t*)EEPROM_SILENT, SILENT_MODE_OFF);

    eeprom_write_byte((uint8_t*)EEPROM_WIZARD_ACTIVE, 1); //run wizard
  }

    lcd_encoder_diff=0;

	st_init();    // Initialize stepper, this enables interrupts!


  plan_set_position_curposXYZE();



	setup_homepin();

  enable_z();

  eeprom_init();


	if (eeprom_read_byte((uint8_t*)EEPROM_TEMP_CAL_ACTIVE) == 255) {
		eeprom_write_byte((uint8_t*)EEPROM_TEMP_CAL_ACTIVE, 0);
	}

	if (eeprom_read_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA) == 255) {
		//eeprom_write_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0);
		eeprom_write_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
		int16_t z_shift = 0;
		for (uint8_t i = 0; i < 5; i++) { EEPROM_save_B(EEPROM_PROBE_TEMP_SHIFT + i * 2, &z_shift); }
		eeprom_write_byte((uint8_t*)EEPROM_TEMP_CAL_ACTIVE, 0);
	}
	if (eeprom_read_byte((uint8_t*)EEPROM_UVLO) == 255) {
		eeprom_write_byte((uint8_t*)EEPROM_UVLO, 0);
	}
	if (eeprom_read_byte((uint8_t*)EEPROM_SD_SORT) == 255) {
		eeprom_write_byte((uint8_t*)EEPROM_SD_SORT, 0);
	}



	for (int i = 0; i<4; i++) { EEPROM_read_B(EEPROM_BOWDEN_LENGTH + i * 2, &bowden_length[i]); }
	
  lcd_update_enable(true);
  lcd_clear();
  lcd_update(2);
  // Store the currently running firmware into an eeprom,
  // so the next time the firmware gets updated, it will know from which version it has been updated.
  update_current_firmware_version_to_eeprom();

  fCheckModeInit();
  KEEPALIVE_STATE(NOT_BUSY);
#ifdef WATCHDOG
  wdt_enable(WDTO_4S);
#endif //WATCHDOG
}

void trace();

#define CHUNK_SIZE 64 // bytes
#define SAFETY_MARGIN 1
char chunk[CHUNK_SIZE+SAFETY_MARGIN];
int chunkHead = 0;


/**
* Output a "busy" message at regular intervals
* while the machine is not accepting commands.
*/
void host_keepalive() {
#ifndef HOST_KEEPALIVE_FEATURE
  return;
#endif //HOST_KEEPALIVE_FEATURE
  if (farm_mode) return;
  long ms = _millis();



  if (host_keepalive_interval && busy_state != NOT_BUSY) {
    if ((ms - prev_busy_signal_ms) < (long)(1000L * host_keepalive_interval)) return;
     switch (busy_state) {
      case IN_HANDLER:
      case IN_PROCESS:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: processing");
      break;
      case PAUSED_FOR_USER:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: paused for user");
      break;
      case PAUSED_FOR_INPUT:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: paused for input");
      break;
      default:
	    break;
    }
  }
  prev_busy_signal_ms = ms;
}


// The loop() function is called in an endless loop by the Arduino framework from the default main() routine.
// Before loop(), the setup() function is called by the main() routine.
void loop() {
	KEEPALIVE_STATE(NOT_BUSY);

	if ((usb_printing_counter > 0) && ((_millis()-_usb_timer) > 1000)) {
		is_usb_printing = true;
		usb_printing_counter--;
		_usb_timer = _millis();
	}
	if (usb_printing_counter == 0) {
		is_usb_printing = false;
	}
    if (isPrintPaused && saved_printing_type == PRINTING_TYPE_USB) {//keep believing that usb is being printed. Prevents accessing dangerous menus while pausing.
		is_usb_printing = true;
	}
    
  get_command();

  if(buflen) {
    cmdbuffer_front_already_processed = false;

    process_commands();

    if (! cmdbuffer_front_already_processed && buflen) {
      // ptr points to the start of the block currently being processed.
      // The first character in the block is the block type.      
      char *ptr = cmdbuffer + bufindr;
      if (*ptr == CMDBUFFER_CURRENT_TYPE_SDCARD) {
        // To support power panic, move the lenght of the command on the SD card to a planner buffer.
        union {
          struct {
              char lo;
              char hi;
          } lohi;
          uint16_t value;
        } sdlen;
        sdlen.value = 0;
        {
          // This block locks the interrupts globally for 3.25 us,
          // which corresponds to a maximum repeat frequency of 307.69 kHz.
          // This blocking is safe in the context of a 10kHz stepper driver interrupt
          // or a 115200 Bd serial line receive interrupt, which will not trigger faster than 12kHz.
          cli();
          // Reset the command to something, which will be ignored by the power panic routine,
          // so this buffer length will not be counted twice.
          *ptr ++ = CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED;
          // Extract the current buffer length.
          sdlen.lohi.lo = *ptr ++;
          sdlen.lohi.hi = *ptr;
          // and pass it to the planner queue.
          planner_add_sd_length(sdlen.value);
          sei();
        }
      } else if((*ptr == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR) && !IS_SD_PRINTING){ 
        cli();
        *ptr ++ = CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED;
        // and one for each command to previous block in the planner queue.
        planner_add_sd_length(1);
        sei();
      }
      // Now it is safe to release the already processed command block. If interrupted by the power panic now,
      // this block's SD card length will not be counted twice as its command type has been replaced 
      // by CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED.
      cmdqueue_pop_front();
    }
    host_keepalive();
  }

  //check heater every n milliseconds
  manage_heater();
  isPrintPaused ? manage_inactivity(true) : manage_inactivity(false);
  checkHitEndstops();
  lcd_update(0);

} //End of loop()



#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)              \
    { return pgm_read_any(&array##_P[axis]); }  \
type array##_ext(int axis)                      \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis) + cs.add_homing[axis];
  min_pos[axis] =          base_min_pos(axis) + cs.add_homing[axis];
  max_pos[axis] =          base_max_pos(axis) + cs.add_homing[axis];
}

//! @return original feedmultiply
static int setup_for_endstop_move(bool enable_endstops_now = true) {
    saved_feedrate = feedrate;
    int l_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = _millis();
    
    enable_endstops(enable_endstops_now);
    return l_feedmultiply;
}

//! @param original_feedmultiply feedmultiply to restore
static void clean_up_after_endstop_move(int original_feedmultiply) {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif
    
    feedrate = saved_feedrate;
    feedmultiply = original_feedmultiply;
    previous_millis_cmd = _millis();
}




#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients) {
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;
    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = cs.zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position_curposXYZE();
}

#else // not AUTO_BED_LEVELING_GRID

static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
    vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
    vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

    vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
    vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
    vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
    planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));

    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = cs.zprobe_zoffset;

    plan_set_position_curposXYZE();

}

#endif // AUTO_BED_LEVELING_GRID

static void run_z_probe() {
    plan_bed_level_matrix.set_to_identity();
    feedrate = homing_feedrate[Z_AXIS];

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

        // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_retract_mm(Z_AXIS);
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // move back down slowly to find bed
    feedrate = homing_feedrate[Z_AXIS]/4;
    zPosition -= home_retract_mm(Z_AXIS) * 2;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position_curposXYZE();
}

static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = homing_feedrate[Z_AXIS];

    current_position[Z_AXIS] = z;
    plan_buffer_line_curposXYZE(feedrate/60, active_extruder);
    st_synchronize();

    feedrate = XY_TRAVEL_SPEED;

    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    plan_buffer_line_curposXYZE(feedrate/60, active_extruder);
    st_synchronize();

    feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
    do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}


/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before) {
  // move to right place
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
  do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

  run_z_probe();
  float measured_z = current_position[Z_AXIS];

  SERIAL_PROTOCOLRPGM(_T(MSG_BED));
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL(x);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL(y);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL(measured_z);
  SERIAL_PROTOCOLPGM("\n");
  return measured_z;
}

#endif // #ifdef ENABLE_AUTO_BED_LEVELING

#ifdef LIN_ADVANCE
/**
  * M900: Set and/or Get advance K factor
  *
  *  K<factor>                  Set advance K factor
  */
inline void gcode_M900() {
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
}
#endif // LIN_ADVANCE

bool check_commands() {
	bool end_command_found = false;
  while (buflen) {
		if ((code_seen("M84")) || (code_seen("M 84"))) { end_command_found = true;}
		if (!cmdbuffer_front_already_processed) {
      cmdqueue_pop_front();
    }
		cmdbuffer_front_already_processed = false;
  }
	return end_command_found;
} //check commands


// raise_z_above: slowly raise Z to the requested height
//
// contrarily to a simple move, this function will carefully plan a move
// when the current Z position is unknown. In such cases, stallguard is
// enabled and will prevent prolonged pushing against the Z tops
void raise_z_above(float target, bool plan) {
  if (current_position[Z_AXIS] >= target) {
    return;
  }
  // Z needs raising
  current_position[Z_AXIS] = target;

#if defined(Z_MIN_PIN) && (Z_MIN_PIN > -1) && !defined(DEBUG_DISABLE_ZMINLIMIT)
    bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
#else
    bool z_min_endstop = false;
#endif

  if (axis_known_position[Z_AXIS] || z_min_endstop) {
    // current position is known or very low, it's safe to raise Z
    if(plan) {plan_buffer_line_curposXYZE(max_feedrate[Z_AXIS]);}
    return;
  }
  // ensure Z is powered in normal mode to overcome initial load
  enable_z();
  st_synchronize();

  // rely on crashguard to limit damage
  bool z_endstop_enabled = enable_z_endstop(true);

  plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 60);
  st_synchronize();

  enable_z_endstop(z_endstop_enabled);
}


void homeaxis(int axis, uint8_t cnt)
{
	bool endstops_enabled  = enable_endstops(true); //RP: endstops should be allways enabled durring homing
#define HOMEAXIS_DO(LETTER) \
((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))
  if ((axis==X_AXIS)?HOMEAXIS_DO(X):(axis==Y_AXIS)?HOMEAXIS_DO(Y):0) {
    int axis_home_dir = home_dir(axis);
    feedrate = homing_feedrate[axis];

    // Move away a bit, so that the print head does not touch the end position,
    // and the following movement to endstop has a chance to achieve the required velocity
    // for the stall guard to work.
    current_position[axis] = 0;
    plan_set_position_curposXYZE();
		set_destination_to_current();
    destination[axis] = -3.f * axis_home_dir;
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();
    // Move away from the possible collision with opposite endstop with the collision detection disabled.
    endstops_hit_on_purpose();
    enable_endstops(false);
    current_position[axis] = 0;
    plan_set_position_curposXYZE();
    destination[axis] = 1. * axis_home_dir;
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();
    // Now continue to move up to the left end stop with the collision detection enabled.
    enable_endstops(true);
    destination[axis] = 1.1 * axis_home_dir * max_length(axis);
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();
		for (uint8_t i = 0; i < cnt; i++) {
			// Move away from the collision to a known distance from the left end stop with the collision detection disabled.
			endstops_hit_on_purpose();
			enable_endstops(false);
			current_position[axis] = 0;
			plan_set_position_curposXYZE();
			destination[axis] = -10.f * axis_home_dir;
			plan_buffer_line_destinationXYZE(feedrate/60);
			st_synchronize();
			endstops_hit_on_purpose();
			// Now move left up to the collision, this time with a repeatable velocity.
			enable_endstops(true);
			destination[axis] = 11.f * axis_home_dir;
#ifdef TMC2130
			feedrate = homing_feedrate[axis];
#else //TMC2130
			feedrate = homing_feedrate[axis] / 2;
#endif //TMC2130
			plan_buffer_line_destinationXYZE(feedrate/60);
			st_synchronize();

		}
		endstops_hit_on_purpose();
		enable_endstops(false);

    axis_is_at_home(axis);
    axis_known_position[axis] = true;
    // Move from minimum

    float dist = - axis_home_dir * 0.01f * 64;

    current_position[axis] -= dist;
    plan_set_position_curposXYZE();
    current_position[axis] += dist;
    destination[axis] = current_position[axis];
    plan_buffer_line_destinationXYZE(0.5f*feedrate/60);
    st_synchronize();

    feedrate = 0.0;
  } else if ((axis==Z_AXIS)?HOMEAXIS_DO(Z):0) {
	
    int axis_home_dir = home_dir(axis);
    current_position[axis] = 0;
    plan_set_position_curposXYZE();
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position_curposXYZE();
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();
    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis]/2 ;
    plan_buffer_line_destinationXYZE(feedrate/60);
    st_synchronize();
#ifdef TMC2130
    check_Z_crash();
#endif //TMC2130
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;
#ifdef TMC2130
		FORCE_HIGH_POWER_END;
#endif	
  }
  enable_endstops(endstops_enabled);
}

/**/
void home_xy() {
  set_destination_to_current();
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  plan_set_position_curposXYZE();
  endstops_hit_on_purpose();
}

void refresh_cmd_timeout(void) {
  previous_millis_cmd = _millis();
}

#ifdef FWRETRACT
void retract(bool retracting, bool swapretract = false) {
  if(retracting && !retracted[active_extruder]) {
    destination[X_AXIS]=current_position[X_AXIS];
    destination[Y_AXIS]=current_position[Y_AXIS];
    destination[Z_AXIS]=current_position[Z_AXIS];
    destination[E_AXIS]=current_position[E_AXIS];
    current_position[E_AXIS]+=(swapretract?retract_length_swap:cs.retract_length)*float(extrudemultiply)*0.01f;
    plan_set_e_position(current_position[E_AXIS]);
    float oldFeedrate = feedrate;
    feedrate=cs.retract_feedrate*60;
    retracted[active_extruder]=true;
    prepare_move();
    current_position[Z_AXIS]-=cs.retract_zlift;
    plan_set_position_curposXYZE();
    prepare_move();
    feedrate = oldFeedrate;
  } else if(!retracting && retracted[active_extruder]) {
    destination[X_AXIS]=current_position[X_AXIS];
    destination[Y_AXIS]=current_position[Y_AXIS];
    destination[Z_AXIS]=current_position[Z_AXIS];
    destination[E_AXIS]=current_position[E_AXIS];
    current_position[Z_AXIS]+=cs.retract_zlift;
    plan_set_position_curposXYZE();
    current_position[E_AXIS]-=(swapretract?(retract_length_swap+retract_recover_length_swap):(cs.retract_length+cs.retract_recover_length))*float(extrudemultiply)*0.01f;
    plan_set_e_position(current_position[E_AXIS]);
    float oldFeedrate = feedrate;
    feedrate=cs.retract_recover_feedrate*60;
    retracted[active_extruder]=false;
    prepare_move();
    feedrate = oldFeedrate;
  }
} //retract
#endif //FWRETRACT

void trace() {
  Sound_MakeCustom(25,440,true);
}



void gcode_M114()
{
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


// G92 - Set current position to coordinates given
static void gcode_G92()
{
    bool codes[NUM_AXIS];
    float values[NUM_AXIS];

    // Check which axes need to be set
    for(uint8_t i = 0; i < NUM_AXIS; ++i)
    {
        codes[i] = code_seen(axis_codes[i]);
        if(codes[i])
            values[i] = code_value();
    }

    if((codes[E_AXIS] && values[E_AXIS] == 0) &&
       (!codes[X_AXIS] && !codes[Y_AXIS] && !codes[Z_AXIS]))
    {
        // As a special optimization, when _just_ clearing the E position
        // we schedule a flag asynchronously along with the next block to
        // reset the starting E position instead of stopping the planner
        current_position[E_AXIS] = 0;
        plan_reset_next_e();
    }
    else
    {
        // In any other case we're forced to synchronize
        st_synchronize();
        for(uint8_t i = 0; i < 3; ++i)
        {
            if(codes[i])
                current_position[i] = values[i] + cs.add_homing[i];
        }
        if(codes[E_AXIS])
            current_position[E_AXIS] = values[E_AXIS];

        // Set all at once
        plan_set_position_curposXYZE();
    }
}

#ifdef EXTENDED_CAPABILITIES_REPORT

static void cap_line(const char* name, bool ena = false) {
    printf_P(PSTR("Cap:%S:%c\n"), name, (char)ena + '0');
}

static void extended_capabilities_report()
{
    cap_line(PSTR("AUTOREPORT_TEMP"), ENABLED(AUTO_REPORT_TEMPERATURES));
    //@todo
}
#endif //EXTENDED_CAPABILITIES_REPORT

#ifdef BACKLASH_X
extern uint8_t st_backlash_x;
#endif //BACKLASH_X
#ifdef BACKLASH_Y
extern uint8_t st_backlash_y;
#endif //BACKLASH_Y

//! \ingroup marlin_main

//! @brief Parse and process commands
//!
//! look here for descriptions of G-codes: https://reprap.org/wiki/G-code
//!
//!
//! Implemented Codes 
//! -------------------
//!
//! * _This list is not updated. Current documentation is maintained inside the process_cmd function._ 
//!
//!@n PRUSA CODES
//!@n P F - Returns FW versions
//!@n P R - Returns revision of printer
//!
//!@n G0  -> G1
//!@n G1  - Coordinated Movement X Y Z E
//!@n G2  - CW ARC
//!@n G3  - CCW ARC
//!@n G4  - Dwell S<seconds> or P<milliseconds>
//!@n G10 - retract filament according to settings of M207
//!@n G11 - retract recover filament according to settings of M208
//!@n G28 - Home all Axes
//!@n G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
//!@n G30 - Single Z Probe, probes bed at current XY location.
//!@n G31 - Dock sled (Z_PROBE_SLED only)
//!@n G32 - Undock sled (Z_PROBE_SLED only)
//!@n G80 - Automatic mesh bed leveling
//!@n G81 - Print bed profile
//!@n G90 - Use Absolute Coordinates
//!@n G91 - Use Relative Coordinates
//!@n G92 - Set current position to coordinates given
//!
//!@n M Codes
//!@n M0   - Unconditional stop - Wait for user to press a button on the LCD
//!@n M1   - Same as M0
//!@n M17  - Enable/Power all stepper motors
//!@n M18  - Disable all stepper motors; same as M84
//!@n M20  - List SD card
//!@n M21  - Init SD card
//!@n M22  - Release SD card
//!@n M23  - Select SD file (M23 filename.g)
//!@n M24  - Start/resume SD print
//!@n M25  - Pause SD print
//!@n M26  - Set SD position in bytes (M26 S12345)
//!@n M27  - Report SD print status
//!@n M28  - Start SD write (M28 filename.g)
//!@n M29  - Stop SD write
//!@n M30  - Delete file from SD (M30 filename.g)
//!@n M31  - Output time since last M109 or SD card start to serial
//!@n M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//!          syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//!          Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//!          The '#' is necessary when calling from within sd files, as it stops buffer prereading
//!@n M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
//!@n M73  - Show percent done and print time remaining
//!@n M80  - Turn on Power Supply
//!@n M81  - Turn off Power Supply
//!@n M82  - Set E codes absolute (default)
//!@n M83  - Set E codes relative while in Absolute Coordinates (G90) mode
//!@n M84  - Disable steppers until next move,
//!          or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
//!@n M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
//!@n M86  - Set safety timer expiration time with parameter S<seconds>; M86 S0 will disable safety timer
//!@n M92  - Set axis_steps_per_unit - same syntax as G92
//!@n M104 - Set extruder target temp
//!@n M105 - Read current temp
//!@n M106 - Fan on
//!@n M107 - Fan off
//!@n M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//!          Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
//!        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
//!@n M112 - Emergency stop
//!@n M113 - Get or set the timeout interval for Host Keepalive "busy" messages
//!@n M114 - Output current position to serial port
//!@n M115 - Capabilities string
//!@n M117 - display message
//!@n M119 - Output Endstop status to serial port
//!@n M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
//!@n M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
//!@n M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
//!@n M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
//!@n M140 - Set bed target temp
//!@n M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
//!@n M155 - Automatically send temperatures
//!@n M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//!          Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
//!@n M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
//!@n M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
//!@n M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
//!@n M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
//!@n M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
//!@n M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
//!@n M206 - set additional homing offset
//!@n M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
//!@n M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
//!@n M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
//!@n M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
//!@n M220 S<factor in percent>- set speed factor override percentage
//!@n M221 S<factor in percent>- set extrude factor override percentage
//!@n M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
//!@n M240 - Trigger a camera to take a photograph
//!@n M250 - Set LCD contrast C<contrast value> (value 0..63)
//!@n M280 - set servo position absolute. P: servo index, S: angle or microseconds
//!@n M300 - Play beep sound S<frequency Hz> P<duration ms>
//!@n M301 - Set PID parameters P I and D
//!@n M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
//!@n M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
//!@n M304 - Set bed PID parameters P I and D
//!@n M400 - Finish all moves
//!@n M401 - Lower z-probe if present
//!@n M402 - Raise z-probe if present
//!@n M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
//!@n M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
//!@n M406 - Turn off Filament Sensor extrusion control
//!@n M407 - Displays measured filament diameter
//!@n M500 - stores parameters in EEPROM
//!@n M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
//!@n M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//!@n M503 - print the current settings (from memory not from EEPROM)
//!@n M509 - force language selection on next restart
//!@n M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
//!@n M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
//!@n M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
//!@n M860 - Wait for PINDA thermistor to reach target temperature.
//!@n M861 - Set / Read PINDA temperature compensation offsets
//!@n M900 - Set LIN_ADVANCE options, if enabled. See Configuration_adv.h for details.
//!@n M907 - Set digital trimpot motor current using axis codes.
//!@n M908 - Control digital trimpot directly.
//!@n M350 - Set microstepping mode.
//!@n M351 - Toggle MS1 MS2 pins directly.
//!
//!@n M928 - Start SD logging (M928 filename.g) - ended by M29
//!@n M999 - Restart after being stopped by error
//! <br><br>

/** @defgroup marlin_main Marlin main */

/** \ingroup GCodes */

//! _This is a list of currently implemented G Codes in Prusa firmware (dynamically generated from doxygen)._ 
/**
They are shown in order of appearance in the code.
There are reasons why some G Codes aren't in numerical order.
*/


void process_commands()
{
#ifdef FANCHECK
    if(fan_check_error){
        if(fan_check_error == EFCE_DETECTED){
            fan_check_error = EFCE_REPORTED;
            // SERIAL_PROTOCOLLNRPGM(MSG_OCTOPRINT_PAUSED);
            lcd_pause_print();
        } // otherwise it has already been reported, so just ignore further processing
        return; //ignore usb stream. It is reenabled by selecting resume from the lcd.
    }
#endif

	if (!buflen) return; //empty command
  #ifdef FILAMENT_RUNOUT_SUPPORT
    SET_INPUT(FR_SENS);
  #endif

#ifdef CMDBUFFER_DEBUG
  SERIAL_ECHOPGM("Processing a GCODE command: ");
  SERIAL_ECHO(cmdbuffer+bufindr+CMDHDRSIZE);
  SERIAL_ECHOLNPGM("");
  SERIAL_ECHOPGM("In cmdqueue: ");
  SERIAL_ECHO(buflen);
  SERIAL_ECHOLNPGM("");
#endif /* CMDBUFFER_DEBUG */
  
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif

  // PRUSA GCODES
  KEEPALIVE_STATE(IN_HANDLER);

#ifdef SNMM
  float tmp_motor[3] = DEFAULT_PWM_MOTOR_CURRENT;
  float tmp_motor_loud[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;
  int8_t SilentMode;
#endif
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
  if (code_seen("M117")) { //moved to highest priority place to be able to to print strings which includes "G", "PRUSA" and "^"
	  starpos = (strchr(strchr_pointer + 5, '*'));
	  if (starpos != NULL)
		  *(starpos) = '\0';
	  lcd_setstatus(strchr_pointer + 5);
  }


  // This prevents reading files with "^" in their names.
  // Since it is unclear, if there is some usage of this construct,
  // it will be deprecated in 3.9 alpha a possibly completely removed in the future:
  // else if (code_seen('^')) {
  //  // nothing, this is a version line
  // }
  else if(code_seen('G'))
  {
	gcode_in_progress = (int)code_value();
//	printf_P(_N("BEGIN G-CODE=%u\n"), gcode_in_progress);
    switch (gcode_in_progress)
    {

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
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {

        #ifdef FILAMENT_RUNOUT_SUPPORT
            
            if(READ(FR_SENS)){

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
                          //lcd_update(0);
                          if(cnt==0)
                          {
                          #if BEEPER > 0
                          
                            if (counterBeep== 500){
                              counterBeep = 0;
                              
                            }
                          
                            
                            SET_OUTPUT(BEEPER);
                            if (counterBeep== 0){
if(eSoundMode!=e_SOUND_MODE_SILENT)
                              WRITE(BEEPER,HIGH);
                            }
                            
                            if (counterBeep== 20){
                              WRITE(BEEPER,LOW);
                            }
                            
                            
                            
                          
                            counterBeep++;
                          #else
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
                 (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR)))
            {
                memcpy(destination, saved_target, sizeof(destination));
                saved_target[0] = SAVED_TARGET_UNSET;
            }

		if (total_filament_used > ((current_position[E_AXIS] - destination[E_AXIS]) * 100)) { //protection against total_filament_used overflow
			total_filament_used = total_filament_used + ((destination[E_AXIS] - current_position[E_AXIS]) * 100);
		}
          #ifdef FWRETRACT
            if(cs.autoretract_enabled)
            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
              float echange=destination[E_AXIS]-current_position[E_AXIS];

              if((echange<-MIN_RETRACT && !retracted[active_extruder]) || (echange>MIN_RETRACT && retracted[active_extruder])) { //move appears to be an attempt to retract or recover
                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
                  retract(!retracted[active_extruder]);
                  return;
              }


            }
          #endif //FWRETRACT
        prepare_move();
        //ClearToSend();
      }
      break;

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
    case 2: 
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
      }
      break;
 
    // -------------------------------
    case 3: 
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
      }
      break;


    /*!
	### G4 - Dwell <a href="https://reprap.org/wiki/G-code#G4:_Dwell">G4: Dwell</a>
	Pause the machine for a period of time.
	
	#### Usage
	
	    G4 [ P | S ]
	
	#### Parameters
	  - `P` - Time to wait, in milliseconds
	  - `S` - Time to wait, in seconds
	
    */
    case 4: 
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
	  if(codenum != 0) LCD_MESSAGERPGM(_n("Sleep..."));////MSG_DWELL
      st_synchronize();
      codenum += _millis();  // keep track of when we started waiting
      previous_millis_cmd = _millis();
      while(_millis() < codenum) {
        manage_heater();
        manage_inactivity();
        lcd_update(0);
      }
      break;
    

    /*!
	### G21 - Sets Units to Millimters <a href="https://reprap.org/wiki/G-code#G21:_Set_Units_to_Millimeters">G21: Set Units to Millimeters</a>
	Units are in millimeters. Prusa doesn't support inches.
    */
    case 21: 
      break; //Doing nothing. This is just to prevent serial UNKOWN warnings.
    
            

    /*!
	### G90 - Switch off relative mode <a href="https://reprap.org/wiki/G-code#G90:_Set_to_Absolute_Positioning">G90: Set to Absolute Positioning</a>
	All coordinates from now on are absolute relative to the origin of the machine. E axis is left intact.
    */
    case 90: {
		axis_relative_modes &= ~(X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK);
    }
    break;

    /*!
	### G91 - Switch on relative mode <a href="https://reprap.org/wiki/G-code#G91:_Set_to_Relative_Positioning">G91: Set to Relative Positioning</a>
    All coordinates from now on are relative to the last position. E axis is left intact.
	*/
    case 91: {
		axis_relative_modes |= X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK;
    }
    break;

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
    case 92: {
        gcode_G92();
    }
    break;


	default:
		printf_P(PSTR("Unknown G code: %s \n"), cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END G-CODE=%u\n"), gcode_in_progress);
	gcode_in_progress = 0;
  } // end if(code_seen('G'))
  /*!
  ### End of G-Codes
  */

  /*!
  ---------------------------------------------------------------------------------
  # M Commands
  
  */

  else if(code_seen('M'))
  {

	  int index;
	  for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);
	   
	 /*for (++strchr_pointer; *strchr_pointer == ' ' || *strchr_pointer == '\t'; ++strchr_pointer);*/
	  if (*(strchr_pointer+index) < '0' || *(strchr_pointer+index) > '9') {
		  printf_P(PSTR("Invalid M code: %s \n"), cmdbuffer + bufindr + CMDHDRSIZE);

	  } else
	  {
	  mcode_in_progress = (int)code_value();
//	printf_P(_N("BEGIN M-CODE=%u\n"), mcode_in_progress);

    switch(mcode_in_progress)
    {

    /*!
	### M0, M1 - Stop the printer <a href="https://reprap.org/wiki/G-code#M0:_Stop_or_Unconditional_stop">M0: Stop or Unconditional stop</a>
    */
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      char *src = strchr_pointer + 2;

      codenum = 0;

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
      while (*src == ' ') ++src;
      if (!hasP && !hasS && *src != '\0') {
        lcd_setstatus(src);
      } else {
        LCD_MESSAGERPGM(_i("Wait for user..."));////MSG_USERWAIT
      }

      lcd_ignore_click();				//call lcd_ignore_click aslo for else ???
      st_synchronize();
      previous_millis_cmd = _millis();
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
      }else{
        marlin_wait_for_click();
      }
      if (IS_SD_PRINTING)
        LCD_MESSAGERPGM(_T(MSG_RESUMING_PRINT));
      else
        LCD_MESSAGERPGM(_T(WELCOME_MSG));
    }
    break;

    /*!
	### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
    */
    case 17:
        LCD_MESSAGERPGM(_i("No move."));////MSG_NO_MOVE
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT

    /*!
	### M20 - SD Card file list <a href="https://reprap.org/wiki/G-code#M20:_List_SD_card">M20: List SD card</a>
    */
    case 20:
      SERIAL_PROTOCOLLNRPGM(_N("Begin file list"));////MSG_BEGIN_FILE_LIST
      card.ls();
      SERIAL_PROTOCOLLNRPGM(_N("End file list"));////MSG_END_FILE_LIST
      break;

    /*!
	### M21 - Init SD card <a href="https://reprap.org/wiki/G-code#M21:_Initialize_SD_card">M21: Initialize SD card</a>
    */
    case 21:
      card.initsd();
      break;

    /*!
	### M22 - Release SD card <a href="https://reprap.org/wiki/G-code#M22:_Release_SD_card">M22: Release SD card</a>
    */
    case 22: 
      card.release();
      break;

    /*!
	### M23 - Select file <a href="https://reprap.org/wiki/G-code#M23:_Select_SD_file">M23: Select SD file</a>
    #### Usage
    
        M23 [filename]
    
    */
    case 23: 
      starpos = (strchr(strchr_pointer + 4,'*'));
	  if(starpos!=NULL)
        *(starpos)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;

    /*!
	### M24 - Start SD print <a href="https://reprap.org/wiki/G-code#M24:_Start.2Fresume_SD_print">M24: Start/resume SD print</a>
    */
    case 24:
	  if (isPrintPaused)
          lcd_resume_print();
      else
      {
          if (!card.get_sdpos())
          {
              // A new print has started from scratch, reset stats
              failstats_reset_print();
#ifndef LA_NOCOMPAT
              la10c_reset();
#endif
          }

          card.startFileprint();
          starttime=_millis();
      }
	  break;

    /*!
	### M26 - Set SD index <a href="https://reprap.org/wiki/G-code#M26:_Set_SD_position">M26: Set SD position</a>
    Set position in SD card file to index in bytes.
    This command is expected to be called after M23 and before M24.
    Otherwise effect of this command is undefined.
    #### Usage
	
	      M26 [ S ]
	
	#### Parameters
	  - `S` - Index in bytes
    */
    case 26: 
      if(card.cardOK && code_seen('S')) {
        long index = code_value_long();
        card.setIndex(index);
        // We don't disable interrupt during update of sdpos_atomic
        // as we expect, that SD card print is not active in this moment
        sdpos_atomic = index;
      }
      break;

    /*!
	### M27 - Get SD status <a href="https://reprap.org/wiki/G-code#M27:_Report_SD_print_status">M27: Report SD print status</a>
    */
    case 27:
      card.getStatus();
      break;

    /*!
	### M28 - Start SD write <a href="https://reprap.org/wiki/G-code#M28:_Begin_write_to_SD_card">M28: Begin write to SD card</a>
    */
    case 28: 
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;

    /*! ### M29 - Stop SD write <a href="https://reprap.org/wiki/G-code#M29:_Stop_writing_to_SD_card">M29: Stop writing to SD card</a>
	Stops writing to the SD file signaling the end of the uploaded file. It is processed very early and it's not written to the card.
    */
    case 29:
      //processed in write to file routine above
      //card,saving = false;
      break;

    /*!
	### M30 - Delete file <a href="https://reprap.org/wiki/G-code#M30:_Delete_a_file_on_the_SD_card">M30: Delete a file on the SD card</a>
    #### Usage
    
        M30 [filename]
    
    */
    case 30:
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;

    /*!
	### M32 - Select file and start SD print <a href="https://reprap.org/wiki/G-code#M32:_Select_file_and_start_SD_print">M32: Select file and start SD print</a>
	@todo What are the parameters P and S for in M32?
    */
    case 32:
    {
      if(card.sdprinting) {
        st_synchronize();

      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        card.openFile(namestartpos,true,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
        card.startFileprint();
        if(!call_procedure)
        {
            if(!card.get_sdpos())
            {
                // A new print has started from scratch, reset stats
                failstats_reset_print();
#ifndef LA_NOCOMPAT
                la10c_reset();
#endif
            }
            starttime=_millis(); // procedure calls count as normal print time.
        }
      }
    } break;

    /*!
	### M928 - Start SD logging <a href="https://reprap.org/wiki/G-code#M928:_Start_SD_logging">M928: Start SD logging</a>
    #### Usage
    
        M928 [filename]
    
    */
    case 928: 
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    /*!
	### M31 - Report current print time <a href="https://reprap.org/wiki/G-code#M31:_Output_time_since_last_M109_or_SD_card_start_to_serial">M31: Output time since last M109 or SD card start to serial</a>
    */
    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=_millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      autotempShutdown();
      }
      break;

    /*!
	### M42 - Set pin state <a href="https://reprap.org/wiki/G-code#M42:_Switch_I.2FO_pin">M42: Switch I/O pin</a>
    #### Usage
    
        M42 [ P | S ]
        
    #### Parameters
    - `P` - Pin number.
    - `S` - Pin value. If the pin is analog, values are from 0 to 255. If the pin is digital, values are from 0 to 1.
    
    */
    case 42:
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;





    /*!
	### M47 - Show end stops dialog on the display <a href="https://reprap.org/wiki/G-code#M47:_Show_end_stops_dialog_on_the_display">M47: Show end stops dialog on the display</a>
    */
    case 47:
        
		KEEPALIVE_STATE(PAUSED_FOR_USER);
        lcd_diag_show_end_stops();
		KEEPALIVE_STATE(IN_HANDLER);
        break;




    /*!
	### M112 - Emergency stop <a href="https://reprap.org/wiki/G-code#M112:_Full_.28Emergency.29_Stop">M112: Full (Emergency) Stop</a>
    It is processed much earlier as to bypass the cmdqueue.
    */
    case 112: 
      kill(MSG_M112_KILL, 3);
      break;


    /*!
	### M82 - Set E axis to absolute mode <a href="https://reprap.org/wiki/G-code#M82:_Set_extruder_to_absolute_mode">M82: Set extruder to absolute mode</a>
	Makes the extruder interpret extrusion as absolute positions.
    */
    case 82:
      axis_relative_modes &= ~E_AXIS_MASK;
      break;

    /*!
	### M83 - Set E axis to relative mode <a href="https://reprap.org/wiki/G-code#M83:_Set_extruder_to_relative_mode">M83: Set extruder to relative mode</a>
	Makes the extruder interpret extrusion values as relative positions.
    */
    case 83:
      axis_relative_modes |= E_AXIS_MASK;
      break;

    /*!
	### M84 - Disable steppers <a href="https://reprap.org/wiki/G-code#M84:_Stop_idle_hold">M84: Stop idle hold</a>
    This command can be used to set the stepper inactivity timeout (`S`) or to disable steppers (`X`,`Y`,`Z`,`E`)
	This command can be used without any additional parameters. In that case all steppers are disabled.
    
    The file completeness check uses this parameter to detect an incomplete file. It has to be present at the end of a file with no parameters.
	
        M84 [ S | X | Y | Z | E ]
	
	  - `S` - Seconds
	  - `X` - X axis
	  - `Y` - Y axis
	  - `Z` - Z axis
	  - `E` - Exruder

	### M18 - Disable steppers <a href="https://reprap.org/wiki/G-code#M18:_Disable_all_stepper_motors">M18: Disable all stepper motors</a>
	Equal to M84 (compatibility)
    */
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
		  if (code_seen('X')) disable_x();
		  if (code_seen('Y')) disable_y();
		  if (code_seen('Z')) disable_z();
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
		  if (code_seen('E')) {
			  disable_e0();
			  disable_e1();
			  disable_e2();
            }
          #endif
        }
      }
	  //in the end of print set estimated time to end of print and extruders used during print to default values for next print
	  snmm_filaments_used = 0;
      break;

    /*!
	### M85 - Set max inactive time <a href="https://reprap.org/wiki/G-code#M85:_Set_Inactivity_Shutdown_Timer">M85: Set Inactivity Shutdown Timer</a>
    #### Usage
    
        M85 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
    case 85: // M85
      if(code_seen('S')) {
        max_inactive_time = code_value() * 1000;
      }
      break;
#ifdef SAFETYTIMER

    /*!
    ### M86 - Set safety timer expiration time <a href="https://reprap.org/wiki/G-code#M86:_Set_Safety_Timer_expiration_time">M86: Set Safety Timer expiration time</a>	
    When safety timer expires, heatbed and nozzle target temperatures are set to zero.
    #### Usage
    
        M86 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
	case 86: 
	  if (code_seen('S')) {
	    safetytimer_inactive_time = code_value() * 1000;
		safetyTimer.start();
	  }
	  break;
#endif

    /*!
	### M92 Set Axis steps-per-unit <a href="https://reprap.org/wiki/G-code#M92:_Set_axis_steps_per_unit">M92: Set axis_steps_per_unit</a>
	Allows programming of steps per unit (usually mm) for motor drives. These values are reset to firmware defaults on power on, unless saved to EEPROM if available (M500 in Marlin)
	#### Usage
    
	    M92 [ X | Y | Z | E ]
	
    #### Parameters
	- `X` - Steps per unit for the X drive
	- `Y` - Steps per unit for the Y drive
	- `Z` - Steps per unit for the Z drive
	- `E` - Steps per unit for the extruder drive
    */
    case 92:
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == E_AXIS) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = cs.axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              cs.max_jerk[E_AXIS] *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            cs.axis_steps_per_unit[i] = value;
#if defined(FILAMENT_SENSOR) && defined(PAT9125)
            fsensor_set_axis_steps_per_unit(value);
#endif
          }
          else {
            cs.axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;

    /*!
	### M110 - Set Line number <a href="https://reprap.org/wiki/G-code#M110:_Set_Current_Line_Number">M110: Set Current Line Number</a>
	Sets the line number in G-code
	#### Usage
    
	    M110 [ N ]
	
    #### Parameters
	- `N` - Line number
    */
    case 110:
      if (code_seen('N'))
	    gcode_LastN = code_value_long();
    break;

    /*!
    ### M113 - Get or set host keep-alive interval <a href="https://reprap.org/wiki/G-code#M113:_Host_Keepalive">M113: Host Keepalive</a>
    During some lengthy processes, such as G29, Marlin may appear to the host to have “gone away.” The “host keepalive” feature will send messages to the host when Marlin is busy or waiting for user response so the host won’t try to reconnect (or disconnect).
    #### Usage
    
        M113 [ S ]
	
    #### Parameters
	- `S` - Seconds. Default is 2 seconds between "busy" messages
    */
	case 113:
		if (code_seen('S')) {
			host_keepalive_interval = (uint8_t)code_value_short();
//			NOMORE(host_keepalive_interval, 60);
		}
		else {
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR("M113 S", (unsigned long)host_keepalive_interval);
			SERIAL_PROTOCOLLN("");
		}
		break;

#ifdef EXTENDED_CAPABILITIES_REPORT
      extended_capabilities_report();
#endif //EXTENDED_CAPABILITIES_REPORT
      
      break;

    /*!
	### M114 - Get current position <a href="https://reprap.org/wiki/G-code#M114:_Get_Current_Position">M114: Get Current Position</a>
    */
    case 114:
		gcode_M114();
      break;

      
      /*
        M117 moved up to get the high priority

    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;*/

    /*!
	### M120 - Enable endstops <a href="https://reprap.org/wiki/G-code#M120:_Enable_endstop_detection">M120: Enable endstop detection</a>
    */
    case 120:
      enable_endstops(false) ;
      break;

    /*!
	### M121 - Disable endstops <a href="https://reprap.org/wiki/G-code#M121:_Disable_endstop_detection">M121: Disable endstop detection</a>
    */
    case 121:
      enable_endstops(true) ;
      break;

    /*!
	### M119 - Get endstop states <a href="https://reprap.org/wiki/G-code#M119:_Get_Endstop_Status">M119: Get Endstop Status</a>
	Returns the current state of the configured X, Y, Z endstops. Takes into account any 'inverted endstop' settings, so one can confirm that the machine is interpreting the endstops correctly.
    */
    case 119:
    SERIAL_PROTOCOLRPGM(_N("Reporting endstop status"));////MSG_M119_REPORT
    SERIAL_PROTOCOLLN("");
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("x_min: "));////MSG_X_MIN
        if(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("x_max: "));////MSG_X_MAX
        if(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("y_min: "));////MSG_Y_MIN
        if(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("y_max: "));////MSG_Y_MAX
        if(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(MSG_Z_MIN);
        if(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(MSG_Z_MAX);
        if(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN("");
      #endif
      break;
      //!@todo update for all axes, use for loop
    

    #ifdef BLINKM
    /*!
	### M150 - Set RGB(W) Color <a href="https://reprap.org/wiki/G-code#M150:_Set_LED_color">M150: Set LED color</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code by defining BLINKM and its dependencies.
    #### Usage
    
        M150 [ R | U | B ]
    
    #### Parameters
    - `R` - Red color value
    - `U` - Green color value. It is NOT `G`!
    - `B` - Blue color value
    */
    case 150:
      {
        byte red;
        byte grn;
        byte blu;

        if(code_seen('R')) red = code_value();
        if(code_seen('U')) grn = code_value();
        if(code_seen('B')) blu = code_value();

        SendColors(red,grn,blu);
      }
      break;
    #endif //BLINKM



    /*!
	### M201 - Set Print Max Acceleration <a href="https://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration">M201: Set max printing acceleration</a>
    For each axis individually.
    */
    case 201:
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				unsigned long val = code_value();
#ifdef TMC2130
				unsigned long val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_ACCEL_XY)
						val = NORMAL_MAX_ACCEL_XY;
					if (val_silent > SILENT_MAX_ACCEL_XY)
						val_silent = SILENT_MAX_ACCEL_XY;
				}
				cs.max_acceleration_units_per_sq_second_normal[i] = val;
				cs.max_acceleration_units_per_sq_second_silent[i] = val_silent;
#else //TMC2130
				max_acceleration_units_per_sq_second[i] = val;
#endif //TMC2130
			}
		}
		// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
		break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * cs.axis_steps_per_unit[i];
      }
      break;
    #endif

    /*!
	### M203 - Set Max Feedrate <a href="https://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate">M203: Set maximum feedrate</a>
    For each axis individually.
    */
    case 203: // M203 max feedrate mm/sec
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				float val = code_value();
#ifdef TMC2130
				float val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_FEEDRATE_XY)
						val = NORMAL_MAX_FEEDRATE_XY;
					if (val_silent > SILENT_MAX_FEEDRATE_XY)
						val_silent = SILENT_MAX_FEEDRATE_XY;
				}
				cs.max_feedrate_normal[i] = val;
				cs.max_feedrate_silent[i] = val_silent;
#else //TMC2130
				max_feedrate[i] = val;
#endif //TMC2130
			}
		}
		break;

    /*!
	### M204 - Acceleration settings <a href="https://reprap.org/wiki/G-code#M204:_Set_default_acceleration">M204: Set default acceleration</a>

    #### Old format:
    ##### Usage
    
        M204 [ S | T ]
        
    ##### Parameters
    - `S` - normal moves
    - `T` - filmanent only moves
    
    #### New format:
    ##### Usage
    
        M204 [ P | R | T ]
    
    ##### Parameters
    - `P` - printing moves
    - `R` - filmanent only moves
    - `T` - travel moves (as of now T is ignored)
	*/
    case 204:
      {
        if(code_seen('S')) {
          // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware,
          // and it is also generated by Slic3r to control acceleration per extrusion type
          // (there is a separate acceleration settings in Slicer for perimeter, first layer etc).
          cs.acceleration = code_value();
          // Interpret the T value as retract acceleration in the old Marlin format.
          if(code_seen('T'))
            cs.retract_acceleration = code_value();
        } else {
          // New acceleration format, compatible with the upstream Marlin.
          if(code_seen('P'))
            cs.acceleration = code_value();
          if(code_seen('R'))
            cs.retract_acceleration = code_value();
          if(code_seen('T')) {
            // Interpret the T value as the travel acceleration in the new Marlin format.
            /*!
            @todo Prusa3D firmware currently does not support travel acceleration value independent from the extruding acceleration value.
            */
            // travel_acceleration = code_value();
          }
        }
      }
      break;

    /*!
	### M205 - Set advanced settings <a href="https://reprap.org/wiki/G-code#M205:_Advanced_settings">M205: Advanced settings</a>
    Set some advanced settings related to movement.
    #### Usage
    
        M205 [ S | T | B | X | Y | Z | E ]
        
    #### Parameters
    - `S` - Minimum feedrate for print moves (unit/s)
    - `T` - Minimum feedrate for travel moves (units/s)
    - `B` - Minimum segment time (us)
    - `X` - Maximum X jerk (units/s)
    - `Y` - Maximum Y jerk (units/s)
    - `Z` - Maximum Z jerk (units/s)
    - `E` - Maximum E jerk (units/s)
    */
    case 205: 
    {
      if(code_seen('S')) cs.minimumfeedrate = code_value();
      if(code_seen('T')) cs.mintravelfeedrate = code_value();
      if(code_seen('B')) cs.minsegmenttime = code_value() ;
      if(code_seen('X')) cs.max_jerk[X_AXIS] = cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Y')) cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Z')) cs.max_jerk[Z_AXIS] = code_value();
      if(code_seen('E'))
      {
          float e = code_value();
#ifndef LA_NOCOMPAT
          e = la10c_jerk(e);
#endif
          cs.max_jerk[E_AXIS] = e;
      }
      if (cs.max_jerk[X_AXIS] > DEFAULT_XJERK) cs.max_jerk[X_AXIS] = DEFAULT_XJERK;
      if (cs.max_jerk[Y_AXIS] > DEFAULT_YJERK) cs.max_jerk[Y_AXIS] = DEFAULT_YJERK;
    }
    break;

    /*!
	### M206 - Set additional homing offsets <a href="https://reprap.org/wiki/G-code#M206:_Offset_axes">M206: Offset axes</a>
    #### Usage
    
        M206 [ X | Y | Z ]
    
    #### Parameters
    - `X` - X axis offset
    - `Y` - Y axis offset
    - `Z` - Z axis offset
	*/
    case 206:
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) cs.add_homing[i] = code_value();
      }
      break;
   
  

    /*!
	### M220 Set feedrate percentage <a href="https://reprap.org/wiki/G-code#M220:_Set_speed_factor_override_percentage">M220: Set speed factor override percentage</a>
	#### Usage
    
        M220 [ B | S | R ]
    
    #### Parameters
    - `B` - Backup current speed factor
	- `S` - Speed factor override percentage (0..100 or higher)
	- `R` - Restore previous speed factor
    */
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
        bool codesWereSeen = false;
        if (code_seen('B')) //backup current speed factor
        {
            saved_feedmultiply_mm = feedmultiply;
            codesWereSeen = true;
        }
        if (code_seen('S'))
        {
            feedmultiply = code_value();
            codesWereSeen = true;
        }
        if (code_seen('R')) //restore previous feedmultiply
        {
            feedmultiply = saved_feedmultiply_mm;
            codesWereSeen = true;
        }
        if (!codesWereSeen)
        {
            printf_P(PSTR("%i%%\n"), feedmultiply);
        }
    }
    break;

    /*!
    ### M226 - Wait for Pin state <a href="https://reprap.org/wiki/G-code#M226:_Wait_for_pin_state">M226: Wait for pin state</a>
    Wait until the specified pin reaches the state required
    #### Usage
    
        M226 [ P | S ]
    
    #### Parameters
    - `P` - pin number
    - `S` - pin state
    */
	case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        int pin_number = code_value(); // pin number
        int pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
          {
            if (sensitive_pins[i] == pin_number)
            {
              pin_number = -1;
              break;
            }
          }

          if (pin_number > -1)
          {
            int target = LOW;

            st_synchronize();

            pinMode(pin_number, INPUT);

            switch(pin_state){
            case 1:
              target = HIGH;
              break;

            case 0:
              target = LOW;
              break;

            case -1:
              target = !digitalRead(pin_number);
              break;
            }

            while(digitalRead(pin_number) != target){
              manage_heater();
              manage_inactivity();
              lcd_update(0);
            }
          }
        }
      }
    }
    break;

    #if NUM_SERVOS > 0

    /*!
	### M280 - Set/Get servo position <a href="https://reprap.org/wiki/G-code#M280:_Set_servo_position">M280: Set servo position</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    #### Usage
    
        M280 [ P | S ]
    
    #### Parameters
    - `P` - Servo index (id)
    - `S` - Target position
    */
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              _delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL(servos[servo_index].read());
          SERIAL_PROTOCOLLN("");
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
    
    /*!
	### M300 - Play tone <a href="https://reprap.org/wiki/G-code#M300:_Play_beep_sound">M300: Play beep sound</a>
	In Prusa Firmware the defaults are `100Hz` and `1000ms`, so that `M300` without parameters will beep for a second.
    #### Usage
    
        M300 [ S | P ]
    
    #### Parameters
    - `S` - frequency in Hz. Not all firmware versions support this parameter
    - `P` - duration in milliseconds
    */
    case 300: // M300
    {
      int beepS = code_seen('S') ? code_value() : 110;
      int beepP = code_seen('P') ? code_value() : 1000;
      if (beepS > 0)
      {
        #if BEEPER > 0
          Sound_MakeCustom(beepP,beepS,false);
        #endif
      }
      else
      {
        _delay(beepP);
      }
    }
    break;
    #endif // M300

    

    
    /*!
	### M400 - Wait for all moves to finish <a href="https://reprap.org/wiki/G-code#M400:_Wait_for_current_moves_to_finish">M400: Wait for current moves to finish</a>
	Finishes all current moves and and thus clears the buffer.
    Equivalent to `G4` with no parameters.
    */
    case 400:
    {
      st_synchronize();
    }
    break;

    /*!
	### M500 - Store settings in EEPROM <a href="https://reprap.org/wiki/G-code#M500:_Store_parameters_in_non-volatile_storage">M500: Store parameters in non-volatile storage</a>
	Save current parameters to EEPROM.
    */
    case 500:
    {
        Config_StoreSettings();
    }
    break;

    /*!
	### M501 - Read settings from EEPROM <a href="https://reprap.org/wiki/G-code#M501:_Read_parameters_from_EEPROM">M501: Read parameters from EEPROM</a>
	Set the active parameters to those stored in the EEPROM. This is useful to revert parameters after experimenting with them.
    */
    case 501:
    {
        Config_RetrieveSettings();
    }
    break;

    /*!
	### M502 - Revert all settings to factory default <a href="https://reprap.org/wiki/G-code#M502:_Restore_Default_Settings">M502: Restore Default Settings</a>
	This command resets all tunable parameters to their default values, as set in the firmware's configuration files. This doesn't reset any parameters stored in the EEPROM, so it must be followed by M500 to write the default settings.
    */
    case 502:
    {
        Config_ResetDefault();
    }
    break;

    /*!
	### M503 - Repport all settings currently in memory <a href="https://reprap.org/wiki/G-code#M503:_Report_Current_Settings">M503: Report Current Settings</a>
	This command asks the firmware to reply with the current print settings as set in memory. Settings will differ from EEPROM contents if changed since the last load / save. The reply output includes the G-Code commands to produce each setting. For example, Steps-Per-Unit values are displayed as an M92 command.
    */
    case 503:
    {
        Config_PrintSettings();
    }
    break;

    /*!
	### M509 - Force language selection <a href="https://reprap.org/wiki/G-code#M509:_Force_language_selection">M509: Force language selection</a>
	Resets the language to English.
	Only on Original Prusa i3 MK2.5/s and MK3/s with multiple languages.
	*/
    case 509:
    {
		lang_reset();
        SERIAL_ECHO_START;
        SERIAL_PROTOCOLPGM(("LANG SEL FORCED"));
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

    /*!
	### M540 - Abort print on endstop hit (enable/disable) <a href="https://reprap.org/wiki/G-code#M540_in_Marlin:_Enable.2FDisable_.22Stop_SD_Print_on_Endstop_Hit.22">M540 in Marlin: Enable/Disable "Stop SD Print on Endstop Hit"</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code. You must define `ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED`.
    #### Usage
    
        M540 [ S ]
    
    #### Parameters
    - `S` - disabled=0, enabled=1
	*/
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif

	/*!
	### M851 - Set Z-Probe Offset <a href="https://reprap.org/wiki/G-code#M851:_Set_Z-Probe_Offset">M851: Set Z-Probe Offset"</a>
    Sets the Z-probe Z offset. This offset is used to determine the actual Z position of the nozzle when using a probe to home Z with G28. This value may also be used by G81 (Prusa) / G29 (Marlin) to apply correction to the Z position.
	This value represents the distance from nozzle to the bed surface at the point where the probe is triggered. This value will be negative for typical switch probes, inductive probes, and setups where the nozzle makes a circuit with a raised metal contact. This setting will be greater than zero on machines where the nozzle itself is used as the probe, pressing down on the bed to press a switch. (This is a common setup on delta machines.)
    #### Usage
    
        M851 [ Z ]
    
    #### Parameters
    - `Z` - Z offset probe to nozzle.
	*/
    #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
    {
      float value;
      if (code_seen('Z'))
      {
        value = code_value();
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          cs.zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
          SERIAL_ECHO_START;
          SERIAL_ECHOLNRPGM(CAT4(MSG_ZPROBE_ZOFFSET, " ", MSG_OK,PSTR("")));
          SERIAL_PROTOCOLLN("");
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHORPGM(MSG_ZPROBE_ZOFFSET);
          SERIAL_ECHORPGM(MSG_Z_MIN);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_ECHORPGM(MSG_Z_MAX);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
          SERIAL_PROTOCOLLN("");
        }
      }
      else
      {
          SERIAL_ECHO_START;
          SERIAL_ECHOLNRPGM(CAT2(MSG_ZPROBE_ZOFFSET, PSTR(" : ")));
          SERIAL_ECHO(-cs.zprobe_zoffset);
          SERIAL_PROTOCOLLN("");
      }
      break;
    }
    #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

  
 
    /*!
	### M907 - Set digital trimpot motor current in mA using axis codes <a href="https://reprap.org/wiki/G-code#M907:_Set_digital_trimpot_motor">M907: Set digital trimpot motor</a>
	Set digital trimpot motor current using axis codes (X, Y, Z, E, B, S).
	#### Usage
    
        M907 [ X | Y | Z | E | B | S ]
	
    #### Parameters
    - `X` - X motor driver
    - `Y` - Y motor driver
    - `Z` - Z motor driver
    - `E` - Extruder motor driver
    - `B` - Second Extruder motor driver
    - `S` - All motors
    */
    case 907:
    {

      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) st_current_set(i,code_value());
        if(code_seen('B')) st_current_set(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) st_current_set(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) st_current_set(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) st_current_set(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) st_current_set(2, code_value());
      #endif
    }
    break;

    /*!
	### M908 - Control digital trimpot directly <a href="https://reprap.org/wiki/G-code#M908:_Control_digital_trimpot_directly">M908: Control digital trimpot directly</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code. Not usable on Prusa printers.
    #### Usage
    
        M908 [ P | S ]
    
    #### Parameters
    - `P` - channel
    - `S` - current
    */
    case 908:
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;

    /*!
	### M350 - Set microstepping mode <a href="https://reprap.org/wiki/G-code#M350:_Set_microstepping_mode">M350: Set microstepping mode</a>
    Printers with TMC2130 drivers have `X`, `Y`, `Z` and `E` as options. The steps-per-unit value is updated accordingly. Not all resolutions are valid!
    Printers without TMC2130 drivers also have `B` and `S` options. In this case, the steps-per-unit value in not changed!
    #### Usage
    
        M350 [ X | Y | Z | E | B | S ]
    
    #### Parameters
    - `X` - X new resolution
    - `Y` - Y new resolution
    - `Z` - Z new resolution
    - `E` - E new resolution
    
    Only valid for MK2.5(S) or printers without TMC2130 drivers
    - `B` - Second extruder new resolution
    - `S` - All axes new resolution
    */
    case 350: 
    {

      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
    }
    break;

    /*!
	### M351 - Toggle Microstep Pins <a href="https://reprap.org/wiki/G-code#M351:_Toggle_MS1_MS2_pins_directly">M351: Toggle MS1 MS2 pins directly</a>
    Toggle MS1 MS2 pins directly.
    #### Usage
    
        M351 [B<0|1>] [E<0|1>] S<1|2> [X<0|1>] [Y<0|1>] [Z<0|1>]
    
    #### Parameters
    - `X` - Update X axis
    - `Y` - Update Y axis
    - `Z` - Update Z axis
    - `E` - Update E axis
    - `S` - which MSx pin to toggle
    - `B` - new pin value
    */
    case 351:
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;





    /*!
    ### M999 - Restart after being stopped <a href="https://reprap.org/wiki/G-code#M999:_Restart_after_being_stopped_by_error">M999: Restart after being stopped by error</a>
    @todo Usually doesn't work. Should be fixed or removed. Most of the time, if `Stopped` it set, the print fails and is unrecoverable.
    */
    case 999:
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
	/*!
	#### End of M-Commands
    */
	default: 
		printf_P(PSTR("Unknown M code: %s \n"), cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END M-CODE=%u\n"), mcode_in_progress);
	mcode_in_progress = 0;
	}
  }
  // end if(code_seen('M')) (end of M codes)
  /*!
  -----------------------------------------------------------------------------------------
  # T Codes
  T<extruder nr.> - select extruder in case of multi extruder printer. select filament in case of MMU_V2.
  #### For MMU_V2:
  T<n> Gcode to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
  @n T? Gcode to extrude shouldn't have to follow, load to extruder wheels is done automatically
  @n Tx Same as T?, except nozzle doesn't have to be preheated. Tc must be placed after extruder nozzle is preheated to finish filament load.
  @n Tc Load to nozzle after filament was prepared by Tc and extruder nozzle is already heated.
  */
  else if(code_seen('T')) {
    int index;
    for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);

	  *(strchr_pointer + index) = tolower(*(strchr_pointer + index));

      if (( *(strchr_pointer + index) < '0' || *(strchr_pointer + index) > '4') && *(strchr_pointer + index) != '?') {
          SERIAL_ECHOLNPGM("Invalid T code.");
      } else { //TCode was Valid
        if (*(strchr_pointer + index) == '?') {
          tmp_extruder = choose_menu_P(_T(MSG_CHOOSE_EXTRUDER), _T(MSG_EXTRUDER));            
        } else {
          tmp_extruder = code_value();
        }
        st_synchronize();
        snmm_filaments_used |= (1 << tmp_extruder); //for stop print
#ifdef SNMM
              mmu_extruder = tmp_extruder;

              _delay(100);

              disable_e0();
              disable_e1();
              disable_e2();

              pinMode(E_MUX0_PIN, OUTPUT);
              pinMode(E_MUX1_PIN, OUTPUT);

              _delay(100);
              SERIAL_ECHO_START;
              SERIAL_ECHO("T:");
              SERIAL_ECHOLN((int)tmp_extruder);
              switch (tmp_extruder) {
              case 1:
                  WRITE(E_MUX0_PIN, HIGH);
                  WRITE(E_MUX1_PIN, LOW);

                  break;
              case 2:
                  WRITE(E_MUX0_PIN, LOW);
                  WRITE(E_MUX1_PIN, HIGH);

                  break;
              case 3:
                  WRITE(E_MUX0_PIN, HIGH);
                  WRITE(E_MUX1_PIN, HIGH);

                  break;
              default:
                  WRITE(E_MUX0_PIN, LOW);
                  WRITE(E_MUX1_PIN, LOW);

                  break;
              }
              _delay(100);

#else //SNMM
              if (tmp_extruder >= EXTRUDERS) {
                  SERIAL_ECHO_START;
                  SERIAL_ECHO('T');
                  SERIAL_PROTOCOLLN((int)tmp_extruder);
                  SERIAL_ECHOLNRPGM(_n("Invalid extruder"));////MSG_INVALID_EXTRUDER
              }
              else {
#if EXTRUDERS > 1
                  boolean make_move = false;
#endif
                  if (code_seen('F')) {
#if EXTRUDERS > 1
                      make_move = true;
#endif
                      next_feedrate = code_value();
                      if (next_feedrate > 0.0) {
                          feedrate = next_feedrate;
                      }
                  }
#if EXTRUDERS > 1
                  if (tmp_extruder != active_extruder) {
                      // Save current position to return to after applying extruder offset
                      memcpy(destination, current_position, sizeof(destination));
                      // Offset extruder (only by XY)
                      int i;
                      for (i = 0; i < 2; i++) {
                          current_position[i] = current_position[i] -
                                  extruder_offset[i][active_extruder] +
                                  extruder_offset[i][tmp_extruder];
                      }
                      // Set the new active extruder and position
                      active_extruder = tmp_extruder;
                      plan_set_position_curposXYZE();
                      // Move to the old position if 'F' was in the parameters
                      if (make_move && Stopped == false) {
                          prepare_move();
                      }
                  }
#endif
                  SERIAL_ECHO_START;
                  SERIAL_ECHORPGM(_n("Active Extruder: "));////MSG_ACTIVE_EXTRUDER
                  SERIAL_PROTOCOLLN((int)active_extruder);
              }

#endif //SNMM
          
      }
  } // end if(code_seen('T')) (end of T codes)
  /*!
  #### End of T-Codes
  */

  /**
  *---------------------------------------------------------------------------------
  *# D codes
  */
  else if (code_seen('D')) // D codes (debug)
  {
    switch((int)code_value())
    {

    /*!
    ### D-1 - Endless Loop <a href="https://reprap.org/wiki/G-code#D-1:_Endless_Loop">D-1: Endless Loop</a>
    */
	case -1:
		dcode__1(); break;
#ifdef DEBUG_DCODES

    /*!
    ### D0 - Reset <a href="https://reprap.org/wiki/G-code#D0:_Reset">D0: Reset</a>
    #### Usage
    
        D0 [ B ]
    
    #### Parameters
    - `B` - Bootloader
    */
	case 0:
		dcode_0(); break;

    /*!
    *
    ### D1 - Clear EEPROM and RESET <a href="https://reprap.org/wiki/G-code#D1:_Clear_EEPROM_and_RESET">D1: Clear EEPROM and RESET</a>
      
          D1
      
    *
    */
	case 1:
		dcode_1(); break;

    /*!
    ### D2 - Read/Write RAM <a href="https://reprap.org/wiki/G-code#D2:_Read.2FWrite_RAM">D3: Read/Write RAM</a>
    This command can be used without any additional parameters. It will read the entire RAM.
    #### Usage
    
        D2 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x1fff)
    - `C` - Count (1-8192)
    - `X` - Data

	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 2:
		dcode_2(); break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE3 || defined DEBUG_DCODES

    /*!
    ### D3 - Read/Write EEPROM <a href="https://reprap.org/wiki/G-code#D3:_Read.2FWrite_EEPROM">D3: Read/Write EEPROM</a>
    This command can be used without any additional parameters. It will read the entire eeprom.
    #### Usage
    
        D3 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x0fff)
    - `C` - Count (1-4096)
    - `X` - Data (hex)
	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 3:
		dcode_3(); break;
#endif //DEBUG_DCODE3
#ifdef DEBUG_DCODES

    /*!
    
    ### D4 - Read/Write PIN <a href="https://reprap.org/wiki/G-code#D4:_Read.2FWrite_PIN">D4: Read/Write PIN</a>
    To read the digital value of a pin you need only to define the pin number.
    #### Usage
    
        D4 [ P | F | V ]
    
    #### Parameters
    - `P` - Pin (0-255)
    - `F` - Function in/out (0/1)
    - `V` - Value (0/1)
    */
	case 4:
		dcode_4(); break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE5 || defined DEBUG_DCODES

    /*!
    ### D5 - Read/Write FLASH <a href="https://reprap.org/wiki/G-code#D5:_Read.2FWrite_FLASH">D5: Read/Write Flash</a>
    This command can be used without any additional parameters. It will read the 1kb FLASH.
    #### Usage
    
        D5 [ A | C | X | E ]
    
    #### Parameters
    - `A` - Address (x00000-x3ffff)
    - `C` - Count (1-8192)
    - `X` - Data (hex)
    - `E` - Erase
 	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
   */
	case 5:
		dcode_5(); break;
#endif //DEBUG_DCODE5
#ifdef DEBUG_DCODES

    /*!
    ### D6 - Read/Write external FLASH <a href="https://reprap.org/wiki/G-code#D6:_Read.2FWrite_external_FLASH">D6: Read/Write external Flash</a>
    Reserved
    */
	case 6:
		dcode_6(); break;

    /*!
    ### D7 - Read/Write Bootloader <a href="https://reprap.org/wiki/G-code#D7:_Read.2FWrite_Bootloader">D7: Read/Write Bootloader</a>
    Reserved
    */
	case 7:
		dcode_7(); break;

    /*!
    ### D8 - Read/Write PINDA <a href="https://reprap.org/wiki/G-code#D8:_Read.2FWrite_PINDA">D8: Read/Write PINDA</a>
    #### Usage
    
        D8 [ ? | ! | P | Z ]
    
    #### Parameters
    - `?` - Read PINDA temperature shift values
    - `!` - Reset PINDA temperature shift values to default
    - `P` - Pinda temperature [C]
    - `Z` - Z Offset [mm]
    */
	case 8:
		dcode_8(); break;

    /*!
    ### D9 - Read ADC <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9: Read ADC</a>
    #### Usage
    
        D9 [ I | V ]
    
    #### Parameters
    - `I` - ADC channel index 
        - `0` - Heater 0 temperature
        - `1` - Heater 1 temperature
        - `2` - Bed temperature
        - `3` - PINDA temperature
        - `4` - PWR voltage
        - `5` - Ambient temperature
        - `6` - BED voltage
    - `V` Value to be written as simulated
    */
	case 9:
		dcode_9(); break;

    /*!
    ### D10 - Set XYZ calibration = OK <a href="https://reprap.org/wiki/G-code#D10:_Set_XYZ_calibration_.3D_OK">D10: Set XYZ calibration = OK</a>
    */
	case 10:
		dcode_10(); break;

    /*!
    ### D12 - Time <a href="https://reprap.org/wiki/G-code#D12:_Time">D12: Time</a>
    Writes the current time in the log file.
    */

#endif //DEBUG_DCODES
#ifdef HEATBED_ANALYSIS

    /*!
    ### D80 - Bed check <a href="https://reprap.org/wiki/G-code#D80:_Bed_check">D80: Bed check</a>
    This command will log data to SD card file "mesh.txt".
    #### Usage
    
        D80 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 80:
		dcode_80(); break;

    /*!
    ### D81 - Bed analysis <a href="https://reprap.org/wiki/G-code#D81:_Bed_analysis">D80: Bed analysis</a>
    This command will log data to SD card file "wldsd.txt".
    #### Usage
    
        D81 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 81:
		dcode_81(); break;
	
#endif //HEATBED_ANALYSIS
#ifdef DEBUG_DCODES

    /*!
    ### D106 - Print measured fan speed for different pwm values <a href="https://reprap.org/wiki/G-code#D106:_Print_measured_fan_speed_for_different_pwm_values">D106: Print measured fan speed for different pwm values</a>
    */
	case 106:
		dcode_106(); break;

#ifdef TMC2130
    /*!
    ### D2130 - Trinamic stepper controller <a href="https://reprap.org/wiki/G-code#D2130:_Trinamic_stepper_controller">D2130: Trinamic stepper controller</a>
    @todo Please review by owner of the code. RepRap Wiki Gcode needs to be updated after review of owner as well.
    
    #### Usage
    
        D2130 [ Axis | Command | Subcommand | Value ]
    
    #### Parameters
    - Axis
      - `X` - X stepper driver
      - `Y` - Y stepper driver
      - `Z` - Z stepper driver
      - `E` - Extruder stepper driver
    - Commands
      - `0`   - Current off
      - `1`   - Current on
      - `+`   - Single step
      - `-`   - Single step oposite direction
      - `NNN` - Value sereval steps
      - `?`   - Read register
      - Subcommands for read register
        - `mres`     - Micro step resolution. More information in datasheet '5.5.2 CHOPCONF – Chopper Configuration'
        - `step`     - Step
        - `mscnt`    - Microstep counter. More information in datasheet '5.5 Motor Driver Registers'
        - `mscuract` - Actual microstep current for motor. More information in datasheet '5.5 Motor Driver Registers'
        - `wave`     - Microstep linearity compensation curve
      - `!`   - Set register
      - Subcommands for set register
        - `mres`     - Micro step resolution
        - `step`     - Step
        - `wave`     - Microstep linearity compensation curve
        - Values for set register
          - `0, 180 --> 250` - Off
          - `0.9 --> 1.25`   - Valid values (recommended is 1.1)
      - `@`   - Home calibrate axis
    
    Examples:
      
          D2130E?wave
      
      Print extruder microstep linearity compensation curve
      
          D2130E!wave0
      
      Disable extruder linearity compensation curve, (sine curve is used)
      
          D2130E!wave220
      
      (sin(x))^1.1 extruder microstep compensation curve used
    
    Notes:
      For more information see https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
    *
	*/
	case 2130:
		dcode_2130(); break;
#endif //TMC2130

#if (defined (FILAMENT_SENSOR) && defined(PAT9125))

    /*!
    ### D9125 - PAT9125 filament sensor <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9125: PAT9125 filament sensor</a>
    #### Usage
    
        D9125 [ ? | ! | R | X | Y | L ]
    
    #### Parameters
    - `?` - Print values
    - `!` - Print values
    - `R` - Resolution. Not active in code
    - `X` - X values
    - `Y` - Y values
    - `L` - Activate filament sensor log
    */
	case 9125:
		dcode_9125(); break;
#endif //FILAMENT_SENSOR

#endif //DEBUG_DCODES
	}
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHORPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(CMDBUFFER_CURRENT_STRING);
    SERIAL_ECHOLNPGM("\"(2)");
  }
  KEEPALIVE_STATE(NOT_BUSY);
  ClearToSend();
}

/*!
#### End of D-Codes
*/

/** @defgroup GCodes G-Code List 
*/

// ---------------------------------------------------

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  printf_P(_N("%S: %ld\n%S\n"), _n("Resend"), gcode_LastN + 1, MSG_OK);
}

// Confirm the execution of a command, if sent from a serial line.
// Execution of a command from a SD card will not be confirmed.
void ClearToSend()
{
    previous_millis_cmd = _millis();
	if ((CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB) || (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR)) 
		SERIAL_PROTOCOLLNRPGM(MSG_OK);
}

#if MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
void update_currents() {
	float current_high[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;
	float current_low[3] = DEFAULT_PWM_MOTOR_CURRENT;
	float tmp_motor[3];
	
	//SERIAL_ECHOLNPGM("Currents updated: ");

	if (destination[Z_AXIS] < Z_SILENT) {
		//SERIAL_ECHOLNPGM("LOW");
		for (uint8_t i = 0; i < 3; i++) {
			st_current_set(i, current_low[i]);		
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(current_low[i]);*/
		}		
	}
	else if (destination[Z_AXIS] > Z_HIGH_POWER) {
		//SERIAL_ECHOLNPGM("HIGH");
		for (uint8_t i = 0; i < 3; i++) {
			st_current_set(i, current_high[i]);
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(current_high[i]);*/
		}		
	}
	else {
		for (uint8_t i = 0; i < 3; i++) {
			float q = current_low[i] - Z_SILENT*((current_high[i] - current_low[i]) / (Z_HIGH_POWER - Z_SILENT));
			tmp_motor[i] = ((current_high[i] - current_low[i]) / (Z_HIGH_POWER - Z_SILENT))*destination[Z_AXIS] + q;
			st_current_set(i, tmp_motor[i]);			
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(tmp_motor[i]);*/
		}
	}
}
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      bool relative = axis_relative_modes & (1 << i);
      destination[i] = (float)code_value();
      if (i == E_AXIS) {
        float emult = extruder_multiplier[active_extruder];
        if (emult != 1.) {
          if (! relative) {
            destination[i] -= current_position[i];
            relative = true;
          }
          destination[i] *= emult;
        }
      }
      if (relative)
        destination[i] += current_position[i];
      seen[i]=true;

    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();

    if(next_feedrate > 0.0) feedrate = next_feedrate;
	if (!seen[0] && !seen[1] && !seen[2] && seen[3])
	{
//		float e_max_speed = 
//		printf_P(PSTR("E MOVE speed %7.3f\n"), feedrate / 60)
	}
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
#ifdef DEBUG_DISABLE_SWLIMITS
	return;
#endif //DEBUG_DISABLE_SWLIMITS

    // Clamp the Z coordinate.
    if (min_software_endstops) {
        float negative_z_offset = 0;
        #ifdef ENABLE_AUTO_BED_LEVELING
            if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset = negative_z_offset + Z_PROBE_OFFSET_FROM_EXTRUDER;
            if (cs.add_homing[Z_AXIS] < 0) negative_z_offset = negative_z_offset + cs.add_homing[Z_AXIS];
        #endif
        if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
    }
    if (max_software_endstops) {
        if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
    }
}

#ifdef MESH_BED_LEVELING
void mesh_plan_buffer_line(const float &x, const float &y, const float &z, const float &e, const float &feed_rate, const uint8_t extruder) {
        float dx = x - current_position[X_AXIS];
        float dy = y - current_position[Y_AXIS];
        int n_segments = 0;

        if (mbl.active) {
            float len = abs(dx) + abs(dy);
            if (len > 0)
                // Split to 3cm segments or shorter.
                n_segments = int(ceil(len / 30.f));
        }

        if (n_segments > 1) {
            // In a multi-segment move explicitly set the final target in the plan
            // as the move will be recalculated in it's entirety
            float gcode_target[NUM_AXIS];
            gcode_target[X_AXIS] = x;
            gcode_target[Y_AXIS] = y;
            gcode_target[Z_AXIS] = z;
            gcode_target[E_AXIS] = e;

            float dz = z - current_position[Z_AXIS];
            float de = e - current_position[E_AXIS];

            for (int i = 1; i < n_segments; ++ i) {
                float t = float(i) / float(n_segments);
                plan_buffer_line(current_position[X_AXIS] + t * dx,
                                 current_position[Y_AXIS] + t * dy,
                                 current_position[Z_AXIS] + t * dz,
                                 current_position[E_AXIS] + t * de,
                                 feed_rate, extruder, gcode_target);
                if (waiting_inside_plan_buffer_line_print_aborted)
                    return;
            }
        }
        // The rest of the path.
        plan_buffer_line(x, y, z, e, feed_rate, extruder);
    }
#endif  // MESH_BED_LEVELING
    
void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = _millis();

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line_destinationXYZE(feedrate/60);
  }
  else {
#ifdef MESH_BED_LEVELING
    mesh_plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply*(1./(60.f*100.f)), active_extruder);
#else
     plan_buffer_line_destinationXYZE(feedrate*feedmultiply*(1./(60.f*100.f)));
#endif
  }

  set_current_to_destination();
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = _millis();
}



void manage_inactivity(bool ignore_stepper_queue) { //default argument set in Marlin.h

#ifdef SAFETYTIMER
	handleSafetyTimer();
#endif //SAFETYTIMER

#if defined(KILL_PIN) && KILL_PIN > -1
	static int killCount = 0;   // make the inactivity button a bit less responsive
   const int KILL_DELAY = 10000;
#endif
	
    if(buflen < (BUFSIZE-1)){
        get_command();
    }

  if( (_millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill(_n("Inactivity Shutdown"), 4);
  if(stepper_inactive_time)  {
    if( (_millis() - previous_millis_cmd) >  stepper_inactive_time ) {
      if(blocks_queued() == false && ignore_stepper_queue == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  
  #ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && (_millis() - chdkHigh > CHDK_DELAY))
    {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif
  
  #if defined(KILL_PIN) && KILL_PIN > -1
    
    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    if( 0 == READ(KILL_PIN) ) {
       killCount++;
    } else if (killCount > 0) {
       killCount--;
    }
    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if ( killCount >= KILL_DELAY) {
       kill(NULL, 5);
    }
  #endif
    
  check_axes_activity();
}

void kill(const char *full_screen_message, unsigned char id)
{
	printf_P(_N("KILL: %d\n"), id);
	//return;
  cli(); // Stop interrupts

  disable_x();
//  SERIAL_ECHOLNPGM("kill - disable Y");
  disable_y();
  poweroff_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNRPGM(_n("Printer halted. kill() called!"));////MSG_ERR_KILLED
  if (full_screen_message != NULL) {
      SERIAL_ERRORLNRPGM(full_screen_message);
      lcd_display_message_fullscreen_P(full_screen_message);
  } else {
      LCD_ALERTMESSAGERPGM(_n("KILLED. "));////MSG_KILLED
  }

  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  for ( int i=5; i--; lcd_update(0))
  {
     _delay(200);	
  }
  cli();   // disable interrupts
  suicide();
  while(1)
  {
#ifdef WATCHDOG
    wdt_reset();
#endif //WATCHDOG
	  /* Intentionally left empty */
	
  } // Wait for reset
}

// Stop: Emergency stop used by overtemp functions which allows recovery
//
//   In addition to stopping the print, this prevents subsequent G[0-3] commands to be
//   processed via USB (using "Stopped") until the print is resumed via M999 or
//   manually started from scratch with the LCD.
//
//   Note that the current instruction is completely discarded, so resuming from Stop()
//   will introduce either over/under extrusion on the current segment, and will not
//   survive a power panic. Switching Stop() to use the pause machinery instead (with
//   the addition of disabling the headers) could allow true recovery in the future.
void Stop()
{
  if(Stopped == false) {
    Stopped = true;
    lcd_print_stop();
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNRPGM(MSG_ERR_STOPPED);
    LCD_MESSAGERPGM(_T(MSG_STOPPED));
  }
}

bool IsStopped() { return Stopped; };

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#ifndef LA_NOCOMPAT
  // Steppers are disabled both when a print is stopped and also via M84 (which is additionally
  // checked-for to indicate a complete file), so abuse this function to reset the LA detection
  // state for the next print.
  la10c_reset();
#endif
}



void long_pause() //long pause print
{
	st_synchronize();
	start_pause_print = _millis();


	//lift z
	current_position[Z_AXIS] += Z_PAUSE_LIFT;
	if (current_position[Z_AXIS] > Z_MAX_POS) current_position[Z_AXIS] = Z_MAX_POS;
	plan_buffer_line_curposXYZE(15);

	//Move XY to side
	current_position[X_AXIS] = X_PAUSE_POS;
	current_position[Y_AXIS] = Y_PAUSE_POS;
	plan_buffer_line_curposXYZE(50);

	// Turn off the print fan
	fanSpeed = 0;
}



void print_world_coordinates()
{
	printf_P(_N("world coordinates: (%.3f, %.3f, %.3f)\n"), current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
}

void print_physical_coordinates()
{
	printf_P(_N("physical coordinates: (%.3f, %.3f, %.3f)\n"), st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));
}




//! @brief Wait for click
//!
//! Set
void marlin_wait_for_click()
{
    int8_t busy_state_backup = busy_state;
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    lcd_consume_click();
    while(!lcd_clicked())
    {
        manage_heater();
        manage_inactivity(true);
        lcd_update(0);
    }
    KEEPALIVE_STATE(busy_state_backup);
}

#define FIL_LOAD_LENGTH 60

#ifdef PSU_Delta
bool bEnableForce_z;

void init_force_z()
{
WRITE(Z_ENABLE_PIN,Z_ENABLE_ON);
bEnableForce_z=true;                              // "true"-value enforce "disable_force_z()" executing
disable_force_z();
}

void check_force_z()
{
if(!(bEnableForce_z||eeprom_read_byte((uint8_t*)EEPROM_SILENT)))
     init_force_z();                              // causes enforced switching into disable-state
}

void disable_force_z()
{
    if(!bEnableForce_z) return;   // motor already disabled (may be ;-p )

    bEnableForce_z=false;

    // switching to silent mode

}

void enable_force_z()
{
if(bEnableForce_z)
     return;                                      // motor already enabled (may be ;-p )
bEnableForce_z=true;



WRITE(Z_ENABLE_PIN,Z_ENABLE_ON);                  // slightly redundant ;-p
}
#endif // PSU_Delta
