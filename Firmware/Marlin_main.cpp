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
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "util.h"
#include "Timer.h"

#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "Dcodes.h"

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

#if NUM_SERVOS > 0
#include "Servo.h"
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



// Extruder offset
#if EXTRUDERS > 1
  #define NUM_EXTRUDER_OFFSETS 3 // XYZ planes
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y) && defined(EXTRUDER_OFFSET_Z)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y, EXTRUDER_OFFSET_Z
#endif
};
#endif

float min_pos[3][2] = { {X_MIN_POS,X_MIN_POS + extruder_offset[0][1]}, {Y_MIN_POS, Y_MIN_POS + extruder_offset[1][1]}, {Z_MIN_POS, Z_MIN_POS + extruder_offset[2][1]} };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};

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
// static unsigned long safetytimer_inactive_time = DEFAULT_SAFETYTIMER_TIME_MINS*60*1000ul;

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
static bool setTargetedHotend(int code, uint8_t &extruder);
static void print_time_remaining_init();
static void wait_for_heater(long codenum, uint8_t extruder);
static void gcode_G28(bool home_x_axis, bool home_y_axis, bool home_z_axis);
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
	// hmm, same size as the above version, the compiler did a good job optimizing the above
	while( uint8_t ch = pgm_read_byte(str) ){
	  MYSERIAL.write((char)ch);
	  ++str;
	}
}

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
}

void softReset() {
    cli();
    wdt_enable(WDTO_15MS);
    while(1);
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

	if ((optiboot_status != 0) || (selectedSerialPort != 0))
		SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;
	printf_P(PSTR(" " FW_VERSION_FULL "\n"));


	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if (mcu & 1) puts_P(MSG_POWERUP);
	if (mcu & 2) puts_P(MSG_EXTERNAL_RESET);
	if (mcu & 4) puts_P(MSG_BROWNOUT_RESET);
	if (mcu & 8) puts_P(MSG_WATCHDOG_RESET);
	if (mcu & 32) puts_P(MSG_SOFTWARE_RESET);
	MCUSR = 0;

#ifdef STRING_VERSION_CONFIG_H
#ifdef STRING_CONFIG_H_AUTHOR
	SERIAL_ECHO_START;
	SERIAL_ECHORPGM(_n(" Last Updated: "));////MSG_CONFIGURATION_VER
	SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
	SERIAL_ECHORPGM(_n(" | Author: "));////MSG_AUTHOR
	SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
	SERIAL_ECHOPGM("Compiled: ");
	SERIAL_ECHOLNPGM(__DATE__);
#endif
#endif

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

	if (w25x20cl_success) {
    lcd_splash(); // we need to do this again, because tp_init() kills lcd
	} else {
    w25x20cl_err_msg();
    printf_P(_n("W25X20CL not responding.\n"));
	}


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
	setup_homepin();
  enable_z();
  eeprom_init();


	if (eeprom_read_byte((uint8_t*)EEPROM_UVLO) == 255) {
		eeprom_write_byte((uint8_t*)EEPROM_UVLO, 0);
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
  min_pos[axis][active_extruder] =          base_min_pos(axis) + cs.add_homing[axis];
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



void refresh_cmd_timeout(void) {
  previous_millis_cmd = _millis();
}


void trace() {
  Sound_MakeCustom(25,440,true);
}


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





// G92 - Set current position to coordinates given
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
    // plan_reset_next_e();
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

#ifdef EXTENDED_CAPABILITIES_REPORT

static void cap_line(const char* name, bool ena = false) {
  printf_P(PSTR("Cap:%S:%c\n"), name, (char)ena + '0');
}

static void extended_capabilities_report() {
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


void process_commands() {

	if (!buflen) {return;} //empty command

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
  // PRUSA GCODES
  KEEPALIVE_STATE(IN_HANDLER);

  /*!
  
  ---------------------------------------------------------------------------------
  ### M117 - Display Message <a href="https://reprap.org/wiki/G-code#M117:_Display_Message">M117: Display Message</a>
  This causes the given message to be shown in the status line on an attached LCD.
  It is processed early as to allow printing messages that contain G, M, N or T.
  
  ---------------------------------------------------------------------------------

 */
  if (code_seen("M117")) { //moved to highest priority place to be able to to print strings which includes "G", "PRUSA" and "^"
	  starpos = (strchr(strchr_pointer + 5, '*'));
	  if (starpos != NULL) {
		  *(starpos) = '\0';
    }
	  lcd_setstatus(strchr_pointer + 5);
  
  } else if(code_seen('G')) {
	  gcode_in_progress = (int)code_value();
    switch (gcode_in_progress) {
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
        if(code_seen('P')) {codenum = code_value();} // milliseconds to wait
        if(code_seen('S')) {codenum = code_value() * 1000; }// seconds to wait
        if(codenum != 0) { LCD_MESSAGERPGM(_n("Sleep...")); }////MSG_DWELL
        st_synchronize();
        codenum += _millis();  // keep track of when we started waiting
        previous_millis_cmd = _millis();
        while(_millis() < codenum) {
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
    gcode_in_progress = 0;
  /*!
  ### End of G-Codes
  */

  /*!
  ---------------------------------------------------------------------------------
  # M Commands
  */

  } else if(code_seen('M')) {

	  int index;
	  for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);
	   
	 /*for (++strchr_pointer; *strchr_pointer == ' ' || *strchr_pointer == '\t'; ++strchr_pointer);*/
	  if (*(strchr_pointer+index) < '0' || *(strchr_pointer+index) > '9') {
		  printf_P(PSTR("Invalid M code: %s \n"), cmdbuffer + bufindr + CMDHDRSIZE);

	  } else {
      mcode_in_progress = (int)code_value();

      switch(mcode_in_progress) {

        /*!
        ### M0, M1 - Stop the printer <a href="https://reprap.org/wiki/G-code#M0:_Stop_or_Unconditional_stop">M0: Stop or Unconditional stop</a>
        */
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: {// M1 - Conditional stop - Wait for user button press on LCD
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
          while (*src == ' ') {++src;}
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
              manage_inactivity(true);
              lcd_update(0);
            }
            KEEPALIVE_STATE(IN_HANDLER);
            lcd_ignore_click(false);
          }else{
            marlin_wait_for_click();
          }
          LCD_MESSAGERPGM(_T(WELCOME_MSG));
        }
        break;

        /*!
        ### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
        */
        case 17:
          LCD_MESSAGERPGM(_i("No move."));////MSG_NO_MOVE
          enable_z();
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
        if (code_seen('S')) {
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
        } else {
          bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
          if(all_axis) {
            st_synchronize();
            disable_e0();
            disable_e1();
            disable_e2();
            finishAndDisableSteppers();
          } else {
            st_synchronize();
            if (code_seen('Z')) disable_z();
          }
        }
        //in the end of print set estimated time to end of print and extruders used during print to default values for next print
        print_time_remaining_init();
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
          // safetytimer_inactive_time = code_value() * 1000;
        // safetyTimer.start();
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
            } else {
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
        }else {
          SERIAL_ECHO_START;
          SERIAL_ECHOPAIR("M113 S", (unsigned long)host_keepalive_interval);
          SERIAL_PROTOCOLLN("");
        }
      break;

      /*!
      ### M115 - Firmware info <a href="https://reprap.org/wiki/G-code#M115:_Get_Firmware_Version_and_Capabilities">M115: Get Firmware Version and Capabilities</a>
      Print the firmware info and capabilities
      Without any arguments, prints Prusa firmware version number, machine type, extruder count and UUID.
      `M115 U` Checks the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware, it will pause the print for 30s and ask the user to upgrade the firmware.
    
    _Examples:_
    
    `M115` results:
    
    `FIRMWARE_NAME:Prusa-Firmware 3.8.1 based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa i3 MK3S EXTRUDER_COUNT:1 UUID:00000000-0000-0000-0000-000000000000`
    
    `M115 V` results:
    
    `3.8.1`
    
    `M115 U3.8.2-RC1` results on LCD display for 30s or user interaction:
    
    `New firmware version available: 3.8.2-RC1 Please upgrade.`
      #### Usage
      
          M115 [ V | U ]
    
      #### Parameters
    - V - Report current installed firmware version
    - U - Firmware version provided by G-code to be compared to current one.  
    */
    case 115: // M115
        if (code_seen('V')) {
            // Report the Prusa version number.
            SERIAL_PROTOCOLLNRPGM(FW_VERSION_STR_P());
        } else if (code_seen('U')) {
            // Check the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware,
            // pause the print for 30s and ask the user to upgrade the firmware.
            show_upgrade_dialog_if_version_newer(++ strchr_pointer);
        } else {
            SERIAL_ECHOPGM("FIRMWARE_NAME:Pep Corp. Pinda Tester");
            SERIAL_ECHORPGM(FW_VERSION_STR_P());
            SERIAL_ECHOPGM(" based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:");
            SERIAL_ECHOPGM(PROTOCOL_VERSION);
            SERIAL_ECHOPGM(" MACHINE_TYPE:");
            SERIAL_ECHOPGM(CUSTOM_MENDEL_NAME); 
            SERIAL_ECHOPGM(" EXTRUDER_COUNT:"); 
            SERIAL_ECHOPGM(STRINGIFY(EXTRUDERS)); 
            SERIAL_ECHOPGM(" UUID:"); 
            SERIAL_ECHOLNPGM(MACHINE_UUID);
        }
      break;

      /*!
      ### M114 - Get current position <a href="https://reprap.org/wiki/G-code#M114:_Get_Current_Position">M114: Get Current Position</a>
      */
      case 114:
        gcode_M114();
      break;


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
      

      /*!
      ### M201 - Set Print Max Acceleration <a href="https://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration">M201: Set max printing acceleration</a>
      For each axis individually.
      */
      case 201:
        for (int8_t i = 0; i < NUM_AXIS; i++) {
          if (code_seen(axis_codes[i]))
          {
            unsigned long val = code_value();

            max_acceleration_units_per_sq_second[i] = val;
          }
        }
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
        reset_acceleration_rates();
      break;

      /*!
      ### M203 - Set Max Feedrate <a href="https://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate">M203: Set maximum feedrate</a>
      For each axis individually.
      */
      case 203: // M203 max feedrate mm/sec
        for (int8_t i = 0; i < NUM_AXIS; i++) {
          if (code_seen(axis_codes[i])) {
            float val = code_value();
            max_feedrate[i] = val;
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
                manage_inactivity();
                lcd_update(0);
              }
            }
          }
        }
      }
      break;


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
        if(code_seen('X')) {
          digitalPotWrite(4, code_value());
        }
        if(code_seen('Y')) {
          digitalPotWrite(5, code_value());
        }
        if(code_seen('Z')) {
          digitalPotWrite(3, code_value());
        }
        if(code_seen('E')) {
          digitalPotWrite(0, code_value());
        }
        if(code_seen('B')) {
          digitalPotWrite(1, code_value());
        }
        if(code_seen('S')) {
          for(int x = 0; x < NUM_AXIS+1; x++){
            digitalPotWrite(x, code_value());
          }
        }
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
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
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
      mcode_in_progress = 0;
    }
  // end if(code_seen('M')) (end of M codes)
  }  else if (code_seen('D')) {// D codes (debug)
    /**
    *---------------------------------------------------------------------------------
    *# D codes
    */
    switch((int)code_value()) {
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
      ### D1 - Clear EEPROM and RESET <a href="https://reprap.org/wiki/G-code#D1:_Clear_EEPROM_and_RESET">D1: Clear EEPROM and RESET</a>
            D1
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

  #endif //DEBUG_DCODES
      }
    } else {
    SERIAL_ECHO_START;
    SERIAL_ECHORPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(CMDBUFFER_CURRENT_STRING);
    SERIAL_ECHOLNPGM("\"(2)");
  }
  KEEPALIVE_STATE(NOT_BUSY);
  ClearToSend();
}


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
        if (target[Z_AXIS] < min_pos[Z_AXIS][active_extruder]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS][active_extruder]+negative_z_offset;
    }
    if (max_software_endstops) {
        if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
    }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = _millis();

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line_destinationXYZE(feedrate/60);
  } else {
     plan_buffer_line_destinationXYZE(feedrate*feedmultiply*(1./(60.f*100.f)));
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





#ifdef SAFETYTIMER
/**
 * @brief Turn off heating after safetytimer_inactive_time milliseconds of inactivity
 *
 * Full screen blocking notification message is shown after heater turning off.
 * Paused print is not considered inactivity, as nozzle is cooled anyway and bed cooling would
 * damage print.
 *
 * If safetytimer_inactive_time is zero, feature is disabled (heating is never turned off because of inactivity)
 */
//Modified 10 Nov 2020 - Parker Drouillard
static void handleSafetyTimer(){

}
#endif //SAFETYTIMER


void manage_inactivity(bool ignore_stepper_queue) { //default argument set in Marlin.h

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

  poweroff_z();

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
void Stop() {
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

}

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

//! @brief Get and validate extruder number
//!
//! If it is not specified, active_extruder is returned in parameter extruder.
//! @param [in] code M code number
//! @param [out] extruder
//! @return error
//! @retval true Invalid extruder specified in T code
//! @retval false Valid extruder specified in T code, or not specifiead

bool setTargetedHotend(int code, uint8_t &extruder)
{
  extruder = active_extruder;
  if(code_seen('T')) {
      extruder = code_value();
    if(extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHORPGM(_n("M104 Invalid extruder "));////MSG_M104_INVALID_EXTRUDER
          break;
        case 105:
          SERIAL_ECHO(_n("M105 Invalid extruder "));////MSG_M105_INVALID_EXTRUDER
          break;
        case 109:
          SERIAL_ECHO(_n("M109 Invalid extruder "));////MSG_M109_INVALID_EXTRUDER
          break;
        case 218:
          SERIAL_ECHO(_n("M218 Invalid extruder "));////MSG_M218_INVALID_EXTRUDER
          break;
        case 221:
          SERIAL_ECHO(_n("M221 Invalid extruder "));////MSG_M221_INVALID_EXTRUDER
          break;
      }
      SERIAL_PROTOCOLLN((int)extruder);
      return true;
    }
  }
  return false;
}

void save_statistics(unsigned long _total_filament_used, unsigned long _total_print_time) //_total_filament_used unit: mm/100; print time in s
{
	if (eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 1) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 2) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 3) == 255)
	{
		eeprom_update_dword((uint32_t *)EEPROM_TOTALTIME, 0);
		eeprom_update_dword((uint32_t *)EEPROM_FILAMENTUSED, 0);
	}

	unsigned long _previous_filament = eeprom_read_dword((uint32_t *)EEPROM_FILAMENTUSED); //_previous_filament unit: cm
	unsigned long _previous_time = eeprom_read_dword((uint32_t *)EEPROM_TOTALTIME); //_previous_time unit: min

	eeprom_update_dword((uint32_t *)EEPROM_TOTALTIME, _previous_time + (_total_print_time/60)); //EEPROM_TOTALTIME unit: min
	eeprom_update_dword((uint32_t *)EEPROM_FILAMENTUSED, _previous_filament + (_total_filament_used / 1000));

	total_filament_used = 0;

}

float calculate_extruder_multiplier(float diameter) {
  float out = 1.f;
  if (cs.volumetric_enabled && diameter > 0.f) {
    float area = M_PI * diameter * diameter * 0.25;
    out = 1.f / area;
  }
  if (extrudemultiply != 100)
    out *= float(extrudemultiply) * 0.01f;
  return out;
}

void calculate_extruder_multipliers() {
	extruder_multiplier[0] = calculate_extruder_multiplier(cs.filament_size[0]);
#if EXTRUDERS > 1
	extruder_multiplier[1] = calculate_extruder_multiplier(cs.filament_size[1]);
#if EXTRUDERS > 2
	extruder_multiplier[2] = calculate_extruder_multiplier(cs.filament_size[2]);
#endif
#endif
}

void delay_keep_alive(unsigned int ms)
{
    for (;;) {
        // Manage inactivity, but don't disable steppers on timeout.
        manage_inactivity(true);
        lcd_update(0);
        if (ms == 0)
            break;
        else if (ms >= 50) {
            _delay(50);
            ms -= 50;
        } else {
			_delay(ms);
            ms = 0;
        }
    }
}


void check_babystep()
{
	int babystep_z = eeprom_read_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->
            s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)));

	if ((babystep_z < Z_BABYSTEP_MIN) || (babystep_z > Z_BABYSTEP_MAX)) {
		babystep_z = 0; //if babystep value is out of min max range, set it to 0
		SERIAL_ECHOLNPGM("Z live adjust out of range. Setting to 0");
		eeprom_write_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->
            s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),
                    babystep_z);
		lcd_show_fullscreen_message_and_wait_P(PSTR("Z live adjust out of range. Setting to 0. Click to continue."));
		lcd_update_enable(true);		
	}	
}


float temp_comp_interpolation(float inp_temperature) {

	//cubic spline interpolation

	int n, i, j;
	float h[10], a, b, c, d, sum, s[10] = { 0 }, x[10], F[10], f[10], m[10][10] = { 0 }, temp;
	int shift[10];
	int temp_C[10];

	n = 6; //number of measured points

	shift[0] = 0;
	for (i = 0; i < n; i++) {
		if (i>0) EEPROM_read_B(EEPROM_PROBE_TEMP_SHIFT + (i-1) * 2, &shift[i]); //read shift in steps from EEPROM
		temp_C[i] = 50 + i * 10; //temperature in C
#ifdef PINDA_THERMISTOR
		constexpr int start_compensating_temp = 35;
		temp_C[i] = start_compensating_temp + i * 5; //temperature in degrees C
#ifdef DETECT_SUPERPINDA
		static_assert(start_compensating_temp >= PINDA_MINTEMP, "Temperature compensation start point is lower than PINDA_MINTEMP.");
#endif //DETECT_SUPERPINDA
#else
		temp_C[i] = 50 + i * 10; //temperature in C
#endif
		x[i] = (float)temp_C[i];
		f[i] = (float)shift[i];
	}
	if (inp_temperature < x[0]) return 0;


	for (i = n - 1; i>0; i--) {
		F[i] = (f[i] - f[i - 1]) / (x[i] - x[i - 1]);
		h[i - 1] = x[i] - x[i - 1];
	}
	//*********** formation of h, s , f matrix **************
	for (i = 1; i<n - 1; i++) {
		m[i][i] = 2 * (h[i - 1] + h[i]);
		if (i != 1) {
			m[i][i - 1] = h[i - 1];
			m[i - 1][i] = h[i - 1];
		}
		m[i][n - 1] = 6 * (F[i + 1] - F[i]);
	}
	//*********** forward elimination **************
	for (i = 1; i<n - 2; i++) {
		temp = (m[i + 1][i] / m[i][i]);
		for (j = 1; j <= n - 1; j++)
			m[i + 1][j] -= temp*m[i][j];
	}
	//*********** backward substitution *********
	for (i = n - 2; i>0; i--) {
		sum = 0;
		for (j = i; j <= n - 2; j++)
			sum += m[i][j] * s[j];
		s[i] = (m[i][n - 1] - sum) / m[i][i];
	}

		for (i = 0; i<n - 1; i++)
			if ((x[i] <= inp_temperature && inp_temperature <= x[i + 1]) || (i == n-2 && inp_temperature > x[i + 1])) {
				a = (s[i + 1] - s[i]) / (6 * h[i]);
				b = s[i] / 2;
				c = (f[i + 1] - f[i]) / h[i] - (2 * h[i] * s[i] + s[i + 1] * h[i]) / 6;
				d = f[i];
				sum = a*pow((inp_temperature - x[i]), 3) + b*pow((inp_temperature - x[i]), 2) + c*(inp_temperature - x[i]) + d;
			}

		return sum;

}

#ifdef PINDA_THERMISTOR
float temp_compensation_pinda_thermistor_offset(float temperature_pinda)
{
	if (!eeprom_read_byte((unsigned char *)EEPROM_TEMP_CAL_ACTIVE)) return 0;
	if (!calibration_status_pinda()) return 0;
	return temp_comp_interpolation(temperature_pinda) / cs.axis_steps_per_unit[Z_AXIS];
}
#endif //PINDA_THERMISTOR

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



//! @brief Immediately stop print moves
//!
//! Immediately stop print moves, save current extruder temperature and position to RAM.
//! If printing from sd card, position in file is saved.
//! If printing from USB, line number is saved.
//!
//! @param z_move
//! @param e_move
void stop_and_save_print_to_ram(float z_move, float e_move)
{
	if (saved_printing) return;

	unsigned char nlines;
	uint16_t sdlen_planner;
	uint16_t sdlen_cmdqueue;
	

	cli();
 if (is_usb_printing) { //reuse saved_sdpos for storing line number
		 saved_sdpos = gcode_LastN; //start with line number of command added recently to cmd queue
		 //reuse planner_calc_sd_length function for getting number of lines of commands in planner:
		 nlines = planner_calc_sd_length(); //number of lines of commands in planner 
		 saved_sdpos -= nlines;
		 saved_sdpos -= buflen; //number of blocks in cmd buffer
		 saved_printing_type = PRINTING_TYPE_USB;
	}
	else {
		 saved_printing_type = PRINTING_TYPE_NONE;
		 //not sd printing nor usb printing
	}

  // save the global state at planning time
  if (current_block)
  {
      memcpy(saved_target, current_block->gcode_target, sizeof(saved_target));
      saved_feedrate2 = current_block->gcode_feedrate;
  }
  else
  {
      saved_target[0] = SAVED_TARGET_UNSET;
      saved_feedrate2 = feedrate;
  }

	planner_abort_hard(); //abort printing
	memcpy(saved_pos, current_position, sizeof(saved_pos));
    saved_feedmultiply2 = feedmultiply; //save feedmultiply
	saved_extruder_relative_mode = axis_relative_modes & E_AXIS_MASK;
	saved_fanSpeed = fanSpeed;
	cmdqueue_reset(); //empty cmdqueue
	saved_printing = true;
  // We may have missed a stepper timer interrupt. Be safe than sorry, reset the stepper timer before re-enabling interrupts.
  st_reset_timer();
	sei();
	if ((z_move != 0) || (e_move != 0)) { // extruder or z move

    // Rather than calling plan_buffer_line directly, push the move into the command queue so that
    // the caller can continue processing. This is used during powerpanic to save the state as we
    // move away from the print.
    char buf[48];

    if(e_move)
    {
        // First unretract (relative extrusion)
        if(!saved_extruder_relative_mode){
            enquecommand(PSTR("M83"), true);
        }
        //retract 45mm/s
        // A single sprintf may not be faster, but is definitely 20B shorter
        // than a sequence of commands building the string piece by piece
        // A snprintf would have been a safer call, but since it is not used
        // in the whole program, its implementation would bring more bytes to the total size
        // The behavior of dtostrf 8,3 should be roughly the same as %-0.3
        sprintf_P(buf, PSTR("G1 E%-0.3f F2700"), e_move);
        enquecommand(buf, false);
    }

    if(z_move)
    {
        // Then lift Z axis
        sprintf_P(buf, PSTR("G1 Z%-0.3f F%-0.3f"), saved_pos[Z_AXIS] + z_move, homing_feedrate[Z_AXIS]);
        enquecommand(buf, false);
    }

    // If this call is invoked from the main Arduino loop() function, let the caller know that the command
    // in the command queue is not the original command, but a new one, so it should not be removed from the queue.
    repeatcommand_front();
    waiting_inside_plan_buffer_line_print_aborted = true; //unroll the stack
  }
}

//! @brief Restore print from ram
//!
//! Restore print saved by stop_and_save_print_to_ram(). Is blocking, restores
//! print fan speed, waits for extruder temperature restore, then restores
//! position and continues print moves.
//!
//! Internally lcd_update() is called by wait_for_heater().
//!
//! @param e_move
void restore_print_from_ram_and_continue(float e_move)
{
	if (!saved_printing) return;
	
#ifdef FANCHECK
	// Do not allow resume printing if fans are still not ok
	if ((fan_check_error != EFCE_OK) && (fan_check_error != EFCE_FIXED)) return;
    if (fan_check_error == EFCE_FIXED) fan_check_error = EFCE_OK; //reenable serial stream processing if printing from usb
#endif
	
//	for (int axis = X_AXIS; axis <= E_AXIS; axis++)
//	    current_position[axis] = st_get_position_mm(axis);
	active_extruder = saved_active_extruder; //restore active_extruder
	fanSpeed = saved_fanSpeed;
	axis_relative_modes ^= (-saved_extruder_relative_mode ^ axis_relative_modes) & E_AXIS_MASK;
	float e = saved_pos[E_AXIS] - e_move;
	plan_set_e_position(e);
  
  #ifdef FANCHECK
    fans_check_enabled = false;
  #endif

	//first move print head in XY to the saved position:
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], current_position[Z_AXIS], saved_pos[E_AXIS] - e_move, homing_feedrate[Z_AXIS]/13, active_extruder);
	//then move Z
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], saved_pos[Z_AXIS], saved_pos[E_AXIS] - e_move, homing_feedrate[Z_AXIS]/13, active_extruder);
	//and finaly unretract (35mm/s)
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], saved_pos[Z_AXIS], saved_pos[E_AXIS], FILAMENTCHANGE_RFEED, active_extruder);
	st_synchronize();

  #ifdef FANCHECK
    fans_check_enabled = true;
  #endif

    // restore original feedrate/feedmultiply _after_ restoring the extruder position
	feedrate = saved_feedrate2;
	feedmultiply = saved_feedmultiply2;

	memcpy(current_position, saved_pos, sizeof(saved_pos));
	memcpy(destination, current_position, sizeof(destination));
  if (saved_printing_type == PRINTING_TYPE_USB) { //was usb printing
		gcode_LastN = saved_sdpos; //saved_sdpos was reused for storing line number when usb printing
		serial_count = 0; 
		FlushSerialRequestResend();
	}

	SERIAL_PROTOCOLLNRPGM(MSG_OK); //dummy response because of octoprint is waiting for this
	lcd_setstatuspgm(_T(WELCOME_MSG));
    saved_printing_type = PRINTING_TYPE_NONE;
	saved_printing = false;
    waiting_inside_plan_buffer_line_print_aborted = true; //unroll the stack
}

// Cancel the state related to a currently saved print
void cancel_saved_printing()
{
    eeprom_update_byte((uint8_t*)EEPROM_UVLO, 0);
    saved_target[0] = SAVED_TARGET_UNSET;
    saved_printing_type = PRINTING_TYPE_NONE;
    saved_printing = false;
}

void print_world_coordinates()
{
	printf_P(_N("world coordinates: (%.3f, %.3f, %.3f)\n"), current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
}

void print_physical_coordinates()
{
	printf_P(_N("physical coordinates: (%.3f, %.3f, %.3f)\n"), st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));
}


uint16_t print_time_remaining() {
	uint16_t print_t = PRINT_TIME_REMAINING_INIT;
	print_t = print_time_remaining_normal;
	if ((print_t != PRINT_TIME_REMAINING_INIT) && (feedmultiply != 0)) print_t = 100ul * print_t / feedmultiply;
	return print_t;
}

uint8_t calc_percent_done()
{
	//in case that we have information from M73 gcode return percentage counted by slicer, else return percentage counted as byte_printed/filesize
	uint8_t percent_done = 0;
	if (print_percent_done_normal <= 100) {
		percent_done = print_percent_done_normal;
	}

	return percent_done;
}

static void runPindaTest(){
  
}

static void print_time_remaining_init()
{
	print_time_remaining_normal = PRINT_TIME_REMAINING_INIT;
	print_time_remaining_silent = PRINT_TIME_REMAINING_INIT;
	print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
	print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
}

void load_filament_final_feed()
{
	current_position[E_AXIS]+= FILAMENTCHANGE_FINALFEED;
	plan_buffer_line_curposXYZE(FILAMENTCHANGE_EFEED_FINAL);
}

//! @brief Wait for user to check the state
//! @par nozzle_temp nozzle temperature to load filament
void M600_check_state(float nozzle_temp)
{
    lcd_change_fil_state = 0;
    while (lcd_change_fil_state != 1)
    {
        lcd_change_fil_state = 0;
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        lcd_alright();
        KEEPALIVE_STATE(IN_HANDLER);
        switch(lcd_change_fil_state)
        {
        // Filament failed to load so load it again
        case 2:
          M600_load_filament_movements();
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
