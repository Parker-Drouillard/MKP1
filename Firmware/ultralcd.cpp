//! @file
//! @date Aug 28, 2019
//! @author mkbel
//! @brief LCD

#include "temperature.h"
#include "ultralcd.h"
#include "Marlin.h"
#include "language.h"
#include "temperature.h"
#include "stepper.h"
#include "ConfigurationStore.h"
#include "printers.h"
#include <string.h>


#include "lcd.h"
#include "menu.h"

#include "backlight.h"

#include "util.h"


//#include "Configuration.h"
#include "cmdqueue.h"

#include "sound.h"

// #include "mmu.h"

#include "static_assert.h"


#include "adc.h"
#include "config.h"


static void lcd_backlight_menu();

int8_t ReInitLCD = 0;
uint8_t scrollstuff = 0;

#ifdef SDCARD_SORT_ALPHA
bool presort_flag = false;
#endif

LcdCommands lcd_commands_type = LcdCommands::Idle;
static uint8_t lcd_commands_step = 0;

CustomMsg custom_message_type = CustomMsg::Status;
unsigned int custom_message_state = 0;


bool isPrintPaused = false;
uint8_t farm_mode = 0;
int farm_no = 0;
int farm_timer = 8;
uint8_t farm_status = 0;
bool printer_connected = true;

unsigned long display_time; //just timer for showing pid finished message on lcd;
float pid_temp = DEFAULT_PID_TEMP;

static bool forceMenuExpire = false;


static float manual_feedrate[] = MANUAL_FEEDRATE;

/* !Configuration settings */

uint8_t lcd_status_message_level;
char lcd_status_message[LCD_WIDTH + 1] = ""; //////WELCOME!
unsigned char firstrun = 1;

static const char separator[] PROGMEM = "--------------------";

/** forward declarations **/

static const char* lcd_display_message_fullscreen_nonBlocking_P(const char *msg, uint8_t &nlines);
// void copy_and_scalePID_i();
// void copy_and_scalePID_d();

/* Different menus */
//static void lcd_status_screen();                // NOT static due to using inside "Marlin_main" module ("manage_inactivity()")
static void lcd_main_menu();
void runPindaTest();
void pindaTest();
void mainTest();
//static void lcd_move_menu();
static void lcd_settings_menu();
static void lcd_calibration_menu();
static void lcd_settings_linearity_correction_menu_save();
static void prusa_stat_printerstatus(int _status);
static void prusa_stat_diameter();
static void prusa_stat_temperatures();
static void prusa_stat_printinfo();
static void lcd_farm_no();
static void lcd_menu_xyz_y_min();
static void lcd_menu_xyz_skew();
static void lcd_menu_xyz_offset();
static void lcd_v2_calibration();
//static void lcd_menu_show_sensors_state();      // NOT static due to using inside "Marlin_main" module ("manage_inactivity()")

static void preheat_or_continue();

#ifdef MMU_HAS_CUTTER
static void mmu_cut_filament_menu();
#endif //MMU_HAS_CUTTER

#if defined(FILAMENT_SENSOR)
static void lcd_menu_fails_stats();
#endif // FILAMENT_SENSOR



static void lcd_selftest_v();

static bool lcd_selfcheck_axis(int _axis, int _travel);
static bool lcd_selfcheck_pulleys(int axis);
static bool lcd_selfcheck_endstops();

static bool lcd_selfcheck_check_heater(bool _isbed);
enum class TestScreen : uint_least8_t
{
    ExtruderFan,
    PrintFan,
    FansOk,
    EndStops,
    AxisX,
    AxisY,
    AxisZ,
    Bed,
    Hotend,
    HotendOk,
    AllCorrect,
    Failed,
    Home
};

enum class TestError : uint_least8_t
{
    Heater,
    Bed,
    Endstops,
    Motor,
    Endstop,
    PrintFan,
    ExtruderFan,
    Pulley,
    Axis,
    SwappedFan
};

static int  lcd_selftest_screen(TestScreen screen, int _progress, int _progress_scale, bool _clear, int _delay);
static void lcd_selftest_screen_step(int _row, int _col, int _state, const char *_name, const char *_indicator);
static bool lcd_selftest_manual_fan_check(int _fan, bool check_opposite,
	bool _default=false);



static bool selftest_irsensor();
static void lcd_selftest_error(TestError error, const char *_error_1, const char *_error_2);
static void lcd_colorprint_change();
static void lcd_disable_farm_mode();
static void lcd_set_fan_check();
static void lcd_cutter_enabled();
#ifdef SDCARD_SORT_ALPHA
 static void lcd_sort_type_set();
#endif
static void lcd_send_status();

//! Beware: has side effects - forces lcd_draw_update to 2, which means clear the display
void lcd_finishstatus();


#define ENCODER_FEEDRATE_DEADZONE 10

#define STATE_NA 255
#define STATE_OFF 0
#define STATE_ON 1

#if (SDCARDDETECT > 0)
bool lcd_oldcardstatus;
#endif

uint8_t selected_sheet = 0;

bool ignore_click = false;
bool wait_for_unclick;


const char STR_SEPARATOR[] PROGMEM = "------------";




// Print Z-coordinate (8 chars total)
void lcdui_print_Z_coord(void) {
    if (custom_message_type == CustomMsg::MeshBedLeveling) {
        lcd_puts_P(_N("Z   --- "));
    } else {
		lcd_printf_P(_N("Z%6.2f%c"), current_position[Z_AXIS], axis_known_position[Z_AXIS]?' ':'?');
	}
}




//Print status line on status screen
void lcdui_print_status_line(void) {
	if (heating_status) { // If heating flag, show progress of heating
		heating_status_counter++;
		if (heating_status_counter > 13) {
			heating_status_counter = 0;
		}
		lcd_set_cursor(7, 3);
		lcd_puts_P(PSTR("             "));

		for (unsigned int dots = 0; dots < heating_status_counter; dots++) {
			lcd_set_cursor(7 + dots, 3);
			lcd_print('.');
		}
		
	} else { // Otherwise check for other special events
   		switch (custom_message_type) {
		case CustomMsg::Status: // Nothing special, print status message normally
			lcd_print(lcd_status_message);
			break;


		}
	}
    
    // Fill the rest of line to have nice and clean output
	for(int fillspace = 0; fillspace < 20; fillspace++)
		if ((lcd_status_message[fillspace] <= 31 ))
			lcd_print(' ');
}

//! @brief Show Status Screen
//!
//! @code{.unparsed}
//! |01234567890123456789|
//! |                    |
//! |      PEP CORP      |
//! | PINDA TEST MACHINE |
//! |                    |
//! ----------------------
//! N - nozzle temp symbol	LCD_STR_THERMOMETER
//! D - Degree sysmbol		LCD_STR_DEGREE
//! B - bed temp symbol 	LCD_STR_BEDTEMP
//! F - feedrate symbol 	LCD_STR_FEEDRATE
//! t - clock symbol 		LCD_STR_THERMOMETER
//! @endcode
void lcdui_print_status_screen(void) {

    lcd_set_cursor(0, 0); //line 3
	lcd_space(LCD_WIDTH);
    lcd_set_cursor(0, 1); //line 0
    //Print the hotend temperature (9 chars total)
	lcd_print("      PEP CORP      ");
	lcd_set_cursor(0, 2);
	lcd_print(" Pinda Test Machine ");


	lcd_set_cursor(0, 3);
	lcd_space(LCD_WIDTH);
// #ifndef DEBUG_DISABLE_LCD_STATUS_LINE
// 	lcdui_print_status_line();
// #endif //DEBUG_DISABLE_LCD_STATUS_LINE

}

// Main status screen. It's up to the implementation specific part to show what is needed. As this is very display dependent
void lcd_status_screen() {                         // NOT static due to using inside "Marlin_main" module ("manage_inactivity()")
	if (firstrun == 1)  {
		firstrun = 0;
		if(lcd_status_message_level == 0) {
			strncpy_P(lcd_status_message, _T(WELCOME_MSG), LCD_WIDTH);
			lcd_finishstatus();
		}
		if (eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 1) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 2) == 255 && eeprom_read_byte((uint8_t *)EEPROM_TOTALTIME + 3) == 255) {
			eeprom_update_dword((uint32_t *)EEPROM_TOTALTIME, 0);
			eeprom_update_dword((uint32_t *)EEPROM_FILAMENTUSED, 0);
		}
	}

#ifdef ULTIPANEL_FEEDMULTIPLY
	// Dead zone at 100% feedrate
	if ((feedmultiply < 100 && (feedmultiply + int(lcd_encoder)) > 100) ||
		(feedmultiply > 100 && (feedmultiply + int(lcd_encoder)) < 100)) {
		lcd_encoder = 0;
		feedmultiply = 100;
	}
	if (feedmultiply == 100 && int(lcd_encoder) > ENCODER_FEEDRATE_DEADZONE) {
		feedmultiply += int(lcd_encoder) - ENCODER_FEEDRATE_DEADZONE;
		lcd_encoder = 0;
	} else if (feedmultiply == 100 && int(lcd_encoder) < -ENCODER_FEEDRATE_DEADZONE) {
		feedmultiply += int(lcd_encoder) + ENCODER_FEEDRATE_DEADZONE;
		lcd_encoder = 0;
	} else if (feedmultiply != 100) {
		feedmultiply += int(lcd_encoder);
		lcd_encoder = 0;
	}
#endif //ULTIPANEL_FEEDMULTIPLY

	if (feedmultiply < 10) {
		feedmultiply = 10;
	} else if (feedmultiply > 999)
		feedmultiply = 999;

	if (lcd_status_update_delay) {
		lcd_status_update_delay--;
	} else {
		lcd_draw_update = 1;
	}

	if (lcd_draw_update) {
		ReInitLCD++;
		if (ReInitLCD == 30) {
			lcd_refresh(); // to maybe revive the LCD if static electricity killed it.
			ReInitLCD = 0 ;
		} else {
			if ((ReInitLCD % 10) == 0) {
				lcd_refresh_noclear();
			} //to maybe revive the LCD if static electricity killed it.
		}

		lcdui_print_status_screen();

		if (farm_mode) {
			farm_timer--;
			if (farm_timer < 1) {
				farm_timer = 10;
			}
			switch (farm_timer)
			{
			case 8:
				break;
			}
		} // end of farm_mode

		lcd_status_update_delay = 10;   /* redraw the main screen every second. This is easier then trying keep track of all things that change on the screen */
		if (lcd_commands_type != LcdCommands::Idle)
			lcd_commands();
	} // end of lcd_draw_update

	bool current_click = LCD_CLICKED;

	if (ignore_click) {
		if (wait_for_unclick)
		{
			if (!current_click)
				ignore_click = wait_for_unclick = false;
			else
				current_click = false;
		} else if (current_click) {
			lcd_quick_feedback();
			wait_for_unclick = true;
			current_click = false;
		}
	}

	if (current_click && ( menu_block_entering_on_serious_errors == SERIOUS_ERR_NONE )) { // or a serious error blocks entering the menu
		menu_depth = 0; //redundant, as already done in lcd_return_to_status(), just to be sure
		menu_submenu(mainTest);
		lcd_refresh(); // to maybe revive the LCD if static electricity killed it.
	}
}

void lcd_commands()
{
	if (lcd_commands_type == LcdCommands::LongPause)
	{
		if (!blocks_queued() && !homing_flag)
		{
			lcd_setstatuspgm(_i("Print paused"));////MSG_PRINT_PAUSED c=20 r=1
            lcd_commands_type = LcdCommands::Idle;
            lcd_commands_step = 0;
            long_pause();
		}
	}
}

void lcd_return_to_status()
{
	lcd_refresh(); // to maybe revive the LCD if static electricity killed it.
	menu_goto(lcd_status_screen, 0, false, true);
	menu_depth = 0;
}


float move_menu_scale;
static void lcd_move_menu_axis();



//! @brief append text label with a colon and format it into a fixed size output buffer
//! It would have been much easier if there was a ':' in the labels.
//! But since the texts like Bed, Nozzle and PINDA are used in other places
//! it is better to reuse these texts even though it requires some extra formatting code.
//! @param [in] ipgmLabel pointer to string in PROGMEM
//! @param [out] pointer to string in RAM which will receive the formatted text. Must be allocated to appropriate size
//! @param [in] dstSize allocated length of dst
static void pgmtext_with_colon(const char *ipgmLabel, char *dst, uint8_t dstSize){
    uint8_t i = 0;
    for(; i < dstSize - 2; ++i){ // 2 byte less than buffer, we'd be adding a ':' to the end
        uint8_t b = pgm_read_byte(ipgmLabel + i);
        if( ! b )
            break;
        dst[i] = b;
    }
    dst[i] = ':';               // append the colon
    ++i;
    for(; i < dstSize - 1; ++i) // fill the rest with spaces
        dst[i] = ' ';
    dst[dstSize-1] = '\0';      // terminate the string properly
}



#ifdef RESUME_DEBUG 
extern void stop_and_save_print_to_ram(float z_move, float e_move);
extern void restore_print_from_ram_and_continue(float e_move);

static void lcd_menu_test_save()
{
	stop_and_save_print_to_ram(10, -0.8);
}

static void lcd_menu_test_restore()
{
	restore_print_from_ram_and_continue(0.8);
}
#endif //RESUME_DEBUG 


//! @brief Show Support Menu
//!
//! @code{.unparsed}
//! |01234567890123456789|
//! | Main               |
//! | Firmware:          |	c=18 r=1
//! |  3.7.2.-2363       |	c=16 r=1
//! | prusa3d.com        |	MSG_PRUSA3D
//! | forum.prusa3d.com  |	MSG_PRUSA3D_FORUM
//! | howto.prusa3d.com  |	MSG_PRUSA3D_HOWTO
//! | --------------     |	STR_SEPARATOR
//! | 1_75mm_MK3         |	FILAMENT_SIZE
//! | howto.prusa3d.com  |	ELECTRONICS
//! | howto.prusa3d.com  |	NOZZLE_TYPE
//! | --------------     |	STR_SEPARATOR
//! | Date:              |	c=17 r=1
//! | MMM DD YYYY        |	__DATE__
//! | --------------     |	STR_SEPARATOR
//! @endcode
//! 
//! If MMU is connected
//! 
//! 	@code{.unparsed}
//! 	| MMU2 connected     |	c=18 r=1
//! 	|  FW: 1.0.6-7064523 |
//! 	@endcode
//! 
//! If MMU is not connected
//! 
//! 	@code{.unparsed}
//! 	| MMU2       N/A     |	c=18 r=1
//! 	@endcode
//! 
//! If Flash Air is connected
//! 
//! 	@code{.unparsed}
//! 	| --------------     |	STR_SEPARATOR
//! 	| FlashAir IP Addr:  |	c=18 r=1
//! 	|  192.168.1.100     |
//! 	@endcode
//! 
//! @code{.unparsed}
//! | --------------     |	STR_SEPARATOR
//! | XYZ cal. details   |	MSG_XYZ_DETAILS c=18
//! | Extruder info      |	MSG_INFO_EXTRUDER
//! | XYZ cal. details   |	MSG_INFO_SENSORS
//! @endcode
//! 
//! 
//! @code{.unparsed}
//! | Temperatures       |	MSG_MENU_TEMPERATURES
//! @endcode
//! 
//! If Voltage Bed and PWR Pin are defined
//! 
//! 	@code{.unparsed}
//! 	| Voltages           |	MSG_MENU_VOLTAGES
//! 	@endcode
//! 
//! 
//! If DEBUG_BUILD is defined
//! 
//! 	@code{.unparsed}
//! 	| Debug              |	c=18 r=1
//! 	@endcode
//! ----------------------
//! @endcode
static void lcd_support_menu() {
	typedef struct
	{	// 22bytes total
		int8_t status;                 // 1byte
		bool is_flash_air;             // 1byte
		uint8_t ip[4];                 // 4bytes
		char ip_str[3*4+3+1];          // 16bytes
	} _menu_data_t;
    static_assert(sizeof(menu_data)>= sizeof(_menu_data_t),"_menu_data_t doesn't fit into menu_data");
	_menu_data_t* _md = (_menu_data_t*)&(menu_data[0]);
    if (_md->status == 0 || lcd_draw_update == 2)
	{
        // Menu was entered or SD card status has changed (plugged in or removed).
        // Initialize its status.
        _md->status = 1;        
    } else if (_md->is_flash_air && 
        _md->ip[0] == 0 && _md->ip[1] == 0 && 
        _md->ip[2] == 0 && _md->ip[3] == 0 &&
        ++ _md->status == 16)
	{
        // Waiting for the FlashAir card to get an IP address from a router. Force an update.
        _md->status = 0;
    }

  MENU_BEGIN();

  MENU_ITEM_BACK_P(_T(MSG_MAIN));

  MENU_ITEM_BACK_P(PSTR("Firmware:"));
  MENU_ITEM_BACK_P(PSTR(" " FW_VERSION_FULL));
      
  MENU_ITEM_BACK_P(_i("Pepcorp.ca"));////MSG_PRUSA3D
  MENU_ITEM_BACK_P(PSTR(ELECTRONICS));
  MENU_ITEM_BACK_P(STR_SEPARATOR);
  MENU_ITEM_BACK_P(_i("Date:"));////MSG_DATE c=17 r=1
  MENU_ITEM_BACK_P(PSTR(__DATE__));

  MENU_ITEM_SUBMENU_P(_i("Sensor info"), lcd_menu_show_sensors_state);////MSG_INFO_SENSORS c=18 r=1
  MENU_END();
}




//! @brief Show filament used a print time
//!
//! If printing current print statistics are shown
//!
//! @code{.unparsed}
//! |01234567890123456789|
//! |Filament used:      | c=19
//! |           0000.00m |
//! |Print time:         | c=19 r=1
//! |        00h 00m 00s |
//! ----------------------
//! @endcode
//!
//! If not printing, total statistics are shown
//!
//! @code{.unparsed}
//! |01234567890123456789|
//! |Total filament:     | c=19 r=1
//! |           0000.00m |
//! |Total print time:   | c=19 r=1
//! |        00d 00h 00m |
//! ----------------------
//! @endcode
//! @todo Positioning of the messages and values on LCD aren't fixed to their exact place. This causes issues with translations. Translations missing for "d"days, "h"ours, "m"inutes", "s"seconds".
void lcd_menu_statistics() {
    lcd_timeoutToStatus.stop(); //infinite timeout

		unsigned long _filament = eeprom_read_dword((uint32_t *)EEPROM_FILAMENTUSED);
		unsigned long _time = eeprom_read_dword((uint32_t *)EEPROM_TOTALTIME); //in minutes
		uint8_t _hours, _minutes;
		uint32_t _days;
		float _filament_m = (float)_filament/100;
		_days = _time / 1440;
		_hours = (_time - (_days * 1440)) / 60;
		_minutes = _time - ((_days * 1440) + (_hours * 60));

		lcd_home();
		lcd_printf_P(_N(
			"%S:\n"
			"%18.2fm \n"
			"%S:\n"
			"%10ldd %02hhdh %02hhdm"
            ),
            _i("Total filament"), _filament_m,  ////c=19 r=1
            _i("Total print time"), _days, _hours, _minutes);  ////c=19 r=1
        menu_back_if_clicked_fb();
}


static void _lcd_move(const char *name, int axis, int min, int max)
{
	typedef struct
	{	// 2bytes total
		bool initialized;              // 1byte
		bool endstopsEnabledPrevious;  // 1byte
	} _menu_data_t;
	static_assert(sizeof(menu_data)>= sizeof(_menu_data_t),"_menu_data_t doesn't fit into menu_data");
	_menu_data_t* _md = (_menu_data_t*)&(menu_data[0]);
	if (!_md->initialized)
	{
		_md->endstopsEnabledPrevious = enable_endstops(false);
		_md->initialized = true;
	}
	if (lcd_encoder != 0)
	{
		refresh_cmd_timeout();
		if (! planner_queue_full())
		{
			current_position[axis] += float((int)lcd_encoder) * move_menu_scale;
			if (min_software_endstops && current_position[axis] < min) current_position[axis] = min;
			if (max_software_endstops && current_position[axis] > max) current_position[axis] = max;
			lcd_encoder = 0;
			plan_buffer_line_curposXYZE(manual_feedrate[axis] / 60);
			lcd_draw_update = 1;
		}
	}
	if (lcd_draw_update)
	{
	    lcd_set_cursor(0, 1);
		menu_draw_float31(name, current_position[axis]);
	}
	if (menu_leaving || LCD_CLICKED) (void)enable_endstops(_md->endstopsEnabledPrevious);
	if (LCD_CLICKED) menu_back();
}




// Save a single axis babystep value.
void EEPROM_save_B(int pos, int* value)
{
  eeprom_update_byte((unsigned char*)pos, (unsigned char)((*value) & 0xff));
  eeprom_update_byte((unsigned char*)pos + 1, (unsigned char)((*value) >> 8));
}

// Read a single axis babystep value.
void EEPROM_read_B(int pos, int* value)
{
  *value = (int)eeprom_read_byte((unsigned char*)pos) | (int)(eeprom_read_byte((unsigned char*)pos + 1) << 8);
}


// Note: the colon behind the text (X, Y, Z) is necessary to greatly shorten
// the implementation of menu_draw_float31
static void lcd_move_z() {
  _lcd_move(PSTR("Z:"), Z_AXIS, Z_MIN_POS, Z_MAX_POS);
}


typedef struct
{	// 12bytes + 9bytes = 21bytes total
    menu_data_edit_t reserved; //12 bytes reserved for number editing functions
	int8_t status;                   // 1byte
	int16_t left;                    // 2byte
	int16_t right;                   // 2byte
	int16_t front;                   // 2byte
	int16_t rear;                    // 2byte
} _menu_data_adjust_bed_t;
static_assert(sizeof(menu_data)>= sizeof(_menu_data_adjust_bed_t),"_menu_data_adjust_bed_t doesn't fit into menu_data");

void lcd_adjust_bed_reset(void)
{
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_VALID, 1);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_LEFT , 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_RIGHT, 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_FRONT, 0);
	eeprom_update_byte((unsigned char*)EEPROM_BED_CORRECTION_REAR , 0);
	_menu_data_adjust_bed_t* _md = (_menu_data_adjust_bed_t*)&(menu_data[0]);
	_md->status = 0;
}



static inline bool pgm_is_whitespace(const char *c_addr)
{
    const char c = pgm_read_byte(c_addr);
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static inline bool pgm_is_interpunction(const char *c_addr)
{
    const char c = pgm_read_byte(c_addr);
    return c == '.' || c == ',' || c == ':'|| c == ';' || c == '?' || c == '!' || c == '/';
}

/**
 * @brief show full screen message
 *
 * This function is non-blocking
 * @param msg message to be displayed from PROGMEM
 * @param nlines
 * @return rest of the text (to be displayed on next page)
 */
static const char* lcd_display_message_fullscreen_nonBlocking_P(const char *msg, uint8_t &nlines)
{
    lcd_set_cursor(0, 0);
    const char *msgend = msg;
    uint8_t row = 0;
    bool multi_screen = false;
    for (; row < 4; ++ row) {
        while (pgm_is_whitespace(msg))
            ++ msg;
        if (pgm_read_byte(msg) == 0)
            // End of the message.
            break;
        lcd_set_cursor(0, row);
        uint8_t linelen = min(strlen_P(msg), 20);
        const char *msgend2 = msg + linelen;
        msgend = msgend2;
        if (row == 3 && linelen == 20) {
            // Last line of the display, full line shall be displayed.
            // Find out, whether this message will be split into multiple screens.
            while (pgm_is_whitespace(msgend))
                ++ msgend;
            multi_screen = pgm_read_byte(msgend) != 0;
            if (multi_screen)
                msgend = (msgend2 -= 2);
        }
        if (pgm_read_byte(msgend) != 0 && ! pgm_is_whitespace(msgend) && ! pgm_is_interpunction(msgend)) {
            // Splitting a word. Find the start of the current word.
            while (msgend > msg && ! pgm_is_whitespace(msgend - 1))
                 -- msgend;
            if (msgend == msg)
                // Found a single long word, which cannot be split. Just cut it.
                msgend = msgend2;
        }
        for (; msg < msgend; ++ msg) {
            char c = char(pgm_read_byte(msg));
            if (c == '~')
                c = ' ';
            lcd_print(c);
        }
    }

    if (multi_screen) {
        // Display the "next screen" indicator character.
        // lcd_set_custom_characters_arrows();
        lcd_set_custom_characters_nextpage();
        lcd_set_cursor(19, 3);
        // Display the down arrow.
        lcd_print(char(1));
    }

    nlines = row;
    return multi_screen ? msgend : NULL;
}

const char* lcd_display_message_fullscreen_P(const char *msg, uint8_t &nlines)
{
    // Disable update of the screen by the usual lcd_update(0) routine.
    lcd_update_enable(false);
    lcd_clear();
//	uint8_t nlines;
    return lcd_display_message_fullscreen_nonBlocking_P(msg, nlines);
}
const char* lcd_display_message_fullscreen_P(const char *msg) 
{
  uint8_t nlines;
  return lcd_display_message_fullscreen_P(msg, nlines);
}


/**
 * @brief show full screen message and wait
 *
 * This function is blocking.
 * @param msg message to be displayed from PROGMEM
 */
void lcd_show_fullscreen_message_and_wait_P(const char *msg)
{
    LcdUpdateDisabler lcdUpdateDisabler;
    const char *msg_next = lcd_display_message_fullscreen_P(msg);
    bool multi_screen = msg_next != NULL;
	lcd_set_custom_characters_nextpage();
	lcd_consume_click();
	KEEPALIVE_STATE(PAUSED_FOR_USER);
	// Until confirmed by a button click.
	for (;;) {
		if (!multi_screen) {
			lcd_set_cursor(19, 3);
			// Display the confirm char.
			lcd_print(char(2));
		}
        // Wait for 5 seconds before displaying the next text.
        for (uint8_t i = 0; i < 100; ++ i) {
            if (lcd_clicked()) {
				if (msg_next == NULL) {
					KEEPALIVE_STATE(IN_HANDLER);
					lcd_set_custom_characters();
					lcd_update_enable(true);
					lcd_update(2);
					return;
				}
				else {
					break;
				}
            }
        }
        if (multi_screen) {
            if (msg_next == NULL)
                msg_next = msg;
            msg_next = lcd_display_message_fullscreen_P(msg_next);
			if (msg_next == NULL) {

				lcd_set_cursor(19, 3);
				// Display the confirm char.
				lcd_print(char(2));
			}
        }
    }
}

bool lcd_wait_for_click_delay(uint16_t nDelay)
// nDelay :: timeout [s] (0 ~ no timeout)
// true ~ clicked, false ~ delayed
{
bool bDelayed;
long nTime0 = _millis()/1000;
	lcd_consume_click();
	KEEPALIVE_STATE(PAUSED_FOR_USER);
    for (;;) {
        manage_inactivity(true);
        bDelayed = ((_millis()/1000-nTime0) > nDelay);
        bDelayed = (bDelayed && (nDelay != 0));   // 0 ~ no timeout, always waiting for click
        if (lcd_clicked() || bDelayed) {
			KEEPALIVE_STATE(IN_HANDLER);
            return(!bDelayed);
        }
    }
}

void lcd_wait_for_click()
{
lcd_wait_for_click_delay(0);
}

//! @brief Show multiple screen message with yes and no possible choices and wait with possible timeout
//! @param msg Message to show
//! @param allow_timeouting if true, allows time outing of the screen
//! @param default_yes if true, yes choice is selected by default, otherwise no choice is preselected
//! @retval 1 yes choice selected by user
//! @retval 0 no choice selected by user
//! @retval -1 screen timed out
int8_t lcd_show_multiscreen_message_yes_no_and_wait_P(const char *msg, bool allow_timeouting, bool default_yes) //currently just max. n*4 + 3 lines supported (set in language header files)
{
    return lcd_show_multiscreen_message_two_choices_and_wait_P(msg, allow_timeouting, default_yes, _T(MSG_YES), _T(MSG_NO));
}
//! @brief Show multiple screen message with two possible choices and wait with possible timeout
//! @param msg Message to show
//! @param allow_timeouting if true, allows time outing of the screen
//! @param default_first if true, fist choice is selected by default, otherwise second choice is preselected
//! @param first_choice text caption of first possible choice
//! @param second_choice text caption of second possible choice
//! @retval 1 first choice selected by user
//! @retval 0 second choice selected by user
//! @retval -1 screen timed out
int8_t lcd_show_multiscreen_message_two_choices_and_wait_P(const char *msg, bool allow_timeouting, bool default_first,
        const char *first_choice, const char *second_choice)
{
	const char *msg_next = lcd_display_message_fullscreen_P(msg);
	bool multi_screen = msg_next != NULL;
	bool yes = default_first ? true : false;

	// Wait for user confirmation or a timeout.
	unsigned long previous_millis_cmd = _millis();
	int8_t        enc_dif = lcd_encoder_diff;
	lcd_consume_click();
	//KEEPALIVE_STATE(PAUSED_FOR_USER);
	for (;;) {
		for (uint8_t i = 0; i < 100; ++i) {
			if (allow_timeouting && _millis() - previous_millis_cmd > LCD_TIMEOUT_TO_STATUS)
				return -1;
			manage_inactivity(true);

			if (abs(enc_dif - lcd_encoder_diff) > 4) {
				if (msg_next == NULL) {
					lcd_set_cursor(0, 3);
					if (enc_dif < lcd_encoder_diff && yes) {
						lcd_puts_P((PSTR(" ")));
						lcd_set_cursor(7, 3);
						lcd_puts_P((PSTR(">")));
						yes = false;
						Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
					}
					else if (enc_dif > lcd_encoder_diff && !yes) {
						lcd_puts_P((PSTR(">")));
						lcd_set_cursor(7, 3);
						lcd_puts_P((PSTR(" ")));
						yes = true;
						Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
					}
					enc_dif = lcd_encoder_diff;
				}
				else {
					Sound_MakeSound(e_SOUND_TYPE_BlindAlert);
					break; //turning knob skips waiting loop
				}
			}
			if (lcd_clicked()) {
				Sound_MakeSound(e_SOUND_TYPE_ButtonEcho);
				if (msg_next == NULL) {
					//KEEPALIVE_STATE(IN_HANDLER);
					lcd_set_custom_characters();
					return yes;
				}
				else break;
			}
		}
		if (multi_screen) {
			if (msg_next == NULL) {
				msg_next = msg;
			}
			msg_next = lcd_display_message_fullscreen_P(msg_next);
		}
		if (msg_next == NULL) {
			lcd_set_cursor(0, 3);
			if (yes) lcd_puts_P(PSTR(">"));
			lcd_set_cursor(1, 3);
			lcd_puts_P(first_choice);
			lcd_set_cursor(7, 3);
			if (!yes) lcd_puts_P(PSTR(">"));
			lcd_set_cursor(8, 3);
			lcd_puts_P(second_choice);
		}
	}
}

//! @brief Show single screen message with yes and no possible choices and wait with possible timeout
//! @param msg Message to show
//! @param allow_timeouting if true, allows time outing of the screen
//! @param default_yes if true, yes choice is selected by default, otherwise no choice is preselected
//! @retval 1 yes choice selected by user
//! @retval 0 no choice selected by user
//! @retval -1 screen timed out
int8_t lcd_show_fullscreen_message_yes_no_and_wait_P(const char *msg, bool allow_timeouting, bool default_yes)
{

	lcd_display_message_fullscreen_P(msg);
	
	if (default_yes) {
		lcd_set_cursor(0, 2);
		lcd_puts_P(PSTR(">"));
		lcd_puts_P(_T(MSG_YES));
		lcd_set_cursor(1, 3);
		lcd_puts_P(_T(MSG_NO));
	}
	else {
		lcd_set_cursor(1, 2);
		lcd_puts_P(_T(MSG_YES));
		lcd_set_cursor(0, 3);
		lcd_puts_P(PSTR(">"));
		lcd_puts_P(_T(MSG_NO));
	}
	int8_t retval = default_yes ? true : false;

	// Wait for user confirmation or a timeout.
	unsigned long previous_millis_cmd = _millis();
	int8_t        enc_dif = lcd_encoder_diff;
	lcd_consume_click();
	KEEPALIVE_STATE(PAUSED_FOR_USER);
	for (;;) {
		if (allow_timeouting && _millis() - previous_millis_cmd > LCD_TIMEOUT_TO_STATUS)
		{
		    retval = -1;
		    break;
		}
		manage_inactivity(true);
		if (abs(enc_dif - lcd_encoder_diff) > 4) {
			lcd_set_cursor(0, 2);
				if (enc_dif < lcd_encoder_diff && retval) {
					lcd_puts_P((PSTR(" ")));
					lcd_set_cursor(0, 3);
					lcd_puts_P((PSTR(">")));
					retval = 0;
					Sound_MakeSound(e_SOUND_TYPE_EncoderMove);

				}
				else if (enc_dif > lcd_encoder_diff && !retval) {
					lcd_puts_P((PSTR(">")));
					lcd_set_cursor(0, 3);
					lcd_puts_P((PSTR(" ")));
					retval = 1;
					Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
				}
				enc_dif = lcd_encoder_diff;
		}
		if (lcd_clicked()) {
			Sound_MakeSound(e_SOUND_TYPE_ButtonEcho);
			KEEPALIVE_STATE(IN_HANDLER);
			break;
		}
	}
    lcd_encoder_diff = 0;
    return retval;
}


static void lcd_show_end_stops() {
	lcd_set_cursor(0, 0);
	lcd_puts_P(PSTR("End stops diag"));
	lcd_set_cursor(0, 1);
	lcd_puts_P(READ(TEST_PIN0)  ? PSTR("X1") : PSTR("X0"));
	lcd_set_cursor(0, 2);
	lcd_puts_P(READ(TEST_PIN1) ? PSTR("Y1") : PSTR("Y0"));
	lcd_set_cursor(0, 3);
	lcd_puts_P(READ(TEST_PIN2) ? PSTR("Z1") : PSTR("Z0"));
}

static void menu_show_end_stops() {
    lcd_show_end_stops();
    if (LCD_CLICKED) menu_back();
}

// Lets the user move the Z carriage up to the end stoppers.
// When done, it sets the current Z to Z_MAX_POS and returns true.
// Otherwise the Z calibration is not changed and false is returned.
void lcd_diag_show_end_stops()
{
    lcd_clear();
	lcd_consume_click();
    for (;;) {
        manage_inactivity(true);
        lcd_show_end_stops();
        if (lcd_clicked()) {
            break;
        }
    }
    lcd_clear();
    lcd_return_to_status();
}

static void lcd_print_state(uint8_t state)
{
	switch (state) {
		case STATE_ON:
			lcd_puts_P(_N("  1"));
		break;
		case STATE_OFF:
			lcd_puts_P(_N("  0"));
		break;
		default: 
			lcd_puts_P(_T(MSG_NA));
		break;
	}
}

//  |01234567890123456789|
//  |Sensor states:      |
//  |1: Y  2: Y  3: Y    |
//  |4: N  5: N  6: N/A  |
//  |                    |
//  ======================

static void lcd_show_sensors_state()
{
	//0: N/A; 1: OFF; 2: ON
	uint8_t pinda_state[NUM_DIST_PROBES] = { STATE_NA };

	
	pinda_state[0] = READ(TEST_PIN1);
	pinda_state[1] = READ(TEST_PIN2);
	pinda_state[2] = READ(TEST_PIN3);
	pinda_state[3] = READ(TEST_PIN4);
	pinda_state[4] = READ(TEST_PIN5);
	

	lcd_puts_at_P(0, 0, _i("Sensor states:"));
	lcd_space(6);
	lcd_set_cursor(0,1);
	lcd_print("1: ");
	lcd_puts_P(READ(TEST_PIN0)  ? PSTR("Y") : PSTR("N"));
	lcd_space(2);
	lcd_print("2: ");
	lcd_puts_P(READ(TEST_PIN1)  ? PSTR("Y") : PSTR("N"));
	lcd_space(2);
	lcd_print("3: ");
	lcd_puts_P(READ(TEST_PIN2)  ? PSTR("Y") : PSTR("N"));
	lcd_space(4);
	lcd_set_cursor(0,2);
	lcd_print("4: ");
	lcd_puts_P(READ(TEST_PIN3)  ? PSTR("Y") : PSTR("N"));
	lcd_space(2);
	lcd_print("5: ");
	lcd_puts_P(READ(TEST_PIN4)  ? PSTR("Y") : PSTR("N"));
	lcd_space(2);
	lcd_print("6: N/A ");
	lcd_space(2);
	
	lcd_set_cursor(0,3);
	lcd_space(LCD_WIDTH);
}

void lcd_menu_show_sensors_state() {               // NOT static due to using inside "Marlin_main" module ("manage_inactivity()")
	lcd_timeoutToStatus.stop();
	lcd_show_sensors_state();
	if(LCD_CLICKED) {
		lcd_timeoutToStatus.start();
		menu_back();
	}
}

void lcd_move_menu_axis() {
	MENU_BEGIN();
	MENU_ITEM_BACK_P(_T(MSG_SETTINGS));
	MENU_ITEM_SUBMENU_P(_i("Move Z"), lcd_move_z);////MSG_MOVE_Z
	MENU_END();
}

static void lcd_move_menu_1mm()
{
  move_menu_scale = 1.0;
  lcd_move_menu_axis();
}


void EEPROM_save(int pos, uint8_t* value, uint8_t size)
{
  do
  {
    eeprom_write_byte((unsigned char*)pos, *value);
    pos++;
    value++;
  } while (--size);
}

void EEPROM_read(int pos, uint8_t* value, uint8_t size)
{
  do
  {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  } while (--size);
}

//-//
static void lcd_sound_state_set(void)
{
Sound_CycleState();
}

void lcd_set_degree() {
	lcd_set_custom_characters_degree();
}


void lcd_pinda_calibration_menu()
{
	MENU_BEGIN();
		MENU_ITEM_BACK_P(_T(MSG_MENU_CALIBRATION));
		MENU_ITEM_SUBMENU_P(_i("Calibrate"), lcd_calibrate_pinda);////MSG_CALIBRATE_PINDA c=17 r=1
	MENU_END();
}


#ifdef HAS_SECOND_SERIAL_PORT
void lcd_second_serial_set() {
	if(selectedSerialPort == 1) selectedSerialPort = 0;
	else selectedSerialPort = 1;
	eeprom_update_byte((unsigned char *)EEPROM_SECOND_SERIAL_ACTIVE, selectedSerialPort);
	MYSERIAL.begin(BAUDRATE);
}
#endif //HAS_SECOND_SERIAL_PORT

void lcd_calibrate_pinda() {
	enquecommand_P(PSTR("G76"));
	lcd_return_to_status();
}


#define SETTINGS_SOUND \
do\
{\
    switch(eSoundMode)\
    {\
        case e_SOUND_MODE_LOUD:\
            MENU_ITEM_TOGGLE_P(_T(MSG_SOUND), _T(MSG_SOUND_LOUD), lcd_sound_state_set);\
            break;\
        case e_SOUND_MODE_ONCE:\
            MENU_ITEM_TOGGLE_P(_T(MSG_SOUND), _T(MSG_SOUND_ONCE), lcd_sound_state_set);\
            break;\
        case e_SOUND_MODE_SILENT:\
            MENU_ITEM_TOGGLE_P(_T(MSG_SOUND), _T(MSG_SILENT), lcd_sound_state_set);\
            break;\
        case e_SOUND_MODE_BLIND:\
            MENU_ITEM_TOGGLE_P(_T(MSG_SOUND), _T(MSG_SOUND_BLIND), lcd_sound_state_set);\
            break;\
        default:\
            MENU_ITEM_TOGGLE_P(_T(MSG_SOUND), _T(MSG_SOUND_LOUD), lcd_sound_state_set);\
    }\
}\
while (0)

//-//
static void lcd_check_mode_set(void)
{
switch(oCheckMode)
     {
     case ClCheckMode::_None:
          oCheckMode=ClCheckMode::_Warn;
          break;
     case ClCheckMode::_Warn:
          oCheckMode=ClCheckMode::_Strict;
          break;
     case ClCheckMode::_Strict:
          oCheckMode=ClCheckMode::_None;
          break;
     default:
          oCheckMode=ClCheckMode::_None;
     }
eeprom_update_byte((uint8_t*)EEPROM_CHECK_MODE,(uint8_t)oCheckMode);
}

#define SETTINGS_MODE \
do\
{\
    switch(oCheckMode)\
         {\
         case ClCheckMode::_None:\
              MENU_ITEM_TOGGLE_P(_T(MSG_NOZZLE), _T(MSG_NONE), lcd_check_mode_set);\
              break;\
         case ClCheckMode::_Warn:\
              MENU_ITEM_TOGGLE_P(_T(MSG_NOZZLE), _T(MSG_WARN), lcd_check_mode_set);\
              break;\
         case ClCheckMode::_Strict:\
              MENU_ITEM_TOGGLE_P(_T(MSG_NOZZLE), _T(MSG_STRICT), lcd_check_mode_set);\
              break;\
         default:\
              MENU_ITEM_TOGGLE_P(_T(MSG_NOZZLE), _T(MSG_NONE), lcd_check_mode_set);\
         }\
}\
while (0)


#define SETTINGS_NOZZLE \
do\
{\
    float fNozzleDiam;\
    switch(oNozzleDiameter)\
    {\
        case ClNozzleDiameter::_Diameter_250: fNozzleDiam = 0.25f; break;\
        case ClNozzleDiameter::_Diameter_400: fNozzleDiam = 0.4f; break;\
        case ClNozzleDiameter::_Diameter_600: fNozzleDiam = 0.6f; break;\
        default: fNozzleDiam = 0.4f; break;\
    }\
    MENU_ITEM_TOGGLE(_T(MSG_NOZZLE_DIAMETER), ftostr12ns(fNozzleDiam), lcd_nozzle_diameter_set);\
}\
while (0)

static void lcd_check_model_set(void)
{
switch(oCheckModel)
     {
     case ClCheckModel::_None:
          oCheckModel=ClCheckModel::_Warn;
          break;
     case ClCheckModel::_Warn:
          oCheckModel=ClCheckModel::_Strict;
          break;
     case ClCheckModel::_Strict:
          oCheckModel=ClCheckModel::_None;
          break;
     default:
          oCheckModel=ClCheckModel::_None;
     }
eeprom_update_byte((uint8_t*)EEPROM_CHECK_MODEL,(uint8_t)oCheckModel);
}

#define SETTINGS_MODEL \
do\
{\
    switch(oCheckModel)\
         {\
         case ClCheckModel::_None:\
              MENU_ITEM_TOGGLE_P(_T(MSG_MODEL), _T(MSG_NONE), lcd_check_model_set);\
              break;\
         case ClCheckModel::_Warn:\
              MENU_ITEM_TOGGLE_P(_T(MSG_MODEL), _T(MSG_WARN), lcd_check_model_set);\
              break;\
         case ClCheckModel::_Strict:\
              MENU_ITEM_TOGGLE_P(_T(MSG_MODEL), _T(MSG_STRICT), lcd_check_model_set);\
              break;\
         default:\
              MENU_ITEM_TOGGLE_P(_T(MSG_MODEL), _T(MSG_NONE), lcd_check_model_set);\
         }\
}\
while (0)

static void lcd_check_version_set(void)
{
switch(oCheckVersion)
     {
     case ClCheckVersion::_None:
          oCheckVersion=ClCheckVersion::_Warn;
          break;
     case ClCheckVersion::_Warn:
          oCheckVersion=ClCheckVersion::_Strict;
          break;
     case ClCheckVersion::_Strict:
          oCheckVersion=ClCheckVersion::_None;
          break;
     default:
          oCheckVersion=ClCheckVersion::_None;
     }
eeprom_update_byte((uint8_t*)EEPROM_CHECK_VERSION,(uint8_t)oCheckVersion);
}



static void lcd_settings_menu()
{
	MENU_BEGIN();
	MENU_ITEM_BACK_P(_T(MSG_MAIN));
	if (!isPrintPaused){
	    MENU_ITEM_GCODE_P(_i("Disable steppers"), PSTR("M84"));////MSG_DISABLE_STEPPERS
	}
	MENU_END();
}


//! @brief Select one of numbered items
//!
//! Create list of items with header. Header can not be selected.
//! Each item has text description passed by function parameter and
//! number. There are 5 numbered items, if mmu_enabled, 4 otherwise.
//! Items are numbered from 1 to 4 or 5. But index returned starts at 0.
//! There can be last item with different text and no number.
//!
//! @param header Header text
//! @param item Item text
//! @param last_item Last item text, or nullptr if there is no Last item
//! @return selected item index, first item index is 0
uint8_t choose_menu_P(const char *header, const char *item, const char *last_item)
{
    //following code should handle 3 to 127 number of items well
    const int8_t items_no = last_item?5:4;
    const uint8_t item_len = item?strlen_P(item):0;
	int8_t first = 0;
	int8_t enc_dif = lcd_encoder_diff;
	int8_t cursor_pos = 1;
	
	lcd_clear();

	KEEPALIVE_STATE(PAUSED_FOR_USER);
	while (1) {
		manage_inactivity(true);

		if (abs((enc_dif - lcd_encoder_diff)) > 4) {
            if (enc_dif > lcd_encoder_diff) {
                cursor_pos--;
            }

            if (enc_dif < lcd_encoder_diff) {
                cursor_pos++;
            }
            enc_dif = lcd_encoder_diff;
			Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
		}

		if (cursor_pos > 3) {		
            cursor_pos = 3;
            if (first < items_no - 3) {
                first++;
                lcd_clear();
            } else { // here we are at the very end of the list
				Sound_MakeSound(e_SOUND_TYPE_BlindAlert);
            }
        }

        if (cursor_pos < 1)
        {
            cursor_pos = 1;
            if (first > 0)
            {
                first--;
                lcd_clear();
            } else { // here we are at the very end of the list
				Sound_MakeSound(e_SOUND_TYPE_BlindAlert);
            }
        }

        if (header) lcd_puts_at_P(0,0,header);

        const bool last_visible = (first == items_no - 3);
        const uint_least8_t ordinary_items = (last_item&&last_visible)?2:3;

        for (uint_least8_t i = 0; i < ordinary_items; i++)
        {
            if (item) lcd_puts_at_P(1, i + 1, item);
        }

        for (uint_least8_t i = 0; i < ordinary_items; i++)
        {
            lcd_set_cursor(2 + item_len, i+1);
            lcd_print(first + i + 1);
        }

        if (last_item&&last_visible) lcd_puts_at_P(1, 3, last_item);

        lcd_set_cursor(0, 1);
        lcd_print(" ");
        lcd_set_cursor(0, 2);
        lcd_print(" ");
        lcd_set_cursor(0, 3);
        lcd_print(" ");
        lcd_set_cursor(0, cursor_pos);
        lcd_print(">");
        _delay(100);

		if (lcd_clicked())
		{
			Sound_MakeSound(e_SOUND_TYPE_ButtonEcho);
		    KEEPALIVE_STATE(IN_HANDLER);
			lcd_encoder_diff = 0;
			return(cursor_pos + first - 1);
		}
	}
}

char reset_menu() {
	int items_no = 4;
	static int first = 0;
	int enc_dif = 0;
	char cursor_pos = 0;
	const char *item [items_no];
	
	item[0] = "Language";
	item[1] = "Statistics";
	item[2] = "Shipping prep";
	item[3] = "All Data";
	enc_dif = lcd_encoder_diff;
	lcd_clear();
	lcd_set_cursor(0, 0);
	lcd_print(">");
	lcd_consume_click();
	while (1) {		

		for (uint_least8_t i = 0; i < 4; i++) {
			lcd_set_cursor(1, i);
			lcd_print(item[first + i]);
		}

		manage_inactivity(true);

		if (abs((enc_dif - lcd_encoder_diff)) > 4) {

			if ((abs(enc_dif - lcd_encoder_diff)) > 1) {
				if (enc_dif > lcd_encoder_diff) {
					cursor_pos--;
				}

				if (enc_dif < lcd_encoder_diff) {
					cursor_pos++;
				}

				if (cursor_pos > 3) {
					cursor_pos = 3;
					Sound_MakeSound(e_SOUND_TYPE_BlindAlert);
					if (first < items_no - 4) {
						first++;
						lcd_clear();
					}
				}

				if (cursor_pos < 0) {
					cursor_pos = 0;
					Sound_MakeSound(e_SOUND_TYPE_BlindAlert);
					if (first > 0) {
						first--;
						lcd_clear();
					}
				}
				lcd_set_cursor(0, 0);
				lcd_print(" ");
				lcd_set_cursor(0, 1);
				lcd_print(" ");
				lcd_set_cursor(0, 2);
				lcd_print(" ");
				lcd_set_cursor(0, 3);
				lcd_print(" ");
				lcd_set_cursor(0, cursor_pos);
				lcd_print(">");
				Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
				enc_dif = lcd_encoder_diff;
				_delay(100);
			}

		}

		if (lcd_clicked()) {
			Sound_MakeSound(e_SOUND_TYPE_ButtonEcho);
			return(cursor_pos + first);
		}

	}

}


void lcd_confirm_print()
{
	uint8_t filament_type;
	int enc_dif = 0;
	int cursor_pos = 1;
	int _ret = 0;
	int _t = 0;

	enc_dif = lcd_encoder_diff;
	lcd_clear();

	lcd_set_cursor(0, 0);
	lcd_print("Print ok ?");

	do
	{
		if (abs(enc_dif - lcd_encoder_diff) > 12) {
			if (enc_dif > lcd_encoder_diff) {
				cursor_pos--;
			}

			if (enc_dif < lcd_encoder_diff) {
				cursor_pos++;
			}
			enc_dif = lcd_encoder_diff;
		}

		if (cursor_pos > 2) { cursor_pos = 2; }
		if (cursor_pos < 1) { cursor_pos = 1; }

		lcd_set_cursor(0, 2); lcd_print("          ");
		lcd_set_cursor(0, 3); lcd_print("          ");
		lcd_set_cursor(2, 2);
		lcd_puts_P(_T(MSG_YES));
		lcd_set_cursor(2, 3);
		lcd_puts_P(_T(MSG_NO));
		lcd_set_cursor(0, 1 + cursor_pos);
		lcd_print(">");
		_delay(100);

		_t = _t + 1;
		if (_t>100)
		{
			_t = 0;
		}
		if (lcd_clicked())
		{
               filament_type = FARM_FILAMENT_COLOR_NONE;
			if (cursor_pos == 1)
			{
				_ret = 1;
//				filament_type = lcd_choose_color();
				no_response = true; //we need confirmation by recieving PRUSA thx
				important_status = 4;
				saved_filament_type = filament_type;
				NcTime = _millis();
			}
			if (cursor_pos == 2)
			{
				_ret = 2;
//				filament_type = lcd_choose_color();
				no_response = true; //we need confirmation by recieving PRUSA thx
				important_status = 5;				
				saved_filament_type = filament_type;
				NcTime = _millis();
			}
		}

		manage_inactivity();
		proc_commands();

	} while (_ret == 0);

}

#include "w25x20cl.h"

//! @brief Resume paused print
//! @todo It is not good to call restore_print_from_ram_and_continue() from function called by lcd_update(),
//! as restore_print_from_ram_and_continue() calls lcd_update() internally.
void lcd_resume_print()
{
    lcd_return_to_status();
    lcd_reset_alert_level(); //for fan speed error

    lcd_setstatuspgm(_T(MSG_FINISHING_MOVEMENTS));
    st_synchronize();

    lcd_setstatuspgm(_T(MSG_RESUMING_PRINT)); ////MSG_RESUMING_PRINT c=20
    isPrintPaused = false;
    restore_print_from_ram_and_continue(default_retraction);
    pause_time += (_millis() - start_pause_print); //accumulate time when print is paused for correct statistics calculation
    refresh_cmd_timeout();
    SERIAL_PROTOCOLLNRPGM(MSG_OCTOPRINT_RESUMED); //resume octoprint
}


void mainTest(){
	MENU_BEGIN();
	// if(lcd_show_fullscreen_message_yes_no_and_wait_P("Begin Probe Test?", false, true)){

	// } else {
	// 	lcd_main_menu();
	// }
	MENU_ITEM_TEXT_P(PSTR("Begin Probe Test?"));
	MENU_ITEM_SUBMENU_P(_i("YES"), runPindaTest);
	MENU_ITEM_SUBMENU_P(_i("NO"), lcd_main_menu);
	MENU_ITEM_BACK_P(_T(MSG_WATCH));
	MENU_END();
}

void runPindaTest(){
	lcd_set_cursor(0,0);
	lcd_space(LCD_WIDTH);
	lcd_set_cursor(0,1);
	lcd_print("Please insert probes");
	lcd_set_cursor(0,2);
	lcd_space(LCD_WIDTH);
	lcd_set_cursor(0,3);
	lcd_space(LCD_WIDTH);
	bool allSet = true;
	do{
		lcd_wait_for_click();
		allSet = lcd_show_fullscreen_message_yes_no_and_wait_P(PSTR(" All probes ready?  "), false, true);
	} while (!allSet);

	enquecommand_P(PSTR("M980"));
}

 void pindaTest(){
    lcd_set_cursor(0, 0); //line 1
	lcd_print("Running test routine");
    lcd_set_cursor(0, 1); //line 0
	lcd_space(LCD_WIDTH);
    //Print the hotend temperature (9 chars total)
	lcd_set_cursor(0, 2);
	lcd_print("Cycles left: ");
	lcd_set_cursor(13, 2);
	lcd_print(itostr3(100-currentCycle));
	lcd_set_cursor(0,3);
	lcd_print("Routine #");
	lcd_set_cursor(9,3);
	lcd_print(routineCycle + 1);
	lcd_set_cursor(10,3);
	lcd_print(" of ");
	lcd_print(NUMROUTINECYCLES);
	lcd_space(5);
}

void waitProbes() { 
	lcd_set_cursor(0,0);
	lcd_space(LCD_WIDTH);
	lcd_set_cursor(0,1);
	lcd_print("Please remove probes");
	lcd_set_cursor(0,2);
	lcd_space(LCD_WIDTH);
	lcd_set_cursor(0,3);
	lcd_space(LCD_WIDTH);
	lcd_wait_for_click();
}


//! |01234567890123456789|
//! | All probes passed! |
//! |Please remove all   |
//! |probes and press to |
//! |resume testing.     |
//! ----------------------
void displayProbeResults() {
	bool allGood = true;
	for(int i = 0; i < NUM_DIST_PROBES; i++) {
		if(!probeResult[i]){
			allGood = false;
			break;
		}
	}
	if(allGood){
		lcd_set_cursor(0,0);
		lcd_print(" All probes passed! ");
		lcd_set_cursor(0,1);
		lcd_print("Please remove all   ");
		lcd_set_cursor(0,2);
		lcd_print("probes and press to ");
		lcd_set_cursor(0,3);
		lcd_print("resume testing.     ");
		lcd_wait_for_click();
	} else {
//! |01234567890123456789|
//! |Some probes rejected|
//! |Please dispose of   |
//! |rejected probes as  |
//! |required.           |
//! ----------------------
		lcd_set_cursor(0,0);
		lcd_print("Some probes rejected");
		lcd_set_cursor(0,1);
		lcd_print("Please dispose of   ");
		lcd_set_cursor(0,2);
		lcd_print("rejected probes as  ");
		lcd_set_cursor(0,3);
		lcd_print("required.           ");
		if(!probeResult[0]){
			lcd_show_fullscreen_message_and_wait_P(PSTR("Reject probe #0"));
		}
		if(!probeResult[1]){
			lcd_show_fullscreen_message_and_wait_P(PSTR("Reject probe #1"));
		}
		if(!probeResult[2]){
			lcd_show_fullscreen_message_and_wait_P(PSTR("Reject probe #2"));
		}
		if(!probeResult[3]){
			lcd_show_fullscreen_message_and_wait_P(PSTR("Reject probe #3"));
		}
		if(!probeResult[4]){
			lcd_show_fullscreen_message_and_wait_P(PSTR("Reject probe #4"));
		}
	}
}

//Main LCD Menu
static void lcd_main_menu() {

	MENU_BEGIN();
	MENU_ITEM_BACK_P(_T(MSG_WATCH));


	MENU_ITEM_SUBMENU_P(_i("Test"), lcd_show_end_stops);
	MENU_ITEM_SUBMENU_P(_T(MSG_SETTINGS), lcd_settings_menu);
	MENU_ITEM_SUBMENU_P(_i("Statistics  "), lcd_menu_statistics);////MSG_STATISTICS
 	MENU_ITEM_SUBMENU_P(_i("Support"), lcd_support_menu);////MSG_SUPPORT

  MENU_END();

}

void stack_error() {
	Sound_MakeCustom(1000,0,true);
	lcd_display_message_fullscreen_P(_i("Error - static memory has been overwritten"));////MSG_STACK_ERROR c=20 r=4
	//err_triggered = 1;
}

#ifdef DEBUG_STEPPER_TIMER_MISSED
bool stepper_timer_overflow_state = false;
uint16_t stepper_timer_overflow_max = 0;
uint16_t stepper_timer_overflow_last = 0;
uint16_t stepper_timer_overflow_cnt = 0;
void stepper_timer_overflow() {
  char msg[28];
  sprintf_P(msg, PSTR("#%d %d max %d"), ++ stepper_timer_overflow_cnt, stepper_timer_overflow_last >> 1, stepper_timer_overflow_max >> 1);
  lcd_setstatus(msg);
  stepper_timer_overflow_state = false;
  if (stepper_timer_overflow_last > stepper_timer_overflow_max)
    stepper_timer_overflow_max = stepper_timer_overflow_last;
  SERIAL_ECHOPGM("Stepper timer overflow: ");
  MYSERIAL.print(msg);
  SERIAL_ECHOLNPGM("");

  WRITE(BEEPER, LOW);
}
#endif /* DEBUG_STEPPER_TIMER_MISSED */



static void mbl_magnets_elimination_toggle() {
	bool magnet_elimination = (eeprom_read_byte((uint8_t*)EEPROM_MBL_MAGNET_ELIMINATION) > 0);
	magnet_elimination = !magnet_elimination;
	eeprom_update_byte((uint8_t*)EEPROM_MBL_MAGNET_ELIMINATION, (uint8_t)magnet_elimination);
}

static void mbl_mesh_toggle() {
	uint8_t mesh_nr = eeprom_read_byte((uint8_t*)EEPROM_MBL_POINTS_NR);
	if(mesh_nr == 3) mesh_nr = 7;
	else mesh_nr = 3;
	eeprom_update_byte((uint8_t*)EEPROM_MBL_POINTS_NR, mesh_nr);
}

static void mbl_probe_nr_toggle() {
	mbl_z_probe_nr = eeprom_read_byte((uint8_t*)EEPROM_MBL_PROBE_NR);
	switch (mbl_z_probe_nr) {
		case 1: mbl_z_probe_nr = 3; break;
		case 3: mbl_z_probe_nr = 5; break;
		case 5: mbl_z_probe_nr = 1; break;
		default: mbl_z_probe_nr = 3; break;
	}
	eeprom_update_byte((uint8_t*)EEPROM_MBL_PROBE_NR, mbl_z_probe_nr);
}



void lcd_print_stop() {
        SERIAL_ECHOLNRPGM(MSG_OCTOPRINT_CANCEL);   // for Octoprint

    CRITICAL_SECTION_START;

    // Clear any saved printing state
    cancel_saved_printing();

    // Abort the planner/queue/sd
    planner_abort_hard();
	cmdqueue_reset();
    st_reset_timer();

    CRITICAL_SECTION_END;

	lcd_setstatuspgm(_T(MSG_PRINT_ABORTED));
	stoptime = _millis();
	unsigned long t = (stoptime - starttime - pause_time) / 1000; //time in s
	pause_time = 0;
	save_statistics(total_filament_used, t);

    lcd_commands_step = 0;
    lcd_commands_type = LcdCommands::Idle;
    cancel_heatup = true; //unroll temperature wait loop stack.

    current_position[Z_AXIS] += 10; //lift Z.
    plan_buffer_line_curposXYZE(manual_feedrate[Z_AXIS] / 60);

    if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) //if axis are homed, move to parked position.
    {
        current_position[X_AXIS] = X_CANCEL_POS;
        current_position[Y_AXIS] = Y_CANCEL_POS;
        plan_buffer_line_curposXYZE(manual_feedrate[0] / 60);
    }
    st_synchronize();

    finishAndDisableSteppers(); //M84

    lcd_setstatuspgm(_T(WELCOME_MSG));
    custom_message_type = CustomMsg::Status;

    planner_abort_hard(); //needs to be done since plan_buffer_line resets waiting_inside_plan_buffer_line_print_aborted to false. Also copies current to destination.
    
    axis_relative_modes = E_AXIS_MASK; //XYZ absolute, E relative
    
    isPrintPaused = false; //clear isPrintPaused flag to allow starting next print after pause->stop scenario.
}


/** End of menus **/

/** LCD API **/

void ultralcd_init() {
    backlight_init();
	lcd_init();
	lcd_refresh();
	lcd_longpress_func = menu_lcd_longpress_func;
	lcd_charsetup_func = menu_lcd_charsetup_func;
	lcd_lcdupdate_func = menu_lcd_lcdupdate_func;
	menu_menu = lcd_status_screen;
	menu_lcd_charsetup_func();

  SET_INPUT(BTN_EN1);
  SET_INPUT(BTN_EN2);
  WRITE(BTN_EN1, HIGH);
  WRITE(BTN_EN2, HIGH);
#if BTN_ENC > 0
  SET_INPUT(BTN_ENC);
  WRITE(BTN_ENC, HIGH);
#endif

  lcd_encoder_diff = 0;
}





void lcd_printer_connected() {
	printer_connected = true;
}

static void lcd_send_status() {
	if (farm_mode && no_response && ((_millis() - NcTime) > (NC_TIME * 1000))) {
		//send important status messages periodicaly
		NcTime = _millis();
#ifdef FARM_CONNECT_MESSAGE
		lcd_connect_printer();
#endif //FARM_CONNECT_MESSAGE
	}
}



void lcd_ping() { //chceck if printer is connected to monitoring when in farm mode
	if (farm_mode) {
		bool empty = is_buffer_empty();
		if ((_millis() - PingTime) * 0.001 > (empty ? PING_TIME : PING_TIME_LONG)) { //if commands buffer is empty use shorter time period
																							  //if there are comamnds in buffer, some long gcodes can delay execution of ping command
																							  //therefore longer period is used
			printer_connected = false;
		}
		else {
			lcd_printer_connected();
		}
	}
}
void lcd_ignore_click(bool b)
{
  ignore_click = b;
  wait_for_unclick = false;
}

void lcd_finishstatus() {
  SERIAL_PROTOCOLLNRPGM(MSG_LCD_STATUS_CHANGED);
  int len = strlen(lcd_status_message);
  if (len > 0) {
    while (len < LCD_WIDTH) {
      lcd_status_message[len++] = ' ';
    }
  }
  lcd_status_message[LCD_WIDTH] = '\0';
  lcd_draw_update = 2;

}

void lcd_setstatus(const char* message)
{
  if (lcd_status_message_level > 0)
    return;
  lcd_updatestatus(message);
}

void lcd_updatestatuspgm(const char *message){
	strncpy_P(lcd_status_message, message, LCD_WIDTH);
	lcd_status_message[LCD_WIDTH] = 0;
	lcd_finishstatus();
	// hack lcd_draw_update to 1, i.e. without clear
	lcd_draw_update = 1;
}

void lcd_setstatuspgm(const char* message)
{
  if (lcd_status_message_level > 0)
    return;
  lcd_updatestatuspgm(message);
}

void lcd_updatestatus(const char *message){
	strncpy(lcd_status_message, message, LCD_WIDTH);
	lcd_status_message[LCD_WIDTH] = 0;
	lcd_finishstatus();
	// hack lcd_draw_update to 1, i.e. without clear
	lcd_draw_update = 1;
}

void lcd_setalertstatuspgm(const char* message)
{
  lcd_setstatuspgm(message);
  lcd_status_message_level = 1;
  lcd_return_to_status();
}

void lcd_setalertstatus(const char* message)
{
  lcd_setstatus(message);
  lcd_status_message_level = 1;
  lcd_return_to_status();
}

void lcd_reset_alert_level()
{
  lcd_status_message_level = 0;
}

uint8_t get_message_level()
{
	return lcd_status_message_level;
}

void menu_lcd_longpress_func(void)
{
	backlight_wake();
    if (homing_flag || menu_menu == lcd_move_z)
    {
        // disable longpress during re-entry, while homing or calibration
        lcd_quick_feedback();
        return;
    }

    // explicitely listed menus which are allowed to rise the move-z or live-adj-z functions
    // The lists are not the same for both functions, so first decide which function is to be performed
    if ( (moves_planned() || is_usb_printing )){ // long press as live-adj-z
        if(( current_position[Z_AXIS] < Z_HEIGHT_HIDE_LIVE_ADJUST_MENU ) // only allow live-adj-z up to 2mm of print height
        && ( menu_menu == lcd_status_screen // and in listed menus...
          || menu_menu == lcd_main_menu
          || menu_menu == lcd_support_menu
           )
        ){
            lcd_clear();
        } else {
            // otherwise consume the long press as normal click
            if( menu_menu != lcd_status_screen )
                menu_back();
        }
    } else { // long press as move-z
        if(menu_menu == lcd_status_screen
        || menu_menu == lcd_main_menu
        || menu_menu == lcd_settings_menu
        || menu_menu == lcd_support_menu
        ){
            move_menu_scale = 1.0;
            menu_submenu(lcd_move_z);
        } else {
            // otherwise consume the long press as normal click
            if( menu_menu != lcd_status_screen )
                menu_back();
        }
    }
}

void menu_lcd_charsetup_func(void)
{
	if (menu_menu == lcd_status_screen)
		lcd_set_custom_characters_degree();
	else
		lcd_set_custom_characters_arrows();
}


static inline bool other_menu_expired()
{
    return (menu_menu != lcd_status_screen
            && lcd_timeoutToStatus.expired(LCD_TIMEOUT_TO_STATUS));
}
static inline bool forced_menu_expire()
{
    bool retval = (menu_menu != lcd_status_screen
            && forceMenuExpire);
    forceMenuExpire = false;
    return retval;
}

void menu_lcd_lcdupdate_func(void) {

    backlight_update();
	if (lcd_next_update_millis < _millis()) {
		if (abs(lcd_encoder_diff) >= ENCODER_PULSES_PER_STEP) {
			if (lcd_draw_update == 0) {
				lcd_draw_update = 1;
			}
			lcd_encoder += lcd_encoder_diff / ENCODER_PULSES_PER_STEP;
			Sound_MakeSound(e_SOUND_TYPE_EncoderMove);
			lcd_encoder_diff = 0;
			lcd_timeoutToStatus.start();
			backlight_wake();
		}

		if (LCD_CLICKED) {
			lcd_timeoutToStatus.start();
			backlight_wake();
		}

		(*menu_menu)();

		if (other_menu_expired() || forced_menu_expire()) {
		// Exiting a menu. Let's call the menu function the last time with menu_leaving flag set to true
		// to give it a chance to save its state.
		// This is useful for example, when the babystep value has to be written into EEPROM.
			if (menu_menu != NULL) {
				menu_leaving = 1;
				(*menu_menu)();
				menu_leaving = 0;
			}
			lcd_clear();
			lcd_return_to_status();
			lcd_draw_update = 2;
		}
		if (lcd_draw_update == 2) {
			lcd_clear();
		}
		if (lcd_draw_update) {
			lcd_draw_update--;
		}
		lcd_next_update_millis = _millis() + LCD_UPDATE_INTERVAL;
	}
	lcd_ping(); //check that we have received ping command if we are in farm mode
	lcd_send_status();
	if (lcd_commands_type == LcdCommands::Layer1Cal) lcd_commands();
}


void lcd_experimental_toggle()
{
    uint8_t oldVal = eeprom_read_byte((uint8_t *)EEPROM_EXPERIMENTAL_VISIBILITY);
    if (oldVal == EEPROM_EMPTY_VALUE)
        oldVal = 0;
    else
        oldVal = !oldVal;
    eeprom_update_byte((uint8_t *)EEPROM_EXPERIMENTAL_VISIBILITY, oldVal);
}

void lcd_experimental_menu()
{
    MENU_BEGIN();
    MENU_ITEM_BACK_P(_T(MSG_BACK));

#ifdef EXTRUDER_ALTFAN_DETECT
    MENU_ITEM_TOGGLE_P(_N("ALTFAN det."), altfanOverride_get()?_T(MSG_OFF):_T(MSG_ON), altfanOverride_toggle);////MSG_MENU_ALTFAN c=18
#endif //EXTRUDER_ALTFAN_DETECT

    MENU_END();
}