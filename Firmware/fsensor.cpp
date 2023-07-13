//! @file

#include "Marlin.h"

#include "fsensor.h"
#include <avr/pgmspace.h>
#include "stepper.h"
#include "cmdqueue.h"
#include "ultralcd.h"
#include "cardreader.h"

#include "adc.h"
#include "temperature.h"
#include "config.h"

//! @name Basic parameters
//! @{
#define FSENSOR_CHUNK_LEN      1.25 //!< filament sensor chunk length (mm)
#define FSENSOR_ERR_MAX           4 //!< filament sensor maximum error/chunk count for runout detection

#define FSENSOR_SOFTERR_CMAX      3 //!< number of contiguous soft failures before a triggering a runout
#define FSENSOR_SOFTERR_DELTA 30000 //!< maximum interval (ms) to consider soft failures contiguous
//! @}

//! @name Optical quality measurement parameters
//! @{
#define FSENSOR_OQ_MAX_ES      2    //!< maximum sum of error blocks during filament recheck
#define FSENSOR_OQ_MIN_YD      2    //!< minimum yd sum during filament check (counts per inch)
#define FSENSOR_OQ_MIN_BR      80   //!< minimum brightness value
#define FSENSOR_OQ_MAX_SH      10   //!< maximum shutter value
//! @}

// PJ7 can not be used (does not have PinChangeInterrupt possibility)
#define FSENSOR_INT_PIN          75 //!< filament sensor interrupt pin PJ4
#define FSENSOR_INT_PIN_MASK   0x10 //!< filament sensor interrupt pin mask (bit4)
#define FSENSOR_INT_PIN_PIN_REG PINJ              // PIN register @ PJ4
#define FSENSOR_INT_PIN_VECT PCINT1_vect          // PinChange ISR @ PJ4
#define FSENSOR_INT_PIN_PCMSK_REG PCMSK1          // PinChangeMaskRegister @ PJ4
#define FSENSOR_INT_PIN_PCMSK_BIT PCINT13         // PinChange Interrupt / PinChange Enable Mask @ PJ4
#define FSENSOR_INT_PIN_PCICR_BIT PCIE1           // PinChange Interrupt Enable / Flag @ PJ4

//! enabled = initialized and sampled every chunk event
bool fsensor_enabled = true;
//! runout watching is done in fsensor_update (called from main loop)
bool fsensor_watch_runout = true;
//! not responding - is set if any communication error occurred during initialization or readout
bool fsensor_not_responding = false;

#ifdef DEBUG_FSENSOR_LOG
//! log flag: 0=log disabled, 1=log enabled
uint8_t fsensor_log = 1;
#endif //DEBUG_FSENSOR_LOG


//! @name filament autoload variables
//! @{

//! autoload feature enabled
bool fsensor_autoload_enabled = true;
//! autoload watching enable/disable flag
bool fsensor_watch_autoload = false;


//! @name filament optical quality measurement variables
//! @{

//! Measurement enable/disable flag
bool fsensor_oq_meassure = false;
//! skip-chunk counter, for accurate measurement is necessary to skip first chunk...
uint8_t  fsensor_oq_skipchunk;
//! number of samples from start of measurement
uint8_t fsensor_oq_samples;
//! sum of steps in positive direction movements
uint16_t fsensor_oq_st_sum;
//! sum of deltas in positive direction movements
uint16_t fsensor_oq_yd_sum;
//! sum of errors during measurement
uint16_t fsensor_oq_er_sum;
//! max error counter value during measurement
uint8_t  fsensor_oq_er_max;
//! minimum delta value
int16_t fsensor_oq_yd_min;
//! maximum delta value
int16_t fsensor_oq_yd_max;
//! sum of shutter value
uint16_t fsensor_oq_sh_sum;
//! @}

#ifdef IR_SENSOR_ANALOG
ClFsensorPCB oFsensorPCB;
ClFsensorActionNA oFsensorActionNA;
bool bIRsensorStateFlag=false;
unsigned long nIRsensorLastTime;
#endif //IR_SENSOR_ANALOG

void fsensor_stop_and_save_print(void)
{
    printf_P(PSTR("fsensor_stop_and_save_print\n"));
    stop_and_save_print_to_ram(0, 0);
    fsensor_watch_runout = false;
}


void fsensor_restore_print_and_continue(void)
{
    printf_P(PSTR("fsensor_restore_print_and_continue\n"));
    fsensor_watch_runout = true;
    restore_print_from_ram_and_continue(0);
}

// fsensor_checkpoint_print cuts the current print job at the current position,
// allowing new instructions to be inserted in the middle
void fsensor_checkpoint_print(void)
{
    printf_P(PSTR("fsensor_checkpoint_print\n"));
    stop_and_save_print_to_ram(0, 0);
    restore_print_from_ram_and_continue(0);
}

#ifdef IR_SENSOR_ANALOG
const char* FsensorIRVersionText()
{
	switch(oFsensorPCB)
	{
		case ClFsensorPCB::_Old:
			return _T(MSG_IR_03_OR_OLDER);
		case ClFsensorPCB::_Rev04:
			return _T(MSG_IR_04_OR_NEWER);
		default:
			return _T(MSG_IR_UNKNOWN);
	}
}
#endif //IR_SENSOR_ANALOG

void fsensor_init(void) {
#ifdef FILAMENT_SENSOR
	uint8_t fsensor_enabled = eeprom_read_byte((uint8_t*)EEPROM_FSENSOR);
	fsensor_autoload_enabled=eeprom_read_byte((uint8_t*)EEPROM_FSENS_AUTOLOAD_ENABLED);
	fsensor_not_responding = false;
#ifdef IR_SENSOR_ANALOG
	bIRsensorStateFlag=false;
	oFsensorPCB = (ClFsensorPCB)eeprom_read_byte((uint8_t*)EEPROM_FSENSOR_PCB);
	oFsensorActionNA = (ClFsensorActionNA)eeprom_read_byte((uint8_t*)EEPROM_FSENSOR_ACTION_NA);

	// If the fsensor is not responding even at the start of the printer,
	// set this flag accordingly to show N/A in Settings->Filament sensor.
	// This is even valid for both fsensor board revisions (0.3 or older and 0.4).
	// Must be done after reading what type of fsensor board we have
	fsensor_not_responding = ! fsensor_IR_check();
#endif //IR_SENSOR_ANALOG
	if (fsensor_enabled){
		fsensor_enable(false);                  // (in this case) EEPROM update is not necessary
	} else {
		fsensor_disable(false);                 // (in this case) EEPROM update is not necessary
	}
	printf_P(PSTR("FSensor %S"), (fsensor_enabled?PSTR("ENABLED"):PSTR("DISABLED")));
#ifdef IR_SENSOR_ANALOG
	printf_P(PSTR(" (sensor board revision:%S)\n"), FsensorIRVersionText());
#else //IR_SENSOR_ANALOG
	MYSERIAL.println();
#endif //IR_SENSOR_ANALOG

#endif //FILAMENT_SENSOR
}

bool fsensor_enable(bool bUpdateEEPROM) {
#ifdef IR_SENSOR_ANALOG
     if(!fsensor_IR_check())
          {
          bUpdateEEPROM=true;
          fsensor_enabled=false;
          fsensor_not_responding=true;
          FSensorStateMenu=0;
          }
     else {
#endif //IR_SENSOR_ANALOG
     fsensor_enabled=true;
     fsensor_not_responding=false;
     FSensorStateMenu=1;
#ifdef IR_SENSOR_ANALOG
          }
#endif //IR_SENSOR_ANALOG
     if(bUpdateEEPROM)
          eeprom_update_byte((uint8_t*)EEPROM_FSENSOR, FSensorStateMenu);
	return fsensor_enabled;
}

void fsensor_disable(bool bUpdateEEPROM)
{ 
	fsensor_enabled = false;
	FSensorStateMenu = 0;
     if(bUpdateEEPROM)
          eeprom_update_byte((uint8_t*)EEPROM_FSENSOR, 0x00); 
}

void fsensor_autoload_set(bool State) {
	fsensor_autoload_enabled = State;
	eeprom_update_byte((unsigned char *)EEPROM_FSENS_AUTOLOAD_ENABLED, fsensor_autoload_enabled);
}

void pciSetup(byte pin)
{
// !!! "digitalPinTo?????bit()" does not provide the correct results for some MCU pins
	*digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin)); // enable pin
	PCIFR |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
	PCICR |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}

bool fsensor_check_autoload(void) {
	if (!fsensor_enabled) {return false;}
	if (!fsensor_autoload_enabled) {return false;}
	return false;
}



//! Common code for enqueing M600 and supplemental codes into the command queue.
void fsensor_enque_M600(){
	printf_P(PSTR("fsensor_update - M600\n"));
	eeprom_update_byte((uint8_t*)EEPROM_FERROR_COUNT, eeprom_read_byte((uint8_t*)EEPROM_FERROR_COUNT) + 1);
	eeprom_update_word((uint16_t*)EEPROM_FERROR_COUNT_TOT, eeprom_read_word((uint16_t*)EEPROM_FERROR_COUNT_TOT) + 1);
	enquecommand_front_P((PSTR("M600")));
}

//! @brief filament sensor update (perform M600 on filament runout)
//!
//! Works only if filament sensor is enabled.
//! When the filament sensor error count is larger then FSENSOR_ERR_MAX, pauses print, tries to move filament back and forth.
//! If there is still no plausible signal from filament sensor plans M600 (Filament change).
void fsensor_update(void) {
    //Originally only PAT sensor compatible
}

#ifdef IR_SENSOR_ANALOG
/// This is called only upon start of the printer or when switching the fsensor ON in the menu
/// We cannot do temporal window checks here (aka the voltage has been in some range for a period of time)
bool fsensor_IR_check(){
    if( IRsensor_Lmax_TRESHOLD <= current_voltage_raw_IR && current_voltage_raw_IR <= IRsensor_Hmin_TRESHOLD ){
        /// If the voltage is in forbidden range, the fsensor is ok, but the lever is mounted improperly.
        /// Or the user is so creative so that he can hold a piece of fillament in the hole in such a genius way,
        /// that the IR fsensor reading is within 1.5 and 3V ... this would have been highly unusual
        /// and would have been considered more like a sabotage than normal printer operation
        printf_P(PSTR("fsensor in forbidden range 1.5-3V - check sensor\n"));
        return false; 
    }
    if( oFsensorPCB == ClFsensorPCB::_Rev04 ){
        /// newer IR sensor cannot normally produce 4.6-5V, this is considered a failure/bad mount
        if( IRsensor_Hopen_TRESHOLD <= current_voltage_raw_IR && current_voltage_raw_IR <= IRsensor_VMax_TRESHOLD ){
            printf_P(PSTR("fsensor v0.4 in fault range 4.6-5V - unconnected\n"));
            return false;
        }
        /// newer IR sensor cannot normally produce 0-0.3V, this is considered a failure 
#if 0	//Disabled as it has to be decided if we gonna use this or not.
        if( IRsensor_Hopen_TRESHOLD <= current_voltage_raw_IR && current_voltage_raw_IR <= IRsensor_VMax_TRESHOLD ){
            printf_P(PSTR("fsensor v0.4 in fault range 0.0-0.3V - wrong IR sensor\n"));
            return false;
        }
#endif
    }
    /// If IR sensor is "uknown state" and filament is not loaded > 1.5V return false
#if 0
    if( (oFsensorPCB == ClFsensorPCB::_Undef) && ( current_voltage_raw_IR > IRsensor_Lmax_TRESHOLD ) ){
        printf_P(PSTR("Unknown IR sensor version and no filament loaded detected.\n"));
        return false;
    }
#endif
    // otherwise the IR fsensor is considered working correctly
    return true;
}
#endif //IR_SENSOR_ANALOG
