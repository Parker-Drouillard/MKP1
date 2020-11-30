/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */


#include "Marlin.h"
#include "ultralcd.h"
#include "sound.h"
#include "temperature.h"
#include <avr/wdt.h>
#include "adc.h"
#include "ConfigurationStore.h"
#include "messages.h"
#include "Timer.h"
#include "Configuration_prusa.h"

#include "config.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;
int current_temperature_raw[EXTRUDERS] = { 0 };
float current_temperature[EXTRUDERS] = { 0.0 };

#ifdef PINDA_THERMISTOR
uint16_t current_temperature_raw_pinda =  0 ; //value with more averaging applied
uint16_t current_temperature_raw_pinda_fast = 0; //value read from adc
float current_temperature_pinda = 0.0;
#endif //PINDA_THERMISTOR

#ifdef AMBIENT_THERMISTOR
int current_temperature_raw_ambient =  0 ;
float current_temperature_ambient = 0.0;
#endif //AMBIENT_THERMISTOR

#ifdef VOLT_PWR_PIN
int current_voltage_raw_pwr = 0;
#endif

#ifdef VOLT_BED_PIN
int current_voltage_raw_bed = 0;
#endif

#ifdef IR_SENSOR_ANALOG
uint16_t current_voltage_raw_IR = 0;
#endif //IR_SENSOR_ANALOG

int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
  

#ifdef PIDTEMP
  float _Kp, _Ki, _Kd;
  int pid_cycle, pid_number_of_cycles;
  bool pid_tuning_finished = false;
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP
  
#ifdef FAN_SOFT_PWM
  unsigned char fanSpeedSoftPwm;
#endif

#ifdef FANCHECK
  volatile uint8_t fan_check_error = EFCE_OK;
#endif

unsigned char soft_pwm_bed;

#ifdef BABYSTEPPING
  volatile int babystepsTodo[3]={0,0,0};
#endif

//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float iState_sum[EXTRUDERS] = { 0 };
  static float dState_last[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  //int output;
  static float pid_error[EXTRUDERS];
  static float iState_sum_min[EXTRUDERS];
  static float iState_sum_max[EXTRUDERS];
  // static float pid_input[EXTRUDERS];
  // static float pid_output[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP
#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[EXTRUDERS];

#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif

uint8_t fanSpeedBckp = 255;

#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1)
  unsigned long extruder_autofan_last_check = _millis();
  
  bool fan_measuring = false;
  uint8_t fanState = 0;
#ifdef EXTRUDER_ALTFAN_DETECT
  struct
  {
    uint8_t isAltfan : 1;
    uint8_t altfanOverride : 1;
  } altfanStatus;
#endif //EXTRUDER_ALTFAN_DETECT
#endif


#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

  static ShortTimer oTimer4minTempHeater[EXTRUDERS],oTimer4minTempBed;

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
#ifdef BED_MINTEMP
static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
#endif
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif
#if defined(AMBIENT_MINTEMP)  && TEMP_SENSOR_AMBIENT > 0
static int ambient_minttemp_raw = AMBIENT_RAW_LO_TEMP;
#endif
#if defined(AMBIENT_MAXTEMP) && TEMP_SENSOR_AMBIENT > 0
static int ambient_maxttemp_raw = AMBIENT_RAW_HI_TEMP;
#endif

static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static float analog2tempAmbient(int raw);
static void updateTemperaturesFromRawValues();


#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//=============================   functions      ============================
//===========================================================================


#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
  if(e >= EXTRUDERS) {
    SERIAL_ERROR_START;
    SERIAL_ERROR((int)e);
    SERIAL_ERRORLNPGM(" - Invalid extruder number !");
    kill(NULL, 6);
    return 0.0;
  } 
  #ifdef HEATER_0_USES_MAX6675
  if (e == 0) {
    return 0.25 * raw;
  }
  #endif

  if(heater_ttbl_map[e] != NULL) {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++) {
      if (PGM_RD_W((*tt)[i][0]) > raw) {
        celsius = PGM_RD_W((*tt)[i-1][1]) + 
          (raw - PGM_RD_W((*tt)[i-1][0])) * 
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) {
      celsius = PGM_RD_W((*tt)[i-1][1]);
    }
    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    byte i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++) {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) + 
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) * 
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) {
      celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);
    }

	// temperature offset adjustment
#ifdef BED_OFFSET
	float _offset = BED_OFFSET;
	float _offset_center = BED_OFFSET_CENTER;
	float _offset_start = BED_OFFSET_START;
	float _first_koef = (_offset / 2) / (_offset_center - _offset_start);
	float _second_koef = (_offset / 2) / (100 - _offset_center);


	if (celsius >= _offset_start && celsius <= _offset_center) {
		celsius = celsius + (_first_koef * (celsius - _offset_start));
	} else if (celsius > _offset_center && celsius <= 100) {
		celsius = celsius + (_first_koef * (_offset_center - _offset_start)) + ( _second_koef * ( celsius - ( 100 - _offset_center ) )) ;
	}	else if (celsius > 100) {
		celsius = celsius + _offset;
	}
#endif


    return celsius;
  #elif defined BED_USES_AD595
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
  #else
    return 0;
  #endif
}

#ifdef AMBIENT_THERMISTOR
static float analog2tempAmbient(int raw) {
    float celsius = 0;
    byte i;

    for (i=1; i<AMBIENTTEMPTABLE_LEN; i++) {
      if (PGM_RD_W(AMBIENTTEMPTABLE[i][0]) > raw) {
        celsius  = PGM_RD_W(AMBIENTTEMPTABLE[i-1][1]) + 
          (raw - PGM_RD_W(AMBIENTTEMPTABLE[i-1][0])) * 
          (float)(PGM_RD_W(AMBIENTTEMPTABLE[i][1]) - PGM_RD_W(AMBIENTTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(AMBIENTTEMPTABLE[i][0]) - PGM_RD_W(AMBIENTTEMPTABLE[i-1][0]));
        break;
      }
    }
    // Overflow: Set to last value in the table
    if (i == AMBIENTTEMPTABLE_LEN) {
      celsius = PGM_RD_W(AMBIENTTEMPTABLE[i-1][1]);
    }
    return celsius;
}
#endif //AMBIENT_THERMISTOR

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues() {
    for(uint8_t e=0;e<EXTRUDERS;e++) {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }

#ifdef PINDA_THERMISTOR
	current_temperature_raw_pinda = (uint16_t)((uint32_t)current_temperature_raw_pinda * 3 + current_temperature_raw_pinda_fast) >> 2;
	current_temperature_pinda = analog2tempBed(current_temperature_raw_pinda);
#endif

#ifdef AMBIENT_THERMISTOR
	current_temperature_ambient = analog2tempAmbient(current_temperature_raw_ambient); //thermistor for ambient is NTCG104LH104JT1 (2000)
#endif
   
#ifdef DEBUG_HEATER_BED_SIM
	current_temperature_bed = target_temperature_bed;
#else //DEBUG_HEATER_BED_SIM
	current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
#endif //DEBUG_HEATER_BED_SIM

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}

void tp_init() {
#if MB(RUMBA) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1))
  //disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
  MCUCR=(1<<JTD); 
  MCUCR=(1<<JTD);
#endif
  
  // Finish init of mult extruder arrays 
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    iState_sum_min[e] = 0.0;
    iState_sum_max[e] = PID_INTEGRAL_DRIVE_MAX / cs.Ki;
#endif //PIDTEMP
#ifdef PIDTEMPBED
    temp_iState_min_bed = 0.0;
    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / cs.bedKi;
#endif //PIDTEMPBED
  }

  #if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1) 
    SET_OUTPUT(HEATER_0_PIN);
  #endif  
  #if defined(HEATER_1_PIN) && (HEATER_1_PIN > -1) 
    SET_OUTPUT(HEATER_1_PIN);
  #endif  
  #if defined(HEATER_2_PIN) && (HEATER_2_PIN > -1) 
    SET_OUTPUT(HEATER_2_PIN);
  #endif  
  #if defined(HEATER_BED_PIN) && (HEATER_BED_PIN > -1) 
    SET_OUTPUT(HEATER_BED_PIN);
  #endif  
  #if defined(FAN_PIN) && (FAN_PIN > -1) 
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / (1 << (8 - FAN_SOFT_PWM_BITS));
    #endif
  #endif

  #ifdef HEATER_0_USES_MAX6675
    #ifndef SDSUPPORT
      SET_OUTPUT(SCK_PIN);
      WRITE(SCK_PIN,0);
    
      SET_OUTPUT(MOSI_PIN);
      WRITE(MOSI_PIN,1);
    
      SET_INPUT(MISO_PIN);
      WRITE(MISO_PIN,1);
    #endif
    /* Using pinMode and digitalWrite, as that was the only way I could get it to compile */
    
    //Have to toggle SD card CS pin to low first, to enable firmware to talk with SD card
	pinMode(SS_PIN, OUTPUT);
	digitalWrite(SS_PIN,0);  
	pinMode(MAX6675_SS, OUTPUT);
	digitalWrite(MAX6675_SS,1);
  #endif

  adc_init();

  timer0_init();
  OCR2B = 128;
  TIMSK2 |= (1<<OCIE2B);
  
  timer4_init(); //for tone and Extruder fan PWM
  
  // Wait for temperature measurement to settle
  _delay(250);

#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 1

#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
  minttemp[2] = HEATER_2_MINTEMP;
  while(analog2temp(minttemp_raw[2], 2) < HEATER_2_MINTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    minttemp_raw[2] += OVERSAMPLENR;
#else
    minttemp_raw[2] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP 2
#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = HEATER_2_MAXTEMP;
  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    maxttemp_raw[2] -= OVERSAMPLENR;
#else
    maxttemp_raw[2] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 2

#ifdef BED_MINTEMP
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP

#if defined(AMBIENT_MINTEMP) && TEMP_SENSOR_AMBIENT > -1
  while(analog2tempAmbient(ambient_minttemp_raw) < AMBIENT_MINTEMP) {
#if HEATER_AMBIENT_RAW_LO_TEMP < HEATER_AMBIENT_RAW_HI_TEMP
    ambient_minttemp_raw += OVERSAMPLENR;
#else
    ambient_minttemp_raw -= OVERSAMPLENR;
#endif
  }
#endif //AMBIENT_MINTEMP
#if defined(AMBIENT_MAXTEMP) && TEMP_SENSOR_AMBIENT > -1
  while(analog2tempAmbient(ambient_maxttemp_raw) > AMBIENT_MAXTEMP) {
#if HEATER_AMBIENT_RAW_LO_TEMP < HEATER_AMBIENT_RAW_HI_TEMP
    ambient_maxttemp_raw -= OVERSAMPLENR;
#else
    ambient_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //AMBIENT_MAXTEMP
}


//! codes of alert messages for the LCD - it is shorter to compare an uin8_t
//! than raw const char * of the messages themselves.
//! Could be used for MAXTEMP situations too - after reaching MAXTEMP and turning off the heater automagically
//! the heater/bed may cool down and a similar alert message like "MAXTERM fixed..." may be displayed.
enum { LCDALERT_NONE = 0, LCDALERT_HEATERMINTEMP, LCDALERT_BEDMINTEMP, LCDALERT_MINTEMPFIXED, LCDALERT_PLEASERESTART };

//! remember the last alert message sent to the LCD
//! to prevent flicker and improve speed
uint8_t last_alert_sent_to_lcd = LCDALERT_NONE;



#ifdef HEATER_0_USES_MAX6675
#define MAX6675_HEAT_INTERVAL 250
long max6675_previous_millis = MAX6675_HEAT_INTERVAL;
int max6675_temp = 2000;

int read_max6675()
{
  if (_millis() - max6675_previous_millis < MAX6675_HEAT_INTERVAL) 
    return max6675_temp;
  
  max6675_previous_millis = _millis();
  max6675_temp = 0;
    
  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif
  
  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
  
  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);
  
  // ensure 100ns delay - a bit extra is fine
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  
  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;
  
  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;
  
  // disable TT_MAX6675
  WRITE(MAX6675_SS, 1);

  if (max6675_temp & 4) 
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else 
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif


extern "C" {


void adc_ready(void) { //callback from adc when sampling finished
	current_temperature_raw[0] = adc_values[ADC_PIN_IDX(TEMP_0_PIN)]; //heater
#if EXTRUDERS > 1
  current_temperature_raw[1] = adc_values[ADC_PIN_IDX(TEMP_1_PIN)]; //Heater 2
#endif
#ifdef PINDA_THERMISTOR
	current_temperature_raw_pinda_fast = adc_values[ADC_PIN_IDX(TEMP_PINDA_PIN)];
#endif //PINDA_THERMISTOR
	current_temperature_bed_raw = adc_values[ADC_PIN_IDX(TEMP_BED_PIN)];
#ifdef VOLT_PWR_PIN
	current_voltage_raw_pwr = adc_values[ADC_PIN_IDX(VOLT_PWR_PIN)];
#endif
#ifdef AMBIENT_THERMISTOR
	current_temperature_raw_ambient = adc_values[ADC_PIN_IDX(TEMP_AMBIENT_PIN)]; // 5->6
#endif //AMBIENT_THERMISTOR
#ifdef VOLT_BED_PIN
	current_voltage_raw_bed = adc_values[ADC_PIN_IDX(VOLT_BED_PIN)]; // 6->9
#endif
#ifdef IR_SENSOR_ANALOG
     current_voltage_raw_IR = adc_values[ADC_PIN_IDX(VOLT_IR_PIN)];
#endif //IR_SENSOR_ANALOG
	temp_meas_ready = true;
}

} // extern "C"

#if HEATER_BED_PIN > -1
  static unsigned char soft_pwm_b;
#endif

FORCE_INLINE static void temperature_isr() {
	if (!temp_meas_ready) {adc_cycle();}
	lcd_buttons_update();

  static uint8_t pwm_count = (1 << SOFT_PWM_SCALE);
  static uint8_t soft_pwm_0;
#ifdef SLOW_PWM_HEATERS
  static unsigned char slow_pwm_count = 0;
  static unsigned char state_heater_0 = 0;
  static unsigned char state_timer_heater_0 = 0;
#endif 
#if (EXTRUDERS > 1) || defined(HEATERS_PARALLEL)
  static unsigned char soft_pwm_1;
#ifdef SLOW_PWM_HEATERS
  static unsigned char state_heater_1 = 0;
  static unsigned char state_timer_heater_1 = 0;
#endif 
#endif
#if EXTRUDERS > 2
  static unsigned char soft_pwm_2;
#ifdef SLOW_PWM_HEATERS
  static unsigned char state_heater_2 = 0;
  static unsigned char state_timer_heater_2 = 0;
#endif 
#endif
#if HEATER_BED_PIN > -1
  // @@DR static unsigned char soft_pwm_b;
#ifdef SLOW_PWM_HEATERS
  static unsigned char state_heater_b = 0;
  static unsigned char state_timer_heater_b = 0;
#endif 
#endif
  
#if defined(FILWIDTH_PIN) &&(FILWIDTH_PIN > -1)
  static unsigned long raw_filwidth_value = 0;  //added for filament width sensor
#endif
  
#ifndef SLOW_PWM_HEATERS
  /*
   * standard PWM modulation
   */
  if (pwm_count == 0) {
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) { 
      WRITE(HEATER_0_PIN,1);
#ifdef HEATERS_PARALLEL
      WRITE(HEATER_1_PIN,1);
#endif
    } else { 
      WRITE(HEATER_0_PIN,0);
    }
#if EXTRUDERS > 1
    soft_pwm_1 = soft_pwm[1];
    if(soft_pwm_1 > 0) {WRITE(HEATER_1_PIN,1);} else {WRITE(HEATER_1_PIN,0);}
#endif
#if EXTRUDERS > 2
    soft_pwm_2 = soft_pwm[2];
    if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,1); else WRITE(HEATER_2_PIN,0);
#endif
  }
#if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
  soft_pwm_b = soft_pwm_bed;
  if(soft_pwm_b > 0) {
    WRITE(HEATER_BED_PIN,1);
  } else {
    WRITE(HEATER_BED_PIN,0);
  }
#endif

  
#ifdef FAN_SOFT_PWM
  if ((pwm_count & ((1 << FAN_SOFT_PWM_BITS) - 1)) == 0) {
    soft_pwm_fan = fanSpeedSoftPwm / (1 << (8 - FAN_SOFT_PWM_BITS));
    if(soft_pwm_fan > 0) {WRITE(FAN_PIN,1); } else {WRITE(FAN_PIN,0);}
  }
#endif
  if(soft_pwm_0 < pwm_count) {
    WRITE(HEATER_0_PIN,0);
#ifdef HEATERS_PARALLEL
    WRITE(HEATER_1_PIN,0);
#endif
  }

#if EXTRUDERS > 1
  if(soft_pwm_1 < pwm_count) WRITE(HEATER_1_PIN,0);
#endif
#if EXTRUDERS > 2
  if(soft_pwm_2 < pwm_count) WRITE(HEATER_2_PIN,0);
#endif

#ifdef FAN_SOFT_PWM
  if (soft_pwm_fan < (pwm_count & ((1 << FAN_SOFT_PWM_BITS) - 1))) WRITE(FAN_PIN,0);
#endif
  
  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

#else //ifndef SLOW_PWM_HEATERS
  /*
   * SLOW PWM HEATERS
   *
   * for heaters drived by relay
   */
#ifndef MIN_STATE_TIME
#define MIN_STATE_TIME 16 // MIN_STATE_TIME * 65.5 = time in milliseconds
#endif
  if (slow_pwm_count == 0) {
    // EXTRUDER 0 
    soft_pwm_0 = soft_pwm[0];
    if (soft_pwm_0 > 0) {
      // turn ON heather only if the minimum time is up 
      if (state_timer_heater_0 == 0) { 
	// if change state set timer 
	if (state_heater_0 == 0) {
	  state_timer_heater_0 = MIN_STATE_TIME;
	}
	state_heater_0 = 1;
	WRITE(HEATER_0_PIN, 1);
#ifdef HEATERS_PARALLEL
	WRITE(HEATER_1_PIN, 1);
#endif
      }
    } else {
      // turn OFF heather only if the minimum time is up 
      if (state_timer_heater_0 == 0) {
	// if change state set timer 
	if (state_heater_0 == 1) {
	  state_timer_heater_0 = MIN_STATE_TIME;
	}
	state_heater_0 = 0;
	WRITE(HEATER_0_PIN, 0);
#ifdef HEATERS_PARALLEL
	WRITE(HEATER_1_PIN, 0);
#endif
      }
    }
    
#if EXTRUDERS > 1
    // EXTRUDER 1
    soft_pwm_1 = soft_pwm[1];
    if (soft_pwm_1 > 0) {
      // turn ON heather only if the minimum time is up 
      if (state_timer_heater_1 == 0) { 
	// if change state set timer 
	if (state_heater_1 == 0) {
	  state_timer_heater_1 = MIN_STATE_TIME;
	}
	state_heater_1 = 1;
	WRITE(HEATER_1_PIN, 1);
      }
    } else {
      // turn OFF heather only if the minimum time is up 
      if (state_timer_heater_1 == 0) {
	// if change state set timer 
	if (state_heater_1 == 1) {
	  state_timer_heater_1 = MIN_STATE_TIME;
	}
	state_heater_1 = 0;
	WRITE(HEATER_1_PIN, 0);
      }
    }
#endif
    
#if EXTRUDERS > 2
    // EXTRUDER 2
    soft_pwm_2 = soft_pwm[2];
    if (soft_pwm_2 > 0) {
      // turn ON heather only if the minimum time is up 
      if (state_timer_heater_2 == 0) { 
	// if change state set timer 
	if (state_heater_2 == 0) {
	  state_timer_heater_2 = MIN_STATE_TIME;
	}
	state_heater_2 = 1;
	WRITE(HEATER_2_PIN, 1);
      }
    } else {
      // turn OFF heather only if the minimum time is up 
      if (state_timer_heater_2 == 0) {
	// if change state set timer 
	if (state_heater_2 == 1) {
	  state_timer_heater_2 = MIN_STATE_TIME;
	}
	state_heater_2 = 0;
	WRITE(HEATER_2_PIN, 0);
      }
    }
#endif
    
#if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    // BED
    soft_pwm_b = soft_pwm_bed;
    if (soft_pwm_b > 0) {
      // turn ON heather only if the minimum time is up 
      if (state_timer_heater_b == 0) { 
	// if change state set timer 
	if (state_heater_b == 0) {
	  state_timer_heater_b = MIN_STATE_TIME;
	}
	state_heater_b = 1;
	//WRITE(HEATER_BED_PIN, 1);
      }
    } else {
      // turn OFF heather only if the minimum time is up 
      if (state_timer_heater_b == 0) {
	// if change state set timer 
	if (state_heater_b == 1) {
	  state_timer_heater_b = MIN_STATE_TIME;
	}
	state_heater_b = 0;
	WRITE(HEATER_BED_PIN, 0);
      }
    }
#endif
  } // if (slow_pwm_count == 0)
  
  // EXTRUDER 0 
  if (soft_pwm_0 < slow_pwm_count) {
    // turn OFF heather only if the minimum time is up 
    if (state_timer_heater_0 == 0) { 
      // if change state set timer 
      if (state_heater_0 == 1) {
	state_timer_heater_0 = MIN_STATE_TIME;
      }
      state_heater_0 = 0;
      WRITE(HEATER_0_PIN, 0);
#ifdef HEATERS_PARALLEL
      WRITE(HEATER_1_PIN, 0);
#endif
    }
  }
    
#if EXTRUDERS > 1
  // EXTRUDER 1 
  if (soft_pwm_1 < slow_pwm_count) {
    // turn OFF heather only if the minimum time is up 
    if (state_timer_heater_1 == 0) { 
      // if change state set timer 
      if (state_heater_1 == 1) {
	state_timer_heater_1 = MIN_STATE_TIME;
      }
      state_heater_1 = 0;
      WRITE(HEATER_1_PIN, 0);
    }
  }
#endif
  
#if EXTRUDERS > 2
  // EXTRUDER 2
  if (soft_pwm_2 < slow_pwm_count) {
    // turn OFF heather only if the minimum time is up 
    if (state_timer_heater_2 == 0) { 
      // if change state set timer 
      if (state_heater_2 == 1) {
	state_timer_heater_2 = MIN_STATE_TIME;
      }
      state_heater_2 = 0;
      WRITE(HEATER_2_PIN, 0);
    }
  }
#endif
  
#if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
  // BED
  if (soft_pwm_b < slow_pwm_count) {
    // turn OFF heather only if the minimum time is up 
    if (state_timer_heater_b == 0) { 
      // if change state set timer 
      if (state_heater_b == 1) {
	state_timer_heater_b = MIN_STATE_TIME;
      }
      state_heater_b = 0;
      WRITE(HEATER_BED_PIN, 0);
    }
  }
#endif
  
#ifdef FAN_SOFT_PWM
  if ((pwm_count & ((1 << FAN_SOFT_PWM_BITS) - 1)) == 0)
    soft_pwm_fan = fanSpeedSoftPwm / (1 << (8 - FAN_SOFT_PWM_BITS));
    if (soft_pwm_fan > 0) WRITE(FAN_PIN,1); else WRITE(FAN_PIN,0);
  }
  if (soft_pwm_fan < pwm_count) WRITE(FAN_PIN,0);
#endif

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;
  
  // increment slow_pwm_count only every 64 pwm_count circa 65.5ms
  if ((pwm_count % 64) == 0) {
    slow_pwm_count++;
    slow_pwm_count &= 0x7f;
    
    // Extruder 0
    if (state_timer_heater_0 > 0) {
      state_timer_heater_0--;
    } 
  
#if EXTRUDERS > 1
    // Extruder 1
    if (state_timer_heater_1 > 0) 
      state_timer_heater_1--;
#endif
    
#if EXTRUDERS > 2
    // Extruder 2
    if (state_timer_heater_2 > 0) 
      state_timer_heater_2--;
#endif
    
#if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    // Bed   
    if (state_timer_heater_b > 0) 
      state_timer_heater_b--;
#endif
  } //if ((pwm_count % 64) == 0) {
  
#endif //ifndef SLOW_PWM_HEATERS

  
#ifdef BABYSTEPPING
  for(uint8_t axis=0;axis<3;axis++)
  {
    int curTodo=babystepsTodo[axis]; //get rid of volatile for performance
   
    if(curTodo>0)
    {
		asm("cli");
      babystep(axis,/*fwd*/true);
      babystepsTodo[axis]--; //less to do next time
		asm("sei");
    }
    else
    if(curTodo<0)
    {
		asm("cli");
      babystep(axis,/*fwd*/false);
      babystepsTodo[axis]++; //less to do next time
		asm("sei");
    }
  }
#endif //BABYSTEPPING

#if (defined(FANCHECK) && defined(TACH_0) && (TACH_0 > -1))
  check_fans();
#endif //(defined(TACH_0))
}

// Timer2 (originaly timer0) is shared with millies
#ifdef SYSTEM_TIMER_2
ISR(TIMER2_COMPB_vect)
#else //SYSTEM_TIMER_2
ISR(TIMER0_COMPB_vect)
#endif //SYSTEM_TIMER_2
{
    static bool _lock = false;
    if (!_lock)
    {
        _lock = true;
        sei();
        temperature_isr();
        cli();
        _lock = false;
    }
}


#ifdef PINDA_THERMISTOR
//! @brief PINDA thermistor detected
//!
//! @retval true firmware should do temperature compensation and allow calibration
//! @retval false PINDA thermistor is not detected, disable temperature compensation and calibration
//!
bool has_temperature_compensation()
{
#ifdef DETECT_SUPERPINDA
    return (current_temperature_pinda >= PINDA_MINTEMP) ? true : false;
#else
    return true;
#endif
}
#endif //PINDA_THERMISTOR
