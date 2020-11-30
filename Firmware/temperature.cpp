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
#include "cardreader.h"

#include "Sd2PinMap.h"

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

enum TempRunawayStates
{
	TempRunaway_INACTIVE = 0,
	TempRunaway_PREHEAT = 1,
	TempRunaway_ACTIVE = 2,
};

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//=============================   functions      ============================
//===========================================================================


// ready for eventually parameters adjusting
void resetPID(uint8_t) {                            // only for compiler-warning elimination (if function do nothing)
//void resetPID(uint8_t extruder)
}

void manage_heater() {
#ifdef WATCHDOG
    wdt_reset();
#endif //WATCHDOG

  float pid_input;
  float pid_output;

  if(temp_meas_ready != true) {   //better readability
    return; 
  }
  // more precisely - this condition partially stabilizes time interval for regulation values evaluation (@ ~ 230ms)
  // ADC values need to be converted before checking: converted values are later used in MINTEMP
  updateTemperaturesFromRawValues();

  // check_max_temp();
  // check_min_temp();


  for(int e = 0; e < EXTRUDERS; e++) {
#ifdef TEMP_RUNAWAY_EXTRUDER_HYSTERESIS
	  temp_runaway_check(e+1, target_temperature[e], current_temperature[e], (int)soft_pwm[e], false);
#endif
#ifdef PIDTEMP
    pid_input = current_temperature[e];
  #ifndef PID_OPENLOOP
    if(target_temperature[e] == 0) {
      pid_output = 0;
      pid_reset[e] = true;
    } else {
      pid_error[e] = target_temperature[e] - pid_input;
      if(pid_reset[e]) {
        iState_sum[e] = 0.0;
        dTerm[e] = 0.0;                       // 'dState_last[e]' initial setting is not necessary (see end of if-statement)
        pid_reset[e] = false;
      }
    #ifndef PonM
      pTerm[e] = cs.Kp * pid_error[e];
      iState_sum[e] += pid_error[e];
      iState_sum[e] = constrain(iState_sum[e], iState_sum_min[e], iState_sum_max[e]);
      iTerm[e] = cs.Ki * iState_sum[e];
      // PID_K1 defined in Configuration.h in the PID settings
      #define K2 (1.0-PID_K1)
      dTerm[e] = (cs.Kd * (pid_input - dState_last[e]))*K2 + (PID_K1 * dTerm[e]); // e.g. digital filtration of derivative term changes
      pid_output = pTerm[e] + iTerm[e] - dTerm[e]; // subtraction due to "Derivative on Measurement" method (i.e. derivative of input instead derivative of error is used)
      if (pid_output > PID_MAX) {
        if (pid_error[e] > 0 ) iState_sum[e] -= pid_error[e]; // conditional un-integration
          pid_output=PID_MAX;
        } else if (pid_output < 0) {
          if (pid_error[e] < 0 ) iState_sum[e] -= pid_error[e]; // conditional un-integration
            pid_output=0;
          }
    #else // PonM ("Proportional on Measurement" method)
      iState_sum[e] += cs.Ki * pid_error[e];
      iState_sum[e] -= cs.Kp * (pid_input - dState_last[e]);
      iState_sum[e] = constrain(iState_sum[e], 0, PID_INTEGRAL_DRIVE_MAX);
      dTerm[e] = cs.Kd * (pid_input - dState_last[e]);
      pid_output = iState_sum[e] - dTerm[e];  // subtraction due to "Derivative on Measurement" method (i.e. derivative of input instead derivative of error is used)
      pid_output = constrain(pid_output, 0, PID_MAX);
    #endif // PonM
    }
    dState_last[e] = pid_input;
  #else 
    pid_output = constrain(target_temperature[e], 0, PID_MAX);
  #endif //PID_OPENLOOP
  #ifdef PID_DEBUG
    SERIAL_ECHO_START;
    SERIAL_ECHO(" PID_DEBUG ");
    SERIAL_ECHO(e);
    SERIAL_ECHO(": Input ");
    SERIAL_ECHO(pid_input);
    SERIAL_ECHO(" Output ");
    SERIAL_ECHO(pid_output);
    SERIAL_ECHO(" pTerm ");
    SERIAL_ECHO(pTerm[e]);
    SERIAL_ECHO(" iTerm ");
    SERIAL_ECHO(iTerm[e]);
    SERIAL_ECHO(" dTerm ");
    SERIAL_ECHOLN(-dTerm[e]);
  #endif //PID_DEBUG
#else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) {
      pid_output = PID_MAX;
    }
#endif

    // Check if temperature is within the correct range
    if((current_temperature[e] < maxttemp[e]) && (target_temperature[e] != 0)) {
      soft_pwm[e] = (int)pid_output >> 1;
    } else {
      soft_pwm[e] = 0;
    }
  } // End extruder for loop

#define FAN_CHECK_PERIOD 5000 //5s
#define FAN_CHECK_DURATION 100 //100ms

#ifndef DEBUG_DISABLE_FANCHECK
  #if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1)

    #ifdef FAN_SOFT_PWM
      #ifdef FANCHECK
  if ((_millis() - extruder_autofan_last_check > FAN_CHECK_PERIOD) && (!fan_measuring)) {
	  extruder_autofan_last_check = _millis();
	  fanSpeedBckp = fanSpeedSoftPwm;
	  
	  if (fanSpeedSoftPwm >= MIN_PRINT_FAN_SPEED) { //if we are in rage where we are doing fan check, set full PWM range for a short time to measure fan RPM by reading tacho signal without modulation by PWM signal
		//  printf_P(PSTR("fanSpeedSoftPwm 1: %d\n"), fanSpeedSoftPwm);
		  fanSpeedSoftPwm = 255;
	  }
	  fan_measuring = true;
  }
  if ((_millis() - extruder_autofan_last_check > FAN_CHECK_DURATION) && (fan_measuring)) {
	  countFanSpeed();
	  checkFanSpeed();
	  //printf_P(PSTR("fanSpeedSoftPwm 1: %d\n"), fanSpeedSoftPwm);
	  fanSpeedSoftPwm = fanSpeedBckp;
	  //printf_P(PSTR("fan PWM: %d; extr fanSpeed measured: %d; print fan speed measured: %d \n"), fanSpeedBckp, fan_speed[0], fan_speed[1]);
	  extruder_autofan_last_check = _millis();
	  fan_measuring = false;
  }
      #endif //FANCHECK
  checkExtruderAutoFans();
    #else //FAN_SOFT_PWM
  if(_millis() - extruder_autofan_last_check > 1000) { // only need to check fan state very infrequently
    #if (defined(FANCHECK) && ((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1))))
	  countFanSpeed();
	  checkFanSpeed();
    #endif //(defined(TACH_0) && TACH_0 >-1) || (defined(TACH_1) && TACH_1 > -1)
    checkExtruderAutoFans();
    extruder_autofan_last_check = _millis();
  }  
    #endif //FAN_SOFT_PWM

  #endif  
#endif //DEBUG_DISABLE_FANCHECK
  
#ifndef PIDTEMPBED
  if(_millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL){
    return;
  }
  previous_millis_bed_heater = _millis();
#endif

#if TEMP_SENSOR_BED != 0

  #ifdef PIDTEMPBED
  pid_input = current_temperature_bed;

    #ifndef PID_OPENLOOP
  pid_error_bed = target_temperature_bed - pid_input;
  pTerm_bed = cs.bedKp * pid_error_bed;
  temp_iState_bed += pid_error_bed;
  temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
  iTerm_bed = cs.bedKi * temp_iState_bed;

  //PID_K1 defined in Configuration.h in the PID settings
		  #define K2 (1.0-PID_K1)
  dTerm_bed= (cs.bedKd * (pid_input - temp_dState_bed))*K2 + (PID_K1 * dTerm_bed);
  temp_dState_bed = pid_input;

  pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
  if (pid_output > MAX_BED_POWER) {
    if (pid_error_bed > 0 ) {
      temp_iState_bed -= pid_error_bed;
    } // conditional un-integration
    pid_output=MAX_BED_POWER;
    } else if (pid_output < 0){
      if (pid_error_bed < 0 ) { 
        temp_iState_bed -= pid_error_bed;
      } // conditional un-integration
      pid_output=0;
    }
    #else 
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif //PID_OPENLOOP

	  if(current_temperature_bed < BED_MAXTEMP) {
	    soft_pwm_bed = (int)pid_output >> 1;
		timer02_set_pwm0(soft_pwm_bed << 1);
	  } else {
	    soft_pwm_bed = 0;
		  timer02_set_pwm0(soft_pwm_bed << 1);
	  }

  #elif !defined(BED_LIMIT_SWITCHING)
    // Check if temperature is within the correct range
    if(current_temperature_bed < BED_MAXTEMP) {
      if(current_temperature_bed >= target_temperature_bed) {
        soft_pwm_bed = 0;
  		  timer02_set_pwm0(soft_pwm_bed << 1);
      } else {
        soft_pwm_bed = MAX_BED_POWER>>1;
		    timer02_set_pwm0(soft_pwm_bed << 1);
      }
    } else {
      soft_pwm_bed = 0;
		  timer02_set_pwm0(soft_pwm_bed << 1);
      WRITE(HEATER_BED_PIN,LOW);
    }
  #else //#ifdef BED_LIMIT_SWITCHING
    // Check if temperature is within the correct band
    if(current_temperature_bed < BED_MAXTEMP) {
      if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS) {
        soft_pwm_bed = 0;
  		  timer02_set_pwm0(soft_pwm_bed << 1);
      } else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS) {
        soft_pwm_bed = MAX_BED_POWER>>1;
        timer02_set_pwm0(soft_pwm_bed << 1);
      }
    } else {
      soft_pwm_bed = 0;
		  timer02_set_pwm0(soft_pwm_bed << 1);
      WRITE(HEATER_BED_PIN,LOW);
    }
  #endif
    if(target_temperature_bed==0) {
      soft_pwm_bed = 0;
  		timer02_set_pwm0(soft_pwm_bed << 1);
	  }
#endif
  
  host_keepalive();
}

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

FORCE_INLINE static void temperature_isr()
{
	if (!temp_meas_ready) adc_cycle();
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
    } else{ WRITE(HEATER_0_PIN,0);}
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
  
#if 0  // @@DR vypnuto pro hw pwm bedu
  // tuhle prasarnu bude potreba poustet ve stanovenych intervalech, jinak nemam moc sanci zareagovat
  // teoreticky by se tato cast uz vubec nemusela poustet
  if ((pwm_count & ((1 << HEATER_BED_SOFT_PWM_BITS) - 1)) == 0)
  {
    soft_pwm_b = soft_pwm_bed >> (7 - HEATER_BED_SOFT_PWM_BITS);
#  ifndef SYSTEM_TIMER_2
	// tady budu krokovat pomalou frekvenci na automatu - tohle je rizeni spinani a rozepinani
	// jako ridici frekvenci mam 2khz, jako vystupni frekvenci mam 30hz
	// 2kHz jsou ovsem ve slysitelnem pasmu, mozna bude potreba jit s frekvenci nahoru (a tomu taky prizpusobit ostatni veci)
	// Teoreticky bych mohl stahnout OCR0B citac na 6, cimz bych se dostal nekam ke 40khz a tady potom honit PWM rychleji nebo i pomaleji
	// to nicemu nevadi. Soft PWM scale by se 20x zvetsilo (no dobre, 16x), cimz by se to posunulo k puvodnimu 30Hz PWM
	//if(soft_pwm_b > 0) WRITE(HEATER_BED_PIN,1); else WRITE(HEATER_BED_PIN,0);
#  endif //SYSTEM_TIMER_2
  }
#endif
#endif
  
#ifdef FAN_SOFT_PWM
  if ((pwm_count & ((1 << FAN_SOFT_PWM_BITS) - 1)) == 0)
  {
    soft_pwm_fan = fanSpeedSoftPwm / (1 << (8 - FAN_SOFT_PWM_BITS));
    if(soft_pwm_fan > 0) WRITE(FAN_PIN,1); else WRITE(FAN_PIN,0);
  }
#endif
  if(soft_pwm_0 < pwm_count)
  { 
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


