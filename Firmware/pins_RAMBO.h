#include "macros.h"

// RAMBO PIN ASSIGNMENTS

#define BOARD_NAME "Rambo"
#define ELECTRONICS "Rambo 1.4"

#ifndef KNOWN_BOARD
  #define KNOWN_BOARD
#endif
#define PINDA_THERMISTOR
//#define AMBIENT_THERMISTOR


// LIMIT SWITCHES
#define X_MIN_PIN   12
//#define X_MAX_PIN  24    //Used for Fil Runout on E0
#define X_MAX_PIN -1
#define Y_MIN_PIN   11
//#define Y_MAX_PIN  23    //Used for Fil Runout on E1
#define Y_MAX_PIN -1
#define Z_MIN_PIN    10
#define Z_MAX_PIN    30


//Fil Runout sensors
#define FIL_RUNOUT_PIN  24
#define FIL_RUNOUT2_PIN 23


//STEPPERS
#define X_STEP_PIN      37
#define X_DIR_PIN       48
#define X_ENABLE_PIN    29

#define Y_STEP_PIN      36
#define Y_DIR_PIN       49
#define Y_ENABLE_PIN    28

#define Z_STEP_PIN      35
#define Z_DIR_PIN       47
#define Z_ENABLE_PIN    27

#define E0_STEP_PIN     34
#define E0_DIR_PIN      43
#define E0_ENABLE_PIN   26

#define E1_STEP_PIN     33
#define E1_DIR_PIN      42
#define E1_ENABLE_PIN   25

//Microsteppping pins
#define X_MS1_PIN       40
#define X_MS2_PIN       41
#define Y_MS1_PIN       69
#define Y_MS2_PIN       39
#define Z_MS1_PIN       68
#define Z_MS2_PIN       67
#define E0_MS1_PIN      65
#define E0_MS2_PIN      66
#define E1_MS1_PIN      63
#define E1_MS2_PIN      64

#define DIGIPOTSS_PIN   38
#define DIGIPOT_CHANNELS  { 4,5,3,0,1 }

#define NUMTEMPPROBES 4
#define PROBE_PIN0 0
#define PROBE_PIN1 1
#define PROBE_PIN2 2
#define PROBE_PIN3 7


//Temp sensors
#define TEMP_0_PIN     0  //Analog input
#define TEMP_1_PIN     0  //Analog input
#define TEMP_BED_PIN   0  //Analog input
#define TEMP_PINDA_PIN 0  //Analog input
//0 1 2 7

// Heaters / Fans
#define HEATER_0_PIN   9
#define HEATER_1_PIN   7
#define HEATER_BED_PIN 3

// #ifndef FAN_PIN
//   #define FAN_PIN      6 //All fans were moved to offboard I2C Slave board
// #endif



#define E0_FAN_PIN -1
#define E1_FAN_PIN -1
#define E2_FAN_PIN -1


//LCD SCREEN
//Extension 1
                                      //1 - 5v vcc        
                                      //2 - Gnd         
#define LCD_PINS_D7     75            //3 - LCD_PIN_D7    PJ4
#define LCD_PINS_D6     74            //4 - LCD_PIN_D6    PJ7
#define LCD_PINS_D5     73            //5 - LCD_PIN_D5    PJ3
#define LCD_PINS_D4     72            //6 - LCD_PIN_D4    PJ2
#define LCD_PINS_RS     70            //7 - LCD_PIN_RS    PG4
#define LCD_PINS_ENABLE 71            //8 - LCD_PIN_EN    PG3
#define BTN_ENC         78            //9 -  BTN_ENC      PE2
#define BEEPER          79            //10 - BEEPER       PE6
//Extension 2
#define LED_PIN         13            //1 - PWR_LED       PH2
                                      //2 - Gnd           
#define KILL_PIN        80            //3 - KILL_PIN      PE7
#define SDCARDDETECT    81            //4 - SDCARDDETECT  PD4
#define MOSI_PIN        51            //5 - MOSI_PIN      PB2
#define BTN_EN2         77            //6 - BTN_EN2       PJ6
#define SDSS            53            //7 - SDSS          PB0
#define BTN_EN1         76            //8 - BTN_EN1       PJ5
// #define SCK_PIN         52            //9 - SCK_PIN       PB1
// #define MISO_PIN        50            //10 - MISO_PIN     PB3

#define SDPOWER -1



#define SWI2C_SDA  20
#define SWI2C_SCL  21

#define SUICIDE_PIN -1
