#ifndef RAMBO_H
#define RAMBO_H

#include "macros.h"

// RAMBO PIN ASSIGNMENTS

#define BOARD_NAME "Rambo"
#define ELECTRONICS "Rambo 1.4"
#define __AVR_ATmega2560__

#ifndef KNOWN_BOARD
  #define KNOWN_BOARD
#endif
#define PINDA_THERMISTOR
//#define AMBIENT_THERMISTOR
#define SDSUPPORT

// LIMIT SWITCHES
#define X_MIN_PIN   12
// #define X_MAX_PIN  24    //Used for Fil Runout on E0
#define X_MAX_PIN -1
#define Y_MIN_PIN   11
// #define Y_MAX_PIN  23    //Used for Fil Runout on E1
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
#define DIGIPOT_CHANNELS { 4,5,3,0,1 } // X Y Z E0 E1 digipot channels to stepper driver mapping
// #ifndef DIGIPOT_MOTOR_CURRENT
//   #define DIGIPOT_MOTOR_CURRENT { 135,135,135,135,135 } // Values 0-255 (RAMBO 135 =~ 0.75A, 185 =~ 1A)
//   #define DIGIPOT_MOTOR_CURRENT_LOUD {135,135,135,135,135}
// #endif



//Temp sensors
#define TEMP_0_PIN     0  //Analog input
#define TEMP_1_PIN     1  //Analog input
#define TEMP_BED_PIN   2  //Analog input
#define TEMP_PINDA_PIN 7  //Analog input

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
//------------------------------------// 1 - 5v vcc        
//------------------------------------// 2 - Gnd         
#define LCD_PINS_D7     75            // 3 - LCD_PIN_D7    PJ4
#define LCD_PINS_D6     74            // 4 - LCD_PIN_D6    PJ7
#define LCD_PINS_D5     73            // 5 - LCD_PIN_D5    PJ3
#define LCD_PINS_D4     72            // 6 - LCD_PIN_D4    PJ2
#define LCD_PINS_RS     70            // 7 - LCD_PIN_RS    PG4
#define LCD_PINS_ENABLE 71            // 8 - LCD_PIN_EN    PG3
#define BTN_ENC         78            // 9 -  BTN_ENC      PE2
#define BEEPER          79            //10 - BEEPER       PE6
//Extension 2
#define LED_PIN         13            // 1 - PWR_LED       PH2
//------------------------------------// 2 - Gnd           
#define KILL_PIN        80            // 3 - KILL_PIN      PE7
#define SDCARDDETECT    81            // 4 - SDCARDDETECT  PD4
#define MOSI_PIN        51            // 5 - MOSI_PIN      PB2
#define BTN_EN2         77            // 6 - BTN_EN2       PJ6
#define SDSS            53            // 7 - SDSS          PB0
#define BTN_EN1         76            // 8 - BTN_EN1       PJ5
// #define SCK_PIN         52            // 9 - SCK_PIN       PB1 - Already declared elsewhere
#define MISO_PIN        50            //10 - MISO_PIN     PB3
// #define EBOARD_SS       53           //Extruder Breakout Board Slave Select

#define SDPOWER  -1

#define SWI2C_SDA  20
#define SWI2C_SCL  21

#define SUICIDE_PIN -1


//LIST OF RAMBO 1.4 PINS AND THEIR ASSIGNMENTS FOR REFERENCE

//  #   Pin Name                Arduino Pin Name              Current Use
//  ======================================================================================
//  1 - PG5 (OC0B)            - Digital Pin 4 (PWM)         - Unused
//  2 - PE0 (RXD0/PCINT8)     - Digital Pin 0 (PWM) (RX0)   - USB Serial Comms, SER0 3
//  3 - PE1 (TXD0)            - Digital Pin 1 (PWM) (TX0)   - USB Serial Comms, SER0 4
//  4 - PE2 (XCK0/AIN0)       - RamboDigital Pin 78         - Unused
//  5 - PE3 (OC3A/AIN1)       - Digital Pin 5 (PWM)         - Unused
//  6 - PE4 (OC3B/INT4)       - Digital Pin 2 (PWM)         - Unused
//  7 - PE5 (OC3C/INT5)       - Digital Pin 3 (PWM)         - Bed Heater
//  8 - PE6 (T3/INT6)         - RamboDigital Pin 79         - Unused
//  9 - PE7 (T3/INT6)         - RamboDigital Pin 80         - Unused
// 10 - VCC                   - VCC                         - VCC
// 11 - GND                   - GND                         - GND
// 12 - PH0 (RXD2)            - Digital Pin 17 (PWM)(RX2)   - Serial 7
// 13 - PH1 (TXD2)            - Digital Pin 16 (PWM)(TX2)   - Serial 8
// 14 - PH2 (XCK2)            - Rambo Digital Pin 84        - Unused
// 15 - PH3 (OC4A)            - Digital Pin 6 (PWM)         - Unused
// 16 - PH4 (OC4B)            - Digital Pin 7 (PWM)         - Heat 1
// 17 - PH5 (OC4C)            - Digital Pin 8 (PWM)         - Unused
// 18 - PH6 (OC2B)            - Digital Pin 9 (PWM)         - Heat 0
// 19 - PB0 (SS/PCINT0)       - Digital Pin 53 (PWM)(SPI-SS)- SPI-Ext SS 6
// 20 - PB1 (SCK/PCINT1)      - Digital Pin 52 (PWM)(SCK)   - SPI-Ext SCK 5
// 21 - PB2 (MOSI/PCINT2)     - Digital Pin 51 (PWM)(MOSI)  - SPI-Ext MOSI 4
// 22 - PB3 (MISO/PCINT3)     - Digital Pin 50 (MISO)       - SPI-Ext MISO 3
// 23 - PB4 (OC2A/PCINT4)     - Digital Pin 10 (PWM)        - Z Min Endstop (Pinda Probe)
// 24 - PB5 (OC1A/PCINT5)     - Digital Pin 11 (PWM)        - Y Min Endstop
// 25 - PB6 (OC1B/PCINT6)     - Digital Pin 12 (PWM)        - X Min Endstop
// 26 - PB7 (OC0A/OC1C/PCINT7)- Digital Pin 13 (PWM)        - LED, PWM-Ext 3
// 27 - PH7 (T4)              - RamboDigital Pin 85         - Ext2 6
// 28 - PG3 (TOSC2)           - RamboDigital Pin 71         - Ext3 7
// 29 - PG4 (TOSC1)           - RamboDigital Pin 70         - Ext3 5
// 30 - RESET                 - RESET                       - RESET
// 31 - VCC                   - VCC                         - VCC
// 32 - GND                   - GND                         - GND
// 33 - XTAL2                 - XTAL2                       - XTAL2
// 34 - XTAL1                 - XTAL1                       - XTAL1
// 35 - PL0 (ICP4)            - Digital Pin 49              - Y Direction
// 36 - PL1 (ICP5)            - Digital Pin 48              - X Direction
// 37 - PL2 (T5)              - Digital Pin 47              - Z Direction
// 38 - PL3 (OC5A)            - Digital Pin 46 (PWM)        - MX3-5 Direction
// 39 - PL4 (OC5B)            - Digital Pin 45 (PWM)        - MX2-5 Direction
// 40 - PL5 (OC5C)            - Digital Pin 44 (PWM)        - MX1-5 Direction
// 41 - PL6                   - Digital Pin 43              - E0 Direction
// 42 - PL7                   - Digital Pin 42              - E1 Direction
// 43 - PD0 (SCL/INT0)        - Digital Pin 21 (SCL)        - I2C SCL
// 44 - PD1 (SDA/INT1)        - Digital Pin 20 (SDA)        - I2C SDA
// 45 - PD2 (RXDI/INT1)       - Digital Pin 19 (RX1)        - Serial 5
// 46 - PD3 (TXD1/INT3)       - Digital Pin 18 (TX1)        - Serial 6
// 47 - PD4 (ICP1)            - RamboDigital Pin 81         - Ext2 14
// 48 - PD5 (XCK1)            - RamboDigital Pin 82         - EXT2 12
// 49 - PD6 (T1)              - RamboDigital Pin 83         - Ext2 10
// 50 - PD7 (T0)              - Digital Pin 38              - Digipot SS
// 51 - PG0 (WR)              - Digital Pin 41              - X Microstep2
// 52 - PG1 (RD)              - Digital Pin 40              - X Microstep1
// 53 - PC0 (A8)              - Digital Pin 37              - X Step
// 54 - PC1 (A9)              - Digital Pin 36              - Y Step
// 55 - PC2 (A10)             - Digital Pin 35              - Z Step
// 56 - PC3 (A11)             - Digital Pin 34              - E0 Step
// 57 - PC4 (A12)             - Digital Pin 33              - E1 Step
// 58 - PC5 (A13)             - Digital Pin 32              - MX1-4 Step
// 59 - PC6 (A14)             - Digital Pin 31              - MX2-4 Step
// 60 - PC7 (A15)             - Digital Pin 30              - Z Max, MX3-4 Step
// 61 - VCC                   - VCC                         - VCC
// 62 - GND                   - GND                         - GND
// 63 - PJ0 (RXD3/PCINT9)     - Digital Pin 15 (RX3)        - Serial 9
// 64 - PJ1 (TXD3/PCINT10)    - Digital Pin 14 (TX3)        - Serial 10
// 65 - PJ2 (XCK3/PCINT11)    - RamboDigital Pin 72         - Ext2 9
// 66 - PJ3 (PCINT 12)        - RamboDigital Pin 73         - Ext2 11
// 67 - PJ4 (PCInt 13)        - RamboDigital Pin 75         - Ext2 15
// 68 - PJ5 (PCInt 14)        - RamboDigital Pin 76         - Ext2 17
// 69 - PJ6 (PCInt 15)        - RamboDigital Pin 77         - Ext2 19
// 70 - PG2 (ALE)             - Digital Pin 39              - Y Microstep2
// 71 - PA7 (AD7)             - Digital Pin 29              - X Enable
// 72 - PA6 (AD6)             - Digital Pin 28              - Y Enable
// 73 - PA5 (AD5)             - Digital Pin 27              - Z Enable
// 74 - PA4 (AD4)             - Digital Pin 26              - E0 Enable
// 75 - PA3 (AD3)             - Digital Pin 25              - E1 Enable
// 76 - PA2 (AD2)             - Digital Pin 24              - X Max, MX3-3 Enable
// 77 - PA1 (AD1)             - Digital Pin 23              - Y Max, MX2-3 Enable
// 78 - PA0 (AD0)             - Digital Pin 22              - MX1-3 Enable
// 79 - PJ7                   - RamboDigital Pin 74         - Ext2 19
// 80 - VCC                   - VCC                         - VCC
// 81 - GND                   - GND                         - GND
// 82 - PK7 (ADC15/PCINT23)   - Analog Pin 15               - Y Microstep1
// 83 - PK6 (ADC14/PCINT22)   - Analog Pin 14               - Z Microstep1
// 84 - PK5 (ADC13/PCINT21)   - Analog Pin 13               - Z Microstep 2
// 85 - PK4 (ADC12/PCINT20)   - Analog Pin 12               - E0 Microstep2
// 86 - PK3 (ADC11/PCINT19)   - Analog Pin 11               - E0 Microstep1
// 87 - PK2 (ADC10/PCINT18)   - Analog Pin 10               - E1 Microstep2
// 88 - PK1 (ADC9/PCINT17)    - Analog Pin 9                - E1 Microstep1
// 89 - PK0 (ADC8/PCINT16)    - Analog Pin 8                - Analog-Ext 1
// 90 - PF7 (ADC7/PCINT15)    - Analog Pin 7                - Thermistor 3, Analog-Ext 5
// 91 - PF6 (ADC6/PCINT14)    - Analog Pin 6                - Analog-Ext 3
// 92 - PF5 (ADC5/TMS)        - Analog Pin 5                - Analog-Ext 8
// 93 - PF4 (ADC4/TMK)        - Analog Pin 4                - Analog-Ext 6
// 94 - PF3 (ADC3)            - Analog Pin 3                - Analog-Ext 4
// 95 - PF2 (ADC2)            - Analog Pin 2                - Thermistor 2
// 96 - PF1 (ADC1)            - Analog Pin 1                - Thermistor 1
// 97 - PF0 (ADC0)            - Analog Pin 0                - Thermistor 0
// 98 - AREF                  - Analog Reference
// 99 - GND                   - GND                         - GND
//100 - AVCC                  - VCC                         -VCC


#endif