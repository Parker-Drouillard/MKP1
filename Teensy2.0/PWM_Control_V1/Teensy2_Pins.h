
//TEENSY 2.0 PINOUT
//             GND
//          SS   0   -- SPI SS  to RAMBO
//          SCK  1   -- SPI SCK to RAMBO
//          MOSI 2   -- SPI MOSI to RAMBO
//          MISO 3   -- SPI MISO to RAMBO
//          PWM  4
//  SCL PWM INT0 5   -- I2C Reserved
//  SDA     INT1 6   -- I2C Reserved  
//       RX INT2 7
//       TX INT3 8
//      PWM      9
//      PWM      10  -- Extruder1 Blower Fan PWM Control
//   (LED)  A10  11
//      PWM  A9  12  -- Extruder1 Axial Cooling Fan PWM Control
//           A8  13  -- TACH 4
//      PWM  A7  14  -- Extruder2 Blower Fan PWM Control
//      PWM  A6  15  -- Extruder2 Axial Cooling Fan PWM Control
//           A5  16  -- TACH 3
//           A4  17  -- TACH 2
//           A3  18  -- TACH 1
//           A2  19  -- TACH 0
//           A1  20  -- Solenoid Retract
//           A0  21  -- Solenoid Extend
//              +5V
//           A11 22  -- TACH 5
//               23  -- Motor Forward Enable
//   (interior)  24  -- Motor Reverse Enable



//Extruder Pins
const int E1BlowerFans_pin = 14;
const int E1AxialFan_pin = 15;
const int E2BlowerFans_pin = 10;
const int E2AxialFan_pin = 12;

//Solenoid Pins
const int solenoidDeploy = 21;
const int solenoidRetract = 20;

//Motor Pins
const int motorForwardEnable = 23;
const int motorReverseEnable = 24;

//LED Pins
const int ledPin = 11;

//SPI Pins
const int SS_pin = 0;
const int SCK_pin = 1;
const int MOSI_pin = 2;
const int MISO_pin = 3;

//Tachometers
const int tach0_pin = 19;  //Extruder 1 Axial Fan (Heatsink)
const int tach1_pin = 18;  //Extruder 1 Front Blower Fan
const int tach2_pin = 17;  //Extruder 1 Rear Blower Fan
const int tach3_pin = 16;  //Extruder 2 Axial Fan (Heatsink)
const int tach4_pin = 13;  //Extruder 2 Front Blower Fan
const int tach5_pin = 22;  //Extruder 2 Rear Blower Fan
