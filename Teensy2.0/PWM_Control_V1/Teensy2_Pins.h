
//TEENSY 2.0 PINOUT
//             GND
//          SS   0   -- SPI SS  to RAMBO
//          SCK  1   -- SPI SCK to RAMBO
//          MOSI 2   -- SPI MOSI to RAMBO
//          MISO 3   -- SPI MISO to RAMBO
//          PWM  4   -- Extruder ?? PWM
//  SCL PWM INT0 5   -- I2C Reserved
//  SDA     INT1 6   -- I2C Reserved  
//       RX INT2 7
//       TX INT3 8
//      PWM      9   -- Extruder ?? PWM
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

const int fanPWMPins[6] = {4,9,10,12,14,15};

//Solenoid Pins
const int solenoidForward = 21;
const int solenoidReverse = 20;

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

const int tachPins[6] = {19,18,17,16,13,22}; 
