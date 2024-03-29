#include <SPI.h>
#include <math.h>
#include "Teensy2_Pins.h"

#define NUMFANS 6

double dutyCycle = 0.5;           // interval at which to blink (milliseconds)

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const int fanDeathSampleCount = 50; //Sample count required to determine a fan is bad/dead
const long fanSampleInterval = 5; //Sample interval in ms to check fans
const long changeTime = 500;
const int fanPins[NUMFANS] = {E1AxialFan_pin, E1BlowerFanFront_pin, E1BlowerFanRear_pin, E2AxialFan_pin, E2BlowerFanFront_pin, E2BlowerFanRear_pin};
const int tachPins[NUMFANS] = {tach0_pin, tach1_pin, tach2_pin, tach3_pin, tach4_pin, tach5_pin};

int ledState = HIGH; // ledState used to set the LED
int fanStates[NUMFANS] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; //State of each fan, default ON.
int fanHealthSamples[NUMFANS] = {0, 0, 0, 0, 0, 0}; //Array used to count the number of bad samples from a fan to determine it is dead.
int fanHealth[NUMFANS] = {1, 1, 1, 1, 1, 1}; //Health of all fans. Default GOOD, if bad, set to 0 for error handling

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long changeMillis = 0;
unsigned long fanMillis = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////         SETUP                    /////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  for(int i = 0; i < NUMFANS; i++){
    pinMode(fanPins[i], OUTPUT);
  }
  pinMode(ledPin,OUTPUT);
  pinMode(solenoidForward,OUTPUT);
  pinMode(solenoidReverse,OUTPUT);
  pinMode(motorForwardEnable, OUTPUT);
  pinMode(motorReverseEnable, OUTPUT);
  Serial.begin(9600);
  Serial.print("PWM output begin.");
  digitalWrite(ledPin, ledState);
  updateAllFanStates(); //DigitalWrite all fan states

  delay(10);

  homeSolenoid();
  resetAllFanHealth();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////         MAIN ARDUINO LOOP         /////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //Blinking light to show life
  blinkLED();

  sampleHealthOfAllFans();

  updateAllFanStates(); //DigitalWrite all fan states
 }

 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////         FUNCTIONS                /////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//============ SOLENOID STUFF ==================================


//Routine to cycle solenoid state to return to a known state
//NOTE - Does not give any indication as to it's success while homing.
void homeSolenoid(){
  //Index: 
  //solenoidForward High && solenoidReverse LOW = Retract 
  //solenoidForward Low && solenoidReverse High = Extend
  //solenoidForward HIGH && solenoidReverse HIGH = neither/inactive
  //solenoidForward LOW && solenoidReverse LOW = neither/inactive

  digitalWrite(solenoidForward,HIGH);
  digitalWrite(solenoidReverse,LOW);
  delay(40);
  digitalWrite(solenoidForward,LOW);
  delay(100);
  digitalWrite(solenoidReverse,HIGH);
  digitalWrite(solenoidForward,LOW);
  delay(40);
  digitalWrite(solenoidReverse,LOW);
  delay(100);
}

//============ FAN STUFF ==================================

//Writes to each fan pin the current state determined by the global fanStates array.
void updateAllFanStates(){
  for(int i = 0; i < NUMFANS; i++){
    // if(fanHealth[i]){
      digitalWrite(fanPins[i],fanStates[i] > 0 ? HIGH : LOW);
    // } else {
      // digitalWrite(fanPins[i],0); //If fan is unhealthy, do not turn it on. Health must first be reset.
    // }
  }
}


// Function determines the health of all fans and stores their health in the global fanHealth array
void sampleHealthOfAllFans(){
  unsigned long currentMillis = millis();
  if(!(currentMillis - fanMillis >= fanSampleInterval)){
    return;
  }
  //If sample interval has been exceeded, sample fans
  fanMillis = currentMillis;

  //Update fanHealth count for all fans
  for(int i = 0; i < NUMFANS; i++){
    if(fanStates[i] && (analogRead(tachPins[i])/4*60 >= 10000 || analogRead(tachPins[i]/4*60 <4000))){ //If fan is on & fan is outside of nominal rpm range
      //TODO: Determine better range for RPM
      fanHealthSamples[i] = fanHealthSamples[i]+1;
    } else {
      fanHealthSamples[i] = 0; //Reset consecutive bad sample counter
    }
  }

  //Check if any fans have errored
  for(int i = 0; i < NUMFANS; i++){
    if(fanHealthSamples[i] >= fanDeathSampleCount && fanStates[i]){
      //If fan is unhealthy and the fan is on, set it to unhealthy and turn it off.
      fanHealth[i] = 0;
      fanStates[i] = -1; 
    } else if(fanStates[i]) {
      //If fan is on and fan is in good health, make sure it is indicated
      fanHealth[i] = 1;
    }
  }
}

void logHealthOfAllFans(){
  Serial.println();
  Serial.print("States: ");
  for(int i = 0; i < NUMFANS; i++){
    Serial.print(String(fanStates[i])+String(" : "));
  }
  Serial.print("Samples: ");
  for(int i = 0; i < NUMFANS; i++){
    Serial.print(String(fanHealthSamples[i]+String(" : ")));
  }
  Serial.println();
  for(int i = 0; i < NUMFANS; i++){
    Serial.print(String(analogRead(tachPins[i])/4*60)+String(" : "));
  }
}


//Function to reset all fan states if needed
void resetAllFanHealth(){
  for(int i = 0; i < NUMFANS; i++){
    fanHealth[i] = 0;
    fanHealthSamples[i] = 0;
  }
}

// Reset the health indicators for a specific fam
// Input: integer representing the fan to be reset (0 - 5)
// Output: 0 on nominal, Non 0 for everything else.
int resetHealthOfSpecificFan(int fanToReset){
  if(fanToReset >= NUMFANS || fanToReset < 0){
    return 1;
  }
  fanHealth[fanToReset] = 0;
  fanHealth[fanToReset] = 0;

  return 0;
}

//Checks the selected fan's health
//Returns 1 on healthy, 0 on unhealthy.
int checkFanHealth(int fanToCheck){
  switch(fanToCheck){
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      if(fanHealth[fanToCheck] >= fanDeathSampleCount && fanStates[fanToCheck]){
        return 0; //Fan is unhealthy
      } else {
        return 1; //Fan is healthy or off
      }
    break;
    default:
      return 0;
    break;
  }
}



//============ LED STUFF ==================================

//Life indicator LED blink
void blinkLED(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= changeTime){
    previousMillis = currentMillis;
    if(ledState == LOW){
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
    logHealthOfAllFans();
  }
}

