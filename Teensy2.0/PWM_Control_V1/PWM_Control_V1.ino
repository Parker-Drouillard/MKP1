#include <SPI.h>
#include <math.h>
#include "Teensy2_Pins.h"

double dutyCycle = 0.5;           // interval at which to blink (milliseconds)

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency


const long changeTime = 500;


int ledState = HIGH;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long changeMillis = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(E1BlowerFans_pin,OUTPUT);
  pinMode(E1AxialFan_pin,OUTPUT);
  pinMode(E2BlowerFans_pin,OUTPUT);
  pinMode(E2AxialFan_pin,OUTPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(solenoidDeploy,OUTPUT);
  pinMode(solenoidRetract,OUTPUT);
  pinMode(motorForwardEnable, OUTPUT);
  pinMode(motorReverseEnable, OUTPUT);


  homeSolenoid();
  Serial.begin(9600);
  Serial.print("PWM output begin.");
  digitalWrite(ledPin, ledState);
//  analogWrite(E1BlowerFans_pin, 255);
//  digitalWrite(E1BlowerFans_pin,ledState);
  digitalWrite(E1AxialFan_pin,ledState);
//  digitalWrite(E2BlowerFans_pin,ledState);
  digitalWrite(E2AxialFan_pin,ledState);
//  digitalWrite(motorForwardEnable,ledState);
}

void loop() {
  //Blinking light to show life
  blinkLED();
  
//  int tach0 = analogRead(tach0_pin); -- E1BF
//  int tach1 = analogRead(tach1_pin);
//  int tach2 = analogRead(tach2_pin); -- E2BF
//  int tach3 = analogRead(tach3_pin);
//  int tach4 = analogRead(tach4_pin);
//  int tach5 = analogRead(tach5_pin);

//  Serial.println(String(analogRead(tach0_pin)/4*60)+String(" : ")+String(analogRead(tach1_pin)/4*60)+String(" : ")+String(analogRead(tach2_pin)/4*60)+String(" : ")+String(analogRead(tach3_pin)/4*60)+String(" : ")+String(analogRead(tach4_pin)/4*60)+String(" : ")+String( analogRead(tach5_pin)/4*60));
//  Serial.println(analogRead(tach5_pin)/6*60);


//  digitalWrite(solenoidDeploy,HIGH);
//  delay(30);
//  digitalWrite(solenoidDeploy,LOW);
//  delay(1000);
//  digitalWrite(solenoidRetract,HIGH);
//  delay(30);
//  digitalWrite(solenoidRetract,LOW);
//  delay(1000);

//
//  if(currentMillis - changeMillis >= changeTime){
//    dutyCycle = (dutyCycle + 0.1);
//    if(dutyCycle > 1){
//      dutyCycle = dutyCycle - 1;
//    }
//    
//    changeMillis = currentMillis;
//    Serial.println(dutyCycle);
//  }
}

void homeSolenoid(){
  digitalWrite(solenoidDeploy,HIGH);
  delay(40);
  digitalWrite(solenoidDeploy,LOW);
  delay(100);
  digitalWrite(solenoidRetract,HIGH);
  delay(40);
  digitalWrite(solenoidRetract,LOW);
  delay(100);
}

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
  }
}

