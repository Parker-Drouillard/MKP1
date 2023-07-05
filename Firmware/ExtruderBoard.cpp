#include "pins_RAMBO.h"
#include "fastio.h"
#include <SPI.h>
#include "config.h"
#include "Dcodes.h"
#include "Marlin.h"
#include "Configuration.h"
#include "language.h"
#include "cmdqueue.h"
#include <EEPROM.h>


void eBoard_init(void) {
    SET_OUTPUT(EBOARD_SS);
    digitalWrite(EBOARD_SS, 1); //slave is selected when pin low
    // SPI.setClockDivider(SPI_CLOCK_DIV8);
}

byte transferAndWait (const byte buf){
    byte a = SPI.transfer(buf);
    delayMicroseconds(20);
    return a;
} // end of transferAndWait

void extruderBoardTest(void) {
    eBoard_init();

    byte a, b, c, d;

    //enable Slave Select
    digitalWrite(EBOARD_SS, LOW);
    transferAndWait ('a');  // add command
    transferAndWait (10);
    a = transferAndWait (17);
    b = transferAndWait (33);
    c = transferAndWait (42);
    d = transferAndWait (0);

    // disable Slave Select
    digitalWrite(EBOARD_SS, HIGH);

    // Serial.println ("Adding results:");
    // Serial.println (a, 10);
    // Serial.println (b, 10);
    // Serial.println (c, 10);
    // Serial.println (d, 10);

    // enable Slave Select
    digitalWrite(EBOARD_SS, LOW);   

    transferAndWait ('s');  // subtract command
    transferAndWait (10);
    a = transferAndWait (17);
    b = transferAndWait (33);
    c = transferAndWait (42);
    d = transferAndWait (0);

    // disable Slave Select
    digitalWrite(EBOARD_SS, HIGH);

    // Serial.println ("Subtracting results:");
    // Serial.println (a, 10);
    // Serial.println (b, 10);
    // Serial.println (c, 10);
    // Serial.println (d, 10);
}
