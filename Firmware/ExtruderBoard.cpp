#include "pins_RAMBO.h"#// #include "fastio.h"
#include <SPI.h>
// #include "config.h"
// #include "Dcodes.h"
#include "Marlin.h"
// #include "Configuration.h"
// #include "language.h"
// #include "cmdqueue.h"
// #include "swspi.h"
// #include "eeprom.h"



byte transferAndWait (const byte buf){
    byte a = SPI.transfer(buf);
    delayMicroseconds(20);
    return a;
} // end of transferAndWait


void eBoard_init(void) {
    printf_P(PSTR("EBoard Init\n"));
    SET_OUTPUT(EBOARD_SS);
    digitalWrite(EBOARD_SS, HIGH); //slave is selected when pin low
    // SPI.setClockDivider(SPI_CLOCK_DIV8);

    // digitalWrite(EBOARD_SS, LOW); //slave is selected when pin low
    // transferAndWait('h');
    // transferAndWait('t');

    // digitalWrite(EBOARD_SS, HIGH);
}

void turnOnFans(){
    digitalWrite(EBOARD_SS, LOW);
    transferAndWait('o');

    digitalWrite(EBOARD_SS, HIGH); //slave is selected when pin low
}



void extruderBoardTest(void) {
    printf_P(PSTR("Running EBoardTest"));
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

	printf_P(PSTR("Adding results:\n %d\n %d\n %d\n %d\n"), a, b, c, d);

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

	printf_P(PSTR("Subtracting results:\n %d\n %d\n %d\n %d\n"), a, b, c, d);
}

void sendText (const char * toSend){
    digitalWrite(EBOARD_SS, LOW);

    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    for (char c; c = *toSend; toSend++){
        delayMicroseconds(20);
        SPI.transfer(c);
        printf_P(c);
    }
    SPI.endTransaction();
    digitalWrite(EBOARD_SS, HIGH); //slave is selected when pin low
}