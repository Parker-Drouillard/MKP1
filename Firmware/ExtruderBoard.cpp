


void eBoard_init(void) {
    SET_OUTPUT(EBOARD_SS);
    digitalWrite(EBoard_SS, HIGH); //slave is selected when pin low
}