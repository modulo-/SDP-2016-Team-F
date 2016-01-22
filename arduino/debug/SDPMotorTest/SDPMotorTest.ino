#include "SDPArduino.h"
#include <Wire.h>
int i = 0;

void setup(){
  SDPsetup();
  helloWorld();
}

void loop(){
Serial.println("All Motors Forward 50%");
motorForward(0, 50);
motorForward(1, 50);
motorForward(2, 50);
motorForward(3, 50);
motorForward(4, 50);
motorForward(5, 50);
delay(2500);
Serial.println("All Motors Forwards 100%");
motorForward(0, 100);
motorForward(1, 100);
motorForward(2, 100);
motorForward(3, 100);
motorForward(4, 100);
motorForward(5, 100);
delay(2500);
Serial.println("All Motors Backwards 50%");
motorBackward(0, 50);
motorBackward(1, 50);
motorBackward(2, 50);
motorBackward(3, 50);
motorBackward(4, 50);
motorBackward(5, 50);
delay(2500);
Serial.println("All Motors Backwards 100%");
motorBackward(0, 100);
motorBackward(1, 100);
motorBackward(2, 100);
motorBackward(3, 100);
motorBackward(4, 100);
motorBackward(5, 100);
delay(2500);
Serial.println("All Motors Stop");
motorAllStop();
delay(2500);

Serial.println("Motor 0 Forward 50%");
motorForward(0, 50);
delay(2500);
Serial.println("Motor 0 Forwards 100%");
motorForward(0, 100);
delay(2500);
Serial.println("Motor 0 Backwards 50%");
motorBackward(0, 50);
delay(2500);
Serial.println("Motor 0 Backwards 100%");
motorBackward(0, 100);
delay(2500);
Serial.println("Motor 0 Stop");
motorStop(0);
delay(2500);

Serial.println("Motor 1 Forward 50%");
motorForward(1, 50);
delay(2500);
Serial.println("Motor 1 Forwards 100%");
motorForward(1, 100);
delay(2500);
Serial.println("Motor 1 Backwards 50%");
motorBackward(1, 50);
delay(2500);
Serial.println("Motor 1 Backwards 100%");
motorBackward(1, 100);
delay(2500);
Serial.println("Motor 1 Stop");
motorStop(1);
delay(2500);

Serial.println("Motor 2 Forward 50%");
motorForward(2, 50);
delay(2500);
Serial.println("Motor 2 Forwards 100%");
motorForward(2, 100);
delay(2500);
Serial.println("Motor 2 Backwards 50%");
motorBackward(2, 50);
delay(2500);
Serial.println("Motor 2 Backwards 100%");
motorBackward(2, 100);
delay(2500);
Serial.println("Motor 2 Stop");
motorStop(2);
delay(2500);

Serial.println("Motor 3 Forward 50%");
motorForward(3, 50);
delay(2500);
Serial.println("Motor 3 Forwards 100%");
motorForward(3, 100);
delay(2500);
Serial.println("Motor 3 Backwards 50%");
motorBackward(3, 50);
delay(2500);
Serial.println("Motor 3 Backwards 100%");
motorBackward(3, 100);
delay(2500);
Serial.println("Motor 3 Stop");
motorStop(3);
delay(2500);

Serial.println("Motor 4 Forward 50%");
motorForward(4, 50);
delay(2500);
Serial.println("Motor 4 Forwards 100%");
motorForward(4, 100);
delay(2500);
Serial.println("Motor 4 Backwards 50%");
motorBackward(4, 50);
delay(2500);
Serial.println("Motor 4 Backwards 100%");
motorBackward(4, 100);
delay(2500);
Serial.println("Motor 4 Stop");
motorStop(4);
delay(2500);

Serial.println("Motor 5 Forward 50%");
motorForward(5, 50);
delay(2500);
Serial.println("Motor 5 Forwards 100%");
motorForward(5, 100);
delay(2500);
Serial.println("Motor 5 Backwards 50%");
motorBackward(5, 50);
delay(2500);
Serial.println("Motor 5 Backwards 100%");
motorBackward(5, 100);
delay(2500);
Serial.println("Motor 5 Stop");
motorStop(5);
delay(2500);
}
