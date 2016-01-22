#include "SDPArduino.h"
#include "Comms.h"
#include <Wire.h>
int i = 0;


/*
Motor mapping
 0

 0 - Left motor (polarity reversed)
 1 - Right motor (polarity reversed)
 2 - Middle motor (left is 'forward')
 3 - kicker
 4 - flippers
 5 - nothing
 */

void setup() {
  SDPsetup();
  helloWorld();
  commsSetup();
}

void loop() {
  char input;
  input = Serial.read();
  switch (input) {
    case '8':
      Serial.write("forward\r\n");
      motorBackward(0, 100);
      motorBackward(1, 100);
      motorStop(2);
      break;

    case '2':
      Serial.write("backward\r\n");
      motorForward(0, 100);
      motorForward(1, 100);
      motorStop(2);
      break;

    case '5':
      Serial.write("stop\r\n");
      motorStop(0);
      motorStop(1);
      motorStop(2);
      break;

    case '6':
      Serial.write("right\r\n");
      motorForward(0, 0);
      motorBackward(1,65);
      motorBackward(2,100);
      break;

    case '4':
      Serial.write("left\r\n");
      motorBackward(0,65);
      motorForward(1, 0);
      motorForward(2, 100);
      break;

    case '9':
      Serial.write("forwards clockwise\r\n");
      motorBackward(0, 100);
      motorForward(1, 50);
      motorBackward(2, 100);
      break;

    case '7':
      Serial.write("forwards anticlockwise\r\n");
      motorForward(0, 50);
      motorBackward(1, 100);
      motorForward(2, 100);
      break;

    case '3':
      Serial.write("backwards clockwise\r\n");
      motorBackward(0, 50);
      motorForward(1, 100);
      motorBackward(0, 50);
      motorBackward(2, 100);
      break;

    case '1':
      Serial.write("backwards anticlockwise\r\n");
      motorForward(0, 100);
      motorBackward(1, 50);
      motorForward(2, 100);
      break;

    //flippers open
    case 'o':
      Serial.write("flippers open\r\n");
      motorForward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //flippers close
    case 'c':
      Serial.write("flippers close\r\n");
      motorBackward(5, 80);
      delay(300);
      motorStop(5);
      break;

//    //left flipper open
//    case 'a':
//      Serial.write("left flipper open\r\n");
//      motorForward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;
//
//    //left flipper close
//    case 's':
//      Serial.write("left flipper close\r\n");
//      motorBackward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;

    //kick
    case 'k':
      Serial.write("kicker down\r\n");
      motorForward(3, 100);
      delay(300);
      motorStop(3);
      break;

    // kicker down
    case 'j':
      Serial.write("kick\r\n");
      motorBackward(3, 30);
      delay(300);
      motorForward(3, 30);
      delay(100);
      motorStop(3);
      break;
  }
}

