#include "SDPArduino.h"
#include "Comms.h"
#include <Wire.h>
int i = 0;


/*
Motor mapping

 0 - Left Reversed
 1 - Right Reversed
 2 - Middle to left
 3 - kicker
 4 - flippers
 5 - free
 */

void setup() {
  SDPsetup();
  helloWorld();
  commsSetup();
}

void loop() {
  react();
}

void react() {
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
    case 'k':
      Serial.write("flippers open\r\n");
      motorForward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //flippers close
    case 'l':
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

    //kicker down
    case 'h':
      Serial.write("kicker down\r\n");
      motorForward(3, 100);
      delay(300);
      motorStop(3);
      break;

    // kick
    case 'y':
      Serial.write("kick\r\n");
      motorBackward(3, 100);
      delay(300);
      motorForward(3, 100);
      delay(100);
      motorStop(3);
      break;
  }
  }
