#include "SDPArduino.h"
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
}

void loop() {
  char input;
  input = Serial.read();
  switch (input) {
    case '8':
      Serial.write("forward\n");
      motorBackward(0, 100);
      motorBackward(1, 100);
      motorStop(2);
      break;

    case '2':
      Serial.write("backward\n");
      motorForward(0, 100);
      motorForward(1, 100);
      motorStop(2);
      break;

    case '5':
      Serial.write("stop\n");
      motorStop(0);
      motorStop(1);
      motorStop(2);
      break;

    case '6':
      Serial.write("right\n");
      motorForward(0, 0);
      motorBackward(1,85);
      motorBackward(2,100);
      break;

    case '4':
      Serial.write("left\n");
      motorBackward(0,85);
      motorForward(1, 0);
      motorForward(2, 100);
      break;

    case '9':
      Serial.write("forwards clockwise\n");
      motorBackward(0, 100);
      motorForward(1, 50);
      motorBackward(2, 100);
      break;

    case '7':
      Serial.write("forwards anticlockwise\n");
      motorForward(0, 50);
      motorBackward(1, 100);
      motorForward(2, 100);
      break;

    case '3':
      Serial.write("backwards clockwise\n");
      motorBackward(0, 50);
      motorForward(1, 100);
      motorBackward(0, 50);
      motorBackward(2, 100);
      break;

    case '1':
      Serial.write("backwards anticlockwise\n");
      motorForward(0, 100);
      motorBackward(1, 50);
      motorForward(2, 100);
      break;

    //flippers open
    case 'k':
      Serial.write("flippers open\n");
      motorForward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //flippers close
    case 'l':
      Serial.write("flippers close\n");
      motorBackward(5, 80);
      delay(300);
      motorStop(5);
      break;

//    //left flipper open
//    case 'a':
//      Serial.write("left flipper open\n");
//      motorForward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;
//
//    //left flipper close
//    case 's':
//      Serial.write("left flipper close\n");
//      motorBackward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;

    //kicker down
    case 'h':
      Serial.write("kicker down\n");
      motorForward(3, 100);
      delay(300);
      motorStop(3);
      break;

    // kick
    case 'y':
      Serial.write("kick\n");
      motorBackward(3, 100);
      delay(300);
      motorForward(3, 100);
      delay(100);
      motorStop(3);
      break;
  }
}

