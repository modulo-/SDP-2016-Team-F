#include "SDPArduino.h"
#include <Wire.h>

unsigned long oldtime;

void setup () {
  SDPsetup();
  Wire.begin(0x45);                // join i2c bus with address #0x45
  Wire.onReceive(receiveByte); // register event
  Serial.begin(115200);
  oldtime = millis();
}


void loop () {
  delay(10);
}

void receiveByte (int howMany) {
  unsigned long newtime = millis();
  char data = Wire.read();
  Serial.print(data + " ");
  Serial.println(newtime-oldtime);
  oldtime = newtime;
}

