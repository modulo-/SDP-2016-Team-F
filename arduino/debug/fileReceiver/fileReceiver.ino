#include "SDPArduino.h"
#include <Wire.h>

unsigned long oldtime;

void setup () {
  SDPsetup();
  Wire.begin(0x45);                // join i2c bus with address #0x45
  Wire.onReceive(receiveByte); // register event
  Serial.begin(115200);
  Serial.println("Receiver up");
  oldtime = millis();
}


void loop () {
  delay(10);
}

void receiveByte (int howMany) {
  unsigned long newtime = millis();
  byte data = Wire.read();
  Serial.print("Data: ");
  Serial.print((int)data);
  Serial.print(" Delay: ");
  Serial.println(newtime-oldtime);
  oldtime = newtime;
}

