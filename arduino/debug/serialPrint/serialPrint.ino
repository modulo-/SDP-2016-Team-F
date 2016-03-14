#include "SDPArduino.h"
#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
  SDPsetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("test\n");
  if(Serial.available()){
    Serial.print(Serial.read());
  }
  delay(100);
}
