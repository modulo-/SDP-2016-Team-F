#include "SDPArduino.h"
#include <Wire.h>
/*
   Joel Hutton
   this script is for testing how to rotate the robot by a given angle
 */


/*
   Motor mapping

   0 - Left Reversed
   1 - Right Reversed
   2 - Middle to left
   3 - kicker
   4 - flippers
   5 - unnasigned
 */
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200
// Motor mapping
#define MOTOR_LEFT 2 // polarity reversed
#define MOTOR_RIGHT 1 // polarity reversed
#define MOTOR_MIDDLE 0 // forward is anticlockwise
#define MOTOR_KICKER 5 // forward is kick
#define MOTOR_GRABBER 3 // forward is grab

#define ENCODER_LEFT 3
#define ENCODER_RIGHT 4
#define ENCODER_MIDDLE 5 //+ve is clockwise

//the radius of the left and right wheels
#define LR_WHEELBASE 0.15
//the radius of the middle wheel
#define MIDDLE_RADIUS 0.11
#define WHEEL_DIAM 0.05
#define PI 3.14159
#define WHEEL_CIRCUM WHEEL_DIAM*PI
//the number of steps in a rotation for the centre wheel
#define FULL_ROT_MIDDLE 25.0
#define SERIAL_DEBUG 1

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};

void setup() {
    digitalWrite(8, HIGH);  // Radio on
    Serial.begin(115200);  // Serial at given baudrate
    Wire.begin();  // Master of the I2C bus
    SDPsetup();
    helloWorld();
}

void resetMotorPositions() {
    for (int i = 0; i < ROTARY_COUNT; i++) {
        positions[i] = 0;
    }
}
void updateMotorPositions() {
    // Request motor position deltas from rotary slave board
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);

    // Update the recorded motor positions
    for (int i = 0; i < ROTARY_COUNT; i++) {
        positions[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
    }
}

void printMotorPositions() {
    Serial.print("Motor positions: ");
    for (int i = 0; i < ROTARY_COUNT; i++) {
        Serial.print(positions[i]);
        Serial.print(' ');
    }
    Serial.println();
    delay(PRINT_DELAY);  // Delay to avoid flooding serial out
}

void turn(long degrees) {
    bool cw = degrees > 0;
    bool finished = false;
    //everything is confusing because the motors are mounted backwards
    degrees = abs(degrees);
    if (degrees > 360)degrees = degrees % 360;
    resetMotorPositions();
    updateMotorPositions();
    int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT], positions[ENCODER_MIDDLE]};
    printMotorPositions();
    if (SERIAL_DEBUG) {
        Serial.write(cw ? "clockwise " : "anticlockwise ");
        Serial.println(degrees);
    }
    float deltaL=0;
    float deltaR=0;
    float deltaC=0;
    int powL=80;
    int powR=80;
    int powC=100;
    int prevTime = millis();
    bool timeout;
    while (!finished) {
        updateMotorPositions();
        deltaL = -360.0*(((positions[ENCODER_LEFT]-start[0])/180.0)*WHEEL_CIRCUM)/(LR_WHEELBASE*PI);
        deltaR = 360.0*(((positions[ENCODER_RIGHT]-start[1])/180.0)*WHEEL_CIRCUM)/(LR_WHEELBASE*PI);
        deltaC = -360.0*(((positions[ENCODER_MIDDLE]-start[2])/FULL_ROT_MIDDLE)*WHEEL_CIRCUM)/(MIDDLE_RADIUS*PI*2);
        if (!cw) {
            deltaL= -deltaL;
            deltaR= -deltaR;
            deltaC= -deltaC;
        }
        if (deltaL >= degrees) {
            finished=true; 
        }
        if(deltaR >= degrees) {
            finished=true;
        }
        if(deltaC >= degrees) {
            finished=true;
        }
        else{
            if(deltaL - deltaR > 5) {
                if(powL>=50)powL--;
                if(powR<=100)powR++;
            }
            else if(deltaR-deltaL > 5) {
                if(powR>=50)powR--;
                if(powL<=50)powL++;
            }
            if(deltaC-((deltaR+deltaL)/2) > 5){
                if(powC>=50)powC--;
                if(powR<=100)powR++;
                if(powL<=100)powL++;
            }
            else if(((deltaR+deltaL)/2)-deltaC > 5){
                if(powC<=100)powC++;
                if(powR>=50)powR--;
                if(powL>=50)powL--;
            }
        }
        if (cw) {
            motorBackward(MOTOR_LEFT, powL);
            motorForward(MOTOR_RIGHT, powR);
            motorBackward(MOTOR_MIDDLE, powC);
        }
        else {
            motorForward(MOTOR_LEFT, powL);
            motorBackward(MOTOR_RIGHT, powR);
            motorForward(MOTOR_MIDDLE, powC);
        }
        delay(10);
    }
    motorAllStop();
    updateMotorPositions();
    if(SERIAL_DEBUG){
        Serial.print("degrees:");
        Serial.print(degrees);
        Serial.print("\ndeltaL:");
        Serial.print(deltaL);
        Serial.print("\ndeltaR:");
        Serial.print(deltaR);
        Serial.print("\ndeltaC:");
        Serial.print(deltaC);
        Serial.print("\npowL:");
        Serial.print(powL);
        Serial.print("\npowR:");
        Serial.print(powR);
        Serial.print("\npowC:");
        Serial.print(powC);
        Serial.print("\n"); 
        printMotorPositions();
    }
}


//Don't ask why, I don't know 
void turnCalibrate(){
    int degrees=90;
    Serial.write("turning calibration:\r\n");
    char ser=Serial.read();
    //quit signal
    int delta;
    bool adjustCw=true;
    while(ser!='q'){
        delta=0;
        switch(ser){
            //Roman Numerals because I'm a horrible back
            case 'A':
                adjustCw=false;
                break;
            case 'C':
                adjustCw=true;
                break;
            case 'I':
                delta=1;
                break;
            case 'V':
                delta=5;
                break;
            case 'X':
                delta=10;
                break;
            case 'L':
                delta=50;
                break;
            case 'i':
                delta=-1;
                break;

            case 'v':
                delta=-5;
                break;
            case 'x':
                delta=-10;
                break;
            case 'l':
                delta=-50;
                break;
            case 'k':
                Serial.write("Turning for:");
                Serial.print(degrees);
                Serial.write("\r\n");
                if(!adjustCw)degrees=-degrees;
                turn(degrees);
                adjustCw=!adjustCw;
                break;
        }
        degrees+=delta;
        if(delta !=0){
            Serial.print("degrees:");
            Serial.print(degrees);
            Serial.write("\r\n");
        }
        ser=Serial.read();
    }
}

void loop(){
    turnCalibrate();	
}

