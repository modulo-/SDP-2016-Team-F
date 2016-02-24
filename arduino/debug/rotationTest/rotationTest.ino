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
#define FULL_ROT_LR 180.0
#define SERIAL_DEBUG

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
    //degrees of rotation of the wheel to rotation of the robot
    //because 1 reported 2 degree is 2 degrees of rotation
    float degToDegLR = (360/FULL_ROT_LR)* ((WHEEL_CIRCUM)/(LR_WHEELBASE*PI));
    float degToDegC =  (360/FULL_ROT_MIDDLE)* ((WHEEL_CIRCUM)/(2*MIDDLE_RADIUS*PI));
#ifdef SERIAL_DEBUG
    Serial.print("degToDegLR:");
    Serial.print(degToDegLR);
    Serial.print(" degToDegC:");
    Serial.println(degToDegC);
#endif
    //target speed in deg/s
    int targetRotV = 10;
    //everything is confusing because the motors are mounted backwards
    degrees = abs(degrees);
    if (degrees > 360)degrees = degrees % 360;
    updateMotorPositions();
    resetMotorPositions();
    updateMotorPositions();
    printMotorPositions();
    int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT], positions[ENCODER_MIDDLE]};
    int prevPositions[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT], positions[ENCODER_MIDDLE]};
    float rotVs[] = {0,0,0};
#ifdef SERIAL_DEBUG
    Serial.write(cw ? "clockwise " : "anticlockwise ");
    Serial.println(degrees);
#endif
    float deltaL=0;
    float deltaR=0;
    float deltaC=0;
    int powL=50;
    int powR=50;
    int powC=70;
    int prevTime[] ={0,0,0};
#ifdef SERIAL_DEBUG
    int prevPrint=millis();
    bool timeout;
#endif
    while (!finished && !Serial.available()) {
#ifdef SERIAL_DEBUG
        timeout=false;
        if(millis()-prevPrint>100){
            timeout=true;
            prevPrint=millis();
        }
#endif
        updateMotorPositions();
        deltaL = -(positions[ENCODER_LEFT]-start[0])*degToDegLR;
        deltaR =  (positions[ENCODER_RIGHT]-start[1])*degToDegLR;
        deltaC = -(positions[ENCODER_MIDDLE]-start[2])*degToDegC;
        if (!cw) {
            deltaL= -deltaL;
            deltaR= -deltaR;
            deltaC= -deltaC;
        }
        if ((deltaL+deltaR+deltaC)/3 >= degrees) {
            finished=true; 
        }
        else{
            if(abs(positions[ENCODER_LEFT]-prevPositions[0])>4||millis()-prevTime[0]>100){
                float rotV = (1000*(positions[ENCODER_LEFT]-prevPositions[0])*degToDegLR)/((millis()-prevTime[0]));
                rotV=abs(rotV);
                if(rotV>200)Serial.print("Anomylous rotV on motor L: ");
                if(rotV>targetRotV&&powL>40){
                    powL-=1;
                }
                else if(rotV<targetRotV){
                    if(powL>=100){
                        if(targetRotV>10)targetRotV--;
                    }
                    else{
                        powL +=1;
                    }
                }
                #ifdef SERIAL_DEBUG
                    /*
                   Serial.print("rotV L:"); 
                   Serial.print(rotV); 
                   Serial.print(" deltaT"); 
                   Serial.println(millis()-prevTime[0]); 
                    */
                #endif
                prevTime[0]=millis();
                prevPositions[0]=positions[ENCODER_LEFT];
                rotVs[0]=rotV;
            }
            if(abs(positions[ENCODER_RIGHT]-prevPositions[1])>4||millis()-prevTime[1]>100){
                float rotV = (1000*(positions[ENCODER_RIGHT]-prevPositions[1])*degToDegLR)/((millis()-prevTime[1]));
                rotV=abs(rotV);
                if(rotV>200)Serial.println("Anomylous rotV on motor R: ");
                if(rotV>targetRotV&&powR>40){
                    powR-=1;
                }
                else if(rotV<targetRotV){
                    if(powR>=100){
                        if(targetRotV>10)targetRotV--;
                    }
                    else{
                        powR +=1;
                    }
                }
                #ifdef SERIAL_DEBUG
                    /*
                   Serial.print("rotV R:"); 
                   Serial.print(rotV); 
                   Serial.print(" deltaT"); 
                   Serial.println(millis()-prevTime[1]); 
                    */
                #endif
                prevTime[1]=millis();
                prevPositions[1]=positions[ENCODER_RIGHT];
                rotVs[1]=rotV;
            }
            if(abs(positions[ENCODER_MIDDLE]-prevPositions[2])>2||millis()-prevTime[2]>100){
                float rotV = (1000*(positions[ENCODER_MIDDLE]-prevPositions[2])*degToDegC)/((millis()-prevTime[2]));
                rotV=abs(rotV);
                if(rotV>200)Serial.print("Anomylous rotV on motor C: ");
                if(rotV>targetRotV&&powC>40){
                    powC-=1;
                }
                else if(rotV<targetRotV){
                    if(powC>=100){
                        if(targetRotV>10)targetRotV--;
                    }
                    else{
                        powC +=1;
                    }
                }
                #ifdef SERIAL_DEBUG
                   /*
                   Serial.print("rotV C:"); 
                   Serial.print(rotV); 
                   Serial.print(" deltaT"); 
                   Serial.println(millis()-prevTime[2]); 
                   */
                #endif
                prevTime[2]=millis();
                prevPositions[2]=positions[ENCODER_MIDDLE];
                rotVs[2]=rotV;
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
    }
    motorAllStop();
    updateMotorPositions();
    #ifdef SERIAL_DEBUG 
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
        Serial.print(" powR:");
        Serial.print(powR);
        Serial.print(" powC:");
        Serial.print(powC);
        Serial.print("\ntargetRotV:");
        Serial.print(targetRotV);
        Serial.print("\nrotVs:");
        Serial.print(rotVs[0]);
        Serial.print(" ");
        Serial.print(rotVs[1]);
        Serial.print(" ");
        Serial.print(rotVs[2]);
        Serial.print("\n"); 
        printMotorPositions();
    #endif
}


//Don't ask why, I don't know 
void turnCalibrate(){
    int deg=90;
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
                Serial.print("Anti-Clockwise\n");
                break;
            case 'C':
                adjustCw=true;
                Serial.print("Clockwise\n");
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
                Serial.print(deg);
                Serial.write("\r\n");
                if(!adjustCw)deg=-deg;
                turn(deg);
                adjustCw=!adjustCw;
                break;
        }
        deg+=delta;
        if(delta !=0){
            Serial.print("degrees:");
            Serial.print(deg);
            Serial.write("\r\n");
        }
        ser=Serial.read();
    }
}

void loop(){
    turnCalibrate();	
}

