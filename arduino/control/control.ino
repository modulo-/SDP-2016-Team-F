#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"
//#define COMPASS
#ifdef COMPASS
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <avr/wdt.h>
#endif


// Motor mapping
#define MOTOR_LEFT 0 // polarity reversed
#define MOTOR_RIGHT 4 // polarity reversed
#define MOTOR_MIDDLE 0 // forward is anticlockwise
#define MOTOR_GRABBER 1 // forward is grab

#define ENCODER_LEFT 3
#define ENCODER_RIGHT 4
#define ENCODER_GRABBER 5


#define PIN_KICKER 6

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
//#define SERIAL_DEBUG

// CMD opcodes
#define CMD_MOVE 'm'
#define CMD_MOVEANDTURN 'M'
#define CMD_TURN 't'
#define CMD_KICK 'k'
#define CMD_DATA 'd'
#define CMD_GRAB 'g'
#define CMD_RELEASE 'r'
#define CMD_CELEBRATE 'c'
#define CMD_PING 'p'
#define CMD_SET_DEBUG 'D'
#define CMD_TEST 'T'
#define OPTIONS 'o'

#define MAX_DATA_SIZE 255
#define DEVICE_ID '2'
#define TURN_ALLOWANCE 3

//motor positioning
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200

#define DETAILED 3
#define WARN     2
#define ERROR    1
#define FATAL    0


int debugLevel=0;

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};
bool turnCorrection = false;
int kickerTime50 = 6;
int kickerTime100 = 7;
int kickerTime150 = 8;
long degToMetre = 1250;
#ifdef COMPASS
Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);
#endif

int targetHeading;

void doMoveAndTurn(byte * message);
void doTurn(byte * message);
void doKick(byte * message);
void doData(byte * message);


byte * dataBytes = (byte *) malloc(MAX_DATA_SIZE * sizeof(byte));
int dataFreq = 2;
int dataLen = 100;

void setup() {
    SDPsetup();
    Serial.flush();
    while (Serial.available()) {
        Serial.read();
    }

    // indicator led for comms system, on indicates error
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    if (!comms::init("67", "~~~")) {
        digitalWrite(13, HIGH);
    }
    pinMode(PIN_KICKER, OUTPUT);

    Wire.begin(ROTARY_SLAVE_ADDRESS);  // I2C slave at given address
    initSensors();

    //Initialise the WDT and flush the serial...
    //wdt_enable(WDTO_1S);
    calibrateGrabbers();
    Serial.flush();
}

void loop() {
    comms::poll();
}

namespace comms {
    const char DEVICEID = DEVICE_ID;

    void process(void const *data, size_t len) {
        byte * message = (byte *) data;

        switch (message[0]) {
            case CMD_MOVE:
                doMove(message);
                break;
            case CMD_MOVEANDTURN:
                doMoveAndTurn(message);
                break;
            case CMD_TURN:
                doTurn(message);
                break;
            case OPTIONS:
                parseOptions(message);
                break;
            case CMD_KICK:
                doKick(message);
                break;
            case CMD_DATA:
                doData(message);
                break;
            case CMD_GRAB: grab();
                           break;
            case CMD_RELEASE:
                           release();
                           break;
            case CMD_PING:
                           ping();
                           break;
            case CMD_SET_DEBUG:
                           setDebug(message);
                           break;
            case CMD_TEST:
                           test();
                           break;
        }
        //free(data);
    }
}

void parseOptions(byte* message) {
    char a = message[1];
    switch (a) {
        case 't':
            turnCorrection = message[2] >= 1;
            break;
        case 'k':
            switch (message[2]) {
                case 50:
                    kickerTime50 += message[3] - 127;
                    break;
                case 100:
                    kickerTime100 += message[3] - 127;
                    break;
                case 150:
                    kickerTime150 += message[3] - 127;
                    break;
            }
        case 'm':
            degToMetre += message[2] - 127;
        default:
            break;
    }
}

void grab() {
    updateMotorPositions();
    //close flippers
    int time=millis();
    motorForward(MOTOR_GRABBER, 55);
    delay(800);
    if(positions[ENCODER_GRABBER]<=8){
        commSend("BC");
    }
    else{
        commSend("NC");
    }
    motorAllStop();
}

void release() {
    //move flippers away
    motorBackward(MOTOR_GRABBER, 55);
    int time=millis();
    delay(800);
    motorAllStop();
}

void calibrateGrabbers() {
    //move flippers away
    motorBackward(MOTOR_GRABBER, 55);
    delay(800);
    positions[ENCODER_GRABBER]=0;
    motorForward(MOTOR_GRABBER, 55);
    delay(800);
    motorAllStop();
}

void ping(){
    updateMotorPositions();
    printMotorPositions();
}

void setDebug(byte * message){
    debugLevel = message[1];
}

void test(){

}

void doMove(byte * message) {
    int distance = (message[1] << 8) | message[2];
move(distance);
}

void doMoveAndTurn(byte * message) {
    int direction = (message[1] << 8) | message[2];
    int distance = (message[3] << 8) | message[4];
    int finalHeading = (message[5] << 8) | message[6]; // relative finish heading
    int startHeading = getCurrentHeading(); // absolute start heading
    finalHeading = (startHeading + finalHeading + 360) % 360; // absulute finish heading
    turn(direction, 0); // turn to calculated final abs heading
    move(distance); // move in relative heading
    delay(1000);
    turn(getHeadingDiff(finalHeading, getCurrentHeading()), 0); // turn to calculated final abs heading
}

void doTurn(byte * message) {
    int heading = (message[1] << 8) | message[2];

    int finalHeading = (getCurrentHeading() + heading + 360) % 360; // absolute finish heading

    turn(getHeadingDiff(finalHeading, getCurrentHeading()), 0); // turn to calculated final abs heading
}

void doKick(byte * message) {
    int distance = (message[1] << 8) | message[2];
    kick(distance);
}

void doData(byte * message) {
    int part =  message[1];
    if (part == 0) {
        dataFreq = message[2];
        dataLen = message[3];

        for (int i = 0; i < MAX_DATA_SIZE; i++) {
            dataBytes[i] = (byte) i; // fixme
        }
    } else if (part == 0xff) {
        sendData();
    } else {
        int firstByte = message[2];
        int chunkLen = message[3];
        byte * chunk = &message[4];
        memcpy(&dataBytes[firstByte], chunk, chunkLen);
    }

}

void sendData() {
    for (int i = 0; i < dataLen; i++) {
        byte data = dataBytes[i]; // get message from queue
        Wire.beginTransmission(0x45);
        Wire.write(data);
        Wire.endTransmission();
        delay(1000 / dataFreq);
    }
}



// move some distance in specified direction, ideally by changing heading minimally
void move(int distance) {
    if (Serial.available()) {
        return;
    };
    long max_data_points=100;
    long history[3][max_data_points];
    int number_data_points=0;
    resetMotorPositions();
    long distanceCovered = 0;
    //long degrees = (distance * degToMetre) / 1000;
    long degrees= distance;

    bool finished = false;
    bool forwards = (distance >= 0);
    //everything is confusing because the motors are mounted backwards
    if (forwards)degrees = -degrees;
    updateMotorPositions();
    int start[] = {positions[0], positions[1]};

    int delta0 = 0;
    int delta1 = 0;

    int targetDelta=0;    
    int targetSpeed=0;

    int leftPower = 100;
    int rightPower = 100;
    int prevTime = millis();
    bool timeout;

    while (!finished && !Serial.available()) {
        updateMotorPositions();
        if(number_data_points<max_data_points && (delta0!=-(positions[ENCODER_LEFT] - start[0]) || delta1!=-(positions[ENCODER_RIGHT] - start[1]))){
            history[0][number_data_points]=positions[ENCODER_LEFT]; 
            history[1][number_data_points]=positions[ENCODER_RIGHT]; 
            history[2][number_data_points]=micros(); 
            number_data_points++;
        }
        
        delta0 = -(positions[ENCODER_LEFT] - start[0]);
        delta1 = -(positions[ENCODER_RIGHT] - start[1]);
        if (!forwards) {
            delta0 = -delta0;
            delta1 = -delta1;
        }
        if (leftPower != 0 && abs(delta0) >= abs(degrees)) {
            leftPower = 0;
        }
        if (rightPower != 0 && abs(delta1) >= abs(degrees)) {
            rightPower = 0;
        }
        if ((abs(delta0) >= abs(degrees) && abs(delta1) >= abs(degrees))) {
            finished = true;
            break;
        }
        else if (delta0 - delta1 > 5) {
            if (leftPower > 90)leftPower = 60;
        }
        else if (delta1 - delta0 > 5) {
            if (rightPower > 90)rightPower = 60;
        }
        else {
            if (leftPower != 0)leftPower = 70;
            if (rightPower != 0)rightPower = 70;
        }
        if (forwards) {
            motorBackward(MOTOR_LEFT, leftPower);
            motorBackward(MOTOR_RIGHT, rightPower);
        }
        else {
            motorForward(MOTOR_LEFT, leftPower);
            motorForward(MOTOR_RIGHT, rightPower);
        }
    }
    debugSend("finished");
    motorAllStop();
    updateMotorPositions();
    for(int i=0;i<number_data_points;i++){
        for(int j=0;j<3;j++){
            String numStr=String(history[j][i]);
            char numChrArray[numStr.length()+1];
            for(int k=0;k<numStr.length();k++){
                numChrArray[k]=numStr.charAt(k);
            }
            numChrArray[numStr.length()+1]='\0';
            debugSend(numChrArray);
            delay(100);
            debugSend(", ");
            delay(100);
        }
        delay(100);
        debugSend("\n");
    }
}

//in a full rotation left should go 550 right should go 550
// turn a certain number of degrees
void turn(long degrees, int depth) {
    if (Serial.available()) {
        return;
    };
    //causes problems, don't know why
    //debugPrint("turning");
    //debugPrint((char*) (Serial.available() ? "true":"false"));
    bool cw = degrees > 0;
    bool finished = false;
    //degrees of rotation of the wheel to rotation of the robot
    //because 1 reported 2 degree is 2 degrees of rotation
    float degToDegLR = (360 / FULL_ROT_LR) * ((WHEEL_CIRCUM) / (LR_WHEELBASE * PI));
    float degToDegC =  (360 / FULL_ROT_MIDDLE) * ((WHEEL_CIRCUM) / (2 * MIDDLE_RADIUS * PI));
    //target speed in deg/s
    int targetRotV = 70;
    //everything is confusing because the motors are mounted backwards
    degrees = abs(degrees);
    if (degrees > 360)degrees = degrees % 360;
    updateMotorPositions();
    resetMotorPositions();
    updateMotorPositions();
    int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT]};
    int prevPositions[] = {0, 0, 0};
    float deltaL = 0;
    float deltaR = 0;
    int pows[] = {40, 40};
    int startTime = millis();
    int prevTime = 0;
    while (!finished && !Serial.available()) {
        updateMotorPositions();
        deltaL = -(positions[ENCODER_LEFT] - start[0]) * degToDegLR;
        deltaR =  (positions[ENCODER_RIGHT] - start[1]) * degToDegLR;
        int deltas[] = {deltaL, deltaR};
        if (!cw) {
            deltaL = -deltaL;
            deltaR = -deltaR;
        }
        int time = millis();
        if ((deltaL + deltaR) / 2 >= degrees) {
            finished = true;
        }
        else if (time - prevTime > 500) {
            int i = 0;
            int diffs[] = {deltaL - prevPositions[0], deltaR - prevPositions[1]};
            for (i = 0; i < 2; i++) {
                float rotV = 1000 * ((float)diffs[i] / (time - prevTime));
                if (rotV > targetRotV) {
                    pows[i] -= 5;
                }
                if (rotV < targetRotV) {
                    pows[i] += 5;
                }
            }
            prevPositions[0] = deltaL;
            prevPositions[1] = deltaR;
            prevTime = time;
        }
        if (cw) {
            motorBackward(MOTOR_LEFT, pows[0]);
            motorForward(MOTOR_RIGHT, pows[1]);
        }
        else {
            motorForward(MOTOR_LEFT, pows[0]);
            motorBackward(MOTOR_RIGHT, pows[1]);
        }
    }
    motorAllStop();
    updateMotorPositions();
}

void kick(int distance) { // distance in cm
    distance=abs(distance);
    int kickerTime = 0;
    int kickerStrength = 100;
    switch (distance) {
        case 50:
            kickerTime = kickerTime50;
            break;
        case 100:
            kickerTime = kickerTime100;
            break;
        case 150:
            kickerTime = kickerTime150;
            break;
        default:
            kickerTime = 6+ ((distance-50)/50);
            break;
    }
    //close flippers
    //motorForward(MOTOR_GRABBER, 20);
    //delay(800);
    //move flippers away
    //motorBackward(MOTOR_GRABBER, 25);
    //delay(800);
    //motorAllStop();
    //kick
    release();
    digitalWrite(PIN_KICKER, HIGH);

    delay(kickerTime);
    //put kicker back down
    digitalWrite(PIN_KICKER, LOW);
    //put grabbers back in
    //motorForward(MOTOR_GRABBER, 25);
    delay(200);
    grab();
    //motorAllStop();
}


void initSensors()
{
#ifdef COMPASS
    if (!accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
    }
    if (!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
    }
    /* Enable auto-ranging */
    gyro.enableAutoRange(true);

    /* Initialise the sensor */
    if (!gyro.begin())
    {
        /* There was a problem detecting the L3GD20 ... check your connections */
    }
#endif
}



int getBatteryVoltage() {
    int relativeVoltage = analogRead(2); // Input on A2.
    float absoluteVoltage = (5.0 * relativeVoltage) / 1023.0;
    /*
       Need to multiply by 2*10 since the potential divider on the board
       divided the voltage by 2, and we want a larger precision number.
       I.e. voltage will go from 0-100.
     */
    int responseVoltage = (int) absoluteVoltage * 20;
    return responseVoltage;
}

/*
getHeadingDiff:

Returns the relative difference between the target heading
and the current heading. Positive direction is
counter-clockwise, so a returned value of eg. 10 means that
the robot should turn 10 degrees left to be on target.
Similarly, a returned value of -20 would indicate that the
robot would have to turn 20 degrees right to be on target.
 */

int getHeadingDiff(int targetHeading, int currentHeading) {
    int diff = (targetHeading - currentHeading + 360) % 360;
    if (diff >= 180) {
        return -360 + diff;
    }
    return diff;
}

int getCurrentHeading() {
#ifdef COMPASS
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
    {
        /* 'orientation' should have valid .heading data now */
        return 360 - ((int) orientation.heading);
    }
#endif
#ifndef COMPASS
    return 0;
#endif
}



void updateMotorPositions() {
    // Request motor position deltas from rotary slave board
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
    int8_t values[ROTARY_COUNT];
    //check for error
    bool error=true;
    bool changed=false;
    for (int i = 0; i < ROTARY_COUNT; i++) {
        int8_t a =(int8_t) Wire.read();  // Must cast to signed 8-bit type
        values[i]=a;
        if(a!=-1){error=false;}
        if(a!=0){changed=true;}
    }
    if(error){
        debugSend("cannot communicate with encoder board");
    }
    else{
        // Update the recorded motor positions
        for (int i = 0; i < ROTARY_COUNT; i++) {
            positions[i] += values[i]; 
        }
    }
    if(changed && debugLevel >= DETAILED){
        printMotorPositions();
    }
}

void resetMotorPositions() {
    for (int i = 0; i < ROTARY_COUNT; i++) {
        //the grabber needs an absolute position
        if(i!=ENCODER_GRABBER){
            positions[i] = 0;
        }
    }
}

void printMotorPositions() {
    debugSend("Motor positions: ");
    String positionStr;
    for(int i=0; i<ROTARY_COUNT;i++){
        positionStr.concat(positions[i]);
        if(i!=ROTARY_COUNT-1){
            positionStr.concat(',');
        }
    }
    char positionString[positionStr.length()+1];
    for(int i=0;i<positionStr.length();i++){
        positionString[i]=positionStr.charAt(i);
    }
    positionString[positionStr.length()+1]='\0';
    debugSend(positionString);
}

void commSend(char* str) {
    comms::send(str, 'c', strlen(str));
}

void debugSend(char* str) { 
    comms::send(str, 'd', strlen(str));
}
