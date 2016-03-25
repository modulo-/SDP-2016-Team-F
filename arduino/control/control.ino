#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"

// Motor mapping
#define MOTOR_LEFT 0 // polarity reversed
#define MOTOR_RIGHT 4 // polarity reversed
#define MOTOR_GRABBER 1 // forward is grab

#define ENCODER_LEFT 3
#define ENCODER_RIGHT 4
#define ENCODER_GRABBER 5

#define PIN_KICKER 6
#define PIN_TRIGGER 3
#define PIN_ECHO 17 


//the radius of the left and right wheels
#define LR_WHEELBASE 0.16
//the radius of the middle wheel
#define MIDDLE_RADIUS 0.11
#define WHEEL_DIAM 0.082
#define PI 3.14159
#define WHEEL_CIRCUM WHEEL_DIAM*PI
//the number of steps in a rotation for the centre wheel
#define FULL_ROT_MIDDLE 25.0
#define FULL_ROT_LR 180.0


#define ACCEL_DETAILS

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
#define CMD_TEST 'T'
#define OPTIONS 'o'

#define MAX_DATA_SIZE 255
#define DEVICE_ID '2'
#define TURN_ALLOWANCE 3

//motor positioning
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200


// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};
int ultraSoundDistance=1000;
int ultraSoundThreshold=150;
bool cautious=true;
long degToMetre = 600;
double degToDegLR = (360 / FULL_ROT_LR) * ((WHEEL_CIRCUM) / (LR_WHEELBASE * PI));

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
            case CMD_GRAB: 
                grab();
                break;
            case CMD_RELEASE:
                release();
                break;
            case CMD_PING:
                ping();
                break;
            case CMD_TEST:
                test();
                break;
            case CMD_CELEBRATE:
                celebrate();
                break;
        }
    }
}

void parseOptions(byte* message) {
    char a = message[1];
    switch (a) {
        case 'm':
            degToMetre += message[2] - 127;
            debugSendInt(degToMetre);
        case 'd':
            degToDegLR += message[2] - 127;
            debugSendInt(degToDegLR);
        case 'c':
            cautious= message[2]=1;
            if(cautious)debugSend("true");
            else debugSend("false");
        case 't':
                cautious= message[2]=1;
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
    motorBrake(MOTOR_GRABBER);
    updateMotorPositions();
    if(positions[ENCODER_GRABBER]<=8){
        commSend("BC");
    }
    else{
        commSend("NC");
    }
    debugSendInt(positions[ENCODER_GRABBER]);
}

void release() {
    int motorPower=55;
    updateMotorPositions();
    //move flippers away
    motorBackward(MOTOR_GRABBER, motorPower);
    int time=millis();
    //open
    while(millis()-time<800&&positions[ENCODER_GRABBER]>0){
        updateMotorPositions();
    } 
    motorPower=0;
    time=millis();
    while(millis()-time<800&&positions[ENCODER_GRABBER]==0){
        motorPower=millis()-time/10;
        motorForward(MOTOR_GRABBER, motorPower);
        updateMotorPositions();
    } 
    motorBrake(MOTOR_GRABBER);
    updateMotorPositions();
}

void calibrateGrabbers() {
    //move flippers away
    updateMotorPositions();
    motorBackward(MOTOR_GRABBER, 55);
    delay(800);
    updateMotorPositions();
    positions[ENCODER_GRABBER]=0;
    motorForward(MOTOR_GRABBER, 55);
    delay(800);
    updateMotorPositions();
    motorAllStop();
}

void ping(){
    updateMotorPositions();
    printMotorPositions();
}

void test(){
    debugSend("test");
    delay(1000);
    bool motorL=false;
    bool motorR=false;
    bool motorG=false;

    motorForward(MOTOR_GRABBER,75);
    delay(800); 
    updateMotorPositions();
    motorG=positions[ENCODER_GRABBER]!=0;
    delay(800);
    motorBackward(MOTOR_GRABBER,75);
    updateMotorPositions();
    motorG=positions[ENCODER_GRABBER]!=0 | motorG;
    if(motorG){debugSend("grabbers OK");}
    else{ debugSend("some problem with grabbers");}

    motorAllStop();

    motorForward(MOTOR_LEFT,100);
    delay(800); 
    updateMotorPositions();
    motorL=positions[ENCODER_LEFT]!=0;
    delay(800);
    motorBackward(MOTOR_LEFT,100);
    updateMotorPositions();
    motorL=positions[ENCODER_LEFT]!=0 | motorL;
    if(motorL){debugSend("left OK");}
    else{ debugSend("some problem with left");}

    motorAllStop();

    motorForward(MOTOR_RIGHT,100);
    delay(800); 
    updateMotorPositions();
    motorR=positions[ENCODER_RIGHT]!=0;
    delay(800);
    motorBackward(MOTOR_RIGHT,100);
    updateMotorPositions();
    motorR=positions[ENCODER_RIGHT]!=0 | motorR;
    if(motorR){debugSend("right OK");}
    else{ debugSend("some problem with right");}

    motorAllStop();
}

void doMove(byte * message) {
    int distance = (message[1] << 8) | message[2];
    move(distance);
}

void doMoveAndTurn(byte * message) {
    int firstTurn = (message[1] << 8) | message[2];
    int distance = (message[3] << 8) | message[4];
    int secondTurn = (message[5] << 8) | message[6]; // relative finish heading
    turn(firstTurn); // turn to calculated final abs heading
    move(distance); // move in relative heading
    delay(1000);
    turn(secondTurn); // turn to calculated final abs heading
}

void doTurn(byte * message) {
    int angle = (message[1] << 8) | message[2];
    turn(angle); // turn to calculated final abs heading
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
    resetMotorPositions();
    long degrees = (distance * degToMetre) / 1000;
    //long degrees= distance;

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
    int time=millis();

    int prevUltraSoundDistance=1000;    

    while (!finished && !Serial.available()) {
        time=millis(); 
        if(forwards&&cautious&&time-prevTime>10){
            refreshUltraSoundDistance(); 
            if(ultraSoundDistance<ultraSoundThreshold&&prevUltraSoundDistance<ultraSoundThreshold){  
                motorBrake(MOTOR_LEFT); 
                motorBrake(MOTOR_RIGHT); 
                break;
            }
            prevUltraSoundDistance=ultraSoundDistance;
        }
        updateMotorPositions();
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
            if (leftPower > 90)leftPower = 230;
        }
        else if (delta1 - delta0 > 5) {
            if (rightPower > 90)rightPower = 230;
        }
        else {
            if (leftPower != 0)leftPower = 255;
            if (rightPower != 0)rightPower = 255;
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
    motorBrake(MOTOR_LEFT);
    motorBrake(MOTOR_RIGHT);
    updateMotorPositions();
}


void turn(long degs) {
    if (Serial.available()) {
        return;
    };
    //causes problems, don't know why
    //debugPrint("turning");
    //debugPrint((char*) (Serial.available() ? "true":"false"));
    bool cw = degs > 0;
    bool finished = false;
    //degrees of rotation of the wheel to rotation of the robot
    //because 1 reported 2 degree is 2 degrees of rotation
    //target speed in deg/s
    int targetRotV = 70;
    //everything is confusing because the motors are mounted backwards
    degs = abs(degs);
    if (degs > 360)degs = degs % 360;
    updateMotorPositions();
    resetMotorPositions();
    updateMotorPositions();
    int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT]};
    int prevPositions[] = {0, 0};
    float deltaL = 0;
    float deltaR = 0;
    int pows[] = {150, 150};
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
        if ((deltaL + deltaR) / 2 >= degs) {
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
    printMotorPositions();
    motorBrake(MOTOR_LEFT);
    motorBrake(MOTOR_RIGHT);
    updateMotorPositions();
}

void kick(int distance) { // distance in cm
    distance=abs(distance);
    int kickerTime = 0;
    int kickerStrength = 100;
    switch (distance) {
        default:
            kickerTime = 6+ ((distance-50)/50);
            break;
    }
    release();
    digitalWrite(PIN_KICKER, HIGH);

    delay(kickerTime);
    //put kicker back down
    digitalWrite(PIN_KICKER, LOW);
    //put grabbers back in
    delay(200);
    grab();
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
}

void resetMotorPositions() {
    updateMotorPositions();
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

void debugSendInt(int num){
    String a =String(num);
    char arr[a.length()+1];
    for(int i=0;i<a.length();i++){
        arr[i]=a.charAt(i);
    }
    arr[a.length()]='\0';
    debugSend(arr);
    delay(10);
}

void refreshUltraSoundDistance(){
    digitalWrite(PIN_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_TRIGGER, LOW);

    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    int duration = pulseIn(PIN_ECHO, HIGH,2500);
    if(duration==0)duration=1000;

    // convert the time into a distance
    ultraSoundDistance=(int) duration/5.8;
}

void celebrate(){
    turn(180);
    release();
    turn(180);
    grab();
}
