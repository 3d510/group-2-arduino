#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define TEST_TIME 0.002
#define FRONT_RIGHT A0
#define FRONT_CENTER A1
#define FRONT_LEFT A2
#define LEFT A3
#define RIGHT A4
#define BACK A5 
#define READ_TIMES 7
#define encoderRPinA 3
#define encoderRPinB 5
#define encoderLPinA 11
#define encoderLPinB 13

volatile int encoderRPos = 0;
int encoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

int prevError = 0;

int n = LOW;
DualVNH5019MotorShield md;
long long Time;

int leftTickPrevError = 0, leftTickPrevPrevError = 0, rightTickPrevError = 0, rightTickPrevPrevError = 0;
int leftPrevOutput = 0, rightPrevOutput = 0;

void setup() {
  pinMode (encoderLPinA, INPUT);
  pinMode (encoderLPinB, INPUT);
  pinMode (encoderRPinA, INPUT);
  pinMode (encoderRPinB, INPUT);
  
  Serial.begin (9600);
  PCintPort::attachInterrupt(encoderLPinA, &doEncoderLeft, RISING);
  PCintPort::attachInterrupt(encoderRPinA, &doEncoderRight, RISING);
  md.init();
}

bool done = false;
int count = 1;
float rpms[200];
int leftTickCount[100];
int rightTickCount[100];
int tickDiff[100];

void loop() {
  if (!done) {    
//    md.setSpeeds(150, 150);
//    
//    delay(3000);
//    
//    leftTickCount[0] = encoderLPos;
//    rightTickCount[0] = encoderRPos;
//    
//    md.setSpeeds(300, 300);
//    
//    while (count < 100) {
//      //rpms[count] = abs(readRpmWithInterrupt(false, 0.02));
//      leftTickCount[count] = encoderLPos;
//      rightTickCount[count] = encoderRPos;
//      encoderLPos = 0;
//      encoderRPos = 0;
//      delay(5);
//      count++;
//    }
//    
//    for (int i = 0; i < count; i++) {
//      Serial.print(i);
//      Serial.print(" "); 
//      Serial.print(leftTickCount[i]);
//      Serial.print(" "); 
//      Serial.println(rightTickCount[i]);
//    }
//    done = true;
    goAnalog(200);
    if (millis() >= 5000) done = true;
  } else {
    md.setSpeeds(0,0);
  }

}

float readRpmWithInterrupt(bool isLeftWheel, float delayTime) { // delayTime in seconds
  if (isLeftWheel) {
    encoderLPos = 0;
    delay(delayTime * 1000);
    float rpm = 60 / (562.25 / ((encoderLPos - 1) / delayTime));
    return rpm;
  } else {
    encoderRPos = 0;
    delay(delayTime * 1000);
    float rpm = 60 / (562.25 / ((encoderRPos - 1) / delayTime));
    return rpm;
  }
}

int readTicksWithInterrupt(bool isLeftWheel, float delayTime) { // delayTime in seconds
  if (isLeftWheel) {
    encoderLPos = 0;
    delay(delayTime * 1000);
    return encoderLPos;
  } else {
    encoderRPos = 0;
    delay(delayTime * 1000);
    return encoderRPos;
  }
}
void doEncoderLeft() {
  encoderLPos++;
}

void doEncoderRight() {
  encoderRPos++;
}

float readSingleSensor(int sensorNumber) {
  // read the pin 7 times to get median value
  int sensorVal[READ_TIMES];
  int sortedSensorVal[READ_TIMES];
  for (int i = 0; i < READ_TIMES; i++) {
    sensorVal[i] = analogRead(sensorNumber);         
    delay(10);
  }
  sortedSensorVal[0] = sensorVal[0];
  for (int i = 1; i < READ_TIMES; i++) {
    int id = 0;
    while (id < i && sortedSensorVal[id] <= sensorVal[i])
      id++;
    if (id == i) {
      sortedSensorVal[i] = sensorVal[i];
    } else {
      for (int j = i - 1; j >= id; j--) {
        sortedSensorVal[j+1] = sensorVal[j]; 
      } 
      sortedSensorVal[id] = sensorVal[i]; 
    }
  }
  int readValue = sortedSensorVal[READ_TIMES/2];

  // deduce distance from value read from analog pin
  switch (sensorNumber) {
    case FRONT_RIGHT: 
      return 8544.11/readValue - 7.29;
    case FRONT_LEFT: 
      return 8068.98/readValue - 5.63;
    case LEFT:
      return 8766.22/readValue - 7.35;
    case RIGHT:
      return 7904.38/readValue - 5.62;
    case BACK:
      return 8068.41/readValue - 6.77;
  }
  
}




void goAnalog(int targetSpeed) {
//  int m1Encoder = readTicksWithInterrupt(true, 0.005);
//  int m2Encoder = readTicksWithInterrupt(false, 0.02);
    encoderLPos = 0;
    encoderRPos = 0;
    delay(10);
    int m1Encoder = encoderLPos;
    int m2Encoder = encoderRPos;
//  Serial.println(readRpmWithInterrupt(true, 0.02));
//  Serial.println(readRpmWithInterrupt(false, 0.02));
  int output = computeAnalogPid(m1Encoder, m2Encoder);
  Serial.println(m1Encoder);
  md.setSpeeds(targetSpeed + 8 - output, targetSpeed + output);
}

int computeAnalogPid(int leftTicks, int rightTicks) {
    int error, pwm1 = 255, pwm2 = 255;
    float integral, derivative, output;
    float Kp = 0.75;  //0-1
    float Kd = 1.65;  //1-2
    float Ki = 0.75;  //0.5-1

    error = leftTicks - rightTicks;
    integral += error;
    derivative = error - prevError;
    output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    pwm1 = output;
    return pwm1;
}



