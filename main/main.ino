#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <PID_v1.h>

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

const double FORWARD_PWM_L = 1750 * 0.087;
const double FORWARD_PWM_R = 1750 * 0.087; 

volatile int encoderRPos = 0;
int encoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

double  motorEncoderDiff = 0;
double  motorDiffOutput = 0;
double  motorTargetDiff = 0;

int prevError = 0;

int n = LOW;
//int counter = 0;
DualVNH5019MotorShield md;
long long Time;

void setup() {
  pinMode (encoderLPinA, INPUT);
  pinMode (encoderLPinB, INPUT);
  pinMode (encoderRPinA, INPUT);
  pinMode (encoderRPinB, INPUT);
  
  Serial.begin (9600);
  // attachInterrupt(1, doEncoder, RISING); 
  PCintPort::attachInterrupt(encoderLPinA, &doEncoderLeft, RISING);
  PCintPort::attachInterrupt(encoderRPinA, &doEncoderRight, RISING);
  md.init();
}

bool done = false;
int count = 1;
float rpms[200];

void loop() {
  if (!done) {    
    //md.setSpeeds(100, 0);
    
    //delay(3000);
    //Serial.print(readRpmWithInterrupt(false, 0.02));
//    rpms[0] = abs(readRpmWithInterrupt(false, 0.02));
//    md.setSpeeds(300, 0);
//    while (count < 200) {
//      rpms[count] = abs(readRpmWithInterrupt(false, 0.02));
//      count++;
//    }
//    for (int i = 0; i < count; i++) {
//      Serial.print(i);
//      Serial.print(","); 
//      Serial.println(rpms[i]);
//    }
//    done = true;
      go();
  } else {
    md.setSpeeds(0,0);
  }

}

void go() {
  int m1Encoder = readTicksWithInterrupt(true, 0.02);
  int m2Encoder = readTicksWithInterrupt(false, 0.02);
  Serial.println(readRpmWithInterrupt(true, 0.02));
  Serial.println(readRpmWithInterrupt(false, 0.02));
  int output = computePid(m1Encoder, m2Encoder);
  md.setSpeeds(300 - output, 300 + output);
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
//  if (digitalRead(encoderLPinA) == digitalRead(encoderLPinB)) {
//    encoderLPos++;
//  } else {
//    encoderLPos--;
//  }
  encoderLPos++;
}

void doEncoderRight() {
//  if (digitalRead(encoderRPinA) == digitalRead(encoderRPinB)) {
//    encoderRPos++;
//  } else {
//    encoderRPos--;
//  }
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

float readRpmWithoutInterrupt(bool isLeftWheel) {
//  Time = micros();
//  md.setSpeeds(400,400);
//  if (Time <= 2000000) {
//    n = digitalRead(encoderRPinA);
//    if ((encoderRPinALast == LOW) && (n == HIGH)) {
//      if (digitalRead(encoderRPinB) == LOW) {
//        encoderRPos--;
//      } else {
//        encoderRPos++;
//      }
//    }
//    encoderRPinALast = n;
//  } else {
//    if (counter == 0)
//      Serial.print(encoderRPos);
//    counter = 1;
//  }
}

// motorTargetDiff is the difference between the motors. set 0 to turn left or right, or set a positive value to move forward
void setPid(float finalLPWM, float finalRPWM, int setPoint) {
    float kp = 1.6, ki = 0.3, kd = 0.6;
    PID motorDiffPID(&motorEncoderDiff, &motorDiffOutput, &motorTargetDiff, kp, ki, kd, DIRECT);
    //motorDiffPID.Reset();
    motorDiffPID.SetMode(AUTOMATIC);
    motorDiffPID.SetOutputLimits(-2000, 2000);
    motorDiffPID.SetSampleTime(10);

    while (true) {
        double left = readRpmWithInterrupt(true, 0.02);
        double right = readRpmWithInterrupt(false, 0.02);
        motorEncoderDiff = right - left;
        motorDiffPID.Compute();
        
        finalLPWM -= motorDiffOutput / 50;
        finalRPWM += motorDiffOutput / 50;

        finalLPWM = constrain(finalLPWM, 0, 255);
        finalRPWM = constrain(finalRPWM, 0, 255);
        
          // Encoder Print
          Serial.print("p##");  
          Serial.print(count);
          Serial.print(" ");
          Serial.print(finalLPWM);
          Serial.print(" ");
          Serial.print(finalRPWM);
          Serial.print(" ");
          //Serial.print(motorLAccmEncoderCount);
          Serial.print(" ");
          //Serial.print(motorRAccmEncoderCount);
          Serial.print(" ");
          Serial.print(motorEncoderDiff);
          Serial.print(" ");
          Serial.println(motorDiffOutput);
          
        if (left < setPoint) {
            md.setM1Speed(finalLPWM / 255.0 * 400.0);
        }
        else {
            md.setM1Brake(400);
        }

        if (right <  setPoint) {
            md.setM2Speed(finalRPWM / 255.0 * 400.0);
        }
        else {
            md.setM2Brake(400);
        }
        delay(10);
    }
}

int computePid(int leftTicks, int rightTicks) {
    int error, pwm1 = 255, pwm2 = 255;
    float integral, derivative, output;
    float Kp = 0.75;  //0-0.1
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

