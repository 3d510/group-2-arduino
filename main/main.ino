#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <string.h>

#define L1 A0
#define F3 A1
#define R2 A2
#define R1 A3
#define F1 A4
#define BACK A5 

#define TEST_TIME 0.002
#define READ_TIMES 17
#define FORWARD 0
#define ROTATE_CW 1
#define ROTATE_CCW 2
#define BACKWARD 3

const int encoderRPinA = 11;
const int encoderRPinB = 13;
const int encoderLPinA = 3;
const int encoderLPinB = 5;
const double iteration_time = 0.02;
const double wheel_radius = 3; // cm
const double robot_radius = 9.5; // cm

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
  
  Serial.begin(9600);
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

int counter = 0;
int ticks[111];

void loop() {
  String command;
  if (Serial.available() > 0) {
    command = Serial.readString();
    Serial.flush();
    if (command.length() > 1) {
      // shortest path
      
//      for (int i = 0; i < command.length(); i++) {
//        readChar(command[i]);
//      }

      shortestPath(command);
    }
    else {
      readChar(command[0]);
    }
  }
}

void readChar(char command) {
  switch(command) {
    case 'f': forward(1);
              break;
    case 'l': turnLeft(90);
              break;
    case 'r': turnRight(90);
              break;
    case 'b': back(1); break;
    case 's': readSensors(); break;
      
  }
}

void shortestPath(String instruction) {
  int curCount = 0, curId = 0;
  char curChar = instruction[0];
  while (curId < instruction.length()) {
    curChar = instruction[curId];
    while (curId + curCount < instruction.length() && instruction[curId + curCount] == curChar) 
      curCount++;

    switch (curChar) {
      case 'f': 
        forward(curCount); break;
      case 'b':
        backward(curCount); break;
      case 'l':
        turnLeft(90 * curCount); break;
      case 'r':
        turnRight(90 * curCount); break;      
    }
    
    curId += curCount;
    curCount = 0;
  }
  
}

double getOffset(int noGrid) {
  double offset;
  switch noGrid {
    case 1: offset = -0.3; break;
    default: offset = -0.3; break;
  }
  return offset;
}

void forward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(60, noGrid * 10 + offset, FORWARD, false);
  readSensors();
}

void back(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(60, 9.7, BACKWARD, false);
  readSensors();
}

void turnLeft(double angle) {
  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CCW, false);
  readSensors();
}

void turnRight(double angle) {
  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CW, false);
  readSensors();
}

void doEncoderLeft() {
  encoderLPos++;
}

void doEncoderRight() {
  encoderRPos++;
}

//---------PID-----------//

double errorPrior = 0, integral = 0;
int leftPrevTicks = 0, rightPrevTicks = 0;

void goDigitalDist(double desired_rpm, float dist, int direction, bool sense) {  
  int leftSpeed = 0, rightSpeed = 0;
  int LMag = 1, RMag = 1;

  if (direction == ROTATE_CW) {
    RMag = -1;
  } 
  else if (direction == ROTATE_CCW) {
    LMag = -1;
  } else if (direction == BACKWARD) {
    RMag = -1;
    LMag = -1;
  }
  
  leftPrevTicks = encoderLPos;
  rightPrevTicks = encoderRPos;

  delay(iteration_time * 1000);

  int totalTicks = (int) (562.25 * dist / (2 * PI * wheel_radius));
  
  int start_ticks = encoderLPos;
  
  while(1) {
    if (encoderLPos >= start_ticks + totalTicks) break;
    
    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 7.8, 4.75, 0.08);
    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.5, 0);

    double newLeftSpeed = constrain(abs(leftSpeed) + leftDigitalPidOutput, 0 , 400) * LMag;
    double newRightSpeed = constrain(abs(rightSpeed) + rightDigitalPidOutput, 0 , 400) * RMag;
    md.setSpeeds(newLeftSpeed, newRightSpeed);
    //md.setSpeeds(newLeftSpeed, 0);
    //md.setSpeeds(0, newRightSpeed);
    
    leftPrevTicks = encoderLPos;
    rightPrevTicks = encoderRPos;

    if (sense) {  
      double d1 = readSingleSensor(R2);
      double d2 = readSingleSensor(L1);
      if ((d1 <= 13.5 && d1 >= 10) || (d2 <= 13.5 && d2 >= 10)) break;
    }
    
    delay(iteration_time * 1000); // iteration_time in seconds
  }
  md.setSpeeds(0,0);
}

int id = 1;
double computeDigitalPid(double desired_value, double actual_value, double kp, double ki, double kd) {
  //Serial.print(millis());
  //Serial.print(" ");
  //Serial.println(actual_value);
  double error = desired_value - actual_value;
  integral += (error * iteration_time);
  double derivative = (error - errorPrior)/iteration_time;
  double output = kp * error + ki * integral + kd * derivative;
  errorPrior = error;
  //Serial.println(output);
  return output;
}

double ticksToRpm(int tickCount, double period) { // period in seconds
  return 60 / (562.25 / ((tickCount) / period));
}

//------------Sensor--------------//


void readSensors() {
  double l1 = readSingleSensor(L1);
  double f3 = readSingleSensor(F3);
  double r2 = readSingleSensor(R2);
  double r1 = readSingleSensor(R1);
  double f1 = readSingleSensor(F1);
  String s = stringify(l1) + ";" + stringify(f3) + ";" + stringify(r2) + ";" + stringify(r1) + ";" + stringify(f1);
  //writeString(s);
  Serial.println(s);
} 

void writeString(String s) {
  for (int i = 0; i < s.length(); i++)
    Serial.write(s[i]);
}

String stringify(double value) {
  return String((int)(value/10 - 0.5));
}

float readSingleSensor(int sensorNumber) {
  // read the pin 7 times to get median value
  int sensorVal[READ_TIMES];
  int sortedSensorVal[READ_TIMES];
  for (int i = 0; i < READ_TIMES; i++) {
    sensorVal[i] = analogRead(sensorNumber);   
    //Serial.println(sensorVal[i]);      
    //delay(10);
  }

  int readValue = kthSmallest(sensorVal, 0 ,READ_TIMES-1, READ_TIMES/2);

  if (readValue <= 3) return 0;

  // deduce distance from value read from analog pin
  switch (sensorNumber) {
    case F1: 
      return 5452.08/(readValue - 3) - 1.246;
    case R1: 
      return 6534.06/(readValue - 3) - 3.83;
    case L1:
      return 6270.15/(readValue - 3) - 2.8977;
    case F3:
      return 14054.34/(readValue - 3) - 5.472;
    case BACK:
      return 6142.70/(readValue - 3) - 3.088;
    case R2:
      return 6189.064 / (readValue - 3) - 2.054;
//      case R2:
//        return 6787 / (readValue - 3) - 4;
  }
}

int partition(int arr[], int l, int r)
{
    int x = arr[r], i = l;
    for (int j = l; j <= r - 1; j++) {
        if (arr[j] <= x) {
            swap(&arr[i], &arr[j]);
            i++;
        }
    }
    swap(&arr[i], &arr[r]);
    return i;
}

void swap(int* a, int* b) {
  int tmp = *a;
  *a = *b;
  *b = tmp;
}
 
// This function returns k'th smallest 
// element in arr[l..r] using QuickSort 
// based method.  ASSUMPTION: ALL ELEMENTS
// IN ARR[] ARE DISTINCT
int kthSmallest(int arr[], int l, int r, int k)
{
    // If k is smaller than number of 
    // elements in array
    if (k > 0 && k <= r - l + 1) {
 
        // Partition the array around last 
        // element and get position of pivot 
        // element in sorted array
        int index = partition(arr, l, r);
 
        // If position is same as k
        if (index - l == k - 1)
            return arr[index];
 
        // If position is more, recur 
        // for left subarray
        if (index - l > k - 1) 
            return kthSmallest(arr, l, index - 1, k);
 
        // Else recur for right subarray
        return kthSmallest(arr, index + 1, r, 
                            k - index + l - 1);
    }
 
    // If k is more than number of 
    // elements in array
    return 1000;
}
