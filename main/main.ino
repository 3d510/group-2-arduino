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
const double distBetweenSensors = 0.5;
const double adjustEpsilon = 0.5;

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
//    if (command.length() > 1) {
//      shortestPath(command);
//    }
//    else {
//      readChar(command[0]);
//    }
    convertCommand(command);
  }
  
//  if (!done) {
//    goDigitalDist(40, 50, FORWARD, false);
//    done = true;
//  } else {
//    md.setSpeeds(0,0);
//  }
}


void readChar(char command) {
  switch(command) {
    case 'f': forward(1);
              break;
    case 'l': turnLeft(90);
              break;
    case 'r': turnRight(90);
              break;
    case 'b': backward(1);
              break;
    case 's': readSensors(true); 
              break;
    case 'a': readSensors(false);
              break;
  }
}

void adjust() {
  double sensorDifference = readSingleSensor(L1) - readSingleSensor(R2);
  //check dist between sensors
  if (abs(sensorDifference) < adjustEpsilon) return;
  if (sensorDifference > 0) {
    turnRight(atan2((sensorDifference), distBetweenSensors));
  }
  else {
    turnLeft(atan2((-1 * sensorDifference), distBetweenSensors));
  }
}

void readCommand(String instruction) {
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
      case 's':
        readSensors(true); break;
      case 'a':
        readSensors(false); break;
    }
    
    curId += curCount;
    curCount = 0;
  }
  
}

double getOffset(int noGrid) {
  double offset;
  switch (noGrid) {
    case 1: offset = -0.3; break;
    default: offset = -0.3; break;
  }
  return offset;
}

void forward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(90, noGrid * 10 + offset, FORWARD, false);
  readSensors(true);
}

void backward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(60, noGrid * 10 + offset, BACKWARD, false);
  readSensors(true);
}

void turnLeft(double angle) {
  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CCW, false);
  readSensors(true);
}

void turnRight(double angle) {
  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CW, false);
  readSensors(true);
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
double kpl, kil, kdl, kpr, kir, kdr;

int rampUp(double desired_rpm) {
  leftPrevTicks = encoderLPos;
  for (int i = 10; i < desired_rpm; i += 10) {
    md.setSpeeds(i, i);
    delay(1);
  }
  return encoderLPos - leftPrevTicks;
}

void rampDown(double desired_rpm) {
  for (int i = desired_rpm; i > 0; i -= 10) {
    md.setSpeeds(i, i);
    delay(1);
  }
  md.setSpeeds(0, 0);
}

void goDigitalDist(double desired_rpm, float dist, int direction, bool sense) {  
  int rampUpTicks = rampUp(desired_rpm);
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
    if (encoderLPos >= start_ticks + totalTicks - rampUpTicks) break;

    //1 grid
//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 7.5, 6.0, 0);
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.5, 0);
    
    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), kpl, kil, kdl);
    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), kpr, kir, kdr);

    //double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 5.0, 5.75, 0);
    //double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.25, 0);
    
    //7.75
//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 5.225, 4.75, 0);  
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.5, 0);

//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 4.875, 5.75, 0.08);
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.25, 4.5, 0);

    double newLeftSpeed = constrain(abs(leftSpeed) + leftDigitalPidOutput, 0 , 400) * LMag;
    double newRightSpeed = constrain(abs(rightSpeed) + rightDigitalPidOutput, 0 , 400) * RMag;
    md.setSpeeds(newLeftSpeed, newRightSpeed);
//    md.setSpeeds(newLeftSpeed, 0);
//    md.setSpeeds(0, newRightSpeed);
    
    leftPrevTicks = encoderLPos;
    rightPrevTicks = encoderRPos;

    if (sense) {  
      double d1 = readSingleSensor(R2);
      double d2 = readSingleSensor(L1);
      
      if ((d1 <= 13.5 && d1 >= 10) || (d2 <= 13.5 && d2 >= 10)) break;
    }
    
    delay(iteration_time * 1000); // iteration_time in seconds
  }
  integral = 0;
  errorPrior = 0;
  //100, 300 for lab2 lounge 100,250
  md.setBrakes(100, 200);
  //rampDown();
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


void readSensors(bool returnGrid) {
  double l1 = readSingleSensor(L1);
  double f3 = readSingleSensor(F3);
  double r2 = readSingleSensor(R2);
  double r1 = readSingleSensor(R1);
  double f1 = readSingleSensor(F1);
  double back = readSingleSensor(BACK);
  String s;
  if (returnGrid) s = stringifyGrid(l1) + ";" + stringifyGrid(back) + ";" + stringifyGrid(r2) + ";" + stringifyGrid(f3) + ";" + stringifyGrid(r1) + ";" + stringifyGrid(f1);
  else s = stringify(l1) + ";" + stringify(back) + ";" + stringify(r2) + ";" + stringify(f3) + ";" + stringify(r1) + ";" + stringify(f1);
  //writeString(s);
  Serial.println(s);
} 

String stringify(double value) {
  return String(value);
}

String stringifyGrid(double value) {
  if (value < 0) return "-1";
  return String((int)(value/10 + 0.5));
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

  if (readValue <= 3) return -10;

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

//---------------Testing----------------//


//ffff;p;k;k;k;k;k;k
//fffff:n:100:300

void convertCommand(String command) {
  String curString, split[11];
  int curIndex = 0, splitId = 0;
  while (curIndex < command.length()) {
    curString = "";
    int index = curIndex;
    while (index < command.length() && command[index] != ';') {
      curString += command[index];
      index++;
    }
    split[splitId] = curString;
    splitId++;
    curIndex = index + 1;
    curString = "";
  }

  if (split[1] == "p") {
    kpl = split[2].toFloat(); 
    kil = split[3].toFloat();
    kdl = split[4].toFloat();
    kpr = split[5].toFloat();
    kir = split[6].toFloat();
    kdr = split[7].toFloat();
    readCommand(split[0]);
    
  } else if (split[1] == "n") {
    md.setSpeeds(split[2].toFloat(), split[3].toFloat());
  }
}



