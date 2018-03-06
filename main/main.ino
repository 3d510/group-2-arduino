#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define L1 A0
#define F3 A1
#define R2 A2
#define R1 A3
#define F1 A4
#define BACK A5 

#define TEST_TIME 0.002
#define READ_TIMES 11
#define FORWARD 0
#define ROTATE_CW 1
#define ROTATE_CCW 2

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

int counter = 0;
int ticks[111];

void loop() {
   
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
  } else if (direction == ROTATE_CCW) {
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
  Serial.println(actual_value);
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

float readSingleSensor(int sensorNumber) {
  // read the pin 7 times to get median value
  int sensorVal[READ_TIMES];
  int sortedSensorVal[READ_TIMES];
  for (int i = 0; i < READ_TIMES; i++) {
    sensorVal[i] = analogRead(sensorNumber);   
    //Serial.println(sensorVal[i]);      
    //delay(10);
  }
  
//  sortedSensorVal[0] = sensorVal[0];
//  for (int i = 1; i < READ_TIMES; i++) {
//    int id = 0;
//    while (id < i && sortedSensorVal[id] <= sensorVal[i])
//      id++;
//    if (id == i) {
//      sortedSensorVal[i] = sensorVal[i];
//    } else {
//      for (int j = i - 1; j >= id; j--) {
//        sortedSensorVal[j+1] = sensorVal[j]; 
//      } 
//      sortedSensorVal[id] = sensorVal[i]; 
//    }
//  }
  
//  int readValue = sortedSensorVal[READ_TIMES/2];

  int readValue = kthSmallest(sensorVal, 0 ,READ_TIMES-1, READ_TIMES/2);

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
      return 6725.72 / (readValue - 3) - 4.077;
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




