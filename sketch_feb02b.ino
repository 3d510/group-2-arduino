#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

DualVNH5019MotorShield md;

#define MOTOR_L_ENCODER_A 3
#define MOTOR_L_ENCODER_B 5

#define MOTOR_R_ENCODER_A 11
#define MOTOR_R_ENCODER_B 13

int motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
// volatile int motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
int motorRAccmEncoderCount = 0;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}



void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();

  pinMode(MOTOR_L_ENCODER_A, INPUT);
  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISRA, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
//  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &moto/rRISRlA, CHANGE); 
}

void loop() {
  md.setSpeeds(200, 185);
  stopIfFault();
//  Serial.println(pulseIn(MOTOR_L_ENCODER_A/, HIGH));
}

void motorLISRA()      //ISR for left motor encoder interrupt
{
    motorLAccmEncoderCount++;
}


void motorRISRA()      //ISR for left motor encoder interrupt
{
    motorRAccmEncoderCount++;
}

