#include "Arduino.h"
#include "gantry.h"
#include "ballScrew.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_Width 500
#define MAX_WIDTH 2500
#define freq 50

#define runButton 35 
#define homeButton 32
#define emergencyStopButton 33

#define limitSwitchXPin 36
#define limitSwitchYPin 39
#define limitSwitchZPin 34

#define testLED 25

#define tube1_Offset -4
#define tube2_Offset -8
#define tube3_Offset -4
#define tube4_Offset 2

#define tube1 15
#define tube2 9
#define tube3 4
#define tube4 0

#define heightAbovePivot_um 50000
#define tubeWidth_mm 27

#define pumpPin 14
#define pumpMicrosteps 16

hw_timer_t * timer = NULL;      //H/W timer defining (Pointer to the Structure)
volatile bool pinState = false;
volatile bool isPumpOn = false;
    //sweep(5, 15);
    //sweep(5, 9);
    //sweep(5, 4);
    //sweep(5, 0);
//this is a test for github



gantry Gantry = gantry();

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int motorPin = 0;
int delayTime = 10;
int servo1Pos = 0;

int goToAngle(int angle, int motorNum, int offset_deg);
int sweepToAngle(int angle, float time_s, int motorNum, int offset_deg);
void sweep(int delay_ms,int motorNum);

void stopAllMotors();
void finalMotionTest();
void performFillingMotion();
void onTimer();
void setPumpRPM(int rpm, int pump_pin, int microstepsPerStep);
//NOTE CAN potentially use all limit switches on same pin, problem is would have to look for short circuits.
//void limitSwitchISR();
void setupLimitSwitchISR(int pinNumber);

void setup() {
  // put your setup code here, to run once:
  pinMode(testLED, OUTPUT);

  pinMode(runButton, INPUT);
  pinMode(homeButton, INPUT);
  pinMode(emergencyStopButton, INPUT);
  
  setupLimitSwitchISR(limitSwitchXPin);
  setupLimitSwitchISR(limitSwitchYPin);
  setupLimitSwitchISR(limitSwitchZPin);

  pinMode(pumpPin, OUTPUT);  
  //attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, RISING);
  attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, HIGH);

  pwm.begin();
  pwm.setPWMFreq(freq);



  timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
 timerAttachInterrupt(timer, &onTimer, true); 	// Attach interrupt
 timerAlarmWrite(timer, 1000000, true);  		// Match value= 1000000 for 1 sec. delay.
 timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)
}

void loop() {
  
  if(digitalRead(homeButton) == HIGH){
    Gantry.homeGantry();
  }
  if (digitalRead(runButton) == HIGH)
  {

    //Adding this line to the test branch
    //performFillingMotion();


    //ADD PUMPING SEQUENCE HERE
    setPumpRPM(3, pumpPin, pumpMicrosteps);
    delay(10000);
    setPumpRPM(5, pumpPin, pumpMicrosteps);
    delay(60000);
    setPumpRPM(7, pumpPin, pumpMicrosteps);
    delay(30000);
    setPumpRPM(10, pumpPin, pumpMicrosteps);
    delay(30000);
    setPumpRPM(15, pumpPin, pumpMicrosteps);
    delay(30000);
    setPumpRPM(0, pumpPin, pumpMicrosteps);

  }
}

void stopAllMotors(){
  Gantry.emergencyStop();
  setPumpRPM(0, pumpPin, pumpMicrosteps);  
}

void setupLimitSwitchISR(int pinNumber){
  pinMode(pinNumber, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinNumber), ballScrew::limitSwitchISR, RISING);
}












//Note that positive angel is when tube tilts away from user standing at front, this is done because we normally want like 60 degrees
//offset is in degrees and is used to calibrate to find true zero position for consistency accross pieces
int goToAngle(int angle, int motorNum, int offset_deg){
  angle = angle + offset_deg;
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
  pwm.setPWM(motorNum, 0, pulseWidth);
  return pulse_wide;
}

int sweepToAngle(int angle, float time_s, int motorNum, int offset_deg){
  angle = angle + offset_deg;
    int pulseWidth, delay_ms;
  int minRes = 5;//microsecond deadband
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int diff = pulse_wide - servo1Pos;
  pulse_wide = servo1Pos;

  //if positive difference then need to move forward, else move backwards
  if (diff > 0){
    int ticks = diff/minRes;
    delay_ms = 1000*time_s/ticks;
    for(int i = 0; i < ticks; i++){
          pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
          pwm.setPWM(motorNum, 0, pulseWidth);
          delay(delay_ms);
          pulse_wide = pulse_wide + minRes;
    }
    return pulse_wide - minRes;
  }
  else if(diff < 0){
    int ticks = -diff/minRes;
    delay_ms = 1000*time_s/ticks;
    for(int i = 0; i < ticks; i++){
          pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
          pwm.setPWM(motorNum, 0, pulseWidth);
          delay(delay_ms);
          pulse_wide = pulse_wide - minRes;
    }
    return pulse_wide + minRes;
  }
  return servo1Pos;
}

void sweep(int delay_ms,int motorNum){
  int pulseWidth, pulse_wide;
  int minRes = 5;//microsecond deadband
  int mid = (MIN_Width + MAX_WIDTH)/2;//start at mid point (0)
  pulse_wide = mid;
  pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
  pwm.setPWM(motorNum, 0, pulseWidth);
  delay(1000);

    //now at 0 position
  
      while(pulse_wide < MAX_WIDTH){
      pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
      pwm.setPWM(motorNum, 0, pulseWidth);
      delay(delay_ms);
      pulse_wide = pulse_wide + minRes;
    }
    delay(1000);
    while(pulse_wide > mid){

      pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
      pwm.setPWM(motorNum, 0, pulseWidth);
      delay(delay_ms);
      pulse_wide = pulse_wide - minRes;
    }
    delay(1000);
    while(pulse_wide > MIN_Width){

      pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
      pwm.setPWM(motorNum, 0, pulseWidth);
      delay(delay_ms);
      pulse_wide = pulse_wide - minRes;
    }
        delay(1000);

    while(pulse_wide < mid){
      pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
      pwm.setPWM(motorNum, 0, pulseWidth);
      delay(delay_ms);
      pulse_wide = pulse_wide + minRes;
    }
    delay(1000);
  
}

void performFillingMotion(){
    servo1Pos = goToAngle(0, tube1, tube1_Offset);
    delay(1000);
    //go to center above the tube.
    Gantry.goToAbsPosition_mm(0, 22, Gantry.getMaxZDisplacement() - 10, 5);
    
    //move to startign position for angle 60 deg, split up in 2 motions to avoid collision
    
    int firstFillAngle = 70;


        Gantry.goToRelativePosition(0, heightAbovePivot_um*sin(PI*firstFillAngle/float(180)), 0, 5000);


    int zOffsetForBottomOfTube = tubeWidth_mm*1000/(2*sin(firstFillAngle*PI/float(180)));
    int intialHeightAboveAxis = 44000;
    Gantry.goToRelativePosition(0, 0, heightAbovePivot_um*cos(PI*firstFillAngle/float(180)) - intialHeightAboveAxis - zOffsetForBottomOfTube, 5000);
    
    servo1Pos = sweepToAngle(firstFillAngle, 2, tube1, tube1_Offset);
    delay(1000);


        int entranceDistance_um = 30000;
    //slide into tube very slowly
    Gantry.goToRelativePosition(0, -entranceDistance_um*sin(PI*firstFillAngle/float(180)), -entranceDistance_um*cos(PI*firstFillAngle/float(180)), 5000);
    //Gantry.goToRelativePosition(0, -entranceDistance_um*3, -entranceDistance_um*3, 1000);

    delay(2000);
      //travel back up tube to the top
      entranceDistance_um = 20000;
        Gantry.goToRelativePosition(0, entranceDistance_um*sin(PI*firstFillAngle/float(180)), entranceDistance_um*cos(PI*firstFillAngle/float(180)), 5000);

}


void finalMotionTest(){
    Gantry.goToAbsPosition_mm(5, 5, 5, 5);

    delay(200);

    Gantry.goToAbsPosition_mm(5, 10, 10, 5);

    delay(200);

    Gantry.goToAbsPosition_mm(285, 140, 30, 10);

    delay(200);

    Gantry.goToAbsPosition_mm(0, 0, 0, 10);

    delay(200);

}

void setPumpRPM(int rpm, int pump_pin, int microstepsPerStep){
  //note this code assumes that the prescaler on the internal timer (timer)
  //is set such that each tick is a microsecond
  int standardSteps = 200;
  int stepsPerSecond = rpm*standardSteps*microstepsPerStep/60;
  int pumpStepDelay_us = 1000000/stepsPerSecond;
  if(rpm == 0){
    isPumpOn = false;
    timerAlarmWrite(timer, 1000000, true);//make it trigger every second and ignore the trigger, in the future make it so the timer actually becomes dissabled
  }
  else{
    isPumpOn = true;
    timerAlarmWrite(timer, pumpStepDelay_us/2, true);  		// Match value= 1000000 for 1 sec. delay.
  }

}
void onTimer(){
  if(isPumpOn){
    pinState = !pinState;
    digitalWrite(pumpPin, -pinState);
  }
}
