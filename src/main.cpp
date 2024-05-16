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

//offsets indegrees
#define tube1_Offset 11
#define tube2_Offset 5
#define tube3_Offset 9
#define tube4_Offset -2

#define tube1 15
#define tube2 9
#define tube3 4
#define tube4 0

//THIS roughly means the distance you want to nozzle to be at before entrance into the tube at 60 degrees
#define heightAbovePivot_um 55000
#define tubeWidth_mm 27

#define pumpPin 14
#define pumpMicrosteps 16

#define pumpDirectionPin 5 //NOTE THAT FORWARD DIRECTION FOR THE PUMP IS TRUE/HIGH

#define volumeSensingPin 23

#define startingX_mm 0
#define startingY_mm 67

#define firstTubeGap 92
#define secondTubeGap 94
#define thirdTubeGap 94

#define NUM_TUBES 4

bool pumpDirection = true;//true is forward false is backwards

int tubeSideToSideGapsOffsets_mm[4] = {0, firstTubeGap, secondTubeGap, thirdTubeGap};

hw_timer_t * timer = NULL;      //H/W timer defining (Pointer to the Structure)
volatile bool pinState = false;
volatile bool isPumpOn = false;

gantry Gantry = gantry();
int startingZ_mm  = Gantry.getMaxZDisplacement() - 50;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int servo1Pos = 0;

int servoPos_pulse[4] = {0, 0, 0, 0};
int tubePins[4] = {tube1, tube2, tube3, tube4};
int tubeOffsets[4] = {tube1_Offset, tube2_Offset, tube3_Offset, tube4_Offset};

int goToAngle(int angle, int motorNum, int offset_deg);
int sweepToAngle(int angle, float time_s, int motorNum, int offset_deg, int currentPosition_pulse);

void stopAllMotors();
void stopPump();
void performFillingMotionforAll4();
void performFillingMotionFor1Tube(int tubeNumber);
void onTimer();
void setPumpRPM(int rpm, int pump_pin, int microstepsPerStep);

//NOTE CAN potentially use all limit switches on same pin, problem is would have to look for short circuits.
void setupLimitSwitchISR(int pinNumber);

void setPumpDirection(bool dir);//true is forward, false is reverse

void setup() {
  //Initialize buttons
  pinMode(runButton, INPUT);
  pinMode(homeButton, INPUT);
  pinMode(emergencyStopButton, INPUT);
  pinMode(pumpDirectionPin, OUTPUT);
  setPumpDirection(true);//true is forward
  
  //Initialize limit switches
  setupLimitSwitchISR(limitSwitchXPin);
  setupLimitSwitchISR(limitSwitchYPin);
  setupLimitSwitchISR(limitSwitchZPin);

  //Initialize pump pin
  pinMode(pumpPin, OUTPUT);

  pinMode(volumeSensingPin, INPUT);

  //Add external interupt for emergency stop button  
  attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, RISING);

  attachInterrupt(digitalPinToInterrupt(volumeSensingPin), stopPump, FALLING);

  //Initialize PWM for servo driver
  pwm.begin();
  pwm.setPWMFreq(freq);

  //Initialize internal timer interupt used to control pump rpm
  timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
  timerAttachInterrupt(timer, &onTimer, true); 	// Attach interrupt
  timerAlarmWrite(timer, 1000000, true);  		// Match value= 1000000 for 1 sec. delay.
  timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)
}

void loop() {

  //Perform homing sequence and tilt tube holders to initial angles
  if(digitalRead(homeButton) == HIGH){
    //go to upright angle
    for(int i = 0; i < NUM_TUBES; i++){
      servoPos_pulse[i] = goToAngle(0, tubePins[i], tubeOffsets[i]);
    }
    delay(1000);
    Gantry.homeGantry();
    //go to loading angle
    int loadingAngle = -30;
    for(int i = 0; i < NUM_TUBES; i++){
      servoPos_pulse[i] = goToAngle(loadingAngle, tubePins[i], tubeOffsets[i]);
    }

  }

  //Perform one of the filling sequences
  if (digitalRead(runButton) == HIGH)
  {
    //performFillingMotionforAll4();  
    performFillingMotionFor1Tube(3);
  }
}


















void performFillingMotionFor1Tube(int tubeNumber){
  //Make sure that the user is calling a valid tube 
  if (tubeNumber > 4 || tubeNumber < 1){
    return;
  }

  int startingXPosition_mm = startingX_mm;
  //add all the gaps.
  for(int i = 0; i < tubeNumber; i++){
    startingXPosition_mm = startingXPosition_mm + tubeSideToSideGapsOffsets_mm[i];
  }

  setPumpDirection(true);

  servoPos_pulse[tubeNumber-1] = goToAngle(0, tubePins[tubeNumber-1], tubeOffsets[tubeNumber-1]);

  delay(1000);
  
  //go to center above the tube.
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);
   
  //move to startign position for angle 60 deg, split up in 2 motions to avoid collision
  int firstFillAngle = 60;
  

  //Offset in z direction so that nozzle travels along bottom side of tube as opposed to along the center of thubes axis
  int zOffsetForBottomOfTube = tubeWidth_mm*1000/(2*sin(firstFillAngle*PI/float(180)));
  
  //note that I 57000 ensures contact but maybe even  bit too much bending of the nozzle
  //note I found that 55000 also works with less bending, can likely still be improved. note that a lower number will make nozzle bend less, i.e lift it up
  int intialHeightAboveAxis = 55000;
  
  
  
  //try combining moves int o1 diagonal pass 
  Gantry.goToRelativePosition(0, heightAbovePivot_um*sin(PI*firstFillAngle/float(180)), heightAbovePivot_um*cos(PI*firstFillAngle/float(180)) - intialHeightAboveAxis - zOffsetForBottomOfTube, 5000);    

  
  
  delay(1000);

  int TUBE_ANGLE_OFFSET_FOR_INSERTION = 15;
  //FIRST MOVE TUBE 5 degs so there are no collisions then move back
  servoPos_pulse[tubeNumber-1] = sweepToAngle(firstFillAngle + TUBE_ANGLE_OFFSET_FOR_INSERTION, 1, tubePins[tubeNumber-1], tubeOffsets[tubeNumber-1], servoPos_pulse[tubeNumber-1]);


  int entranceDistance_um = 70000;
  //slide into tube very slowly as deep as posssible
  Gantry.goToRelativePosition(0, -entranceDistance_um*sin(PI*(firstFillAngle)/float(180)), -entranceDistance_um*cos(PI*(firstFillAngle)/float(180)), 5000);
   
  servoPos_pulse[tubeNumber-1] = sweepToAngle(firstFillAngle, 1, tubePins[tubeNumber-1], tubeOffsets[tubeNumber-1], servoPos_pulse[tubeNumber-1]);

  //initizl pump prime
  setPumpRPM(300, pumpPin, pumpMicrosteps);
  delay(1900);
  //ADD CODE HERE TO DO INITIAL FILLING WHILE THE TUBE IS FULLY IN.


  //ONCE THE BLOOD HAS REACHED WHERE THE NOZZLE IS THE CONTINUE TO NEXT SECTION.

  //find distance to move out of the tube
    //this is the diagonal distance out of the tube you with to travel, I assume it is just 1cm shy of where you started so as to ensure you are in the tube at the end
    int exitDistance_um = entranceDistance_um - 10000;
  //the line below pulls the tube out in 1 shot where as the loop lets you set different pump speeds as you pull it out.
  //Gantry.goToRelativePosition(0, exitDistance_um*sin(PI*firstFillAngle/float(180)), exitDistance_um*cos(PI*firstFillAngle/float(180)), 5000);
   
   
      //HERE I LET YOU DO DIFFERENT PUMP SEQUENCES AS YOU FILL IT UP
   int pumpRPMS[] = {10, 40, 60, 80};
   //MAKE SURE BOTH THESE ARRAYS HAVE SAME NUMBER OF ELEMENTS!!
   int delays_ms_Per_pumpingInterval[] = {3000, 5000, 3000, 2000};


   int numberOfPumpingSequencesWhileExitingTube = sizeof(pumpRPMS)/sizeof(int);
   int exitDistancePerPumpSequence_um = exitDistance_um/numberOfPumpingSequencesWhileExitingTube;
   for(int i = 0; i < numberOfPumpingSequencesWhileExitingTube; i++){
    //set new pumping speed and slide gantry up tube, take sliding time as delay before new pumping speed
    setPumpRPM(pumpRPMS[i], pumpPin, pumpMicrosteps);
    Gantry.goToRelativePosition(0, exitDistancePerPumpSequence_um*sin(PI*firstFillAngle/float(180)), exitDistancePerPumpSequence_um*cos(PI*firstFillAngle/float(180)), delays_ms_Per_pumpingInterval[i]);
   }
  
  //take the nozzle out by traveling straight up.
  Gantry.goToRelativePosition(0, 0, 40000, 5000);

  // straighten out 
  int finalFillAngle = -10;
  servoPos_pulse[tubeNumber-1] = sweepToAngle(finalFillAngle, 1, tubePins[tubeNumber-1], tubeOffsets[tubeNumber-1], servoPos_pulse[tubeNumber-1]);


  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  int depthBelowTubeTop_um = 25000;//this is for the depth along the tube wall you want the final fill position to be at.


  //take the nozzle to tube wall, old logic
  Gantry.goToRelativePosition(0, -(tubeWidth_mm*1000/2 + sin(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um), -cos(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um, 5000);
  
  setPumpRPM(50, pumpPin, pumpMicrosteps);
  delay(10000);

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  //go back to center
  servoPos_pulse[tubeNumber-1] = sweepToAngle(0, 2, tubePins[tubeNumber-1], tubeOffsets[tubeNumber-1], servoPos_pulse[tubeNumber-1]);
}

//DO NOT CHANGE ANYTHING BELOW THIS LINE
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
*/

void performFillingMotionforAll4(){
  //cycle through all 4 motions.
  for(int i = 0; i < NUM_TUBES; i++){
    performFillingMotionFor1Tube(i+1);
  }

  //once it is finished then go to top left
  Gantry.goToAbsPosition_mm(0, Gantry.getMaxYDisplacement(), Gantry.getMaxZDisplacement(), 10);
}


//true is forward, false is reverse
void setPumpDirection(bool dir){
  pumpDirection = dir;
  digitalWrite(pumpDirectionPin, pumpDirection);
}

void stopAllMotors(){
  Gantry.emergencyStop();
  setPumpRPM(0, pumpPin, pumpMicrosteps);  
}

void setupLimitSwitchISR(int pinNumber){
  pinMode(pinNumber, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinNumber), ballScrew::limitSwitchISR, RISING);
}

void onTimer(){
  if(isPumpOn){
    pinState = !pinState;
    digitalWrite(pumpPin, -pinState);
  }
}

void stopPump(){
  setPumpRPM(0, pumpPin, pumpMicrosteps);
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

//Note that positive angle is when tube tilts away from user standing at front, this is done because we normally want like 60 degrees
//offset is in degrees and is used to calibrate to find true zero position for consistency accross pieces
int goToAngle(int angle, int motorNum, int offset_deg){
  angle = angle + offset_deg;
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
  pwm.setPWM(motorNum, 0, pulseWidth);
  return pulse_wide;
}

int sweepToAngle(int angle, float time_s, int motorNum, int offset_deg, int currentPosition_pulse){
  angle = angle + offset_deg;
  int pulseWidth, delay_ms;
  int minRes = 5;//microsecond deadband
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int diff = pulse_wide - currentPosition_pulse;
  pulse_wide = currentPosition_pulse;

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
  return currentPosition_pulse;
}