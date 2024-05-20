#include "Arduino.h"
#include "gantry.h"
#include "ballScrew.h"


//NOTE: MAKE SURE MOTORS ARE CONNECTED IN PROPER OriENtATION, I.E CCW is taken as the positive direction, => moves actuators forward, away from motor, consult photos.


//int minPulseWidth = 50;

gantry::gantry(){

    //For parameter reference see ballScrewConstructor
    //ballScrew(int stepPinNumber, int directionPinNumber, int stepsPerRevolutions, int pitchMm, int maxSpeed,  int maxDis_um,  bool positiveDir, bool isHomingDirPositive, bool isHomeZero);

    //BELOW IS THE CONFIGURATION FOR THE X AXIS
    ballScrew X = ballScrew(limitSwitchXPin, xStepPin, xDirPin, stepsPerRev, xPitch_mm, maxSpeed_mmps, XmaxDisplacementFrom0_mm*1000,true, false,true);

    //BELOW IS THE CONFIGURATION FOR THE Y AXIS (back and forth)
    ballScrew Y = ballScrew(limitSwitchYPin, yStepPin, yDirPin, stepsPerRev, yPitch_mm, maxSpeed_mmps, YmaxDisplacementFrom0_mm*1000,true, false,true);

    //BELOW IS ballscrew for z axis NEMA 14, note this motor is rated for 0.9A, 2mm pitch
    //ballScrew Z = ballScrew(zStepPin, zDirPin, stepsPerRev, zPitch_mm, 30, ZmaxDisplacementFrom0_mm*1000,false, true,false);
    ballScrew Z = ballScrew(limitSwitchZPin, zStepPin, zDirPin, stepsPerRev, zPitch_mm, maxSpeed_mmps, ZmaxDisplacementFrom0_mm*1000,false, true,false);

    //NOTE THAT CALLING THAT CREATING THE BALL SCREW OBJ SHOULD HAVE INITIALIZED PINS
    axis[0] = X;
    axis[1] = Y;
    axis[2] = Z;

};


void gantry::homeGantry(){
    isEmergencyStopped = false;
    for(int i = NUMBER_OF_MOTORS-1; i >= 0; i--){
        axis[i].homingBallScrew();
        if(isEmergencyStopped){
            return;
        }
    }
    isHomed = true;
}
int gantry::getMaxZDisplacement(){
  return ZmaxDisplacementFrom0_mm;
}

int gantry::getMaxYDisplacement(){
  return YmaxDisplacementFrom0_mm;
}

int gantry::getMaxXDisplacement(){
  return XmaxDisplacementFrom0_mm;
}

int gantry::getXDisplacement_um(){
  return axis[0].getDisplacement_um();
}

bool gantry::isGantryHomed(){
  return isHomed;
}


void gantry::emergencyStop(){
    isEmergencyStopped = true;
    isHomed = false;
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
        axis[i].emergencyStopMotor();
    }
}
void gantry::goToAbsPosition_mm(float x, float y, float z, int time_s){
  goToAbsPosition(x*1000,y*1000,z*1000,time_s*1000);
}
void gantry::goToAbsPosition(int x, int y, int z, int time_ms){
    int coordinates[NUMBER_OF_MOTORS] = {x, y, z};
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
        axis[i].moveToAbsPosition(coordinates[i]);
    }
    runMotors(time_ms);
}

void gantry::goToRelativePosition(int x, int y, int z, int time_ms){
    int displacement[NUMBER_OF_MOTORS] = {x, y, z};
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
        axis[i].moveToRelativePosition(displacement[i]);
    }
    runMotors(time_ms);
}



bool gantry::allStepsDone(){
  bool stepsDone = true;
  for(int i = 0; i < NUMBER_OF_MOTORS; i++){
    if(axis[i].getRemainingSteps() > 0){
      stepsDone = false;
      break;
    }
  }  
  return stepsDone;
}

unsigned long  gantry::getMinimumTimeToCompleteSteps(unsigned long desiredTime_ms){
    unsigned long longestTime = 0;
    unsigned long timeRequired = 0;
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
        timeRequired = axis[i].getMinimumTimeToCompleteSteps(desiredTime_ms);
        if(timeRequired > longestTime){
            longestTime = timeRequired;
        }
    }
  return longestTime;
}



void gantry::runMotors(int completionTime_ms){
  //get steps to move, then go to the given position

  //must figure out which motor has to go the farthest to set the total time
  bool stepsDone = false;


    //NOTE COULD ALTERNATIVELY HAVE THIS AS A FUCNTION WITH NO WHILE LOOP RUNNING IN THE FOREVER LOOP SO CAN ADDITIONALLY CHECK STATUS OF OTHER THINGS, current implementation is more precise but blocks code
    unsigned long totalTime_us = getMinimumTimeToCompleteSteps(completionTime_ms);

  setStepDelays(totalTime_us);

  //Poll timer until all motors are finished moving
  while (!stepsDone)
  {
    currentTime_us = micros();
    //reset every time
    checkAndTakeStep();
    stepsDone = allStepsDone();
  }

  if(isEmergencyStopped){
    return;
  }

  //can optionally update displacement at the end based on the total steps, or could update incrementally each step, here I just do it once but should change later to allow for non blocking implementation and error checking of out of bounds position
  for(int i = 0; i < NUMBER_OF_MOTORS; i++){
    axis[i].updateDisplacement();
  }
}


void gantry::setStepDelays(unsigned long totalTime_us){
  currentTime_us = micros();
  for(int i = 0; i < NUMBER_OF_MOTORS; i++){
    if(axis[i].getRemainingSteps() > 0){
      axis[i].setPulseDelay_us(totalTime_us/axis[i].getRemainingSteps());
    }
    else{//if 0 steps then set dealy to max but even if it does go off there is a != 0, so it wont step, mostly to prevent /0
      axis[i].setPulseDelay_us(totalTime_us);
    }
    axis[i].prevTime = currentTime_us;
  }
}

//MUST UPDATE TOTAL STEPS OF EACH MOTOR WHEN TAKING A STEP!!!!!

void gantry::checkAndTakeStep(){
  bool takeStep = false;
  for(int i = 0; i < NUMBER_OF_MOTORS; i++){
    //This implementation protects against micros() overflowing and returning back to 0
    //When a small unsigned is subtracted by a larger unsigned, normally if signed the answer would be negative (2's complement reprewsentation)
    //Since it is unsigned it does not interpret the signed bit so instead of a negative result you get a huge number like 2^63 (msb = 1), so even
    //when micros overflows and you would get a negative value for that single iteration, it can handle itself, and after that instance, the prev value is set small so will
    //return to conventional big - small
    if(currentTime_us - axis[i].prevTime  > axis[i].getDelay_us() && axis[i].getRemainingSteps() > 0){
      axis[i].timeToStep = true;
      axis[i].remainingSteps--;
      axis[i].prevTime = currentTime_us;
      takeStep = true;

      if(axis[i].direction == axis[i].positiveDirection){
        axis[i].totalSteps++;
        //THIS IS WHERE I COULD UPDATE DISPLACEMENT INCREMENTALLY FOR THE NON_BLOCKING IMPLEMENTATION
      }
      else{
        axis[i].totalSteps--;
      }
    }
  }
  if (takeStep){
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
      if(axis[i].timeToStep){
        digitalWrite(axis[i].getStepPin(), HIGH);
        axis[i].timeToStep = false;
      }
    }
    delayMicroseconds(axis[0].minimumMicrosHigh);
    for(int i = 0; i < NUMBER_OF_MOTORS; i++){
        digitalWrite(axis[i].getStepPin(), LOW);
    }
    delayMicroseconds(axis[0].minimumMicrosHigh);
  } 
}
