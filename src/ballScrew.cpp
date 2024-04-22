#include "ballScrew.h"
#include "Arduino.h"

#define testLED 25

volatile bool ballScrew ::switchTriggered = false;

ballScrew :: ballScrew(){
    //empty default constructor so can at least initialize gantry array
}

ballScrew :: ballScrew(int stepPinNumber, int directionPinNumber, int stepsPerRevolutions, int pitchMm, int maxSpeed,  int maxDis_um,  bool positiveDir, bool isHomingDirPositive, bool isHomeZero){
    
    //pinMode(testLED, OUTPUT);
    //digitalWrite(testLED, LOW);

    
    stepPin = stepPinNumber;
    dirPin = directionPinNumber;
    stepsPerRevolution = stepsPerRevolutions;
    pitch_mm = pitchMm;
    maxSpeed_mmps = maxSpeed;
    maxDisplacement_um = maxDis_um;
    positiveDirection = positiveDir;
    direction = positiveDirection;
    isHomingDirectionPositive = isHomingDirPositive;
    isHomingSideZero = isHomeZero;
    initializeMotorPins();
    //setLimitSwitchInterupt();
    //does go through here
    pinMode(testLED,OUTPUT);
    digitalWrite(testLED, LOW);

}

void ballScrew::setPulseDelay_us(int delayUs){
    delay_us = delayUs;
}
int ballScrew::getDelay_us(){
    return delay_us;
}

void ballScrew::moveToAbsPosition(int disp_um){
    //start mith micros but consider switching to floating point mm after proof of concept
    
    //is this more of a set function or a run function?
    //I think I want this to be a set function so I will use a different naming convention.
    int relativeDistance_um;
    if(disp_um > maxDisplacement_um){
        disp_um = maxDisplacement_um;
    }
    else if(disp_um < 0 ){
        disp_um = 0;
    }
    relativeDistance_um = disp_um - displacement_um;
    moveToRelativePosition(relativeDistance_um);
}

void ballScrew::moveToRelativePosition(int dis_um){

    //Confirm move will be within bounds
    if(dis_um + displacement_um > maxDisplacement_um){
        dis_um = maxDisplacement_um - displacement_um;
    }
    else if(dis_um + displacement_um < 0 ){
        dis_um = -displacement_um;
    }

    //determine direction of motion
    bool travelDirection;
    if (dis_um >= 0){
        travelDirection = positiveDirection;
    }
    else{
        travelDirection = !positiveDirection;
        dis_um = dis_um*-1;//make direction positive again
    }
    setSteps(getLinearSteps(dis_um),travelDirection);
}

void ballScrew::updateDisplacement(){
    displacement_um = totalSteps*(pitch_mm*1000)/stepsPerRevolution;
}

void ballScrew::setStepDelays(unsigned long totalTime_us){
    
    if(remainingSteps != 0){
      delay_us = totalTime_us/remainingSteps;
    }
    else{//if 0 steps then set dealy to max but even if it does go off there is a != 0, so it wont step, mostly to prevent /0
      delay_us = totalTime_us;
    }
}

unsigned long  ballScrew::getMinimumTimeToCompleteSteps(unsigned long desiredTime_ms){
    //must ensure that the time is able to be accomodated, have option for speed as well.
    //ensure motor is homed
    //consider where I want to check the limits
    //probably gonna have to do this in the setSteps function


    //get steps to move, then go to the given position

    //must figure out which motor has to go the farthest to set the total time

    unsigned long totalTime_us;
    int minimumPossibleTime_us = remainingSteps*minPulseWidth;
    //If given 0 or an impossible time then go as fast as possible
    if (desiredTime_ms == 0 || desiredTime_ms < minimumPossibleTime_us/1000){
    //set delay to minimum/deafault
        totalTime_us = minimumPossibleTime_us;
    }
    else{
        totalTime_us = desiredTime_ms * 1000;
    }


    //Additional check based on speed, I think this one is more releavent because prev min time is based on no load.
    int distanceToGo_um = remainingSteps*(pitch_mm*1000)/stepsPerRevolution;
    int fastestTime_us = 1000*distanceToGo_um/(maxSpeed_mmps);//max speed is in mm per s
    if (totalTime_us < fastestTime_us){
        totalTime_us = fastestTime_us;
    }
    return totalTime_us;
}


void ballScrew::runMotor(int completionTime_ms){
    if(!homingDone){
        return;
    }


  unsigned long totalTime_us = getMinimumTimeToCompleteSteps(completionTime_ms);

  //set delay for each motor and the next step
  setStepDelays(totalTime_us);

  //Poll timer until all motors are finished moving
  while (remainingSteps > 0)
  {
    digitalWrite(getStepPin(), HIGH);
    delayMicroseconds(minimumMicrosHigh);
    digitalWrite(getStepPin(), LOW);
    remainingSteps--;
    delayMicroseconds(delay_us);
    if(direction == positiveDirection){
        totalSteps++;
    }
    else{
        totalSteps--;
    }
  }
  if(forcedStop){
    return;
  }
  updateDisplacement();
}


void ballScrew::setSteps(unsigned int steps, bool dir){
    if(!homingDone){
        return;
    }
    direction = dir;
    remainingSteps = steps;
    digitalWrite(dirPin, direction);//SHOULD MOVE  to runMotor code
    /*
        if(direction){
        digitalWrite(dirPin, HIGH);
    }
    else{
        digitalWrite(dirPin, LOW);
    }
    */
}

int ballScrew::getRemainingSteps(){
    return remainingSteps;
}

void ballScrew::emergencyStopMotor(){
    remainingSteps = 0;
    forcedStop = true;
    homingDone = false;
}

void ballScrew::initializeMotorPins(){
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, positiveDirection);
}



int ballScrew::getLinearSteps(int length_um){
  //note this does not have the specific motor must fix
  //motor1.remainingSteps = length_um*motor1.stepsPerRevolution/(pitchmm * 1000)
    return length_um*stepsPerRevolution/(pitch_mm * 1000);

}

void ballScrew::homingBallScrew(){
    //very importatn to set to 0 first

    //note the big downside to this implementation is that if the switch is hit, the motor will only stop if it is homing, if it is not then it can hit but that would only happen if it skips steps, and thus is the risk you take with open loop control
    //delay(1000);

    forcedStop = false;
    //problem is it wont instantly stop the it will still take those additional 500 um step which can be bad maybe, basically would need to wait for runMotor to finish
    switchTriggered = false;
    int distance_um_FromSwitch = 3000;
    int stepsToTake = getLinearSteps(distance_um_FromSwitch);
    //prevInteruptTime = 0;
    //int distance_um = 2000;
    //int stepsToTake = getLinearSteps(distance_um, pitch_mm);
    //homingDone = false;
    /*
        while(!homingDone){
        //I think if I go this approach would need some custome homing function where it actually checks each step that way it stops as soon as interupt is triggered

        setSteps(stepsToTake, false);
        runMotor(500);
    }
    */
   //will need to check this speed with the different motor tupes
   //int homingDelay_us = 100; //set different homing delays depending on pitch, could make a 
    
    
    //digitalWrite(getDirPin(), false);//go in direction towards motor

    bool homingDir;
    if (isHomingDirectionPositive){
        homingDir = positiveDirection;
    }
    else{
        homingDir = !positiveDirection;
    }

    digitalWrite(getDirPin(), homingDir);

    //limit switch ISR will trigger this interupt
    while(!switchTriggered){
        digitalWrite(getStepPin(), HIGH);
        delayMicroseconds(minimumMicrosHigh);
        digitalWrite(getStepPin(), LOW);
        delayMicroseconds(homingDelay_us); 
        if(forcedStop){
            return;
            //break;
        }
        //just move motor at set speed step by step till home (note that this speed will be different and is pitch dependent)
    }

    //NOTE NEED TO MAKE ALL THESE NUMBERS variables that can be tuned at the begening of hte function, no random numbers allowed, i,e properly name all dealy etc, use proper data type too liek short or unsigned int, unsigned short
    //move1mm forward off the limit switch
    delay(200);

    homingDone = true;
    setSteps(stepsToTake, !homingDir);
    runMotor(1000);
    //NOW I DEFINE THIS LOCATION AS THE ZERO OF MY SYSTEM, INITIALIZE ALL ATTRIBUTES
    setHomePosition();

    //UNCOMMENT LATER
    digitalWrite(testLED, LOW);
}

void ballScrew::setHomePosition(){
    homingDone = true;
    remainingSteps = 0;

    if(isHomingSideZero){
        totalSteps = 0;
        displacement_um = 0;
    }
    else{
        displacement_um = maxDisplacement_um;//if at other end then must be at maximum position
        totalSteps = stepsPerRevolution*displacement_um/(pitch_mm*1000);
    }

}


/*
//I GUESS I NEED TO INCLUDE THE ISR's IN MAIN but luckily it is very simple.
*/
void ballScrew::limitSwitchISR(){
    /*
    int debounceTime_ms = 1000;
    if(millis() - prevInteruptTime >  debounceTime_ms){
        remainingSteps = 0;
        prevInteruptTime = millis();
        homingDone = true;
    }
    */
   //basically alternatively just use single ISR and fix the problem in code, this will work fine as long as we are homing 1 thing at a time, which we are because it will block the code
    //dont actually need any debouncing because I just care when it strikes switch for the first time
    switchTriggered = true;
    pinMode(testLED, OUTPUT);
            digitalWrite(testLED, LOW);

            digitalWrite(testLED, HIGH);
    //digitalWrite(testLED, HIGH);

    /*
        int debounceTime_ms = 500;
    if(millis() - prevInteruptTime >  debounceTime_ms){
        prevInteruptTime = millis();//note this doesnt even really matter here anymore
        switchTriggered = true;
    }
    */
}

/*//this did not work unfortunately
void ballScrew::setLimitSwitchInterupt(){
    pinMode(limitSwitchPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(limitSwitchPin), limitSwitchISR, RISING);
}
*/


int ballScrew::getDirPin(){
    return dirPin;
}

int ballScrew:: getStepPin(){
    return stepPin;
}


