#pragma once
#include "Arduino.h"

class ballScrew {
  public:

    //used for gantry implementation
    unsigned long prevTime;
    bool timeToStep = false;


    //
    //
    //bool positiveDirection = true;//make this a private variable, don't want to be able to cahnge it, maybe even make it const for testing purposes
    //CONSIDER USING VARIABLE FOR FORWARD DIRECTION so then can just toggle it and don't need to look at all the true false and not sure what it is, either it gets closer or farther 
    //  


    //static volatile unsigned long prevInteruptTime;// = 0; NOTE THIS MUST BE SET TO 0 AT THE BEGENING OF CODE BEFORE HOMING or could just set in the homing function.
    static volatile bool switchTriggered;
    //static bool switchTriggered;

    ballScrew();
    ballScrew(int limitSwitchPin, int stepPinNumber, int directionPinNumber, int stepsPerRevolutions, int pitchMm, int maxSpeed,  int maxDis_um,  bool positiveDir, bool isHomingDirPositive, bool isHomeZero);
    void initializeMotorPins();
    void setStepDelays(unsigned long totalTime_us);
    //NOTE I DONT KNOW if ISR WILL BE LINKED TO INITIAL OBJ OR ARRAY OBJ so maybe leave as function to set and then can call when ready
    void homingBallScrew();
    void setHomePosition();
    void runMotor(int completionTime_ms);
    void moveToAbsPosition(int disp_um);
    void moveToRelativePosition(int dis_um);
    void setSteps(unsigned int steps, bool dir);
    int getDirPin();
    int getStepPin();
    int getLinearSteps(int length_um);
    void updateDisplacement();
    int getDisplacement_um();
    void emergencyStopMotor();
    unsigned long  getMinimumTimeToCompleteSteps(unsigned long desiredTime_ms);
    void setPulseDelay_us(int delayUs);
    int getDelay_us();

    int getRemainingSteps();
    void checkLimitSwitch();
    int remainingSteps = 0;//make this private again later and add proper accessors and mutators, just easier to make public for now.
    bool direction;
    bool positiveDirection;
    int totalSteps = 0;
    int minimumMicrosHigh = 1;



  private:
    bool isLimitSwitchPressed;
    int dirPin;
    int stepPin;
    int limitPin;
    int pitch_mm;
    int maxSpeed_mmps = 20;
    int maxDisplacement_um;
    int homingDelay_us = 100;
    int minPulseWidth = 50;
    int displacement_um = 0;
    bool homingDone = false;
    int stepsPerRevolution = 3200;
    int delay_us = 0;
    bool forcedStop = false;
    
    //bool positiveDirection;//INDICATE WHICH DIRECTION IS POSITIVE
    bool isHomingDirectionPositive;
    bool isHomingSideZero;

};
