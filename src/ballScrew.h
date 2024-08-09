#pragma once
#include "Arduino.h"

class ballScrew {
  public:
    //used for gantry implementation
    unsigned long prevTime;
    bool timeToStep = false;
    static volatile bool switchTriggered;

    ballScrew();
    ballScrew(int limitSwitchPin, int stepPinNumber, int directionPinNumber, int stepsPerRevolutions, int pitchMm, int maxSpeed,  int maxDis_um,  bool positiveDir, bool isHomingDirPositive, bool isHomeZero);
    void initializeMotorPins();
    void setStepDelays(unsigned long totalTime_us);
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
    
    bool isHomingDirectionPositive;
    bool isHomingSideZero;

};
