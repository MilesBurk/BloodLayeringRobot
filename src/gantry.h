#pragma once
/*
#ifndef gantry_h
#define gantry_h
#endif
#ifndef ballScrew_h
#define ballScrew_h
#endif
*/

#include "Arduino.h"
#include "ballScrew.h"

class gantry{
  public:
    //Choose to define motors etc in the gantry class so bloat can be contained, normally wont have multiple gantrys
    gantry();
    void homeGantry();
    void goToAbsPosition(int x, int y, int z, int time_ms);
    void goToAbsPosition_mm(float x, float y, float z, int time_s);
    void goToRelativePosition(int x, int y, int z, int time_ms);
    void emergencyStop();
    int getMaxZDisplacement();
    int getMaxYDisplacement();
    int getMaxXDisplacement();

    int getXDisplacement_um();

    bool isGantryHomed();
    //To be completed Later for enabeling and dissabling
    void powerDown();
    void powerUp();

  private:
    ballScrew axis[3];
    unsigned long currentTime_us = 0;    
    short const NUMBER_OF_MOTORS = 3;
    bool isHomed = false;
    bool isEmergencyStopped = false;

    //void emergencyStop();

    bool allStepsDone();
    void checkAndTakeStep();
    void setStepDelays(unsigned long totalTime_us);
    void initializeMotorPins();
    //int getMostSteps();
    void runMotors(int completionTime_ms);
    unsigned long getMinimumTimeToCompleteSteps(unsigned long desiredTime_ms);
    
};