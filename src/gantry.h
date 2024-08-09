#pragma once
#include "Arduino.h"
#include "ballScrew.h"

//MUST ADD ALL PIN CONNECTIONS
#define xStepPin 19
#define xDirPin 18
#define yStepPin 5
#define yDirPin 17
#define zStepPin 16 
#define zDirPin 4
#define xPitch_mm 5
#define yPitch_mm 5
#define zPitch_mm 2
#define limitSwitchXPin 36
#define limitSwitchYPin 39
#define limitSwitchZPin 34
#define XmaxDisplacementFrom0_mm 285
#define YmaxDisplacementFrom0_mm 140
#define ZmaxDisplacementFrom0_mm 200
#define maxSpeed_mmps 24 
#define stepsPerRev 3200//assuming all are the same microstepping

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

    bool allStepsDone();
    void checkAndTakeStep();
    void setStepDelays(unsigned long totalTime_us);
    void initializeMotorPins();
    //int getMostSteps();
    void runMotors(int completionTime_ms);
    unsigned long getMinimumTimeToCompleteSteps(unsigned long desiredTime_ms);
    
};