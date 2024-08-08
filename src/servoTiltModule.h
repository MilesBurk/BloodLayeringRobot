#pragma once
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//PWM specifications for servo motors being used
#define MIN_Width  500
#define MAX_WIDTH  2500
#define freq  50

//offsets in degrees
#define tube1_Offset 11
#define tube2_Offset 5
#define tube3_Offset 9
#define tube4_Offset -2

//tube pins from left most to right
#define tube1 15
#define tube2 9
#define tube3 4
#define tube4 0

#define firstTubeGap 92
#define secondTubeGap 94
#define thirdTubeGap 94

#define NUM_TUBES 4



class servoTiltModule{
    public:
        servoTiltModule();
        int getAbsoluteStartingXPositionOfTube(int startingX_mm, int tubeNum);
        void goDirectlyToTubeAngle(int angle, int tubeNum);
        void sweepTubeToAngle(int angle, float time_s, int tubeNum);
        int getNumberOfTubes();
        void setAllTubesToAngle(int angle_deg);
        Adafruit_PWMServoDriver pwm;
        static volatile bool isForcedStop;


        //basically I want to only interact with the tube tilting module for initialization. and to either go directly to an angle or sweep to an angle
    private:
        int tubeSideToSideGapsOffsets_mm[4] = {0, firstTubeGap, secondTubeGap, thirdTubeGap};
        int servoPos_pulse[4] = {0, 0, 0, 0};
        int tubePins[4] = {tube1, tube2, tube3, tube4};
        int tubeOffsets[4] = {tube1_Offset, tube2_Offset, tube3_Offset, tube4_Offset};
};