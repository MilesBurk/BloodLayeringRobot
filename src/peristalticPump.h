#pragma once
#include "Arduino.h"

#define pumpPin 23
#define pumpMicrosteps 16
#define pumpDirectionPin 15 //NOTE THAT FORWARD DIRECTION FOR THE PUMP IS TRUE/HIGH

class peristalticPump{
    public:
        peristalticPump();
        static void onTimer();
        void setPumpRPM(int rpm);
        void setPumpDirection(bool dir);//true is forward, false is reverse
        static void stopPump();
        hw_timer_t *timer = NULL;      //H/W timer defining (Pointer to the Structure)
        static volatile bool isPumpOn;
        static volatile bool isForcedStop;


    private:
        static volatile bool pumpDirection;//true is forward false is backwards
        static volatile bool pinState;

};


