#include "Arduino.h"
#include "servoTiltModule.h"

volatile bool servoTiltModule::isForcedStop = false;

servoTiltModule::servoTiltModule(){

}

int servoTiltModule::getAbsoluteStartingXPositionOfTube(int startingX_mm, int tubeNum){
  if(tubeNum < 1 || tubeNum > NUM_TUBES){
    return 0;
  }
  int absoluteStartOfTube = startingX_mm;
  //add all the gaps.
  for(int i = 0; i < tubeNum; i++){
    absoluteStartOfTube = absoluteStartOfTube + tubeSideToSideGapsOffsets_mm[i];
  }
  return absoluteStartOfTube;

}

void servoTiltModule::setAllTubesToAngle(int angle_deg){
    for(int i = 0; i < NUM_TUBES; i++){
        goDirectlyToTubeAngle(angle_deg, i+1);
        delay(500);
    }
}


//Note that positive angle is when tube tilts away from user standing at front, this is done because we normally want like 60 degrees
//offset is in degrees and is used to calibrate to find true zero position for consistency accross pieces
void servoTiltModule::goDirectlyToTubeAngle(int angle, int tubeNum){
  //first check to make sure the requested tube is valid
  if(tubeNum < 1 || tubeNum > NUM_TUBES || isForcedStop){
    return;
  }
  angle = angle + tubeOffsets[tubeNum-1];
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
  pwm.setPWM(tubePins[tubeNum-1], 0, pulseWidth);
  servoPos_pulse[tubeNum-1] = pulse_wide;
}

void servoTiltModule::sweepTubeToAngle(int angle, float time_s, int tubeNum){

  if(tubeNum < 1 || tubeNum > NUM_TUBES || isForcedStop){
    return;
  }
  angle = angle + tubeOffsets[tubeNum-1];
  int pulseWidth, delay_ms;
  int minRes = 5;//microsecond deadband
  int pulse_wide = map(angle, -90,90,MIN_Width,MAX_WIDTH);
  int diff = pulse_wide - servoPos_pulse[tubeNum-1];
  pulse_wide = servoPos_pulse[tubeNum-1];

  //if positive difference then need to move forward, else move backwards
  if (diff > 0){
    int ticks = diff/minRes;
    delay_ms = 1000*time_s/ticks;
    for(int i = 0; i < ticks; i++){
          if(isForcedStop){return;};
          pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
          pwm.setPWM(tubePins[tubeNum-1], 0, pulseWidth);
          delay(delay_ms);
          pulse_wide = pulse_wide + minRes;
    }
    servoPos_pulse[tubeNum-1] =  pulse_wide - minRes;
  }
  else if(diff < 0){
    int ticks = -diff/minRes;
    delay_ms = 1000*time_s/ticks;
    for(int i = 0; i < ticks; i++){
      if(isForcedStop){return;};
      pulseWidth = int(float(pulse_wide)/ 1000000 * freq * 4096);
      pwm.setPWM(tubePins[tubeNum-1], 0, pulseWidth);
      delay(delay_ms);
      pulse_wide = pulse_wide - minRes;
    }
    servoPos_pulse[tubeNum-1] = pulse_wide + minRes;
  }
}

int servoTiltModule::getNumberOfTubes(){
    return NUM_TUBES;
}


