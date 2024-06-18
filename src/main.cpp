#include "Arduino.h"
#include "gantry.h"
#include "ballScrew.h"
#include "servoTiltModule.h"
#include "peristalticPump.h"
#include "volumeSenseModule.h"
//For ESP NOW
#include <esp_now.h>
#include <WiFi.h>
//##########################################################

#define runButton 27 
#define homeButton 14
#define emergencyStopButton 13

//THIS roughly means the distance you want to nozzle to be at before entrance into the tube at 60 degrees
#define heightAbovePivot_um 55000
#define tubeWidth_mm 27


#define startingX_mm 0
#define startingY_mm 63

//# ########################################################################
//# ##############ESP-NOW Declaration#######################################
uint8_t broadcastAddress[] = {0xEC,0xDA,0x3B,0x8D,0x2A,0x04};// REPLACE WITH OTHER TRANSCEIVER MAC ADDRESS

// Structure example to send data
// Must match the receiver structure
typedef struct Message_Struct {
  boolean Run;
  boolean Home;
  boolean Stop;
  boolean Abort;
  uint16_t tubes[4];
  boolean Process; // Default
} Message_Struct;

Message_Struct message_object;
char dataRcv[15];

// Important 
esp_now_peer_info_t peerInfo;

void initializePushButtons();
void initializeInterupts();
void stopAllMotors();
void performFillingMotionforAll4();
void performFillingMotionFor1Tube(int tubeNumber);
void performFastFillingMotionFor1Tube(int tubeNumber);
void performFastFillingMotionForAll4();
void lockerStorageSequence();
void AdjustTubeValueAndSend();
void ResetTubeVolAndSend();
void delayWithAbort_ms(int delayTime_ms);

bool ESPNOWSendStatBool ;
bool PreventSendProcessWhenAbort;
volatile bool RunSignal = 0;
volatile bool AbortSignal = 0;
volatile bool aborted = 0;

uint16_t DisplayTubes[4];
//# ########################################################################
// ESP 2.0.3 FW version (Confirmed)
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {  //Automated function to show data is sent, ignore for now
  Serial.print("\r\nLast Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS){
    Serial.print("Delivery Success");
    ESPNOWSendStatBool = 1;
  }
  else{
    Serial.print("Delivery Fail");
  }
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  Serial.print("\r\nESP-Now Data Received\t");
  if (len == sizeof(Message_Struct)) {
    // Cast the received data pointer to Message_Struct
    Message_Struct *receivedData = (Message_Struct *)incomingData;

    // Set PreventSendProcessWhenAbort based on received Abort value
    PreventSendProcessWhenAbort = receivedData->Abort;

    // // Extract the received boolean values
    // boolean Run = receivedData->Run;
    // boolean Home = receivedData->Home;
    // boolean Stop = receivedData->Stop;
    // boolean Abort = receivedData->Abort; 

    Serial.println("Run :");
    Serial.print(receivedData->Run);

    Serial.println("Abort :");
    Serial.print(receivedData->Abort);

    Serial.println("Process :");
    Serial.print(receivedData->Process);

    // Run if Run = 1
    if(receivedData->Run == 1)
    {
      Serial.println("Received Run command");
      RunSignal = receivedData->Run;
      // performFillingMotionforAll4();
    }
    else
    {
      Serial.println("Not Running");
    }

    // Stop if Abort = 1
    if(receivedData->Abort == 1)
    {
      Serial.println("Received Abort command");
      AbortSignal = receivedData->Abort;
      stopAllMotors();
    }
    else
    {
      Serial.println("Not Aborting");
    }

  } 
  else {
    Serial.println("Received invalid data length");
  }
}

//# ########################################################################
//# ########################################################################

volumeSenseModule VolumeSensors = volumeSenseModule();
peristalticPump Pump = peristalticPump();

servoTiltModule TiltModule = servoTiltModule();
gantry Gantry = gantry();
int startingZ_mm  = Gantry.getMaxZDisplacement() - 50;

// void initializePushButtons();
// void initializeInterupts();
// void stopAllMotors();
// void performFillingMotionforAll4();
// void performFillingMotionFor1Tube(int tubeNumber);
// void lockerStorageSequence();


void setup() {
  //# ########################################################################
  // Set device as a Wi-Fi Station (FOR ESP NOW)
  // Init Serial Monitor
 	Serial.begin(115200);

 	WiFi.mode(WIFI_STA);
 
  // Init ESP-NOW
 	if (esp_now_init() != ESP_OK) {
 			Serial.println(F("Error initializing ESP-NOW"));
 			return;
 	}
 	Serial.print(F("Transceiver initialized : "));
 	Serial.println(WiFi.macAddress());
 	
 	// Define callback functions
 	esp_now_register_send_cb(OnDataSent);
 	esp_now_register_recv_cb(OnDataRecv);
 	
  // Register peer
 	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
 	peerInfo.channel = 0;
 	peerInfo.encrypt = false;
 	
  // Add peer
 	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
 			Serial.println(F("Failed to add peer"));
 			return;
 	}

  ESPNOWSendStatBool = 0; //Default is 0, 0 means process incomplete/idle/not in process, 1 = process complete
  PreventSendProcessWhenAbort = 0; //Default is 0, 0 means normal, 1 means Abort sending Process
  
  ResetTubeVolAndSend();
  //# ########################################################################

  //Initialize tilt module pwm
  TiltModule.pwm = Adafruit_PWMServoDriver(); // Uncomment if needed
  TiltModule.pwm.begin();
  TiltModule.pwm.setPWMFreq(freq);
  
  initializePushButtons();
  initializeInterupts();
}

int count = 0;
void loop() {
  if(count == 0)
  {
    Serial.println("In First Loop");
    count++;
  }

  // //Perform homing sequence and tilt tube holders to initial angles
  // if(digitalRead(homeButton) == HIGH){ // Uncomment if needed
  //   //go to upright angle
  //   TiltModule.setAllTubesToAngle(0);
  //   delay(1000);
  //   Gantry.homeGantry();
  //   //go to loading angle
  //   int loadingAngle = -30;
  //   TiltModule.setAllTubesToAngle(loadingAngle);
  // }

  // //Perform one of the filling sequences
  // if (digitalRead(runButton) == HIGH) // Uncomment if needed
  // {

  //   //performFillingMotionforAll4(); 
  //   //lockerStorageSequence(); 

  //   //ONLY TEST WITH FIRST TUBE, OTHER VOLUYME SENSORES ARE NOT YET CONNECTED
  //   performFillingMotionFor1Tube(2);
  // }
  // Serial.println("AbortSignal : ");
  // Serial.println(AbortSignal);

  if(RunSignal == 1 || digitalRead(runButton))
  { 
    Serial.println("Jumping to Run");

    //go to upright angle
    AbortSignal = 0;
    aborted = 0;
    Pump.isForcedStop = false;
    TiltModule.isForcedStop = false;
    TiltModule.setAllTubesToAngle(0);
    delay(1000);
    Gantry.homeGantry();


    
    //go to loading angle
    int loadingAngle = -30;
    TiltModule.setAllTubesToAngle(loadingAngle);

    RunSignal = 0;
    performFastFillingMotionForAll4();
    aborted = 0;


    if(AbortSignal == 1)
    {
      Serial.println("Aborted Signal received in void loop");
      ResetTubeVolAndSend();
      stopAllMotors();
    }
    else
    {
      Serial.println("Run seccessfully, now quitting if and go to void loop");
    }
    AbortSignal = 0;
  }
}


// Modified performFillingMotionFor1Tube function
void performFillingMotionFor1Tube(int tubeNumber){
  Serial.println("Performing performFillingMotionFor1Tube...");

  delayWithAbort_ms(5000); //Delay for 5s

  //Make sure that the user is calling a valid tube 
  if (tubeNumber > 4 || tubeNumber < 1){
    return;
  }

  volumeSenseModule::performingFinalFill = false;
  VolumeSensors.currentTubeBeingFilled = tubeNumber;
  int startingXPosition_mm = TiltModule.getAbsoluteStartingXPositionOfTube(startingX_mm, tubeNumber);

  //only for testing get rid after
  delayWithAbort_ms(1000);

  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delayWithAbort_ms(3000);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);


  TiltModule.goDirectlyToTubeAngle(0, tubeNumber);

  delayWithAbort_ms(1000);

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

  delayWithAbort_ms(1000);

  int TUBE_ANGLE_OFFSET_FOR_INSERTION = 15;
  //FIRST MOVE TUBE 5 degs so there are no collisions then move back
  TiltModule.sweepTubeToAngle(firstFillAngle + TUBE_ANGLE_OFFSET_FOR_INSERTION, 1, tubeNumber);


  int entranceDistance_um = 63500;
  //slide into tube very slowly as deep as posssible
  Gantry.goToRelativePosition(0, -entranceDistance_um*sin(PI*(firstFillAngle)/float(180)), -entranceDistance_um*cos(PI*(firstFillAngle)/float(180)), 5000);
   
  TiltModule.sweepTubeToAngle(firstFillAngle, 1, tubeNumber);

  //initizl pump prime
  Pump.setPumpRPM(300);
  delayWithAbort_ms(1900);
  Pump.setPumpRPM(0);
  delayWithAbort_ms(1000);

  //ADD CODE HERE TO DO INITIAL FILLING WHILE THE TUBE IS FULLY IN.
  Pump.setPumpRPM(10);
  delayWithAbort_ms(30000);


  //ONCE THE BLOOD HAS REACHED WHERE THE NOZZLE IS THE CONTINUE TO NEXT SECTION.

  ///find distance to move out of the tube
  //this is the diagonal distance out of the tube you with to travel, I assume it is just 1cm shy of where you started so as to ensure you are in the tube at the end
  int exitDistance_um = entranceDistance_um - 11000;
  //the line below pulls the tube out in 1 shot where as the loop lets you set different pump speeds as you pull it out.
  //Gantry.goToRelativePosition(0, exitDistance_um*sin(PI*firstFillAngle/float(180)), exitDistance_um*cos(PI*firstFillAngle/float(180)), 5000);

  //HERE I LET YOU DO DIFFERENT PUMP SEQUENCES AS YOU FILL IT UP
  int pumpRPMS[] = {14, 16, 18, 25, 30, 0};
  //MAKE SURE BOTH THESE ARRAYS HAVE SAME NUMBER OF ELEMENTS!!
  int delays_ms_Per_pumpingInterval[] = {30000, 30000, 20000, 20000, 22000, 1000};

  int numberOfPumpingSequencesWhileExitingTube = sizeof(pumpRPMS)/sizeof(int);
  int exitDistancePerPumpSequence_um = exitDistance_um/numberOfPumpingSequencesWhileExitingTube;

  for(int i = 0; i < numberOfPumpingSequencesWhileExitingTube; i++){
    //set new pumping speed and slide gantry up tube, take sliding time as delay before new pumping speed
    Pump.setPumpRPM(pumpRPMS[i]);
    Gantry.goToRelativePosition(0, exitDistancePerPumpSequence_um*sin(PI*firstFillAngle/float(180)), exitDistancePerPumpSequence_um*cos(PI*firstFillAngle/float(180)), delays_ms_Per_pumpingInterval[i]);
  }

  //take the nozzle out by traveling straight up.
  Gantry.goToRelativePosition(0, 0, 40000, 5000);

  // straighten out 
  int finalFillAngle = -10;
  TiltModule.sweepTubeToAngle(finalFillAngle, 1, tubeNumber);

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  int depthBelowTubeTop_um = 25000;//this is for the depth along the tube wall you want the final fill position to be at.

  //take the nozzle to tube wall, old logic
  Gantry.goToRelativePosition(0, -(tubeWidth_mm*1000/2 + sin(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um), -cos(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um, 5000);

  Pump.setPumpRPM(50);
  volumeSenseModule::performingFinalFill = true;

  Serial.println("Reached above the volume line");
  //here you would fill until the volume sensors is triggered.
  while(!volumeSenseModule::timeToStopPump() && !aborted){};
  //basically wait until the pump turns itself off.

  peristalticPump::stopPump();
  volumeSenseModule::performingFinalFill = false;

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  //go back to center
  TiltModule.sweepTubeToAngle(0, 2, tubeNumber);

  //will need to be changed so it just runs once at the end of filling four tubes
  delayWithAbort_ms(1000);

  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delayWithAbort_ms(3000);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);

  delayWithAbort_ms(500);
  Serial.println("0.5 second");
  if (PreventSendProcessWhenAbort) return;
  DisplayTubes[tubeNumber-1] = 100;
  // Call tube vol update func
  AdjustTubeValueAndSend();

  Serial.println("# ////////////////////////////////////////////////////////////////////");
}


void performFillingMotionforAll4(){
  //cycle through all 4 motions.
  for(int i = 0; i < TiltModule.getNumberOfTubes(); i++){
    if (PreventSendProcessWhenAbort) return;
    performFillingMotionFor1Tube(i+1);
  }
  if (PreventSendProcessWhenAbort) return;
  //once it is finished then go to top left
  Gantry.goToAbsPosition_mm(0, Gantry.getMaxYDisplacement(), Gantry.getMaxZDisplacement(), 10);
  // #############################################################################################
  // Send ESP-Now Process = 1
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  Serial.println("Tubes Filled !");
  Serial.println("Sending ESP-NOW Process");
  ESPNOWSendStatBool = 0;

  message_object.Process = 1;
  int attempt = 0;
  for (attempt = 0; attempt < 20; attempt++) {
    // Simulate some operation that assigns a value to 'result'
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message_object, sizeof(message_object));
    delay(50); // 1000 = 1s
    Serial.println();
    Serial.printf("Attempt %d: Result = %d\n", attempt + 1, result);
    Serial.println();
    // Check if the result is ESP_OK
    if (ESPNOWSendStatBool == 1) 
    {
      Serial.println("Process confirm sent successful, breaking the loop.");
      ESPNOWSendStatBool = 0;
      break;
    }
  }
  AbortSignal = 0;
  message_object.Process = 0;

  if (attempt == 21) {Serial.println("Max attempts on sending Process confirm reached without success.");}
  ResetTubeVolAndSend();
  // #############################################################################################
}

void stopAllMotors(){
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  Serial.print("In stopAllMotors");
  Gantry.emergencyStop();
  Pump.stopPump(); 
  aborted = 1;
  Pump.isForcedStop = true;
  TiltModule.isForcedStop = true;

  Serial.println("Motors stopped");
  message_object.Process = 0;
  ESPNOWSendStatBool = 0;
  
  int attempt = 0;
  for (attempt = 0; attempt < 20; attempt++) 
  {
    // Simulate some operation that assigns a value to 'result'
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message_object, sizeof(message_object));
    //delay(800); // 1000 = 1s
    Serial.println();
    Serial.printf("Attempt %d: Result = %d\n", attempt + 1, result);
    Serial.println();
    // Check if the result is ESP_OK
    if (ESPNOWSendStatBool == 1) 
      {
        Serial.println("Abort Confirm, breaking the loop.");
        ESPNOWSendStatBool = 0;
        break;
      }
  }

  if (attempt == 21) {Serial.println("Max attempts on sending Abort Confirm reached without success.");}
  AbortSignal = 0;
}

void initializePushButtons(){
  //Initialize buttons
  pinMode(runButton, INPUT);
  pinMode(homeButton, INPUT);
  pinMode(emergencyStopButton, INPUT);
}

void initializeInterupts(){
  //Add external interupt for emergency stop button  
  attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, RISING);
  //Interupt for pump control
  timerAttachInterrupt(Pump.timer, peristalticPump::onTimer, true); 	// Attach interrupt for pump
}

void lockerStorageSequence(){
  Gantry.goToAbsPosition_mm(Gantry.getMaxXDisplacement()/2, Gantry.getMaxYDisplacement()/2, Gantry.getMaxZDisplacement(), 10);
}

void AdjustTubeValueAndSend()
{
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  memcpy(message_object.tubes,DisplayTubes, sizeof(DisplayTubes));
  // Send Volume update
  ESPNOWSendStatBool = 0;
  int attempt;  
  for (attempt = 0; attempt < 20; attempt++) {
      // Simulate some operation that assigns a value to 'result'
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message_object, sizeof(message_object));
      delay(00); // 1000 = 1s
      Serial.println();
      Serial.printf("Attempt %d: Result = %d\n", attempt + 1, result);
      Serial.println();
      // Check if the result is ESP_OK
      if (ESPNOWSendStatBool == 1) 
      {
        Serial.println("Tube volume update sent successful, breaking the loop.");
        ESPNOWSendStatBool = 0;
        break;
      }
    }

  if (attempt == 21) {Serial.println("Max attempts on sending Process confirm reached without success.");}
  
}

void ResetTubeVolAndSend()
{
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  for(int i = 0; i<4 ; i++)
  {
    DisplayTubes[i] = 0;
  }
  memcpy(message_object.tubes,DisplayTubes, sizeof(DisplayTubes));
  // Send Volume update
  ESPNOWSendStatBool = 0;
  int attempt;  
  for (attempt = 0; attempt < 20; attempt++) {
      // Simulate some operation that assigns a value to 'result'
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message_object, sizeof(message_object));
      delay(500); // 1000 = 1s
      Serial.println();
      Serial.printf("Attempt %d: Result = %d\n", attempt + 1, result);
      Serial.println();
      // Check if the result is ESP_OK
      if (ESPNOWSendStatBool == 1) 
      {
        Serial.println("Tube volume RESET update sent successful, breaking the loop.");
        ESPNOWSendStatBool = 0;
        break;
      }
    }

  if (attempt == 21) {Serial.println("Max attempts on sending Process confirm reached without success.");}
  
}

void performFastFillingMotionForAll4(){
    //cycle through all 4 motions.
  for(int i = 0; i < TiltModule.getNumberOfTubes(); i++){
    performFastFillingMotionFor1Tube(i+1);
  }

  if (PreventSendProcessWhenAbort) return;
  //once it is finished then go to top left
  Gantry.goToAbsPosition_mm(0, Gantry.getMaxYDisplacement(), Gantry.getMaxZDisplacement(), 10);
  // #############################################################################################
  // Send ESP-Now Process = 1
  Serial.println("# ////////////////////////////////////////////////////////////////////");
  Serial.println("Tubes Filled !");
  Serial.println("Sending ESP-NOW Process");
  ESPNOWSendStatBool = 0;

  message_object.Process = 1;
  int attempt = 0;
  for (attempt = 0; attempt < 20; attempt++) {
    // Simulate some operation that assigns a value to 'result'
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message_object, sizeof(message_object));
    delay(500); // 1000 = 1s
    Serial.println();
    Serial.printf("Attempt %d: Result = %d\n", attempt + 1, result);
    Serial.println();
    // Check if the result is ESP_OK
    if (ESPNOWSendStatBool == 1) 
    {
      Serial.println("Process confirm sent successful, breaking the loop.");
      ESPNOWSendStatBool = 0;
      break;
    }
  }
  AbortSignal = 0;
  message_object.Process = 0;

  if (attempt == 21) {Serial.println("Max attempts on sending Process confirm reached without success.");}
  ResetTubeVolAndSend();
  // #############################################################################################
}

//for testing purposes only
void performFastFillingMotionFor1Tube(int tubeNumber){
   Serial.println("Performing performFastFillingMotionFor1Tube...");

  delayWithAbort_ms(1000); //Delay for 5s

  //Make sure that the user is calling a valid tube 
  if (tubeNumber > 4 || tubeNumber < 1){
    return;
  }

  volumeSenseModule::performingFinalFill = false;
  VolumeSensors.currentTubeBeingFilled = tubeNumber;
  int startingXPosition_mm = TiltModule.getAbsoluteStartingXPositionOfTube(startingX_mm, tubeNumber);

  //only for testing get rid after
  delayWithAbort_ms(1000);

  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delayWithAbort_ms(1000);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);


  TiltModule.goDirectlyToTubeAngle(0, tubeNumber);

  delayWithAbort_ms(1000);

  //go to center above the tube.
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 0);

  //move to startign position for angle 60 deg, split up in 2 motions to avoid collision
  int firstFillAngle = 60;

  //Offset in z direction so that nozzle travels along bottom side of tube as opposed to along the center of thubes axis
  int zOffsetForBottomOfTube = tubeWidth_mm*1000/(2*sin(firstFillAngle*PI/float(180)));

  //note that I 57000 ensures contact but maybe even  bit too much bending of the nozzle
  //note I found that 55000 also works with less bending, can likely still be improved. note that a lower number will make nozzle bend less, i.e lift it up
  int intialHeightAboveAxis = 55000;
  
  //try combining moves int o1 diagonal pass 
  Gantry.goToRelativePosition(0, heightAbovePivot_um*sin(PI*firstFillAngle/float(180)), heightAbovePivot_um*cos(PI*firstFillAngle/float(180)) - intialHeightAboveAxis - zOffsetForBottomOfTube, 000);    

  delayWithAbort_ms(1000);

  int TUBE_ANGLE_OFFSET_FOR_INSERTION = 15;
  //FIRST MOVE TUBE 5 degs so there are no collisions then move back
  TiltModule.sweepTubeToAngle(firstFillAngle + TUBE_ANGLE_OFFSET_FOR_INSERTION, 1, tubeNumber);


  int entranceDistance_um = 63500;
  //slide into tube very slowly as deep as posssible
  Gantry.goToRelativePosition(0, -entranceDistance_um*sin(PI*(firstFillAngle)/float(180)), -entranceDistance_um*cos(PI*(firstFillAngle)/float(180)), 000);
   
  TiltModule.sweepTubeToAngle(firstFillAngle, 1, tubeNumber);

  //initizl pump prime
  Pump.setPumpRPM(300);
  delayWithAbort_ms(500);
  Pump.setPumpRPM(0);
  delayWithAbort_ms(500);

  //ADD CODE HERE TO DO INITIAL FILLING WHILE THE TUBE IS FULLY IN.
  Pump.setPumpRPM(10);
  delayWithAbort_ms(1000);


  //ONCE THE BLOOD HAS REACHED WHERE THE NOZZLE IS THE CONTINUE TO NEXT SECTION.

  ///find distance to move out of the tube
  //this is the diagonal distance out of the tube you with to travel, I assume it is just 1cm shy of where you started so as to ensure you are in the tube at the end
  int exitDistance_um = entranceDistance_um - 11000;
  //the line below pulls the tube out in 1 shot where as the loop lets you set different pump speeds as you pull it out.
  //Gantry.goToRelativePosition(0, exitDistance_um*sin(PI*firstFillAngle/float(180)), exitDistance_um*cos(PI*firstFillAngle/float(180)), 5000);

  //HERE I LET YOU DO DIFFERENT PUMP SEQUENCES AS YOU FILL IT UP
  int pumpRPMS[] = {14, 16, 18, 25, 30, 0};
  //MAKE SURE BOTH THESE ARRAYS HAVE SAME NUMBER OF ELEMENTS!!
  int delays_ms_Per_pumpingInterval[] = {0000, 0000, 0000, 0000, 000, 000};

  int numberOfPumpingSequencesWhileExitingTube = sizeof(pumpRPMS)/sizeof(int);
  int exitDistancePerPumpSequence_um = exitDistance_um/numberOfPumpingSequencesWhileExitingTube;


  for(int i = 0; i < numberOfPumpingSequencesWhileExitingTube; i++){

    //set new pumping speed and slide gantry up tube, take sliding time as delay before new pumping speed
    Pump.setPumpRPM(pumpRPMS[i]);
    Gantry.goToRelativePosition(0, exitDistancePerPumpSequence_um*sin(PI*firstFillAngle/float(180)), exitDistancePerPumpSequence_um*cos(PI*firstFillAngle/float(180)), delays_ms_Per_pumpingInterval[i]);
  }

  //take the nozzle out by traveling straight up.
  Gantry.goToRelativePosition(0, 0, 40000, 000);

  // straighten out 
  int finalFillAngle = -10;
  TiltModule.sweepTubeToAngle(finalFillAngle, 1, tubeNumber);

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 0);

  int depthBelowTubeTop_um = 25000;//this is for the depth along the tube wall you want the final fill position to be at.

  //take the nozzle to tube wall, old logic
  Gantry.goToRelativePosition(0, -(tubeWidth_mm*1000/2 + sin(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um), -cos(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um, 000);

  Pump.setPumpRPM(50);
  volumeSenseModule::performingFinalFill = true;
  Serial.println("Reached above the volume line");
  //here you would fill until the volume sensors is triggered.
  while(!volumeSenseModule::timeToStopPump() && !aborted){};
  //basically wait until the pump turns itself off.

  peristalticPump::stopPump();
  volumeSenseModule::performingFinalFill = false;

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 0);

  //go back to center
  TiltModule.sweepTubeToAngle(0, 1, tubeNumber);

  //will need to be changed so it just runs once at the end of filling four tubes
  delayWithAbort_ms(000);

  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delayWithAbort_ms(500);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);

  delayWithAbort_ms(500);
  Serial.println("0.5 second");
  DisplayTubes[tubeNumber-1] = 100;
  // Call tube vol update func
  AdjustTubeValueAndSend();

  Serial.println("# ////////////////////////////////////////////////////////////////////");
}

void delayWithAbort_ms(int delayTime_ms){
  for(int i = 0; i < delayTime_ms; i++){
    delay(1);
    if(aborted){
      return;
    }
  }  
}