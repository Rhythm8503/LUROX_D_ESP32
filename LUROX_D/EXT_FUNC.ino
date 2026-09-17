/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

#define DEBUGSYS false
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2

/***************************************************************************************** 
                                 Basic Motor Assignments
******************************************************************************************/
void Standby() {  // Wander
  #if DEBUGSYS
  Serial.println("Position Change");
  #endif

  ArmPA[0] = random(135, 155);    //Shoulder Pitch
  ArmRA[0] = random(127, 133);    //Shoulder Roll
  ArmYA[0] = random(125, 145);     //Shoulder Yaw
  ElbowPA[0] = random(145, 200);  //Elbow Pitch
  WristPA[0] = random(90, 100);   //Wrist Pitch
  WristRA[0] = random(115, 155);   //Wrist Roll
}

void Sleep() {  // Place Arm to Sleep
  #if DEBUGSYS
  Serial.println("Sleep Mode Activated");
  #endif

  // Rotate back to home
  digitalWrite(SHY_EN, HIGH);  //Shoulder Yaw Sleep
  digitalWrite(FR_EN, HIGH);   //Forearm Roll Sleep

  //Servo Sleep
  SHR.detach();
  SHP.detach();
  EP.detach();
  FP.detach();
}

void Wake() {  // Initalizing Objects
  #if DEBUGSYS
  Serial.println("Waking Arm Up!");
  #endif

  //Servo Wake
  FP.attach(WRIST, 500, 2500);
  EP.attach(ELBOW, 500, 2500);
  SHR.attach(SHR_RO, 500, 2500);
  SHP.attach(SHR_PI, 500, 2500);
}

void Extended_Position() { /* Extended out on the XYZ Plane */
  #if DEBUGSYS
  Serial.println("Extending the Arm out!");
  #endif

  ArmRA[0] = 135;
  ArmPA[0] = 160;
  ArmYA[0] = 135;
  ArmYA_Lock();

  ElbowPA[0] = 190;
  WristRA[0] = 135;
  WristRA_Lock();

  WristPA[0] = 90;
  Gestures[0] = 0; // Open the Hand
  HandCode();
}

void Neutral_Position() { /* Straight Down position */
  #if DEBUGSYS
  Serial.println("Returning Arm Back to Home!");
  #endif

  ArmRA[0] = 135;
  ArmPA[0] = 135;
  ArmYA[0] = 135;
  ElbowPA[0] = 135;
  WristRA[0] = 135;
  WristPA[0] = 90;
  Gestures[0] = 0; // Open the Hand
  HandCode();
}

/***************************************************************************************** 
                                  Motor Safety Functions
******************************************************************************************/

void WristRA_Lock() {
  while(WristRA[1] != WristRA[0]) {
        vTaskDelay(pdMS_TO_TICKS(1)); /* Waiting for Function Completion */
        if (millis() - FR_currentTime) > 5000) {
            break;                    /* Force break after 3 seconds */
        }
  }
}

void ArmYA_Lock() {
  while(ArmYA[1] != ArmYA[0]) {
        vTaskDelay(pdMS_TO_TICKS(1)); /* Waiting for Function Completion */
        if (millis() - SHY_currentTime) > 7500) {
            break;                    /* Force break after 3 seconds */
        }
  }
}

/***************************************************************************************** 
                                  Hand Gesture Functions
******************************************************************************************/
void HandCode() {   // Hand Servos Control
  if (HandRot == false && SleepState == false && (Gestures[0] != Gestures[1])) {  //Gesture Control
    HandRot = true;
    Wander = false;

    #if DEBUGSYS
    Serial.println("Changing Gestures");
    #endif 

    switch (Gestures[0]) {  // 0 Thumb, 1 Index, 2, Middle, 3 Ring, 4, Pinky
      case 0:               // Open
        for (int f = 0; f < 5; f++) {
          HandFunc(500, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        break;

      case 1:                    // Close
        HandFunc(500, true, 0);  //Thumb first
        for (int f = 1; f < 5; f++) {
          HandFunc(500, true, f);  //Close da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        break;

      case 2:  // Thumbs Up/Down
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        for (int f = 1; f < 5; f++) {
          HandFunc(250, true, f);  //Close da hand but thumb
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        break;

      case 3:  // Peace
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        HandFunc(250, true, 0);  //Thumb close
        vTaskDelay(pdMS_TO_TICKS(5));
        HandFunc(250, true, 3);  //Ring close
        HandFunc(250, true, 4);  //Pinky close
        break;

      case 4:  // Middle
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        HandFunc(250, true, 0);  // Thumb first
        vTaskDelay(pdMS_TO_TICKS(5));
        HandFunc(250, true, 1);  // Index First
        HandFunc(250, true, 3);  // Ring
        HandFunc(250, true, 4);  // Pinky
        break;

      case 5:  //Point
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        HandFunc(250, true, 0);  // Thumb first
        vTaskDelay(pdMS_TO_TICKS(5));
        for (int f = 2; f < 5; f++) {
          HandFunc(400, true, f);  //Close da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        break;

      case 6:  // Rock
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        HandFunc(250, true, 0);  // Thumb first
        vTaskDelay(pdMS_TO_TICKS(5));
        HandFunc(250, true, 1);  // Index First
        HandFunc(250, true, 4);  // Pinky
        break;

      case 7:  // Rock
        for (int f = 0; f < 5; f++) {
          HandFunc(400, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        HandFunc(250, true, 1);  // Thumb first
        vTaskDelay(pdMS_TO_TICKS(5));
        HandFunc(250, true, 1);  // Index First
        break;

      case 8:
        for (int f = 0; f < 5; f++) {
          HandFunc(500, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (int f = 0; f < 3; f++) {
          WristPA[0] = 90;
          delay(500);
          WristPA[0] = 110;
        }
        break;

      case 9:
        for (int f = 0; f < 5; f++) {
          HandFunc(500, false, f);  //Open da hand
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        WristPA[0] = 90;
        delay(500);
        WristPA[0] = 110;
        break;
    }
    Wander = true;
    HandRot = false;
    Gestures[2] = Gestures[1];
    Gestures[1] = Gestures[0];  //Log Changes
  }

  if (Grab == true && Gestures[1] == 1) {  // Hold and apply grip force
    HandFunc(750, true, 0);                //Thumb first
    for (int f = 1; f < 5; f++) {
      HandFunc(500, true, f);  //Close da hand
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

int HandFunc(int TSPer, bool CF, byte SF) {  //Writing to Hand
  // 360 Servo Drive
  switch (SF) {
    case 0:
      if (CF == 1) {
        ThumbRA[0] = 0;
      } else {
        ThumbRA[0] = 180;
      }
      break;

    case 1:
      if (CF == 1) {
        IndexRA[0] = 0;
      } else {
        IndexRA[0] = 180;
      }
      break;

    case 2:
      if (CF == 1) {
        MiddleRA[0] = 0;
      } else {
        MiddleRA[0] = 180;
      }
      break;

    case 3:
      if (CF == 1) {
        RingRA[0] = 0;
      } else {
        RingRA[0] = 180;
      }
      break;

    case 4:
      if (CF == 1) {
        PinkyRA[0] = 0;
      } else {
        PinkyRA[0] = 180;
      }
      break;
  }
  return 1;
}
