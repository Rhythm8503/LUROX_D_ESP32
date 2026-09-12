/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

/***************************************************************************************** 
                                  Basic Motor Functions
******************************************************************************************/
#define DEBUGSYS true
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2

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
  //Gestures[0] = random(0,1);
  //HandCode();
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

void Search_Position() { // Hunting for Object
  #if DEBUGSYS
  Serial.println("Searching for Object!");
  #endif
  WristPA[0] = 110; /* Locked rotation */

  /* Randomly moving positions to try finding */
  WristRA[0] = random(110, 150);
  ArmYA[0] = random(110, 150); 
  ArmPA[0] = random(170, 175);
  ArmYA[0] = random(120, 150);
  ElbowPA[0] = random(210, 220);
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
  ArmPA[0] = 150;
  ArmYA[0] = 135;
  ElbowPA[0] = 190;
  WristRA[0] = 135;
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

void Wrist_Wave() {
  for (int Wave = 0; Wave < 2; Wave++) {
    WristRA[0] = 125;
    vTaskDelay(pdMS_TO_TICKS(700));
    WristRA[0] = 145;
    vTaskDelay(pdMS_TO_TICKS(700));
  }
  WristRA[0] = 135;
}

void Wave_Movement() { // Pre-Defined Wave Animation
  /* Bring to Extended Position */
  Extended_Position();
  #if DEBUGSYS
  Serial.println("Beginning Wave Animation!");
  #endif

  WristPA[0] = 40;

  /* Wave Animation */
  for (int Wave = 0; Wave < 1; Wave++) {
    ArmYA[0] = 125;
    vTaskDelay(pdMS_TO_TICKS(1500));
    Wrist_Wave();
    ArmYA[0] = 145;
    vTaskDelay(pdMS_TO_TICKS(1500));
    Wrist_Wave();
    if (Anim_Break == true) {
      break;
    }
  }
  ArmYA[0] = 135;
  WristRA[0] = 135;
  WristPA[0] = 90;
  vTaskDelay(pdMS_TO_TICKS(500));
  Neutral_Position();
}

void Handshake() {  // Pre-Defined Handshake
  Extended_Position(); /* Extended Position */
  WristRA[0] = 225; /* 90 Degrees for the Handshake */ 
  #if DEBUGSYS
  Serial.println("Beginning Handshake!");
  #endif 

  unsigned long Anim_Timer = millis(); /* Time out counter */
  vTaskDelay(pdMS_TO_TICKS(1000));

  while((millis() - Anim_Timer) < 15000) { /* It will wait 15 seconds before timing out */
    if (Anim_Break == true) { /* If Animation is requested to break, then it will break */
      break;
    }

    if (Obj_Dist < 40 && Obj_Dist > 0) { /* Someone places their hand or object infront of the hand */
      #if DEBUGSYS
      Serial.println(Obj_Dist);
      #endif

      Gestures[0] = 1; // Close the Hand
      HandCode(); //Push the Change

      for (int Anim_Count = 0; Anim_Count < 2; Anim_Count++) {
        if (Obj_Dist > 500 || Anim_Break == true) {
          Extended_Position();
          WristRA[0] = 135; /* 90 Degrees for the Handshake */ 
          break;
        }
        ElbowPA[0] = 170; /* Shake the Person's Hand */
        vTaskDelay(pdMS_TO_TICKS(500));
        ElbowPA[0] = 180;
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
  /* Return the hand back to open */
    Gestures[0] = 0;
    HandCode();
    Neutral_Position();
}

void HighFive() {
  Extended_Position(); /* Extended Position */
  ArmPA[0] = 150;
  ElbowPA[0] = 180;
  WristPA[0] = 40; /* High Five Position */

  #if DEBUGSYS
  Serial.println("Beginning High-Five!");
  #endif 

  unsigned long Anim_Timer = millis(); /* Time out counter */

  while((millis() - Anim_Timer) < 10000) { /* It will wait 15 seconds before timing out */
    if (Anim_Break == true) { /* If Animation is requested to break, then it will break */
      break;
    }
     vTaskDelay(pdMS_TO_TICKS(1000));
     if (Obj_Dist < 500) { /* Someone is approaching! */
      /* Pull Arm Forward */
        ElbowPA[0] = 200;
        ArmPA[0] = 160; /* Pull Arm Forward */
        vTaskDelay(pdMS_TO_TICKS(10));
        if (Obj_Dist > 100) {
          break;
        }
      }
    }

    Extended_Position(); /* Return back! */
    vTaskDelay(pdMS_TO_TICKS(1000));
    Neutral_Position();
}

/***************************************************************************************** 
                                  Hand Gesture Functions
******************************************************************************************/
void HandCode() {                   // Hand Servos Control
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
