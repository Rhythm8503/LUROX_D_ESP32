/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/
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

/***************************************************************************************** 
                                Communication Functions
******************************************************************************************/

void K210_Handle() {
  /******************************** COMMAND MAP ****************************************
      LAYER 1:  <OPEN> <REQUEST> <INTENT> <OBJECTIVE> <SPECIFICATION> <CLOSE>
      LAYER 2:  <OPEN>  <ObjX>    <ObjY>    <ObjW>       <ObjH>       <CLOSE>
      LAYER 3:  <OPEN>                                                <CLOSE>

    LAYER 1: <OPEN> = 0x1, <CLOSE> = 0x3 | LAYER 2: <OPEN> = 0x2, <CLOSE> = 0X4
    LAYER 3: <OPEN> = 0xB, <CLOSE> = 0xF
  *************************************************************************************/

  if (K210Serial.available()) {
    byte b = K210Serial.read();

    switch (state) {
      case STATE_WAITING_LAYER1:
        if (b == 0x1) {  // Layer 1 <OPEN>
          state = STATE_READING_LAYER1;
          layer1_counter = 0;
        } else if (b == 0xB) {  // Layer 3 <OPEN> (start of exit sequence)
          state = STATE_WAITING_LAYER3_CLOSE;
        }
        // Ignore other bytes
        break;

      case STATE_READING_LAYER1:
        if (layer1_counter < 4) {
          // Store the 4 bytes of Layer 1
          if (layer1_counter == 0) request = b;
          else if (layer1_counter == 1) intent = b;
          else if (layer1_counter == 2) objective = b;
          else if (layer1_counter == 3) specification = b;
          layer1_counter++;
        } else {
          if (b == 0x3) {                  // Layer 1 <CLOSE>
            state = STATE_WAITING_LAYER2;  // Layer 1 complete, ready for Layer 2
          } else {
            state = STATE_WAITING_LAYER1;  // Invalid sequence, reset
          }
        }
        break;

      case STATE_WAITING_LAYER2:
        if (b == 0x2) {  // Layer 2 <OPEN>
          state = STATE_READING_LAYER2;
          layer2_counter = 0;
        } else if (b == 0xB) {  // Layer 3 <OPEN> (start of exit sequence)
          state = STATE_WAITING_LAYER3_CLOSE;
        }
        // Ignore other bytes
        break;

      case STATE_READING_LAYER2:
        if (layer2_counter < 4) {
          // Store the 4 bytes of Layer 2
          if (layer2_counter == 0) objX = b;
          else if (layer2_counter == 1) objY = b;
          else if (layer2_counter == 2) objW = b;
          else if (layer2_counter == 3) objH = b;
          layer2_counter++;
        } else {
          if (b == 0x4) {                  // Layer 2 <CLOSE>
            state = STATE_WAITING_LAYER1;  // Layer 2 complete, reset to Layer 1
                                           // Variables are now stored and can be used
          } else {
            state = STATE_WAITING_LAYER1;  // Invalid sequence, reset
          }
        }
        break;

      case STATE_WAITING_LAYER3_CLOSE:
        if (b == 0xF) {                  // Layer 3 <CLOSE> (exit complete)
          state = STATE_WAITING_LAYER1;  // Reset to waiting for Layer 1
        } else {
          state = STATE_WAITING_LAYER1;  // Invalid sequence, reset
        }
        break;
    }
  }
}

void Bluetooth_Handle() {
  /******************************** COMMAND MAP ****************************************
      |    RAW COMMANDS     |     OPTION     |    OPTIONS     |     OPTION    | 
              MODE               STANDARD           CON             PARTY
                                   SLEEP           MANUAL    
  **************************************************************************************
                  MANUAL MODE       |     LIMB       |      ANGLE
  *************************************************************************************/
  if (newCommandReceived) {
    handleCommand(commandBuffer);
    newCommandReceived = false;
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void handleCommand(char *cmd) {
  char response[32];

  /* MODE SELECTION -> RAW COMMAND */
  if (currentContext == MAIN) {
    if (strcmp(cmd, "MODE") == 0) {
      currentContext = MODE_MENU;
      sendResponse("Select mode: STANDBY, AUTO, MANUAL");
    } else if (strcmp(cmd, "STATUS") == 0) {
      //sendStatus();
    } else {
      sendResponse("Invalid command. Use MODE, STATUS");
    }
  }

  else if (currentContext == MODE_MENU) {
    if (strcmp(cmd, "STANDARD") == 0) {
      GeneralMode = MODE_STANDARD;
      sprintf(response, "Mode set to STANDARD");
      sendResponse(response);
      currentContext = MAIN;
    } else if (strcmp(cmd, "CON") == 0) {
      GeneralMode = MODE_CON;
      sprintf(response, "Mode set to CON");
      sendResponse(response);
      currentContext = MAIN;
    } else if (strcmp(cmd, "PARTY") == 0) {
      GeneralMode = MODE_PARTY;
      sprintf(response, "Mode set to PARTY");
      sendResponse(response);
      currentContext = MAIN;
    } else if (strcmp(cmd, "MANUAL") == 0) {
      GeneralMode = MODE_MANUAL;
      sprintf(response, "Mode set to MANUAL");
      sendResponse(response);
      currentContext = MANUAL_MENU;
    } else if (strcmp(cmd, "SLEEP") == 0) {
      GeneralMode = MODE_SLEEP;
      sprintf(response, "Mode set to SLEEP");
      sendResponse(response);
      currentContext = MAIN;
    } else {
      sendResponse("Invalid mode. Use STANDBY, AUTO, MANUAL");
    }
  } else if (currentContext == MANUAL_MENU) {
    if (strcmp(cmd, "L1") == 0) {
      selectedLimb = 0;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L1. Enter angle (0-270)");
    } else if (strcmp(cmd, "L2") == 0) {
      selectedLimb = 1;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L2. Enter angle (0-270)");
    } else if (strcmp(cmd, "L3") == 0) {
      selectedLimb = 2;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L3. Enter angle (0-270)");
    } else if (strcmp(cmd, "L4") == 0) {
      selectedLimb = 3;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L4. Enter angle (0-270)");
    } else if (strcmp(cmd, "L5") == 0) {
      selectedLimb = 4;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L5. Enter angle (0-180)");
    } else if (strcmp(cmd, "L6") == 0) {
      selectedLimb = 5;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L6. Enter angle (0-180)");
    } else if (strcmp(cmd, "L6") == 0) {
      selectedLimb = 6;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected Hand. Enter Gesture (0-10)");
    } else {
      sendResponse("Invalid limb. Select L1 to L6");
    }
  } else if (currentContext == SELECTED_LIMB) {
    int angle = atoi(cmd);
    if (angle >= 0 && angle <= 180) {
      switch (selectedLimb) {
        case 0:
          ArmRA[0] = (uint8_t)angle;
          sprintf(response, "Set L1 to %d degrees", angle);
          break;
        case 1:
          ArmPA[0] = (uint8_t)angle;
          sprintf(response, "Set L2 to %d degrees", angle);
          break;
        case 2:
          ArmYA[0] = (uint8_t)angle;
          sprintf(response, "Set L3 to %d degrees", angle);
          break;
        case 3:
          ElbowPA[0] = (uint8_t)angle;
          sprintf(response, "Set L4 to %d degrees", angle);
          break;
        case 4:
          WristRA[0] = (uint8_t)angle;
          sprintf(response, "Set L5 to %d degrees", angle);
          break;
        case 5:
          WristPA[0] = (uint8_t)angle;
          sprintf(response, "Set L6 to %d degrees", angle);
          break;
        case 6:
          Gestures[0] = (uint8_t)angle;
          sprintf(response, "Set Gesture to %d", angle);
        default:
          sprintf(response, "Invalid limb selected");
          break;
      }
      sendResponse(response);
      currentContext = MANUAL_MENU;
    } else {
      sendResponse("Invalid angle. Enter 0-180");
    }
  }
}

void sendResponse(const char *response) {
  char fullResponse[64];
  sprintf(fullResponse, "Received: %s\n%s", originalCommand, response);
  pTxCharacteristic->setValue((uint8_t *)fullResponse, strlen(fullResponse));
  pTxCharacteristic->notify();
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0 && rxValue.length() < 32) {
      strncpy(originalCommand, rxValue.c_str(), 31);
      originalCommand[31] = '\0';
      strncpy(commandBuffer, rxValue.c_str(), 31);
      commandBuffer[31] = '\0';
      newCommandReceived = true;
    }
  }
};

void Serial_Terminal() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    // Help menu
    if (input == "help" || input == "?") {
      Serial.println("\n=== LUROX D Kinematics Terminal ===");
      Serial.println("move X Y Z MODE - Run trajectory to X,Y,Z (mm)");
      Serial.println("  MODE: 1=TOP_DOWN, 2=SIDE_SWIPE");
      Serial.println("test - Run test trajectory sequence");
      Serial.println("pos - Show current arm angles");
      Serial.println("fk - Show current end effector position");
      Serial.println("help - Show this menu");
      Serial.println("=== Position Commands ===");
      Serial.println("set - Show current setpoints");
      Serial.println("set neutral - Go to neutral position");
      Serial.println("set extended - Go to extended position");
      Serial.println("set all <r> <p> <y> <e> <wr> <wp> - Set all joint angles");
      Serial.println("set <joint> <value> - Set individual joint (roll/r, pitch/p, yaw/y, elbow/e, wristr/wr, wristp/wp)");
      Serial.println("set gesture/g <0-9> - Set hand gesture");
      return;
    }

    // Show current angles
    if (input == "pos") {
      Serial.print("Current Angles: ");
      Serial.print("Roll="); Serial.print(ArmRA[1]);
      Serial.print(" Pitch="); Serial.print(ArmPA[1]);
      Serial.print(" Yaw="); Serial.print(ArmYA[1]);
      Serial.print(" Elbow="); Serial.println(ElbowPA[1]);
      return;
    }

    // Show forward kinematics position
    if (input == "fk") {
      double theta[4] = {(double)ArmRA[1], (double)ArmPA[1], (double)ArmYA[1], (double)ElbowPA[1]};
      double pos[3];
      double R_out[3][3];
      Pos_Fwrd_Kin(theta, pos, R_out);
      Serial.print("End Effector Position: X=");
      Serial.print(pos[0]);
      Serial.print("mm Y=");
      Serial.print(pos[1]);
      Serial.print("mm Z=");
      Serial.print(pos[2]);
      Serial.println("mm");
      return;
    }

    // Test trajectory sequence
    if (input == "test") {
      Serial.println("Running test trajectory sequence...");

      // Test point 1: Extended forward
      double test1[3] = {0, 200, -200};
      Run_Trajectory(test1, MODE_TOP_DOWN);
      vTaskDelay(pdMS_TO_TICKS(2000));

      // Test point 2: Up and right
      double test2[3] = {100, 200, -250};
      Run_Trajectory(test2, MODE_SIDE_SWIPE);
      vTaskDelay(pdMS_TO_TICKS(2000));

      // Return to neutral
      double test3[3] = {100, 200, -300};
      Run_Trajectory(test3, MODE_TOP_DOWN);

      Serial.println("Test sequence complete.");
      return;
    }

    // Move command: "move X Y Z MODE"
    if (input.startsWith("move ")) {
      // Parse the command
      input = input.substring(5); // Remove "move "
      
      // Find the space positions
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);
      int space3 = input.indexOf(' ', space2 + 1);

      if (space3 == -1) space3 = input.length();

      if (space1 == -1 || space2 == -1) {
        Serial.println("Error: Usage: move X Y Z MODE");
        return;
      }

      // Extract values
      double x = input.substring(0, space1).toFloat();
      double y = input.substring(space1 + 1, space2).toFloat();
      double z = input.substring(space2 + 1, space3).toFloat();
      int mode = 1; // Default to TOP_DOWN

      if (space3 < input.length()) {
        mode = input.substring(space3 + 1).toInt();
        if (mode != 1 && mode != 2) {
          Serial.println("Error: MODE must be 1 (TOP_DOWN) or 2 (SIDE_SWIPE)");
          return;
        }
      }

      Serial.print("Moving to X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.print(y);
      Serial.print(" Z=");
      Serial.print(z);
      Serial.print(" Mode=");
      Serial.println(mode);

      double target[3] = {x, y, z};
      float error = Run_Trajectory(target, mode);

      if (error < 0) {
        Serial.println("Error: Target out of reach!");
      } else {
        Serial.println("Movement complete.");
      }
      return;
    }

    if (input == "set") {
  Serial.println("\nCurrent Setpoints:");
  Serial.print("  Roll="); Serial.print(ArmRA[0]);
  Serial.print(" Pitch="); Serial.print(ArmPA[0]);
  Serial.print(" Yaw="); Serial.print(ArmYA[0]);
  Serial.print(" Elbow="); Serial.print(ElbowPA[0]);
  Serial.print(" WristR="); Serial.print(WristRA[0]);
  Serial.print(" WristP="); Serial.println(WristPA[0]);
  Serial.print("  Gesture="); Serial.println(Gestures[0]);
  return;
}

// Set arm position commands
if (input.startsWith("set ")) {
  input = input.substring(4);

  // Preset positions
  if (input == "neutral") {
    Neutral_Position();
    Serial.println("Set to neutral position.");
    return;
  }
  if (input == "extended") {
    Extended_Position();
    Serial.println("Set to extended position.");
    return;
  }

  // Parse command
  int space1 = input.indexOf(' ');
  String cmd = (space1 != -1) ? input.substring(0, space1) : input;
  String args = (space1 != -1) ? input.substring(space1 + 1) : "";

  // Set ALL joints at once: set all <roll> <pitch> <yaw> <elbow> <wristr> <wristp>
  if (cmd == "all") {
    int values[6];
    int count = 0;
    int start = 0;
    int end;

    while (count < 6 && start < args.length()) {
      end = args.indexOf(' ', start);
      if (end == -1) end = args.length();
      if (start >= end) break;

      values[count] = args.substring(start, end).toInt();
      count++;
      start = end + 1;
    }

    if (count == 6) {
      // Apply your physical joint limits + offsets
      ArmRA[0]   = constrain(values[0], 115, 155);   // 120-140° physical
      ArmPA[0]   = constrain(values[1], 135, 205);   // 120-150° physical
      ArmYA[0]   = constrain(values[2], 0, 270);   // 125-145° physical
      ElbowPA[0] = constrain(values[3], 130, 205);   // 132-138° physical
      WristRA[0] = constrain(values[4], 30, 240);   // Conservative range
      WristPA[0] = constrain(values[5], 40, 120);    // Conservative range

      Serial.println("All joints set:");
      Serial.print("  Roll="); Serial.print(ArmRA[0]);
      Serial.print(" Pitch="); Serial.print(ArmPA[0]);
      Serial.print(" Yaw="); Serial.print(ArmYA[0]);
      Serial.print(" Elbow="); Serial.print(ElbowPA[0]);
      Serial.print(" WristR="); Serial.print(WristRA[0]);
      Serial.print(" WristP="); Serial.println(WristPA[0]);
    } else {
      Serial.println("Error: Need 6 values for 'set all' (roll pitch yaw elbow wristr wristp)");
    }
    return;
  }

  // Individual joint commands
    int value = args.toInt();
    bool valid = true;

    if (cmd == "roll" || cmd == "r") {
      ArmRA[0] = constrain(value, 115, 155);
      Serial.print("Arm Roll set to: "); Serial.println(ArmRA[0]);
    }
    else if (cmd == "pitch" || cmd == "p") {
      ArmPA[0] = constrain(value, 135, 205);
      Serial.print("Arm Pitch set to: "); Serial.println(ArmPA[0]);
    }
    else if (cmd == "yaw" || cmd == "y") {
      ArmYA[0] = constrain(value, 0, 270);
      Serial.print("Arm Yaw set to: "); Serial.println(ArmYA[0]);
    }
    else if (cmd == "elbow" || cmd == "e") {
      ElbowPA[0] = constrain(value, 130, 205);
      Serial.print("Elbow Pitch set to: "); Serial.println(ElbowPA[0]);
    }
    else if (cmd == "wristr" || cmd == "wr") {
      WristRA[0] = constrain(value, 30, 240);
      Serial.print("Wrist Roll set to: "); Serial.println(WristRA[0]);
    }
    else if (cmd == "wristp" || cmd == "wp") {
      WristPA[0] = constrain(value, 50, 110);
      Serial.print("Wrist Pitch set to: "); Serial.println(WristPA[0]);
    }
    else if (cmd == "gesture" || cmd == "g") {
      if (value >= 0 && value <= 9) {
        Gestures[0] = value;
        HandCode();
        Serial.print("Gesture set to: "); Serial.println(Gestures[0]);
      } else {
        Serial.println("Error: Gesture must be 0-9");
        valid = false;
      }
    }
    else {
      Serial.println("Unknown set command. Usage:");
      Serial.println("  set - show current setpoints");
      Serial.println("  set neutral - go to neutral position");
      Serial.println("  set extended - go to extended position");
      Serial.println("  set all <r> <p> <y> <e> <wr> <wp> - set all joints");
      Serial.println("  set <joint> <value> - set individual joint");
      Serial.println("  Joints: roll/r, pitch/p, yaw/y, elbow/e, wristr/wr, wristp/wp, gesture/g");
      valid = false;
    }

    if (valid && cmd != "all") {
      Serial.println("Setpoint updated. Use 'fk' to see forward kinematics result.");
    }
    return;
  }

    Serial.println("Unknown command. Type 'help' for options.");
  }
}