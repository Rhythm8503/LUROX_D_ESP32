/*********************************************************************************************** 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board & Maix-Bit K210
    LUROX D: Mark II Software
***********************************************************************************************/

#define DEBUGSYS true
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2

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
          CMD_IN = true;  // Set the command input flag
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

void K210_Write_Comm() {
  /* Forward Bluetooth Command to K210 */
    K210Serial.write(0x01); /* Open Layer 1 */
    K210Serial.write(request);
    K210Serial.write(intent);
    K210Serial.write(objective);
    K210Serial.write(specification);
    K210Serial.write(0x03); /* Close Layer 1 */
}

void K210_Write_HALT() {
    K210Serial.write(0x01); /* Open Layer 1 */
    K210Serial.write(50);
    K210Serial.write(50);
    K210Serial.write(50);
    K210Serial.write(50);
    K210Serial.write(0x03); /* Close Layer 1 */
}

/****************************** Bluetooth Communication ***********************************/

void Bluetooth_Handle() {
  /******************************** COMMAND MAP *********************************************
      |    RAW COMMANDS     |     OPTION     |    OPTION     |     OPTION    |    OPTION    |
              MODE               STANDARD          MANUAL          SLEEP      
  *******************************************************************************************
             INPUTS               Request         Intention      Objective    Specification   
  *******************************************************************************************
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
      sendResponse("Select mode: STANDARD, MANUAL, SLEEP");
    } else if (strcmp(cmd, "STATUS") == 0) {
      //sendStatus();
    } else {
      sendResponse("Invalid command. Use MODE, STATUS");
    }
  }

  else if (currentContext == MODE_MENU) {
    if (strcmp(cmd, "STANDARD") == 0) {
      sendResponse("Mode set to STANDARD, enter cmd");
      currentContext = COMMAND_MENU;

    } else if (strcmp(cmd, "MANUAL") == 0) {
      sprintf(response, "Mode set to MANUAL");
      sendResponse(response);
      currentContext = MANUAL_MENU;

    } else if (strcmp(cmd, "SLEEP") == 0) {
      sprintf(response, "Mode set to SLEEP");
      sendResponse(response);
      currentContext = MAIN;

    } else {
      sendResponse("Invalid mode. Use STANDARD, MANUAL, SLEEP");
    }
  } else if (currentContext == COMMAND_MENU) {
    if (parseCommandSequence(cmd)) {
       // Success - use the values
       sprintf(response, "Req: %d, Int: %d, Spec: %d, Obj: %d", request, intent, specification, objective);
       sendResponse(response);
       K210_Write_Comm();
       CMD_IN = true;
    }
    else if (strcmp(cmd, "EXIT") == 0) {
      currentContext = MODE_MENU;
      sendResponse("Main menu, Select MODE or STATUS");
    } 
    else {
        sendResponse("Invalid format. Enter 4 numbers separated by spaces (e.g., '1 3 0 3')");
    }
    
  } else if (currentContext == MANUAL_MENU) {
    if (strcmp(cmd, "L1") == 0) {
      selectedLimb = 0;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L1. Enter angle (115 - 155)");
    } else if (strcmp(cmd, "L2") == 0) {
      selectedLimb = 1;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L2. Enter angle (135 - 200)");
    } else if (strcmp(cmd, "L3") == 0) {
      selectedLimb = 2;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L3. Enter angle (0 - 270)");
    } else if (strcmp(cmd, "L4") == 0) {
      selectedLimb = 3;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L4. Enter angle (135 - 200)");
    } else if (strcmp(cmd, "L5") == 0) {
      selectedLimb = 4;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L5. Enter angle (30 - 240)");
    } else if (strcmp(cmd, "L6") == 0) {
      selectedLimb = 5;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected L6. Enter angle (30 - 110)");
    } else if (strcmp(cmd, "L6") == 0) {
      selectedLimb = 6;
      currentContext = SELECTED_LIMB;
      sendResponse("Selected Hand. Enter Gesture (0-10)");
    } else if (strcmp(cmd, "EXIT") == 0) {
      currentContext = MODE_MENU;
      sendResponse("Main menu, Select MODE or STATUS");
    } else {
      sendResponse("Invalid limb. Select L1 to L6");
    }
  } else if (currentContext == SELECTED_LIMB) {
    int angle = atoi(cmd);
    if (angle >= 0 && angle <= 270) {
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
    } 
    else if (strcmp(cmd, "EXIT") == 0) {
      currentContext = MODE_MENU;
      sendResponse("Main menu, Select MODE or STATUS");
    } else {
      sendResponse("Invalid angle. Enter 0-270");
    }
  }
}

bool parseCommandSequence(const char* input) {
    // 1. NULL/empty check
    if (input == NULL || *input == '\0') {
        return false;
    }

    // 2. Length check - "0 0 0 0" = 7 chars, "10 10 10 10" = 11 chars
    size_t len = strlen(input);
    if (len < 7 || len > 14) {  // Allow some margin for extra spaces
        return false;
    }

    // 3. Parse with validation
    int request_char, intention_char, specification_char, objective_char;
    int count = sscanf(input, "%d %d %d %d", &request_char, &intention_char, &specification_char, &objective_char);

    if (count != 4) {
        return false;
    }

    // 4. Range validation (0-10)
    if (request_char < 0 || request_char > 10 || intention_char < 0 || intention_char > 10 ||
        specification_char < 0 || specification_char > 10 || objective_char < 0 || objective_char > 10) {
        return false;
    }

    // 5. Only set outputs on success
    request = (uint8_t)request_char;
    intent = (uint8_t)intention_char;
    specification = (uint8_t)specification_char;
    objective = (uint8_t)objective_char;

    CMD_IN = true;
    return true;

    // int count = sscanf(input, "%d %d %d %d", request_char, intention_char, specification_char, objective_char);
    // CMD_IN = true;
    // return (count == 4);
}

void sendResponse(const char *response) {
  char fullResponse[128];
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
    rxValue.trim(); /* Trim off White Space */

    if (rxValue.length() > 0 && rxValue.length() < 32) {
      
      strncpy(originalCommand, rxValue.c_str(), 31);
      originalCommand[31] = '\0';
      strncpy(commandBuffer, rxValue.c_str(), 31);
      commandBuffer[31] = '\0';

      #if DEBUGSYS
      Serial.println(commandBuffer);
      #endif

      newCommandReceived = true;
    }
  }
};

/****************************** Serial Communication ***********************************/

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