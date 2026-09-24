/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

#define DEBUGSYS true

/***************************************************************************************** 
                                      Decision Mapping
******************************************************************************************/

/* Look Up Table: [Layer][Group] -> Next Node */
const Node transition_lut[5][4] = {
  /* Layer 1 = Request */       {INTENTION, SPECIFICATION_RESTRICTED, GESTURE, HALT},   /* Request -> Intention, Request -> Specification, Gesture or Halt */
  /* Layer 2 = Intention */     {SPECIFICATION, GESTURE, HALT, ACTION},                 /* Intention -> Specification, Gesture, or Halt. Action potentially? */
  /* Layer 3 = Specification */ {OBJECTIVE, ACTION, HALT, NOTHING},                     /* Color Selection -> Objective, or Action or Halt */
  /* Layer 4 = Objective */     {ACTION, HALT, NOTHING, NOTHING},                       /* Final Stage, Action or Halt */
  /* Layer 3 = Restricted */    {NOTHING, ACTION, HALT, NOTHING}                        /* Just grab object */
};

const int* const node_input_maps[] ={
  Request_Map,
  Intention_Map,
  Specification_Map, 
  Objective_Map,
  Specification_Restricted_Map
};

void Decision_Backbone(int req_input, int int_input, int spec_input, int obj_input) {
    #if DEBUGSYS 
      Serial.println("Processing Command");
    #endif

    CMD_PROC = true;     /* Declare in progress */
    ACT_Break = false;   /* Reset */
    Anim_Break = false;  /* Reset */
    Node current_node = REQUEST;

    while (current_node <= SPECIFICATION_RESTRICTED) { /* Only Layers 1 - 5 will be processed */
      int raw_val;
        switch (current_node) {
            case REQUEST:              raw_val = req_input; break;
            case INTENTION:            raw_val = int_input; break;
            case SPECIFICATION:        raw_val = spec_input; break;
            case SPECIFICATION_RESTRICTED: raw_val = spec_input; break;
            case OBJECTIVE:            raw_val = obj_input; break;
            default:                   raw_val = 0;
        }
        
        // Safety: ensure input doesn't exceed the specific layer's array size
        if (raw_val < 0 || raw_val > node_input_max[current_node]) {
            current_node = HALT; 
            break;
        }

        // Dynamically pull the correct map and get the group
        int group = node_input_maps[current_node][raw_val];
        // Move to next node
        current_node = transition_lut[current_node][group];
    }

    switch (current_node) {
        case HALT:   Halt_Function(); break;
        case GESTURE: Gesture_Function(req_input, int_input); break;
        case ACTION: Action_Function(int_input, spec_input, obj_input); break;
        default: break;
    }
}

void Halt_Function() {
  #if DEBUGSYS
  Serial.println("Halting Actions!");
  #endif
  /* Move to Pose Position */
  Extended_Position();
  K210_Write_HALT(); /* Force Halt, even if the instruction came from the K210 */
  ACT_Break = true;
  Anim_Break = true;
  Open_Hand(); /* Open hand*/
  CMD_END();
}

void Gesture_Function(int req_ges, int int_ges) {
  #if DEBUGSYS
    Serial.println("Displaying Gesture on Hand!");
  #endif

  /* Move to Pose Position */
  Anim_Break = false; 
  Extended_Position();
  
  /* Input to Action */
  if (req_ges >= 6 && req_ges <= 10) {
    switch(req_ges) {
      case 6: Wave_Movement(); break; /* Wave Function */
      case 7: Handshake(); break; /* Run Handshake Animation */
      case 8: /* Thumbs Up */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristRA[0] = 30;
        WristRA_Lock(); 
        break;
      case 9: /* Thumbs Down */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristRA[0] = 240;
        WristRA_Lock();
        break;
      case 10: HighFive(); break;
    }
  }

  if (int_ges >= 4 && int_ges <= 10) {
    switch(int_ges) {
      case 4: Wave_Movement(); break;
      case 5: Handshake(); break;

      case 6: /* Thumbs Up */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristRA[0] = 30;
        WristRA_Lock();
        break;

      case 7: /* Thumbs Down */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristRA[0] = 240;
        WristRA_Lock();
        break;

      case 8: HighFive(); break;

      case 9: /* Point Function */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristPA[0] = 30;
        break;

      case 10: /* Peace Function */
        Extended_Position();
        Gestures[0] = 3;
        HandCode();
        WristPA[0] = 110;
        break;

      case 11:  /* Ok! Function */
        Extended_Position();
        Gestures[0] = 7;
        HandCode();
        WristRA[0] = 30;
        WristRA_Lock();
    }
  }
  
  /* Force Reset Everything */
  Extended_Position();
  CMD_END();
}

void Action_Function(int int_input, int spec_action, int obj_action) {
  #if DEBUGSYS
    Serial.println("Action Activated!");
  #endif

  /* Start Function */
  ACT_Break = false;
  uint8_t Search_timeout = 0;     /* Trying to find Object */
  uint16_t HandInv_Timeout = 0;   /* Trying to Center */
  bool Attempt = false;           /* Another attempt before total timeout */
  int32_t Cam_Centered = 0;       /* State machine for finding object */

  while((ACT_Break == false) && (CMD_PROC == true)) {

  /* Inital Stage Object Search */
  if (ObjFound == false && HandTrack == false) {
    for (; Search_timeout < 10; Search_timeout++) {
      #if DEBUGSYS
        Serial.println("Searching for Object!");
      #endif

      /* Move relative to cycle */
      Search_Position(Search_timeout); 
      if (ObjFound) break;
      if (ACT_Break == true) {
        #if DEBUGSYS
          Serial.println("Force Halt!");
        #endif

        CMD_END();
        break;
      }
    }

    if (Search_timeout >= 10) {
      Search_timeout = 0;
      CMD_END();

      #if DEBUGSYS
        Serial.println("Object not found, search timed out!");
      #endif

      break;
    }
  }

  /* Object Found, Centering with Object */
  if (ObjFound == true && HandTrack == false) {
    #if DEBUGSYS
        Serial.println("Running Alignment Function");
    #endif
    while (!(Cam_Centered == 1)) {  /* Until the function centers the object, it will run */
      Cam_Centered = Hand_CenterCam(objX[0], objY[0], WristRA[1], WristPA[1], &WristRA[0], &WristPA[0]);
      HandInv_Timeout++;
      WristRA_Lock(); /* Wait for Wrist to fully rotate before attempting again */
      vTaskDelay(pdMS_TO_TICKS(1500)); /* Wait for motor movement + OV5640 sensor gathering */
      #if DEBUGSYS
        Serial.println("Attempting to Align!");
      #endif

      if (Cam_Centered == 1 && ObjFound == true) { /* Once it has found the object it will progress */
        HandTrack = true;
        #if DEBUGSYS
          Serial.println("ALIGNED! Attempting to Grab!");
        #endif
      }

      if (HandInv_Timeout > 50 && Attempt == false) {
        /* The object was found previously, try searching again and seeing if it can be aligned */
        ObjFound = false;
        Attempt = true;  
        HandInv_Timeout = 0;
        Search_timeout = 0;

        #if DEBUGSYS
          Serial.println("Timeout Alignment, Attempting Search!");
        #endif
        break;
      }

      if (HandInv_Timeout > 50 && Attempt == true) {
        ObjFound = false;
        CMD_END();

        #if DEBUGSYS
          //Serial.println("Timeout Alignment, gave up!");
        #endif
        break;
      }

      if (ACT_Break == true) {
        #if DEBUGSYS
          Serial.println("Force Halt!");
        #endif

        CMD_END();
        break;
      }
    }
  }
  
  /* Object Location determined, moving to object and completing action */
  if (ObjFound == true && HandTrack == true) {
    #if DEBUGSYS
      Serial.println("Grabbing Object!");
    #endif
    Grab = true; 

    double Obj_Pos[3] = {0, 0, 0};
    double Pos_Angles[4] = {ArmRA[1], ArmPA[1], ArmYA[1], ElbowPA[1]}; /* Grab the Values from Variables */
    Get_Current_Angles(Pos_Angles);
    Object_Position(Pos_Angles, WristRA[1], WristPA[1], Obj_Pos);

    #if DEBUGSYS
      Serial.println("Object Position!");
      Serial.println(Obj_Pos[0]);
      Serial.println(Obj_Pos[1]);
      Serial.println(Obj_Pos[2]);
    #endif

    Run_Trajectory(Obj_Pos, MODE_TOP_DOWN);
    ArmYA_Lock();
    WristRA_Lock();
    vTaskDelay(pdMS_TO_TICKS(10000));                                   /* Hold and Wait */

                                                                        /* Based on Intention with Object */
    if (int_input == 3) Push_Obj(Obj_Pos);
    else if (int_input == 4) Pull_Obj(Obj_Pos);
    else Extended_Position();                                           /* Bring to Home */

    CMD_END();                                                          /* End Function, Objective Achieved */
    }
  }
}

void CMD_END() {
  #if DEBUGSYS
        Serial.println("Function Ended, returning back to main state!");
  #endif

  /* Reset Variables back to original states */
  request = 0;
  intent = 0;
  specification = 0;
  objective = 0;
  objX[0] = 0;
  objY[0] = 0;
  objX[1] = 0; 
  objY[1] = 0;

  CMD_PROC = false;
  CMD_IN = false;
  ObjFound = false;
  HandTrack = false;
  Grab = false;

  K210_Write_HALT();
}





