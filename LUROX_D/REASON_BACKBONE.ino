/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

const double Max_Reach = 1550.0;     /* mm  */
const int Traj_Points = 50;
/* Trajectory modes */
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2
#define STEP_DELAY_MS  15   /* dwell per micro-step so servos physically settle */
#define DEBUGSYS true

/***************************************************************************************** 
                                Trajectory Functions
******************************************************************************************/

void Get_Current_Angles(double theta[4]) { // Grab Current Angles from Robotic Arm
    //theta[0] = (double)ArmRA[1];
    theta[1] = (double)ArmPA[1];
    //theta[2] = (double)ArmYA[1];
    theta[3] = (double)ElbowPA[1];

    theta[0] = 270.0 - (double)ArmRA[1];;   // Flip Arm Roll around 135°
    theta[2] = 270.0 - (double)ArmYA[1];   // Flip Arm Yaw around 135°
}

void Move_Arm_Pose(const double theta[4]) { //Update Arm with New Angles
    int Mod_theta0 = 270.0 - theta[0];   // Flip Arm Roll around 135°
    int Mod_theta2 = 270.0 - theta[2];   // Flip Arm Yaw around 135°

    ArmRA[0]   = (int)lround(Mod_theta0);
    ArmPA[0]   = (int)lround(theta[1]);
    ArmYA[0]   = (int)lround(Mod_theta2);
    ElbowPA[0] = (int)lround(theta[3]);
}

void Gen_Trajectory(const double p_start[3], const double p_target[3], int mode, double path[3][50]) { // Generates the Parametric Bezier Curve Points to Object
    /* --- 1. control point --- */
    double p_ctrl[3] = {0.0, 0.0, 0.0};
    p_ctrl[1] = p_start[1];                       /* lock Y axis */

    if (mode == MODE_TOP_DOWN) {
        p_ctrl[0] = p_target[0];                  /* X = target */
        p_ctrl[2] = p_start[2];                   /* Z = start  */
    } else { /* SIDE_SWIPE */
        p_ctrl[0] = p_start[0];                   /* X = start  */
        p_ctrl[2] = p_target[2];                  /* Z = target */
    }

    /* --- 2. 50-point Bezier sweep --- */
    for (int i = 0; i < 50; ++i) {
        double t = (double)i / (double)(Traj_Points - 1);
        double u = 1.0 - t;
        double pt[3];
        for (int k = 0; k < 3; ++k)
            pt[k] = u*u * p_start[k] + 2.0*u*t * p_ctrl[k] + t*t * p_target[k];

        /* origin-aware reach clamp */
        double r = sqrt(pt[0]*pt[0] + pt[1]*pt[1] + pt[2]*pt[2]);
        if (r > Max_Reach) {
            double s = Max_Reach / r;
            pt[0] *= s; pt[1] *= s; pt[2] *= s;
        }
        for (int k = 0; k < 3; ++k)
            path[k][i] = pt[k];
    }
}

void Solve_Trajectory(const double theta_init[4], const double path[3][50], double theta_traj[4][50], double *avg_iters) { //Solves the Parametric Curve Points, so Point to Point Kinematics
    #if DEBUGSYS
      Serial.println(" Solving Trajectory!");
    #endif
    double seed[4];
    memcpy(seed, theta_init, sizeof(seed));
    long total_iters_unused = 0;  /* (iteration count not surfaced here) */
    (void)total_iters_unused;

    double sum_iters = 0.0;

    for (int i = 0; i < Traj_Points; ++i) {
        double p_des[3] = { path[0][i], path[1][i], path[2][i] };
        double th_out[4];
        Invrs_Kin(p_des, seed, th_out);     /* 10° filter inside */
        for (int k = 0; k < 4; ++k) {
            theta_traj[k][i] = th_out[k];
            seed[k] = th_out[k];                   /* chain seed forward */
        }
        sum_iters += 1.0;                         /* per-point counter */
    }
    if (avg_iters) *avg_iters = sum_iters / (double)Traj_Points;
}

float Move_Trajectory(const double theta_init[4], const double path[3][50]) { // Updates the Motor Angles from Point to Point
    double seed[4];
    memcpy(seed, theta_init, sizeof(seed));

    for (int i = 0; i < Traj_Points; ++i) {
        double p_des[3] = { path[0][i], path[1][i], path[2][i] };

        /* solve this micro-step within ±10° of the current pose */
        double th_out[4];
        float err = Invrs_Kin(p_des, seed, th_out);
        (void)err;

        /* command the arm to the solved pose — point to point */
        Move_Arm_Pose(th_out);

        /* dwell so the servos reach the pose before the next step */
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

        /* chain the solved pose forward as the next seed */
        memcpy(seed, th_out, sizeof(seed));
    }
    return 1.0f;
}

float Run_Trajectory(const double p_target[3], int mode) { //Plug in the XYZ and Mode and the Arm will move.
    #if DEBUGSYS
      Serial.println("Running Trajectory!");
    #endif
    /* sanity: reject unreachable targets */
    double r = sqrt(p_target[0]*p_target[0] +
                    p_target[1]*p_target[1] +
                    p_target[2]*p_target[2]);
    if (r > Max_Reach) return -1.0f;     /* caller should re-prompt */

    /* seed from the arm's real current pose */
    double theta_now[4];
    Get_Current_Angles(theta_now);

    double p_start[3];
    double R_dummy[3][3];
    Pos_Fwrd_Kin(theta_now, p_start, R_dummy);

    /* generate the Bezier path */
    static double path[3][Traj_Points];
    Gen_Trajectory(p_start, p_target, mode, path);

    /* drive the arm through the path, point to point */
    Move_Trajectory(theta_now, path);
    return 1.0f;
}

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
    Serial.println(" Grab is true ");
    double Obj_Pos[3] = {0, 0, 0};
    Serial.println("Declaring Obj_Pos");
    double Pos_Angles[4] = {ArmRA[1], ArmPA[1], ArmYA[1], ElbowPA[1]}; /* Grab the Values from Variables */
    Serial.println("Declared variables");
    Get_Current_Angles(Pos_Angles);
    Serial.println("Get Current Angles Functioned");
    Object_Position(Pos_Angles, WristRA[1], WristPA[1], Obj_Pos);
    Serial.println("Object Position!");
    Serial.println(Obj_Pos[0]);
    Serial.println(Obj_Pos[1]);
    Serial.println(Obj_Pos[2]);
    Run_Trajectory(Obj_Pos, MODE_TOP_DOWN);
    ArmYA_Lock();
    WristRA_Lock();
    vTaskDelay(pdMS_TO_TICKS(5000));                                    /* Hold and Wait */

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
  ObjFound == false;
  HandTrack == false;
  Grab = false;

  K210_Write_HALT();
}





