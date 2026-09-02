/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

const double Max_Reach = 550.0;     /* mm  */
const int Traj_Points = 50;
/* Trajectory modes */
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2
#define STEP_DELAY_MS  10   /* dwell per micro-step so servos physically settle */
#define DEBUGSYS false

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
        delay(STEP_DELAY_MS);

        /* chain the solved pose forward as the next seed */
        memcpy(seed, th_out, sizeof(seed));
    }
    return 1.0f;
}

float Run_Trajectory(const double p_target[3], int mode) { //Plug in the XYZ and Mode and the Arm will move.
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
  /* Layer 1 = Request */       {INTENTION, SPECIFICATION_RESTRICTED, GESTURE, HALT},
  /* Layer 2 = Intention */     {SPECIFICATION, ACTION, GESTURE, HALT},
  /* Layer 3 = Specification */ {COLOR, OBJECTIVE, ACTION, HALT},
  /* Layer 4 = Objective */     {ACTION, HALT, NOTHING, NOTHING},
  /* Layer 3 = Restricted */    {HALT, OBJECTIVE, ACTION, HALT}
};

const int* const node_input_maps[] ={
  Request_Map,
  Intention_Map,
  Specification_Map, 
  Objective_Map,
  Specification_Restricted_Map
};

void Decision_Backbone(int req_input, int int_input, int spec_input, int obj_input) {
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
        case ACTION: Action_Function(spec_input, obj_input); break;
        default: break;
    }
}


void Halt_Function() {
  #if DEBUGSYS
  Serial.println("Halting Actions!");
  #endif
  /* Move to Pose Position */
  Extended_Position();
}

void Gesture_Function(int req_ges, int int_ges) {
  #if DEBUGSYS
  Serial.println("Displaying Gesture on Hand!");
  #endif
  /* Move to Pose Position */
  Extended_Position();
  
  /* Input to Action */
  if (req_ges >= 6 && req_ges <= 10) {
    switch(req_ges) {
      case 6: Wave_Movement(); break; /* Wave Function */
      case 7: Handshake(); break; /* Run Handshake Animation */
      case 8: /* Thumbs Up */
        Extended_Position();
        Gestures[0] = 2;
        WristPA[0] = 30;
        break;
      case 9: /* Thumbs Down */
        Extended_Position();
        Gestures[0] = 2;
        WristPA[0] = 240;
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
        break;

      case 7: /* Thumbs Down */
        Extended_Position();
        Gestures[0] = 2;
        HandCode();
        WristRA[0] = 240;
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
    }
  }
  
}

void Action_Function(int spec_action, int obj_action) {
  /* Inital Stage Object Search */
  if (ObjFound == false && HandTrack == false) {
    int Search_timeout = 0;
    for (Search_timeout < 60; Search_timeout++;) {
      Search_Position(); /* Randomly Move to find object */
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (ObjFound == true) {
        break;
      }
    }

    if (Search_timeout >= 60) {
      Search_timeout = 0;
      CMD_IN = false;

      #if DEBUGSYS
      Serial.println("Object not found, search timed out!");
      #endif
    }
  }

  if (ObjFound == true && HandTrack == false) {
    int HandInv_Timeout = 0;
    while (!Hand_CenterCam(objX, objY, WristRA[1], WristPA[1], &WristRA[0], &WristPA[0])) { /* Until the function centers the object, it will run */
      HandInv_Timeout++;
      if (HandInv_Timeout > 1000) {
        ObjFound = false;
        CMD_IN = false;

        #if DEBUGSYS
        Serial.println("Timeout Alignment, Cannot grab!");
        #endif

        break;
      }
    }
    if (Hand_CenterCam(objX, objY, WristRA[1], WristPA[1], &WristRA[0], &WristPA[0])) { /* Once it has found the object it will progress */
      HandTrack = true;
    }
  }

  if (ObjFound == true && HandTrack == true) {
    #if DEBUGSYS
      Serial.println("Grabbing Object!");
    #endif
    Grab = true; 

    double Obj_Pos[3] = {0, 0, 0};
    double Pos_Angles[4] = {ArmRA[1], ArmPA[1], ArmYA[1], ElbowPA[1]}; /* Grab the Values from Variables */

    Object_Position(Pos_Angles, WristRA[1], WristPA[1], Obj_Pos);
    Run_Trajectory(Obj_Pos, MODE_TOP_DOWN);
  }
}






