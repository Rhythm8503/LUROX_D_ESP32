/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/
#define DEBUGSYS true
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2

/***************************************************************************************** 
                                  Mapped Motor Functions
******************************************************************************************/

void Search_Position(uint8_t Section) { // Hunting for Object
  #if DEBUGSYS
    Serial.println("Searching for Object!");
  #endif

  /* Sweeping Search Mode */
  /* First Position Sweep */
  if (Section == 0) {
    /* Force Move to Position [0, 277, -375.7] */
    ArmPA[0] = 150;     //Shoulder Pitch
    ArmRA[0] = 135;     //Shoulder Roll
    ArmYA[0] = 135;     //Shoulder Yaw
    ElbowPA[0] = 190;   //Elbow Pitch
    WristPA[0] = 100;    //Wrist Pitch
    WristRA[0] = 135;   //Wrist Roll
    ArmYA_Lock();
    WristRA_Lock();

    /* Wrist Sweep */
    for (int Sw = 0; Sw <= 5; Sw++) {
      if (ACT_Break == true) break;      /* Break before movement */
      if (ObjFound) break;
      WristPA[0] = 100 - (Sw * 8);       /* Wrist Pitch Slowly 100 -> 60 */
      for (int Sr = 0; Sr <= 10; Sr++) {
        if (ObjFound) break;             /* Break before movement */
        if (ACT_Break == true) break;
        WristRA[0] = 100 + (Sr * 8);     /* Wrist Roll Sweep 100 -> 180 */
        WristRA_Lock(); 
        vTaskDelay(pdMS_TO_TICKS(2500)); /* Pause for OV5640 Lock On*/
      }
    }
  }

  /* Pushing Trajectory Sweep 1 - 3 */
  else if (Section >= 1 && Section <= 3) {
    double Traj[3] = {0, (277 + (Section * 25)), -375};
    Run_Trajectory(Traj, 1);

    for (int Sw = 0; Sw <= 5; Sw++) {
        if (ACT_Break == true) break;      /* Break before movement */
        if (ObjFound) break;
        WristPA[0] = 100 - (Sw * 8);       /* Wrist Pitch Slowly 100 -> 60 */
        for (int Sr = 0; Sr <= 10; Sr++) {
          if (ObjFound) break;             /* Break before movement */
          if (ACT_Break == true) break;
          WristRA[0] = 100 + (Sr * 8);     /* Wrist Roll Sweep 100 -> 180 */
          WristRA_Lock(); 
          vTaskDelay(pdMS_TO_TICKS(2500)); /* Pause for OV5640 Lock On*/
        }
      }
  }

  /* Random Point Search*/
  else {
    ArmPA[0] = random(150, 180);    //Shoulder Pitch
    ArmRA[0] = random(135, 145);    //Shoulder Roll
    ArmYA[0] = random(125, 145);     //Shoulder Yaw
    ElbowPA[0] = random(150, 190);  //Elbow Pitch
    WristRA[0] = random(125, 145);   //Wrist Roll
    WristPA[0] = random(60, 100);   //Wrist Pitch
    ArmYA_Lock();
    WristRA_Lock();
    vTaskDelay(pdMS_TO_TICKS(4000)); /* Pause for OV5640 Lock On*/
  }
}

void Wrist_Wave() { /* Directly Utilize Wrist to Wave */
  /* Return Back */
  WristRA[0] = 135;
  WristRA_Lock();

  /* Wave */
  for (int Wave = 0; Wave < 2; Wave++) {
    WristRA[0] = 125;
    WristRA_Lock();
    WristRA[0] = 145;  
    WristRA_Lock();
  }

  /* Return Back */
  WristRA[0] = 135;
  WristRA_Lock();
}

void Wave_Movement() { // Pre-Defined Wave Animation
    /* Bring to Extended Position */
    Extended_Position();
    #if DEBUGSYS
        Serial.println("Beginning Wave Animation!");
    #endif

    WristPA[0] = 40; /* Tilt Up to Wave at People*/

    /* Wave Animation */
    ArmYA[0] = 125;   // <--- Movement Triggered
    Wrist_Wave();     // <- While YA moves, Wrist Moves
    ArmYA_Lock();     // <-- ArmYA locking Wave_Movement till completion

    ArmYA[0] = 145;  
    Wrist_Wave();
    ArmYA_Lock();

    ArmYA[0] = 135;   // Reset to Original Position
    ArmYA_Lock();

    WristPA[0] = 90;
    vTaskDelay(pdMS_TO_TICKS(200));
}

void Handshake() {  // Pre-Defined Handshake

  Extended_Position(); /* Extended Position */

  WristRA[0] = 225; /* 90 Degrees for the Handshake */ 
  WristRA_Lock();

  #if DEBUGSYS
    Serial.println("Beginning Handshake!");
  #endif 

  unsigned long Anim_Timer = millis(); /* Time out counter */

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
          break;
        }
        ElbowPA[0] = 170; /* Shake the Person's Hand */
        vTaskDelay(pdMS_TO_TICKS(500)); /* Small Delay */
        ElbowPA[0] = 180;
        vTaskDelay(pdMS_TO_TICKS(500)); /* Small Delay */
      }
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
    /* Return the hand back to open */
    Gestures[0] = 0;
    HandCode();
}

void HighFive() {
  Extended_Position(); /* Extended Position */
  ArmPA[0] = 150;
  ElbowPA[0] = 180;
  WristPA[0] = 40; /* High Five Position */

  #if DEBUGSYS
  Serial.println("Beginning High-Five!");
  #endif 

  unsigned long Anim_Timer = millis();      /* Time out counter */

  while((millis() - Anim_Timer) < 10000) {  /* It will wait 15 seconds before timing out */
    if (Anim_Break == true) {               /* If Animation is requested to break, then it will break */
      break;
    }
     vTaskDelay(pdMS_TO_TICKS(10));
     if (Obj_Dist < 500) {                   /* Someone is approaching! */

        /* Pull Arm Forward */
        ElbowPA[0] = 200;
        ArmPA[0] = 160;  /* Pull Arm Forward */
        vTaskDelay(pdMS_TO_TICKS(10));
        if (Obj_Dist > 100) {
          break;
        }
      }
    }

    Extended_Position(); /* Return back! */
}

/***************************************************************************************** 
                                  Complex Motor Functions
******************************************************************************************/

void Push_Obj(double Push_Pos[3]) {
  /* Basic Kinematics Push Function */
  Push_Pos[1] = Push_Pos[1] + 50; /* 50mm Push in Y */
  Run_Trajectory(Push_Pos, MODE_TOP_DOWN);
}

void Pull_Obj(double Pull_Pos[3]) {
  /* Basic Kinematics Pull Function */
  Pull_Pos[1] = Pull_Pos[1] - 50; /* 50mm Pull in Y */
  Run_Trajectory(Pull_Pos, MODE_TOP_DOWN);
}