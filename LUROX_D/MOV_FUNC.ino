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
                                  Mapped Motor Functions
******************************************************************************************/

void Search_Position() { // Hunting for Object
  #if DEBUGSYS
    Serial.println("Searching for Object!");
  #endif

  /* Sweeping Search Mode */
  /* Freeze for now */
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

void Push_Obj() {

}

void Pull_Obj() {

}