/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

/***************************************************************************************** 
                                  Arm Position Functions
******************************************************************************************/
#define DEBUGSYS true

void ARMYA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Arm Yaw Task Handle Opened");
  #endif

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1)); /* Minor Delay for function activation */
    if ((abs(ArmYA[0] - ArmYA[1])) > 1) {  // Initate motor function
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #if DEBUGSYS
      Serial.println("Shoulder Yaw Change!");
      #endif

      int SHY_Error = (ArmYA[0] - ArmYA[1]);  /* Desired - Actual Angle */
      bool SDir = (SHY_Error  >= 0);  // 1 for positive, 0 for negative;
      digitalWrite(SHY_DIR, SDir);

      /* Include function to convert angles to step, 400 steps = 360 */
      uint32_t SHY_steps = (round(abs(SHY_Error) / 0.9)) * 16;

      for (int St = 0; St < SHY_steps; St++) {
        digitalWrite(SHY_STEP, HIGH);  // Assumes Sstep2pin is defined
        delayMicroseconds(SYSP);        // Adjust for motor speed
        digitalWrite(SHY_STEP, LOW);
        delayMicroseconds(SYSP);
      }

      // Update Position history
      xSemaphoreGive(motorSemaphore);
      ArmYA[1] = ArmYA[0];
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void ARMPA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Arm Pitch Task Handle Opened");
  #endif

  int8_t ARMPA_Offset = -15; /* 120 is the actual value for neutral */

  while (1) {
    if (ArmPA[0] != ArmPA[1]) {  // Shoulder Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);   // Flag enable

      #if DEBUGSYS
      Serial.println("Shoulder Pitch Change!");            
      #endif    

      int ArmPA_Dir = (ArmPA[0] > ArmPA[1]) ? 1 : -1;

      while (ArmPA[0] != ArmPA[1]) {
        ArmPA[1] += ArmPA_Dir;
        SHP.write(ArmPA[1] + ARMPA_Offset);

        int ArmPA_Dist = abs(ArmPA[1] - ArmPA[0]);
        int APA_DelayMs = map(ArmPA_Dist, 1, 270, 8, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(APA_DelayMs)); // Non-blocking RTOS delay
      }

      xSemaphoreGive(motorSemaphore);
      ArmPA[2] = ArmPA[1];
      ArmPA[1] = ArmPA[0];  // Log
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    else {
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);
      SHP.write(ArmPA[1] + ARMPA_Offset);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void ARMRA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Arm Roll Task Handle Opened");
  #endif

  int8_t ARMRA_Offset = 15; /* 135 needs + 15 to make 150 */

  while (1) {
    if (ArmRA[0] != ArmRA[1]) {  // Shoulder Roll
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);  // Flag enable

      #if DEBUGSYS
      Serial.println("Shoulder Roll Change!");  
      #endif

      int ArmRA_Dir = (ArmRA[0] > ArmRA[1]) ? 1 : -1;

      while (ArmRA[0] != ArmRA[1]) {
        ArmRA[1] += ArmRA_Dir;
        SHR.write(ArmRA[1] + ARMRA_Offset);

        int ArmRA_dist = abs(ArmRA[1] - ArmRA[0]);
        int ARA_DelayMs = map(ArmRA_dist, 1, 180, 25, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(ARA_DelayMs)); // Non-blocking RTOS delay
      }

      xSemaphoreGive(motorSemaphore);
      ArmRA[2] = ArmRA[1];
      ArmRA[1] = ArmRA[0];  // Log
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    else {
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);
      SHR.write(ArmRA[1] + ARMRA_Offset);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void ELPA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Elbow Pitch Task Handle Opened");
  #endif

  int8_t ELPA_Offset = -27; /* Angle 108 is the true neutral */

  while (1) {
    if (ElbowPA[0] != ElbowPA[1]) {  // Elbow Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);    // Flag enable

      #if DEBUGSYS
      Serial.println("Elbow Pitch Change!");          
      #endif 

      int ElbowPA_Dir = (ElbowPA[0] > ElbowPA[1]) ? 1 : -1;

      while (ElbowPA[0] != ElbowPA[1]) {
        ElbowPA[1] += ElbowPA_Dir;
        EP.write(ElbowPA[1] + ELPA_Offset);

        int ELP_Dist = abs(ElbowPA[1] - ElbowPA[0]);
        int ELP_DelayMs = map(ELP_Dist, 1, 225, 8, 1); // 100ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(ELP_DelayMs)); // Non-blocking RTOS delay
        }

      xSemaphoreGive(motorSemaphore);
      ElbowPA[2] = ElbowPA[1];
      ElbowPA[1] = ElbowPA[0];
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    else {
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);
      EP.write(ElbowPA[1] + ELPA_Offset);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void WRPA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Wrist Pitch Task Handle Opened");
  #endif

  int8_t WristPA_Offset = 0;

  while (1) {
    if (WristPA[0] != WristPA[1]) {  // Wrist Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #if DEBUGSYS
      Serial.println("Wrist Pitch Change!");
      #endif

      int WristPA_Dir = (WristPA[0] > WristPA[1]) ? 1 : -1;

      while (WristPA[0] != WristPA[1]) {
        WristPA[1] += WristPA_Dir;
        FP.write(WristPA[1] + WristPA_Offset);

        int WristPA_Dist = abs(WristPA[1] - WristPA[0]);
        int WPA_DelayMs = map(WristPA_Dist, 1, 180, 25, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(WPA_DelayMs)); // Non-blocking RTOS delay
      }
      
      xSemaphoreGive(motorSemaphore);
      WristPA[2] = WristPA[1];
      WristPA[1] = WristPA[0];  // Log
      vTaskDelay(pdMS_TO_TICKS(50));
    } 
    else {
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);
      FP.write(WristPA[1] + WristPA_Offset);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void WRRA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
    Serial.println("Wrist Roll Task Handle Opened");
  #endif
  
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1)); /* Minor delay for function activation */
    if (abs(WristRA[0] - WristRA[1]) > 1) {  // Initate motor function
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #if DEBUGSYS
        Serial.println("Wrist Roll Change!");
      #endif

      int FR_Error = (WristRA[0] - WristRA[1]);  /* Desired vs Current Angle*/
      bool FDir = (FR_Error>= 0);        // Call Direction
      digitalWrite(FR_DIR, FDir);

      /* Include function to convert angles to step, 400 steps = 360 */
      uint32_t FR_Steps = round(abs(FR_Error) / 0.9) * 3.7;

      digitalWrite(FR_EN, LOW);
      vTaskDelay(pdMS_TO_TICKS(5));

      for (int St = 0; St < FR_Steps; St++) {
        digitalWrite(FR_STEP, HIGH);  // Assumes Sstep2pin is defined
        delayMicroseconds(FRSP);       // Adjust for motor speed
        digitalWrite(FR_STEP, LOW);
        delayMicroseconds(FRSP);
      }

      if (Grab == false) digitalWrite(FR_EN, HIGH);  /* Disable Motor after movement */

      xSemaphoreGive(motorSemaphore);
      WristRA[1] = WristRA[0]; 
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

/***************************************************************************************** 
                                  Finger Control Functions
******************************************************************************************/

void Thumb_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
    Serial.println("Thumb Task Handle Opened");
  #endif

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    if (ThumbRA[0] != ThumbRA[1]) {
      /* Change Servo State */

      #if DEBUGSYS
      Serial.println("Thumb Active");
      #endif

      HT.write(ThumbRA[0]);
      vTaskDelay(pdMS_TO_TICKS(40));
      ThumbRA[1] = ThumbRA[0];

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Index_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
    Serial.println("Index Task Handle Opened");
  #endif

  while (1) {
  vTaskDelay(pdMS_TO_TICKS(2));
  if (IndexRA[0] != IndexRA[1]) {
      /* Change Servo State */
      #if DEBUGSYS
        Serial.println("Index Active");
      #endif

      HI.write(IndexRA[0]);
      vTaskDelay(pdMS_TO_TICKS(40));
      IndexRA[1] = IndexRA[0];

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Middle_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
   Serial.println("Middle Task Handle Opened");
  #endif

  while (1) {
  vTaskDelay(pdMS_TO_TICKS(3));
  if (MiddleRA[0] != MiddleRA[1]) {
      /* Change Servo State */

      #if DEBUGSYS
      Serial.println("Middle Active");
      #endif

      HM.write(MiddleRA[0]);
      vTaskDelay(pdMS_TO_TICKS(60));
      MiddleRA[1] = MiddleRA[0];

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Ring_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Ring Task Handle Opened");
  #endif

  while (1) {
  vTaskDelay(pdMS_TO_TICKS(2));
  if (RingRA[0] != RingRA[1]) {
      /* Change Servo State */
      #if DEBUGSYS
      Serial.println("Ring Active");
      #endif
      HR.write(RingRA[0]);
      vTaskDelay(pdMS_TO_TICKS(40));
      RingRA[1] = RingRA[0];

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Pinky_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Pinky Task Handle Opened");
  #endif

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    if (PinkyRA[0] != PinkyRA[1]) {
      /* Change Servo State */

      #if DEBUGSYS
      Serial.println("Pinky Active");
      #endif

      HP.write(PinkyRA[0]);
      vTaskDelay(pdMS_TO_TICKS(50));
      PinkyRA[1] = PinkyRA[0];

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Embedded_Comm(void* pvParameters) {

  #if DEBUGSYS
    Serial.println("Embedded Comm Task Handle Opened");
  #endif
  uint16_t IR_Counter = 0; /* Temporary tracker to make sure distance sensor is alive. */

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms Poll Period for Sensor Readings
    K210_Handle();                 /* Read UART Commands from the K210 */
    Obj_Dist = IRSen.readRangeContinuousMillimeters(); // Sensor Read
    IR_Counter++;
    Object_Grasp();                /* Wait for Object to Grasp */

    #if DEBUGSYS
      if (IR_Counter > 1500) { /* After 1500 cycles it will output the distance ~15 - 16 seconds */
        Serial.println(Obj_Dist);
        IR_Counter = 0;
      }
    #endif
  }
}
