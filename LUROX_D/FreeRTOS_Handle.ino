/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                                  Arm Position Functions
******************************************************************************************/
#define DEBUGSYS false

void ARMYA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Arm Yaw Task Handle Opened");
  #endif

  while (1) {
    /* Motor PID setup */
    SHY_currentTime = millis(); //Present
    SHY_deltaTime = (SHY_currentTime = SHY_prevTime); //Delta
    SHY_prevTime = SHY_currentTime; //Record

    if (abs(ArmYA[0] - ArmYA[1]) > 1) {  // Initate motor function
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #if DEBUGSYS
      Serial.println("Shoulder Yaw Change!");
      #endif

      SHY_Error = (ArmYA[0] - ArmYA[1]);  //AS5600 Angle Reading vs Desired -> Similar to ArmYA[0] - ArmYA[1] However now closed loop.
      SHY_eDot = ((SHY_Error - SHY_prevError) / SHY_deltaTime);
      SHY_EInt += (SHY_Error * SHY_deltaTime);

      KSUM_SHY = (Kp * SHY_Error) + (Ki * SHY_EInt) + (Kd * SHY_eDot); /* Wow PID! */
      SHY_prevError = SHY_Error;

      bool SDir = (SHY_Error  >= 0);  // 1 for positive, 0 for negative;
      digitalWrite(SHY_DIR, SDir);

      /* Include function to convert angles to step, 400 steps = 360 */
      uint32_t SHY_steps = (round(abs(SHY_Error) / 0.9)) * 16;

      // Enable Motor
      digitalWrite(SHY_EN, LOW);
      vTaskDelay(pdMS_TO_TICKS(5));
      for (int St = 0; St >= SHY_steps; St++) {
        digitalWrite(SHY_STEP, HIGH);  // Assumes Sstep2pin is defined
        delayMicroseconds(500);        // Adjust for motor speed
        digitalWrite(SHY_STEP, LOW);
        delayMicroseconds(500);
      }

      // Update Position history
      ArmYA[1] = ArmYA[0];
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
      //digitalWrite(SHY_EN, HIGH);
    }
  }
}

void ARMPA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #if DEBUGSYS
  Serial.println("Arm Pitch Task Handle Opened");
  #endif

  int ARMPA_Offset = -15; /* 120 is the actual value for neutral */

  while (1) {
    if (ArmPA[0] != ArmPA[1]) {  // Shoulder Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);   // Flag enable

      #ifdef DEBUGSYS
      Serial.println("Shoulder Pitch Change!");            
      #endif    

      int ArmPA_Dir = (ArmPA[0] < ArmPA[1]) ? 1 : -1;

      while (ArmPA[0] != ArmPA[1]) {
        ArmPA[0] += ArmPA_Dir;
        SHP.write(ArmPA[0] + ARMPA_Offset);

        int ArmPA_Dist = abs(ArmPA[1] - ArmPA[0]);
        int APA_DelayMs = map(ArmPA_Dist, 1, 270, 25, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(APA_DelayMs)); // Non-blocking RTOS delay
      }

      ArmPA[2] = ArmPA[1];
      ArmPA[1] = ArmPA[0];  // Log
      xSemaphoreGive(motorSemaphore);
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

  int ARMRA_Offset = 15; /* 135 needs + 15 to make 150 */

  while (1) {
    if (ArmRA[0] != ArmRA[1]) {  // Shoulder Roll
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);  // Flag enable

      #if DEBUGSYS
      Serial.println("Shoulder Roll Change!");  
      #endif

      int ArmRA_Dir = (ArmRA[0] < ArmRA[1]) ? 1 : -1;

      while (ArmRA[0] != ArmRA[1]) {
        ArmRA[0] += ArmRA_Dir;
        SHR.write(ArmRA[0] + ARMRA_Offset);

        int ArmRA_dist = abs(ArmRA[1] - ArmRA[0]);
        int ARA_DelayMs = map(ArmRA_dist, 1, 180, 25, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(ARA_DelayMs)); // Non-blocking RTOS delay
      }

      ArmRA[2] = ArmRA[1];
      ArmRA[1] = ArmRA[0];  // Log
      xSemaphoreGive(motorSemaphore);
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

  #ifdef DEBUGSYS
  Serial.println("Elbow Pitch Task Handle Opened");
  #endif

  int ELPA_Offset = 3; /* Angle 138 is the true neutral */

  while (1) {
    if (ElbowPA[0] != ElbowPA[1]) {  // Elbow Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);    // Flag enable

      #ifdef DEBUGSYS
      Serial.println("Elbow Pitch Change!");          
      #endif 

      int ElbowPA_Dir = (ElbowPA[0] < ElbowPA[1]) ? 1 : -1;

      while (ElbowPA[0] != ElbowPA[1]) {
        ElbowPA[0] += ElbowPA_Dir;
        EP.write(ElbowPA[0] + ELPA_Offset);

        int ELP_Dist = abs(ElbowPA[1] - ElbowPA[0]);
        int ELP_DelayMs = map(ELP_Dist, 1, 270, 25, 1); // 100ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(ELP_DelayMs)); // Non-blocking RTOS delay
        }

      ElbowPA[2] = ElbowPA[1];
      ElbowPA[1] = ElbowPA[0];
      xSemaphoreGive(motorSemaphore);
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

  #ifdef DEBUGSYS
  Serial.println("Wrist Pitch Task Handle Opened");
  #endif

  int WristPA_Offset = 10;

  while (1) {
    if (WristPA[0] != WristPA[1]) {  // Wrist Pitch
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Wrist Pitch Change!");
      #endif

      int WristPA_Dir = (WristPA[0] < WristPA[1]) ? 1 : -1;

      while (WristPA[0] != WristPA[1]) {
        WristPA[0] += WristPA_Dir;
        FP.write(WristPA[0]);

        int WristPA_Dist = abs(WristPA[1] - WristPA[0]);
        int WPA_DelayMs = map(WristPA_Dist, 1, 180, 25, 1); // 50ms delay near target, 5ms far away

        vTaskDelay(pdMS_TO_TICKS(WPA_DelayMs)); // Non-blocking RTOS delay
      }

      WristPA[2] = WristPA[1];
      WristPA[1] = WristPA[0];  // Log
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(50));
    } 
    else {
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);
      FP.write(WristPA[1]);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void WRRA_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Wrist Roll Task Handle Opened");
  #endif
  
  while (1) {
  /* Motor PID setup */
  FR_currentTime = millis(); //Present
  FR_deltaTime = (FR_currentTime = FR_prevTime); //Delta
  FR_prevTime = FR_currentTime; //Record

    if (abs(WristRA[0] - WristRA[1]) > 1) {  // Initate motor function
      xSemaphoreTake(motorSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Wrist Roll Change!");
      #endif

      FR_Error = (WristRA[0] - WristRA[1]);  //AS5600 Angle Reading vs Desired -> Similar to ArmYA[0] - ArmYA[1] However now closed loop.
      FR_eDot = ((FR_Error - FR_prevError) / FR_deltaTime);
      FR_EInt += (FR_Error * FR_deltaTime);

      KSUM_FR = (Kp * FR_Error) + (Ki * FR_EInt) + (Kd * FR_eDot); /* Wow PID! */
      FR_prevError = FR_Error;

      bool FDir = (FR_Error>= 0);        // Call Direction
      digitalWrite(FR_DIR, FDir);

      /* Include function to convert angles to step, 400 steps = 360 */
      uint32_t FR_Steps = round(abs(FR_Error) / 0.9) * 3.7;

      digitalWrite(FR_EN, LOW);
      vTaskDelay(pdMS_TO_TICKS(5));

      for (int St = 0; St >= FR_Steps; St++) {
        digitalWrite(FR_STEP, HIGH);  // Assumes Sstep2pin is defined
        delayMicroseconds(500);       // Adjust for motor speed
        digitalWrite(FR_STEP, LOW);
        delayMicroseconds(500);
      }

      WristRA[1] = WristRA[0]; 
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
      //digitalWrite(FR_EN, HIGH);
    }
  }
}

/***************************************************************************************** 
                                  Finger Control Functions
******************************************************************************************/

void Thumb_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Thumb Task Handle Opened");
  #endif

  while (1) {
    if (ThumbRA[0] != ThumbRA[1]) {
      /* Change Servo State */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Thumb Active");
      #endif

      HT.attach(THUMB, 500, 2500);
      HT.write(ThumbRA[0]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HT.detach();
      ThumbRA[1] = ThumbRA[0];

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    else {
      /* Pulse to Ensure Power */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      //Serial.println("Thumb Active Pulse");

      HT.attach(THUMB, 500, 2500);
      HT.write(ThumbRA[1]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HT.detach();

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void Index_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Index Task Handle Opened");
  #endif

  while (1) {
  if (IndexRA[0] != IndexRA[1]) {
      /* Change Servo State */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Index Active");
      #endif

      HI.attach(INDEX, 500, 2500);
      HI.write(IndexRA[0]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HI.detach();
      IndexRA[1] = IndexRA[0];

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    else {
      /* Pulse to ensure power */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      //Serial.println("Index Active Pulse");

      HI.attach(INDEX, 500, 2500);
      HI.write(IndexRA[1]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HI.detach();

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void Middle_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Middle Task Handle Opened");
  #endif

  while (1) {
  if (MiddleRA[0] != MiddleRA[1]) {
      /* Change Servo State */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      Serial.println("Middle Active");

      HM.attach(MIDDLE, 500, 2500);
      HM.write(MiddleRA[0]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HM.detach();
      MiddleRA[1] = MiddleRA[0];

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    else {
      /* Pulse to ensure power */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      //Serial.println("Middle Active Pulse");

      HM.attach(MIDDLE, 500, 2500);
      HM.write(MiddleRA[1]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HM.detach();

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void Ring_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Ring Task Handle Opened");
  #endif

  while (1) {
  if (RingRA[0] != RingRA[1]) {
      /* Change Servo State */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Ring Active");
      #endif

      HR.attach(RING, 500, 2500);
      HR.write(RingRA[0]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HR.detach();
      RingRA[1] = RingRA[0];

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    else {
      /* Pulse to ensure Power */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      //Serial.println("Ring Active");

      HR.attach(RING, 500, 2500);
      HR.write(RingRA[1]);
      vTaskDelay(pdMS_TO_TICKS(20));
      HR.detach();

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void Pinky_Mot(void* pvParameters) {
  int motorID = (int)pvParameters;

  #ifdef DEBUGSYS
  Serial.println("Pinky Task Handle Opened");
  #endif

  while (1) {
    if (PinkyRA[0] != PinkyRA[1]) {
      /* Change Servo State */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);

      #ifdef DEBUGSYS
      Serial.println("Pinky Active");
      #endif

      HP.attach(PINKY, 500, 2500);
      HP.write(PinkyRA[0]);
      vTaskDelay(pdMS_TO_TICKS(50));
      HP.detach();
      PinkyRA[1] = PinkyRA[0];

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    else {
      /* Pulse to ensure power */
      xSemaphoreTake(fingerSemaphore, portMAX_DELAY);
      //Serial.println("Pinky Active");

      HP.attach(PINKY, 500, 2500);
      HP.write(PinkyRA[1]);
      vTaskDelay(pdMS_TO_TICKS(20)); /* 20ms equates to 1 50Hz bleep */
      HP.detach();

      xSemaphoreGive(fingerSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void Sensor_Feedback(void* pvParameters) {
  Serial.println("Sensor Gathering Task Handle Opened");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms Poll Period for Sensor Readings

    WristRA[2] = HandSensor(); /* Read and Declare Angle */
    ArmYA[2] = UpperArmSensor(); /* Read and Declare Angle */
   
  }
}
