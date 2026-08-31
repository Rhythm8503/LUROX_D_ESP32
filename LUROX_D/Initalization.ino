/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                                  Primary Init Functions
******************************************************************************************/

#define DEBUGSYS true
#define i2C_EN true

void FreeRTOS_Initalization() {
  #if DEBUGSYS
    Serial.begin(115200); //Allow for Debug Reports to Serial Terminal----------/
  #endif

  //Core 0, Event Handling, Feedback Collection, Decision Making and Inverse Kinematics
  //Core 1, Stepper Motor Management, Servo Motor Management

  /* FreeRTOS Thread Management Initalization */
  motorSemaphore = xSemaphoreCreateCounting(MAX_CONCURRENT_MOTORS, MAX_CONCURRENT_MOTORS);
  fingerSemaphore = xSemaphoreCreateCounting(MAX_FINGERCONCURRENT_MOTORS,MAX_FINGERCONCURRENT_MOTORS);


  xTaskCreatePinnedToCore(ARMYA_Mot, "Shoulder Yaw", 10000, NULL, 0, &ArmYAMot, 1);
  xTaskCreatePinnedToCore(ARMPA_Mot, "Shoulder Pitch", 10000, NULL, 0, &ArmPAMot, 1);
  xTaskCreatePinnedToCore(ARMRA_Mot, "Shoulder Roll", 10000, NULL, 0, &ArmRAMot, 1);
  xTaskCreatePinnedToCore(ELPA_Mot, "Elbow Pitch", 10000, NULL, 0, &ElbowPAMot, 1);
  xTaskCreatePinnedToCore(WRPA_Mot, "Wrist Pitch", 10000, NULL, 0, &WristPAMot, 1);
  xTaskCreatePinnedToCore(WRRA_Mot, "Wrist Roll", 10000, NULL, 0, &WristRAMot, 1);
  xTaskCreatePinnedToCore(Thumb_Mot, "Thumb Motor", 4096, NULL, 0, &ArmPAMot, 1);
  xTaskCreatePinnedToCore(Index_Mot, "Index Motor", 4096, NULL, 0, &ArmPAMot, 1);
  xTaskCreatePinnedToCore(Middle_Mot, "Middle Motor", 4096, NULL, 0, &ArmPAMot, 1);
  xTaskCreatePinnedToCore(Ring_Mot, "Ring Motor", 4096, NULL, 0, &ArmPAMot, 1);
  xTaskCreatePinnedToCore(Pinky_Mot, "Pinky Motor", 4096, NULL, 0, &ArmPAMot, 1);
  
  xTaskCreatePinnedToCore(Sensor_Feedback, "Sensor Feedback", 20000, NULL, 1, &Feedback, 0);
  
  #if DEBUGSYS
  Serial.println("FreeRTOS Tasks have been Handled");
  #endif
}

void Library_Initalization() {
  /* Allow allocation of timers */
  ESP32PWM::allocateTimer(0);  //Shoulder Timer
  ESP32PWM::allocateTimer(1);  //Arm Timer
  ESP32PWM::allocateTimer(2);  //Hand Timer
  ESP32PWM::allocateTimer(3);  //Hand Timer

  #if DEBUGSYS
  Serial.println("Timers Allocated");
  #endif

  /* Embedded Protocol Ports */
  K210Serial.begin(115200, SERIAL_8N1, 17, 18);  //Open K210 Port
  randomSeed(analogRead(1));                     //Activate randomness

  //#if i2c_EN
  Wire.begin();                   //i2C at 100KHz
  //#endif

  #if DEBUGSYS
  Serial.println("Embedded Communication Settled");
  #endif

  /* Palm sensor initalization */
  //#if i2c_EN
  IRSen.setTimeout(500); //500ms Read Periods
  IRSen.init();
  IRSen.startContinuous();

  #if DEBUGSYS
  Serial.println("VL53L0X Sensor Initalized");
  #endif

  /* Stepper Motor Encoder */
  //SHYAS.begin(); 
  //SHYAS.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  #if DEBUGSYS
  Serial.println("Shoulder AS5600 Initalized");
  #endif

  //FRAS.begin();  
  //FRAS.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  #if DEBUGSYS
  Serial.println("Forearm AS5600 Initalized");
  #endif

  //#endif
}

void Motor_Initalization() {
  /* Defining Servo Motors Frequency */
  HT.setPeriodHertz(50);   // Standard 50hz servo (Thumb)
  HI.setPeriodHertz(50);   // Standard 50hz servo (Index)
  HM.setPeriodHertz(50);   // Standard 50hz servo (Middle)
  HR.setPeriodHertz(50);   // Standard 50hz servo (Ring)
  HP.setPeriodHertz(50);   // Standard 50hz servo (Pinky)
  FP.setPeriodHertz(50);   // Standard 50hz servo (Wrist)
  EP.setPeriodHertz(50);   // Standard 50hz servo (Elbow)
  SHR.setPeriodHertz(50);  // Standard 50hz servo (Shoulder Roll)
  SHP.setPeriodHertz(50);  // Standard 50hz servo (Shoulder Pitch)

  #if DEBUGSYS
  Serial.println("Servo set to 50Hz");
  #endif

  /* Attaching Servo to Pin */
  FP.attach(WRIST, 500, 2500);
  EP.attach(ELBOW, 500, 2500);
  SHR.attach(SHR_RO, 500, 2500);
  SHP.attach(SHR_PI, 500, 2500);

  #if DEBUGSYS
  Serial.println("Attached Position Servos to Pins");
  #endif

  /* Stepper Motor Pinout */
  pinMode(SHY_DIR, OUTPUT);
  pinMode(SHY_STEP, OUTPUT);
  pinMode(SHY_EN, OUTPUT);
  pinMode(SHY_HallEffect, INPUT);
  digitalWrite(SHY_EN, HIGH);

  #if DEBUGSYS
  Serial.println("Shoulder Stepper Initalized");
  #endif

  pinMode(FR_DIR, OUTPUT);
  pinMode(FR_STEP, OUTPUT);
  pinMode(FR_EN, OUTPUT);
  pinMode(FR_HallEffect, INPUT);
  digitalWrite(FR_EN, HIGH);

  #if DEBUGSYS
  Serial.println("Forearm Stepper Initalizated");
  #endif
}

void Bluetooth_Initialization() {
  #if BUILD == 1
    BLEDevice::init("LUROX D: AZAMI");

  #elif BUILD == 2
    BLEDevice::init("LUROX D: ENE");

  #elif BUILD == 3
    BLEDevice::init("LUROX D: ARES");

  #elif BUILD == 4
    BLEDevice::init("LUROX D: WIM");

  #endif
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();
  pServer->getAdvertising()->start();
  startTime = millis();
}

/***************************************************************************************** 
                                Start-Up Procedure Functions
******************************************************************************************/

void Stepper_Home() {
  /* Using the Hall Effect Sensor read the Forearm and Stepper and home */
  // StepperTimer = millis();

  // // Enable Motor
  // digitalWrite(SHY_EN, LOW);

  #if DEBUGSYS
  Serial.println("Homing SHY Motor");
  #endif

  /* Initial force to overcome Static Friction */
  for (int Init = 0; Init < 3; Init++) {
    digitalWrite(SHY_STEP, HIGH);  // Assumes Sstep2pin is defined
    delayMicroseconds(500);        // Adjust for motor speed
    digitalWrite(SHY_STEP, LOW);
    delayMicroseconds(500);
  }

  // while(analogRead(SHY_HallEffect) < 100) {
  //   digitalWrite(SHY_STEP, HIGH);  // Assumes Sstep2pin is defined
  //   delayMicroseconds(500);        // Adjust for motor speed
  //   digitalWrite(SHY_STEP, LOW);
  //   delayMicroseconds(500);

  //   if ((millis() - StepperTimer) > 10000) {

  //     #if DEBUGSYS
  //     Serial.println("SHY Home Timed Out!");
  //     #endif

  //     digitalWrite(SHY_EN, HIGH);
  //     break;
  //   }
  // } 

  // if((millis() - StepperTimer) <= 10000) {
  //   int32_t SHY_home_pos = static_cast<int32_t>(round(0 * COUNTS_PER_DEGREE));
  //   SHYAS.resetPosition(SHY_home_pos);
  //   SHY_home = true;
  //   ArmYA[2] = UpperArmSensor();
  //   ArmYA[1] = ArmYA[2];

  //   #if DEBUGSYS
  //   Serial.println("Homed SHY Motor!");
  //   #endif
  // }
  
  // digitalWrite(SHY_EN, HIGH); /*Disable SHY Motor */
  // StepperTimer = millis();    /* Restart Timer for Forearm motor */

  // digitalWrite(FR_EN, LOW);

  #if DEBUGSYS
  Serial.println("Homing FR Motor");
  #endif

  /* Initial force to overcome Static Friction */
  for (int Init = 0; Init < 3; Init++) {
    digitalWrite(FR_STEP, HIGH);  
    delayMicroseconds(500);        // Adjust for motor speed
    digitalWrite(FR_STEP, LOW);
    delayMicroseconds(500);
  }

  // while(analogRead(FR_HallEffect) < 100) {
  //   digitalWrite(FR_STEP, HIGH);  
  //   delayMicroseconds(500);       // Adjust for motor speed
  //   digitalWrite(FR_STEP, LOW);
  //   delayMicroseconds(500);

  //   if ((millis() - StepperTimer > 10000)) {

  //     #if DEBUGSYS
  //     Serial.println("FR Home Timed Out!");
  //     #endif

  //     digitalWrite(FR_EN, HIGH);
  //     break;
  //   }
  // }

  // if(millis() - StepperTimer <= 10000) {
  //   int32_t FR_home_pos = static_cast<int32_t>(round(0 * COUNTS_PER_DEGREE));
  //   FRAS.resetPosition(FR_home_pos);
  //   FR_home = true;
  //   WristRA[2] = HandSensor();
  //   WristRA[1] = WristRA[2];

  //   #if DEBUGSYS
  //   Serial.println("Homed FR Motor");
  //   #endif
  // }

  // digitalWrite(FR_EN, HIGH);  /* Disable FR Motor */
}
