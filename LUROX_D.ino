/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software
    CORE VERSION
*/

/***************************************************************************************** 
                                LUROX D Global Definitions
******************************************************************************************/
/* Timers */
unsigned long GlobalTimer = 0;
unsigned long startTime = 0;
unsigned long ResetTimer = 0;
unsigned long StepperTimer = 0;

/* Global Function Flags */
bool HandRot = false;
bool SleepT = false;
bool Grab = false;
bool Wander = true;
bool SleepState = false;
bool manualMode = false;

/***************************************************************************************** 
                             Primary Initalizations Functions
******************************************************************************************/

void setup() {
    /* Initalization for LUROX D */
    Serial.begin(115200);
    Library_Initalization(); /* Primary Libraries, Sensors and etc */
    UART_Bootcheck();        /* Test UARY connection to K210 */
    Motor_Initalization();   /* Initialize all Servo Motors */
    Stepper_Home();          /* Home Stepper Motors */
    Bluetooth_Initialization(); /* Initalize Bluetooth for Remote Control */
    FreeRTOS_Initalization();   /* FreeRTOS Task Functions Initalize */

    vTaskDelay(pdMS_TO_TICKS(100));
}

/***************************************************************************************** 
                             Primary Comms/Decision Functions
******************************************************************************************/

void loop() {  
  //GlobalTimer = millis(); // Keep Track of everything
  vTaskDelay(pdMS_TO_TICKS(1)); //Watchdog Trigger
  K210_Handle();      //Read UART Commands from the K210
  Bluetooth_Handle(); //Read Commands from BLE Terminal

  /* Standy By Function (For Now) */
  if (((millis() - ResetTimer) > 15000) && Wander == true) {
    ResetTimer = millis();
    Serial.println("===========================");
    Standby();  // Standby Random Move Function
  }

  /* Sleep Mode */
  if (Wander == false && SleepState == true) {
    Sleep();    // Power Saving
  }

  /* This loop wil primarily focus on the control loop reading information and kinematics instruction to grab objects */
  /* Fake Function */
  if (CMD_IN == true) {
    Wander = false; //Disable wandering

    /* Base Position */
    WristPA[0] = 90;  //Wrist Pitch
    WristRA[0] = 135; //Wrist Roll

  if (ObjFound == false && HandTrack == false) { /* Inital Stage*/
    Search_timeout++;
    Search_Position(); /* Randomly Move to find object */
    delay(1000); 
    if (Search_timeout > 60) {
      Search_timeout = 0;
      CMD_IN = false;
      Serial.println("Object not found, search timed out!");
    }
  }
 
   if (ObjFound == true && HandTrack == false) {   /* Object Found, Aligning with Object */
    uint8_t HandInv_timeout = 0;
    /* Attempt Kinematics 10,000 times */
    while(Hand_InvsKin(objX, objY, WristRA[1], WristPA[1], &WristRA[0], &WristPA[1])) {
      HandInv_timeout++;
      if (HandInv_timeout > 10000) {
        ObjFound = false;
        CMD_IN = false;
        Serial.println("Timeout Alignment, Cannot grab!");
        break;
      }
    }
    if ((Hand_InvsKin(objX, objY, WristRA[1], WristPA[1], &WristRA[0], &WristPA[1])) == 0) {
      HandTrack = true; /* Activate FreeRTOS Thread to constantly track as arm moves */
    }
  }

  if (ObjFound == true && HandTrack == true) { /* Final Stage */
  /* Second, we need to determine the distance with infrared */
  Serial.println("Determining Distance Vector");
  Object_Vector(WristRA[1], WristPA[1], ObjDist, &CameraL[0], &CameraL[1], &CameraL[2], &Obj_Vect[0], &Obj_Vect[1], &Obj_Vect[2]);

  /* Third, we align with the Y axis relative from the home point */
  double Arm_Theta[4] = {ArmRA[1], ArmPA[1], ArmYA[1], ElbowPA[1]};
  double finalAngles[4];
  double AlignY[3] = {0, CameraL[1], ArmPos[2]}; /* X = 0, Y = Object Location, Z = Arm Z Position */

  Serial.println("Aligning with Y Axis");

  Invrs_Kin(AlignY, Arm_Theta, finalAngles);

  ArmRA[0] = finalAngles[0];
  ArmPA[0] = finalAngles[1];
  ArmYA[0] = finalAngles[2];
  ElbowPA[0] = finalAngles[3];
  
  /* Determine regression approach */

  ApproachT(ObjW, ObjH);

  }
  }
}

