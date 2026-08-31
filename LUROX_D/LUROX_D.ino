/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                                LUROX D Global Definitions
******************************************************************************************/

/* Timers */
unsigned long GlobalTimer = 0;
unsigned long startTime = 0;
unsigned long ResetTimer = 0;
unsigned long StepperTimer = 0;

/* Decision Backbone Functions */
bool CMD_IN = false;
bool ObjFound = false;
bool HandTrack = false;
uint8_t request, intent, objective, specification;

/* Global Function Flags */
bool HandRot = false;
bool SleepT = false;
bool Grab = false;
bool Wander = true;
bool SleepState = false;
bool manualMode = false;
bool Anim_Break = false;
#define DEBUGSYS true

/***************************************************************************************** 
                             Primary Initalizations Functions
******************************************************************************************/

void setup() {
    /* Initalization for LUROX D */
    #if DEBUGSYS
    Serial.begin(115200);
    #endif 

    Library_Initalization(); /* Primary Libraries, Sensors and etc */
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

    #if DEBUGSYS
    Serial.println("===========================");
    #endif
    //Standby();
     Wave_Movement();
     vTaskDelay(pdMS_TO_TICKS(10000));
     Handshake();
     vTaskDelay(pdMS_TO_TICKS(10000));
  }
  
  /* Sleep Mode */
  if (Wander == false && SleepState == true) {
    Neutral_Position();
    Sleep();    // Power Saving
  }

  /* This loop wil primarily focus on the control loop reading information and kinematics instruction to grab objects */
  if (CMD_IN == true) {
    Wander = false; // Disable Wandering
    Extended_Position(); //Return to Standby-State
    Decision_Backbone(request, intent, specification, objective); /* Actions will be governed */
  }

}

