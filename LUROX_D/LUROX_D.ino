/*********************************************************************************************** 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                    L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board & Maix-Bit K210
    LUROX D: Mark II Software
***********************************************************************************************/

SET_LOOP_TASK_STACK_SIZE(16384); 

/***************************************************************************************** 
                                LUROX D Global Definitions
******************************************************************************************/

/* Timers */
unsigned long GlobalTimer = 0;
unsigned long startTime = 0;
unsigned long ResetTimer = 0;
unsigned long StepperTimer = 0;

/* Decision Backbone Functions */
bool CMD_IN = false;   /* Notify when CMD has been received */
bool CMD_PROC = false; /* Semaphore of when processing is in operation */
bool ObjFound = false;
bool HandTrack = false;
bool Anim_Break = false;
bool ACT_Break = false;

/* Global Function Flags */
bool HandRot = false;
bool SleepT = false;
bool Grab = false;
bool Wander = false;
bool SleepState = false;
bool manualMode = false;

uint8_t request, intent, objective, specification;

#define DEBUGSYS true

/***************************************************************************************** 
                             Primary Initalizations Functions
******************************************************************************************/

void setup() {
    /* Initalization for LUROX D */
    #if DEBUGSYS
      Serial.begin(115200);
    #endif 

    Library_Initalization();    /* Primary Libraries, Sensors and etc */
    UART_Test();                /* Test UART Communication with K210 */
    Motor_Initalization();      /* Initialize all Servo Motors */
    Bluetooth_Initialization(); /* Initalize Bluetooth for Remote Control */
    FreeRTOS_Initalization();   /* FreeRTOS Task Functions Initalize */

    vTaskDelay(pdMS_TO_TICKS(2000));
    Extended_Position(); /* Move the Robotic arm to an extended position*/
}

/***************************************************************************************** 
                             Primary Comms/Decision Functions
******************************************************************************************/

void loop() {  
  //GlobalTimer = millis(); // Keep Track of everything
  vTaskDelay(pdMS_TO_TICKS(1));     /* Watchdog Trigger */
  Bluetooth_Handle();               /* Read Commands from BLE Terminal */
  Serial_Terminal();                /* Direct Connection from the Computer */

  /* Standy By Function */
  // if (((millis() - ResetTimer) > 15000) && Wander == true && SleepState == false) {
  //   ResetTimer = millis();

  //   #if DEBUGSYS
  //     Serial.println("===========================");
  //   #endif
  //   Standby();
  // }

  /* This loop wil primarily focus on the control loop reading information and kinematics instruction to grab objects */
  if (CMD_IN == true && CMD_PROC == false) {
    Wander = false;                                                 /* Disable Wandering */
    Extended_Position();                                            /* Return to Standby-State  */
    Decision_Backbone(request, intent, specification, objective);   /* Actions will be governed */
  }

}

