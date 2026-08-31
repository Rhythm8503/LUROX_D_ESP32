/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                              Libraries and Library Pointers
******************************************************************************************/
#include <SPI.h>                    //Serial Peripheral Interface for K210
#include <math.h>                   //Mathmatics Library
#include <string.h>                 //Converting strings from UART
#include <HardwareSerial.h>         //Open Serial Ports
#include <Wire.h>                   //i2C Initalization for ESP32-S3
#include "freertos/FreeRTOS.h"      //FreeRTOS OTA
#include "freertos/task.h"          //FreeRTOS Task
#include "freertos/event_groups.h"  //FreeRTOS Event Groups
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include "AS5600.h"                 //Stepper motor encoders
#include <VL53L0X.h>                //VL53L0X Distance Sensor
#include <ESP32Servo.h>             //ESP32-S3 ISR Servo Manager
#include <float.h>

/* Pointers for Libraries */
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

VL53L0X IRSen;  //Palm sensor
//AS5600L SHYAS(AS5600L_DEFAULT_ADDRESS, &Wire);  //Encoder Shoulder Yaw
//AS5600L FRAS(AS5600L_DEFAULT_ADDRESS, &Wire);   //Encoder Forearm Roll
ESP32PWM pwm;   //PWM Controller on ESP32-S3

Servo HT;       //Thumb
Servo HI;       //Index
Servo HM;       //Middle
Servo HR;       //Ring
Servo HP;       //Pinky
Servo FP;       //Wrist
Servo EP;       //Elbow
Servo SHP;      //Shoulder pitch
Servo SHR;      //Shoulder roll

//UART Port for K210
HardwareSerial K210Serial(2);

/***************************************************************************************** 
                                  FreeRTOS Thread Handles
******************************************************************************************/

// FreeRTOS Task Handle
TaskHandle_t ArmYAMot;    //Shoulder Yaw Control
TaskHandle_t ArmPAMot;    //Shoulder Pitch Control
TaskHandle_t ArmRAMot;    //Shoulder Roll Control
TaskHandle_t ElbowPAMot;  //Elbow Pitch Control
TaskHandle_t WristPAMot;  //Wrist Pitch Control
TaskHandle_t WristRAMot;  //Wrist Roll Control

TaskHandle_t ThumbMot;    //Thumb Servo Control
TaskHandle_t IndexMot;    //Index Servo Control
TaskHandle_t MiddleMot;   //Middle Servo Control
TaskHandle_t RingMot;     //Ring Servo Control
TaskHandle_t PinkyMot;    //Pinky Servo Control

TaskHandle_t Feedback;           //i2C Sensor Feedback & Management

/***************************************************************************************** 
                                      Definitions
******************************************************************************************/

/* Definitions for the ARM */
#define BUILD 4  // 1 Azami, 2 Ene, 3 Wim, 4 Ares
#define PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / PI)

/* Servo Motor Pinout */
#define THUMB 38      //Servo Thumb
#define INDEX 37      //Servo Index
#define MIDDLE 36     //Servo Middle
#define RING 35       //Servo Ring
#define PINKY 48      //Servo Pinky
#define WRIST 47      //Servo Wrist
#define ELBOW 21      //Servo Elbow
#define SHR_PI 20     //Shoulder Pitch
#define SHR_RO 41     //Shoulder Roll

/* General Pin Out for Motors */
#define SHY_DIR 15     //Shoulder Yaw Direction Pin
#define SHY_STEP 16    //Shoulder Yaw Step Pin
#define SHY_EN 17      //Shoulder Yaw Enable Pin
uint16_t SYSP = 400;   //Microsecond Pulse Width for Shoulder Yaw Steps

#define FR_DIR 1      //Forearm Roll Direction Pin
#define FR_STEP 2     //Forearm Roll Step Pin
#define FR_EN 42      //Forearm Roll Enable Pin
uint16_t FRSP = 500;  //Microsecond Pulse Width for Forearm Roll Steps

/* Hall Effect Sensor Pins */
#define FR_HallEffect 6
#define SHY_HallEffect 7

/* Communication Sepecifications */
#define RX_PIN 10
#define TX_PIN 11
#define STATE_WAITING_LAYER1       0
#define STATE_READING_LAYER1       1
#define STATE_WAITING_LAYER2       2
#define STATE_READING_LAYER2       3
#define STATE_WAITING_LAYER3_CLOSE 4

#define MODE_STANDARD 0
#define MODE_CON    1
#define MODE_PARTY  2
#define MODE_MANUAL 3
#define MODE_SLEEP  4

/* Semaphore Definitions */
#define MAX_CONCURRENT_MOTORS 5
#define MAX_FINGERCONCURRENT_MOTORS 3

/* DEBUG & SETTINGS */
#define DEBUGSYS true
#define i2C_EN true

/* BLUETOOTH UUID PER BUILD */
#define SERVICE_UUID "2a1f5803-d723-4b20-885a-36fcf9cfe56a"  // UART ID
#define CHARACTERISTIC_UUID_RX "ad532eb5-0db3-4f1f-b0ca-111e50a9d2ec"
#define CHARACTERISTIC_UUID_TX "f47feb80-c7cd-41dc-b72c-6e6d6273287c"


/***************************************************************************************** 
                                    Global Variables
******************************************************************************************/

/* Motor Positions & Logs */

/* Desired | Motor Moved | Encoder Actual | */

uint8_t  WristPA[3] = {90, 90, 0 };   // Wrist Pitch
uint16_t ElbowPA[3] = {135, 135, 0 };  // Elbow Pitch
uint8_t  WristRA[3] = {135, 135, 0 };  // Wrist Roll 
uint16_t ArmRA[3] =   {135, 135, 0 };  // Shoulder Roll
uint16_t ArmPA[3] =   {135, 135, 0 };  // Shoulder Pitch
uint16_t ArmYA[3] =   {135, 135, 0 };  // Shoulder Yaw

uint8_t ThumbRA[2] =  {180, 0};       // Thumb 
uint8_t IndexRA[2] =  {180, 0};       // Index
uint8_t MiddleRA[2] = {180, 0};       // Middle
uint8_t RingRA[2] =   {180, 0};       // Ring
uint8_t PinkyRA[2] =  {180, 0};       // Pinky

/* 0 -> Handshake, 1 = Request, 2 = Intention, 3 = Objective, 4 = Specification, 5 = Position X, 6 = Position Y, 7 = Box Area, 8 = Object */
uint8_t Gestures[3] = {0, 0, 0};  // 0 = Open, 1 = Close, 2 = Thumbs Up/Down, 3 = Peace, 4 = Middle, 5 = Point, 6 = Rock, 7 = Ok, 8 = Wave, 9 = Highfive

/* ################################################################################################ */

/* Positioning and Inverse Kinematics */
double CameraL[3] = {0, 0, 0};  /* Desired XYZ from Inverse Kinematics */
double CurrentL[3] = {0, 0, 0}; /* Current XYZ from Forward Kinematics */
int16_t Obj_Dist = 500;

/* ################################################################################################ */

/* General Modes: 0 = Standard, 1 = Convention Cosplay, 2 = Party Cosplay */
uint8_t GeneralMode;

/* Communication Variables */
uint8_t txValue = 0;
int state = STATE_WAITING_LAYER1;
int layer1_counter = 0;
int layer2_counter = 0;
uint8_t objX, objY, objW, objH; // Object XY and Box Size used to determine Desired Position
char commandBuffer[32];         // Buffer for command parsing
char originalCommand[32];       // Buffer to store original command for echo
uint8_t selectedLimb = 0;      

/* ################################################################################################ */

/* Decision Maps */
const int Request_Map[13] = {0, 0, 0, 0, 1, 1, 2, 2, 2, 2, 2, 3, 3};
const int Intention_Map[14] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
const int Specification_Map[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
const int Objective_Map[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1};
const int Specification_Restricted_Map[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};

/* ################## MAPPING ####################### 

Request -> 0 - 3 = Request {Can [0], Would[1], Could[2], Please[3]}
Request -> 4 - 5 = Direct Command {Grab[4], Hold[5]}
Request -> 6 - 10 = Gestures {Wave[6], Handshake[7], ThumbsUp[8], ThumbsDown[9], High-Five[10]}
Request -> 11 - 12 = Interrupt/HALT {Stop[11], Cancel[12]}

Intention -> 0 - 3 = Command (Grab[0], Hold[1], Push[2], Pull[3])
Intention -> 4 - 11 = Gestures {Wave[4], Handshake[5], Thumbs Up[6], Thumbs Down[7], High-Five[8], Point[9], Peace[10], Ok! [11]}
Intention -> 12 - 13 =  Interrupt / HALT {Stop[12], Cancel[13]}

Specification -> 0 - 9 = Colors (Red[0], Orange[1], Yellow[2], Green[3], Blue[4], Purple[5], White[6], Black[7], Brown[8], Grey[9])
Specification -> 10 - 20 = Objective (Wallet[10], Pliers[11], Wrench[12], Cup[13], Phone[14], Screwdrivers[15], Scissors[16], Drill[17], Hammer[18], Can[19], Bottle[20])
Specification -> 21 - 22 = Interrupt (Stop[21], Cancel[22])

Objective -> 0 - 10 = Objective (Wallet[0], Pliers[1], Wrench[2], Cup[3], Phone[4], Screwdrivers[5], Scissors[6], Drill[7], Hammer[8], Can[9], Bottle[10])
Objective -> 11 - 12 = Interrupt (Stop[11], Cancel[12])

################## MAPPING #######################  */


const int node_input_max[] = {
  12, /* Max Input for Request */
  13, /* Max Input for Intention */
  22, /* Max Input for Specification */
  12, /* Max Input for Objective */
  22  /* Max Input for Specification, Restricted */
};

/* ################################################################################################ */

/* AS5600 Magnetic PID Controller */
const double Kp = 1.3;
const double Ki = 0.0005;
const double Kd = 0.01;

float KSUM_FR = 0.0;
float FR_Error;
float FR_eDot;
float FR_EInt;
float FR_Pos;
float FR_prevError = 0.0;

float KSUM_SHY = 0.0;
float SHY_Error;
float SHY_eDot;
float SHY_EInt;
float SHY_Pos;
float SHY_prevError = 0.0;

float FR_currentTime = 0;
float FR_deltaTime = 0;
float FR_prevTime = 0;
float SHY_currentTime = 0;
float SHY_deltaTime = 0;
float SHY_prevTime = 0;

/* ################################################################################################ */

/* AS5600 Angle System */
const float COUNTS_PER_DEGREE = 4096.0 / 360.0;

/***************************************************************************************** 
                              Boolean/Semaphore Definitions
******************************************************************************************/

/* Communication or Control Specific */ 
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool newCommandReceived = false;

/* Stepper Motor Homing */
bool FR_home = false;
bool SHY_home = false;

// Servo Motor Management
SemaphoreHandle_t motorSemaphore;
SemaphoreHandle_t fingerSemaphore;

/***************************************************************************************** 
                              Enumeration  Definitions
******************************************************************************************/

// Context enum for menu state
enum Context {
  MAIN,
  MODE_MENU,
  MANUAL_MENU,
  SELECTED_LIMB
};

// State variables
Context currentContext = MAIN;

/* Defining the Layers & Blocks in Decision Map */
typedef enum {
  REQUEST = 0,
  INTENTION = 1,
  SPECIFICATION = 2,
  OBJECTIVE = 3,
  SPECIFICATION_RESTRICTED = 4,
  GESTURE = 5,
  ACTION = 6,
  COLOR = 7,
  NOTHING = 8,
  HALT = 9,
} Node;


/***************************************************************************************** 
                                   Key Point Definitions
******************************************************************************************/

const int16_t Arm_Home[3] = {-150, 300, -100}; /* General Home Position */
const int16_t Arm_Extended[3] = {0, 300, -325}; /* Arm Extended out */

