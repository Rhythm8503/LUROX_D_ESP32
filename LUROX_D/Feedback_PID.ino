/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                                  Sensor Feedback Functions
******************************************************************************************/

float HandSensor() { //P2 Vector
  //Read Stepper Motor Position/Revolutions
  FR_Pos = FRAS.getCumulativePosition();
  Obj_Dist =  IRSen.readRangeContinuousMillimeters(); // Sensor Read

  return static_cast<float>(FR_Pos) * (360.0 / 4096.0);
}

float UpperArmSensor() { //P1 Vector
  //Read Stepper Motor Position/Revolutions
  SHY_Pos = SHYAS.getCumulativePosition();
  return static_cast<float>(FR_Pos) * (360.0 / 4096.0);
}



