/*********************************************************************************************** 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

/***************************************************************************************** 
                                  Sensor Feedback Functions
******************************************************************************************/

float HandSensor() { //P2 Vector
  //Read Stepper Motor Position/Revolutions
  //FR_Pos = FRAS.getCumulativePosition();
  Obj_Dist = IRSen.readRangeContinuousMillimeters(); // Sensor Read
  // Serial.println(Obj_Dist);
  return 1;
}

void Object_Grasp() {
  if (Obj_Dist < 75 && Grab == true) {
    Gestures[0] = 1; // Close Hand
    HandCode(); // Push Update to Hand
  }

  else {
    Gestures[0] = 0; //Open Hand
    HandCode(); // Push Update to Hand
  }
}

bool VL53_DataReady() {
  return IRSen.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x01;   /* RESULT_INTERRUPT_STATUS, GPIO bit */
}



