/***********************************************************************************************
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                                  L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

***********************************************************************************************/

/***************************************************************************************** 
                            Vector Position Kinematics Functions
******************************************************************************************/

#define DEBUGSYS true
const uint8_t joint5Limits[2] = { 30, 225 }; /* 135 is Nominal, range from 30 to 225 */
const uint8_t joint6Limits[2] = { 40, 110 }; /* 90 is Nominal, range from 40 to 110 */
const uint8_t Hand_Link = 25; /* Actual rotation distance is 25mm */

/* Mathmatics Variables */
const uint8_t Vector_Length[4] = {70, 25, 210, 230}; /* MM */
const uint8_t Neutral_Pos = 135;
const uint16_t Joint_limits[4][2] = {
                                   {115, 155}, /* Shoulder Roll */
                                   {133, 205}, /* Shoulder Pitch */
                                   {0, 270},   /* Shoulder Yaw */
                                   {130, 205}  /* Elbow Pitch */
                                   }; 
const float W[4] = {0.25, 0.2, 0.1, 0.005}; /* Angle Abuse Weights */
const double IK_Filter_Deg = 10.0;     /* kinematic bounding box   */

const double Grasp_Y_Budget = 100.0;  /* XY-plane pullback at x=0 */
const double Grasp_X_Scale  = 200.0; /* |X| where pullback fades to 0 */
const double Grasp_Z_Offset = 75.0; /* sensor depth offset reduce */

/* Rotation matrix helpers (inline for speed) */
static inline void rot_x(double phi, double R[3][3]) {
    double c = cos(phi), s = sin(phi);
    R[0][0] = 1.0;  R[0][1] = 0.0;  R[0][2] = 0.0;
    R[1][0] = 0.0;  R[1][1] =    c;  R[1][2] =   -s;
    R[2][0] = 0.0;  R[2][1] =    s;  R[2][2] =    c;
}

static inline void rot_y(double phi, double R[3][3]) {
    double c = cos(phi), s = sin(phi);
    R[0][0] =    c;  R[0][1] = 0.0;  R[0][2] =    s;
    R[1][0] = 0.0;  R[1][1] = 1.0;  R[1][2] = 0.0;
    R[2][0] =   -s;  R[2][1] = 0.0;  R[2][2] =    c;
}

static inline void rot_z(double phi, double R[3][3]) {
    double c = cos(phi), s = sin(phi);
    R[0][0] =    c;  R[0][1] =    s;  R[0][2] = 0.0;
    R[1][0] =   -s;  R[1][1] =    c;  R[1][2] = 0.0;
    R[2][0] = 0.0;  R[2][1] = 0.0;  R[2][2] = 1.0;
}

static inline void rot_z_reg(double phi, double R[3][3]) {
    double c = cos(phi), s = sin(phi);
    R[0][0] =    c;  R[0][1] =   -s;  R[0][2] = 0.0; // Negative sine here
    R[1][0] =    s;  R[1][1] =    c;  R[1][2] = 0.0; // Positive sine here
    R[2][0] = 0.0;   R[2][1] = 0.0;   R[2][2] = 1.0;
}

/* Matrix-vector multiply (3x3 * 3x1) */
static inline void mat_vec_mul(const double M[3][3], const double v[3], double result[3]) {
    for (int i = 0; i < 3; ++i)
        result[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
}

float Pos_Fwrd_Kin(double theta_deg[4], double pos[3], double R_out[3][3]) {  //Forward Kinematics -> Return Position

  double R_Sum[3][3]; /* 3 x 3 Matrix which is the sum of all transformations */
  double phi[4];
  double p[3] = {0.0, 0.0, 0.0};

  /* Identity matrix as initial cumulative rotation */
  for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_Sum[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

  for (int i = 0; i < 4; i++) {
    phi[i] = DEG_TO_RAD(theta_deg[i] - Neutral_Pos); /* Setting the Angles to Radians and subtracting from Neutral */
  }
  
  for (int i = 0; i < 4; ++i) {
        double R[3][3];
        if (i == 0)
            rot_y(phi[i], R);                 /* Joint 1 : roll  (Y) */
        else if (i == 1)
            rot_x(phi[i], R);                 /* Joint 2 : pitch (X) */
        else if (i == 2)
            rot_z(phi[i], R);                 /* Joint 3 : yaw   (Z) */
        else
            rot_x(phi[i], R);                 /* Joint 4 : pitch (X) */

        /* Update cumulative rotation: R_cum = R_cum * R */
        double tmp[3][3];
        memcpy(tmp, R_Sum, sizeof(tmp));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                R_Sum[r][c] = tmp[r][0]*R[0][c] + 
                              tmp[r][1]*R[1][c] + 
                              tmp[r][2]*R[2][c];
            }

        double d[3] = {0.0, 0.0, -Vector_Length[i]};
        double v[3];
        mat_vec_mul(R_Sum, d, v);

        p[0] += v[0];
        p[1] += v[1];
        p[2] += v[2];
    }
    pos[0] = p[0];
    pos[1] = p[1];
    pos[2] = p[2];    

    memcpy(R_out, R_Sum, sizeof(R_Sum));
    return 1;
}

float Invrs_Kin(const double p_d[3], const double theta_init[4], double theta_out[4]) {  //Inverse Kinematics desired X,Y,Z
    const double alpha = 0.001;
    const double beta = 0.4;
    const int max_iter = 2000;
    const double tol = 1.0; /* mm */

    double R_arm[3][3];

    double Inv_theta[4];
    memcpy(Inv_theta, theta_init, sizeof(Inv_theta));

    double v[4] = {0.0, 0.0, 0.0, 0.0};
    double prev_total_error = INFINITY;
    int iter = 0;

    /* ---- KINEMATIC FILTER : ±10° bounding box (seed-centred) ---- */
    double local_lim[4][2];
    for (int i = 0; i < 4; ++i) {
        local_lim[i][0] = fmax((double)Joint_limits[i][0], theta_init[i] - IK_Filter_Deg);
        local_lim[i][1] = fmin((double)Joint_limits[i][1], theta_init[i] + IK_Filter_Deg);
    }

    /* ---- DIRECT HEADING INJECTION for joint 1 ---- */
    Inv_theta[0] = (p_d[0] * (-2.0 / 7.0)) + 135.0;
    /* safety clamp to HARDWARE limits */
    if (Inv_theta[0] < Joint_limits[0][0]) Inv_theta[0] = Joint_limits[0][0];
    if (Inv_theta[0] > Joint_limits[0][1]) Inv_theta[0] = Joint_limits[0][1];

    for (iter = 1; iter <= max_iter; ++iter) {
        double p_curr[3]; /* Array of Vector Current Position */
        Pos_Fwrd_Kin(Inv_theta, p_curr, R_arm); /* Calculate Actual Current Position relative to Angles */

        double e[3] = { /* Distance difference */
            p_d[0] - p_curr[0],
            p_d[1] - p_curr[1],
            p_d[2] - p_curr[2]
        };

        double dist_err = sqrt( (e[0]*e[0]) + (e[1]*e[1]) + (e[2]*e[2])); /* Distance Function for error */

        if (dist_err < tol) break;

        /* Angle abuse term */
        double angle_abuse = 0.0;
        for (int i = 0; i < 4; i++) {
            double dev = Inv_theta[i] - Neutral_Pos;
            angle_abuse += W[i] * dev * dev;
        }

        angle_abuse *= 0.75;

        double total_error = dist_err + angle_abuse;

        /* Dynamic step size */
        double step_size = (total_error / 5.0) * alpha;

        /* Heuristic error → joint mapping */
        double delta_theta[4] = {0.0, 0.0, 0.0, 0.0};

        if (fabs(e[0]) > 0.1 || fabs(e[1]) > 0.1) {
            delta_theta[0] = -e[0] * step_size;   /* θ1 mainly affects X */
            delta_theta[1] = (e[1] + e[2]) * step_size;   /* θ2 mainly affects Y */
        }
        if (fabs(e[2]) > 0.1 || fabs(e[0]) > 0.1) {
            delta_theta[2] = (e[0] + e[1] + e[2]) * step_size * 0.5;  /* θ3 helps X/Z */
            delta_theta[3] = e[2] * step_size;        /* θ4 mainly affects Z */
        }

        /* Momentum update */
        for (int i = 0; i < 4; ++i)
            v[i] = beta * v[i] + (1.0 - beta) * delta_theta[i];

        /* Apply velocity */
        for (int i = 0; i < 4; ++i)
            Inv_theta[i] += v[i];

        /* Clamp to joint limits */
        for (int i = 0; i < 4; ++i) {
            if (Inv_theta[i] < local_lim[i][0]) Inv_theta[i] = local_lim[i][0];
            if (Inv_theta[i] > local_lim[i][1]) Inv_theta[i] = local_lim[i][1];
        }

        /* Stagnation backtrack */
        if (total_error >= prev_total_error) {
            for (int i = 0; i < 4; ++i) {
                Inv_theta[i] -= v[i] * 0.825;
                v[i] *= 0.7;
            }
            /* Re-clamp after backtrack */
            for (int i = 0; i < 4; ++i) {
                if (Inv_theta[i] < local_lim[i][0]) Inv_theta[i] = local_lim[i][0];
                if (Inv_theta[i] > local_lim[i][1]) Inv_theta[i] = local_lim[i][1];
            }
        }

        prev_total_error = total_error;
    }

    /* Final evaluation */
    double p_final[3];
    Pos_Fwrd_Kin(Inv_theta, p_final, R_arm);

    double err = sqrt(pow(p_final[0]-p_d[0],2) +
                      pow(p_final[1]-p_d[1],2) +
                      pow(p_final[2]-p_d[2],2));
    memcpy(theta_out, Inv_theta, sizeof(Inv_theta));
    return (float)err;
}

float Hand_Fwrd_Kin(float pitch, float roll, int16_t magnitude, double pos_out[3], double R_out[3][3]) {
    double R_Sum[3][3];
    double p[3] = {0.0, 0.0, 0.0};

    /* Sensor Calibration */
    magnitude = magnitude * 0.8; /* Slight dampening */

    /* Identity matrix as initial cumulative rotation */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_Sum[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    /* Joint 5 : WristRA - roll about Z (neutral 135) */
    double R[3][3];
    rot_z_reg(DEG_TO_RAD((double)roll - 135.0), R);

    /* Update cumulative rotation: R_cum = R_cum * R */
    double tmp[3][3];
    memcpy(tmp, R_Sum, sizeof(tmp));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            R_Sum[r][c] = tmp[r][0]*R[0][c] +
                          tmp[r][1]*R[1][c] +
                          tmp[r][2]*R[2][c];
        }

    double d[3] = {0.0, 0.0, -Hand_Link};   /* 25mm link to WristPA */
    double v[3];
    mat_vec_mul(R_Sum, d, v);
    p[0] += v[0];
    p[1] += v[1];
    p[2] += v[2];

    /* Joint 6 : WristPA - pitch about X (neutral 90) */
    rot_x(DEG_TO_RAD((double)pitch - 90.0), R);

    memcpy(tmp, R_Sum, sizeof(tmp));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            R_Sum[r][c] = tmp[r][0]*R[0][c] +
                          tmp[r][1]*R[1][c] +
                          tmp[r][2]*R[2][c];
        }

    double d2[3] = {0.0, -magnitude, 0.0};   /* IR distance to object */
    mat_vec_mul(R_Sum, d2, v);
    p[0] += v[0];
    p[1] += v[1];
    p[2] += v[2];

    pos_out[0] = p[0];
    pos_out[1] = p[1];
    pos_out[2] = p[2];

    memcpy(R_out, R_Sum, sizeof(R_Sum));

    return 1.0f;
}

int32_t Hand_CenterCam(float CamX, float CamY, uint8_t A5, uint8_t A6, uint8_t* A5N, uint8_t* A6N) {
  /* Verify new X,Y have been achieved before moving to prevent runaway */
  if ((fabsf(CamX - objX[1]) > 1) || (fabsf(CamY - objY[1]) > 1)) {
  // Iteration Method of Inverse Kinematics
  if (!A5N || !A6N) return -1;

  const float Kp = 0.5;  // Proportional Gain
  const float DEG_PER_PX = 0.2; 
  const float MAX_STEP = 20;

  const uint8_t CenterX = 112;
  const uint8_t CenterY = 112;
  const uint8_t DEADZONE = 36; /* Heavy Deadzone especially if close to camera */

  if ((CamX <= 1 && CamY <= 1) || (CamX >= 254 && CamY >= 254)) {
    *A5N = round(A5);
    *A6N = round(A6);
    return -1;  /* target lost, hold */
  }

  // Error from Center
  float ErrorX = CamX - CenterX;
  float ErrorY = CenterY - CamY;

  /* Error Distance from X and Y */
  float Cam_DistR = sqrt((ErrorX * ErrorX) + (ErrorY * ErrorY));

  /* Camera generally centered = Stop moving and hold */
  if (Cam_DistR < DEADZONE) {
    *A5N = round(A5);
    *A6N = round(A6);
    objX[1] = CamX;
    objY[1] = CamY;
    return 1; /* Locked onto Target */
  }
  
  // Roll Alignment
  float targetRollDeg = RAD_TO_DEG(atan2(ErrorY, ErrorX));
  float rollError = 90.0 - targetRollDeg;
  rollError = fmodf((rollError + 180.0), 360.0);
  rollError -= 180.0;

  float pitchError = Cam_DistR * cos(DEG_TO_RAD(rollError)) * DEG_PER_PX;

  // 4. Update Kinematics proportionally
  float rollStep  = constrain(Kp * rollError, -MAX_STEP, MAX_STEP);
  float pitchStep = constrain(Kp * pitchError, -MAX_STEP, MAX_STEP);

  // Constrain to angles
  *A5N = (int)round(constrain(A5 + rollStep, joint5Limits[0], joint5Limits[1]));
  *A6N = (int)round(constrain(A6 - pitchStep, joint6Limits[0], joint6Limits[1]));
  objX[1] = CamX; /* Load the Camera values into the Past State */
  objY[1] = CamY;

  return 0; /* Actively Tracking for Object */
  }

  else {
    return 0; /* Not detected likely! */
  }
}

uint8_t Hand_Align(double theta_deg[4], uint8_t mode, uint8_t* A5_out, uint8_t* A6_out) {
    if (!A5_out || !A6_out) return 0;

    double pos[3];
    double R_arm[3][3];
    Pos_Fwrd_Kin(theta_deg, pos, R_arm);                /* Current arm X,Y,Z position */
    const double dx = R_arm[2][0];
    const double dy = R_arm[2][1];
    const double dz = R_arm[2][2];

    double alpha = atan2(dx, dy);                       /* azimuth of the dial (rad) */
    double beta  = asin(constrain(dz, -1.0, 1.0));      /* elevation split   (rad) */

    if (mode == MODE_SIDE_SWIPE) alpha += PI / 2.0;     /* 90 deg roll about forearm */

    double A5 = 135.0 + RAD_TO_DEG(alpha);
    double A6 = 90.0 - RAD_TO_DEG(beta);

    *A5_out = (uint8_t)lround(constrain(A5, joint5Limits[0], joint5Limits[1]));
    *A6_out = (uint8_t)lround(constrain(A6, joint6Limits[0], joint6Limits[1]));
    return 1;
}

void Object_Position(double theta_deg[4], float A5, float A6, double global_obj_pos[3]) {
    Serial.println("Objective Position Active!");

    double wrist_pos[3];
    double R_arm[3][3];

    // 1. Get Arm's Global Position and Rotation Matrix (Joints 1 to 4)
    Pos_Fwrd_Kin(theta_deg, wrist_pos, R_arm);

    // 2. Get Object's Local XYZ Vector (Joints 5 to 6)
    double hand_local[3];
    double R_hand[3][3];
    Hand_Fwrd_Kin(A6, A5, Obj_Dist, hand_local, R_hand);

    Serial.println("Hand Forward Kinematics Ran");
    
    /* Rotate hand-local vector into global space: R_arm * p_local */
    double global_vec[3];
    mat_vec_mul(R_arm, hand_local, global_vec);

    Serial.println("Rotate scaled vector to global space ");

    // 6. Translate: Add Global Ray to True Camera Origin
    global_obj_pos[0] = wrist_pos[0] + global_vec[0];
    global_obj_pos[1] = wrist_pos[1] + global_vec[1];
    global_obj_pos[2] = wrist_pos[2] + global_vec[2];

    double xn = global_obj_pos[0] / Grasp_X_Scale;
    if (xn >  1.0) xn =  1.0;
    if (xn < -1.0) xn = -1.0;
    global_obj_pos[1] -= Grasp_Y_Budget * sqrt(1.0 - xn * xn);
    global_obj_pos[2] += Grasp_Z_Offset;

    Serial.println("Add global ray to camera origin");
}

