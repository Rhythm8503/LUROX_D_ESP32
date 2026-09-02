/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2026
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                            Vector Position Kinematics Functions
******************************************************************************************/

#define DEBUGSYS false

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
const double   IK_Filter_Deg = 10.0;     /* kinematic bounding box   */

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

float Hand_Fwrd_Kin(float pitch, float roll, float* magnitude, float* outX, float* outY, float* outZ) {
    // Convert angles to radians
    float pitch_rad = DEG_TO_RAD(pitch);
    float roll_rad = DEG_TO_RAD(roll);

    // Compute direction vector using spherical coordinates
    // Pitch (θ) is the angle from the positive z-axis (0° points along +z)
    // Roll (φ) is the angle in the x-y plane from the positive x-axis
    float sin_pitch = sin(pitch_rad);
    float cos_pitch = cos(pitch_rad);
    float sin_roll = sin(roll_rad);
    float cos_roll = cos(roll_rad);

    // Unit direction vector
    *outX = sin_pitch * cos_roll; // x = sin(θ) * cos(φ)
    *outY = sin_pitch * sin_roll; // y = sin(θ) * sin(φ)
    *outZ = cos_pitch;            // z = cos(θ)

    *magnitude = Obj_Dist; // Retrieve Distance from IR Sensor
}

int32_t Hand_CenterCam(float CamX, float CamY, uint8_t A5, uint8_t A6, uint8_t* A5N, uint8_t* A6N) {
  // Iteration Method of Inverse Kinematics
  if (!A5N || !A6N) return 1;

  const float Kp = 0.5;  // Proportional Gain

  uint16_t joint5Neutral = 135;
  uint16_t joint6Neutral = 90;

  const uint8_t joint5Limits[2] = { 30, 240 }; /* 135 is Nominal, range from 30 to 240 */
  const uint8_t joint6Limits[2] = { 60, 120 }; /* 90 is Nominal, range from 60 to 120 */

  const uint8_t CenterX = 112;
  const uint8_t CenterY = 112;
  const uint8_t DEADZONE = 16;


  if (CamX < 0 || CamY < 0 || isnan(CamX) || isnan(CamY)) {
    *A5N = round(A5); // Retain current joint angle
    *A6N = round(A6); // Retain current joint angle
    return -1;  // Target lost, holding position
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
    return 1; /* Locked onto Target */
  }

  float angleRad = atan2(ErrorY, ErrorX);
  float targetRollDeg = RAD_TO_DEG(angleRad);
  float rollError = 90.0 - targetRollDeg;

  rollError = fmodf((rollError + 180.0), 360.0);

  if (rollError < 0) rollError += 360.0;
  rollError -= 180.0;

  // 4. Update Kinematics proportionally
  float nextA5 = A5 + (Kp * rollError);
  float nextA6 = A6 + (Kp * Cam_DistR);

  // Constrain to angles
  *A5N = (int)round(constrain(nextA5, joint5Limits[0], joint5Limits[1]));
  *A6N = (int)round(constrain(nextA6, joint6Limits[0], joint6Limits[1]));

  return 0; /* Actively Tracking for Object */
}

void Object_Position(double theta_deg[4], float A5, float A6, double global_obj_pos[3]) {
    double wrist_pos[3];
    double R_arm[3][3];
    float hand_local[3];
    float Magnitude;

    // 1. Get Arm's Global Position and Rotation Matrix (Joints 1 to 4)
    Pos_Fwrd_Kin(theta_deg, wrist_pos, R_arm);

    // 2. Get Object's Local XYZ Vector (Joints 5 to 6)
    Hand_Fwrd_Kin(A6, A5, &Magnitude, &hand_local[0], &hand_local[1], &hand_local[2]);

    // 3. Rotate local hand vector into global orientation: V_global = R_arm * hand_local
    double rotated_hand_vec[3];
    rotated_hand_vec[0] = R_arm[0][0]*hand_local[0] + R_arm[0][1]*hand_local[1] + R_arm[0][2]*hand_local[2];
    rotated_hand_vec[1] = R_arm[1][0]*hand_local[0] + R_arm[1][1]*hand_local[1] + R_arm[1][2]*hand_local[2];
    rotated_hand_vec[2] = R_arm[2][0]*hand_local[0] + R_arm[2][1]*hand_local[1] + R_arm[2][2]*hand_local[2];

    // 4. Translate: Add rotated hand vector to the global wrist position
    global_obj_pos[0] = wrist_pos[0] + rotated_hand_vec[0];
    global_obj_pos[1] = wrist_pos[1] + rotated_hand_vec[1];
    global_obj_pos[2] = wrist_pos[2] + rotated_hand_vec[2];
}

