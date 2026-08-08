/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

/***************************************************************************************** 
                            Vector Position Kinematics Functions
******************************************************************************************/

/* Mathmatics Variables */
const uint8_t Vector_Length[4] = {70, 25, 210, 230}; /* MM */
const uint8_t Neutral_Pos = 135;
const uint16_t Joint_limits[4][2] = {
                                   {115, 155}, /* Shoulder Roll */
                                   {130, 220}, /* Shoulder Pitch */
                                   {0, 270},   /* Shoulder Yaw */
                                   {130, 225}  /* Elbow Pitch */
                                   }; 
const float W[4] = {0.25, 0.2, 0.1, 0.005}; /* Angle Abuse Weights */

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

float Pos_Fwrd_Kin(double theta_deg[4], double pos[3]) {  //Forward Kinematics -> Return Position

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
            rot_y(phi[i], R);
        else if (i == 1 || i == 3)
            rot_x(phi[i], R);
        else /* i == 2 */
            rot_z(phi[i], R);

        /* Update cumulative rotation: R_cum = R_cum * R */
        double tmp[3][3];
        memcpy(tmp, R_Sum, sizeof(tmp));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                R_Sum[r][c] = tmp[r][0]*R[0][c] + tmp[r][1]*R[1][c] + tmp[r][2]*R[2][c];
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

    return 1;
}

float Invrs_Kin(double Target[3], double theta_start[4], double Output_theta[4]) {  //Inverse Kinematics desired X,Y,Z
    const double alpha = 0.002;
    const double beta = 0.7;
    const int max_iter = 1000;
    const double tol = 1.0; /* mm */

    double Inv_theta[4];
    memcpy(Inv_theta, theta_start, sizeof(Inv_theta));

    double v[4] = {0.0, 0.0, 0.0, 0.0};
    double prev_total_error = INFINITY;
    double p_curr[3] = {0.0, 0.0, 0.0}; /* Array of Vector Current Position */
    int iter;

    for (iter = 1; iter <= max_iter; ++iter) {
        Pos_Fwrd_Kin(Inv_theta, p_curr); /* Calculate Actual Current Position relative to Angles */

        double e[3] = { /* Distance difference */
            Target[0] - p_curr[0],
            Target[1] - p_curr[1],
            Target[2] - p_curr[2]
        };
        double dist_err = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]); /* Distance Function for error */

        if (dist_err < tol) break;

        /* Angle abuse term */
        double angle_abuse = 0.0;
        for (int i = 0; i < 4; i++) {
            double dev = Inv_theta[i] - Neutral_Pos;
            angle_abuse += W[i] * dev * dev;
        }

        double total_error = dist_err + (angle_abuse * 0.5);

        /* Dynamic step size */
        double step_size = (total_error / 5.0) * alpha;

        /* Heuristic error → joint mapping */
        double delta_theta[4] = {0.0, 0.0, 0.0, 0.0};

        if (fabs(e[0]) > 0.1 || fabs(e[1]) > 0.1) {
            delta_theta[0] = e[0] * step_size;   /* θ1 mainly affects X */
            delta_theta[1] = e[1] * step_size;   /* θ2 mainly affects Y */
        }
        if (fabs(e[2]) > 0.1 || fabs(e[0]) > 0.1) {
            delta_theta[2] = e[0] * step_size * 0.5;  /* θ3 helps X/Z */
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
            if (Inv_theta[i] < Joint_limits[i][0]) Inv_theta[i] = Joint_limits[i][0];
            if (Inv_theta[i] > Joint_limits[i][1]) Inv_theta[i] = Joint_limits[i][1];
        }

        /* Stagnation backtrack */
        if (total_error >= prev_total_error) {
            for (int i = 0; i < 4; ++i) {
                Inv_theta[i] -= v[i] * 0.825;
                v[i] *= 0.7;
            }
            /* Re-clamp after backtrack */
            for (int i = 0; i < 4; ++i) {
                if (Inv_theta[i] < Joint_limits[i][0]) Inv_theta[i] = Joint_limits[i][0];
                if (Inv_theta[i] > Joint_limits[i][1]) Inv_theta[i] = Joint_limits[i][1];
            }
        }
        prev_total_error = total_error;
    }

    /* Final evaluation */
    Pos_Fwrd_Kin(Inv_theta, p_curr);
    memcpy(Output_theta, Inv_theta, sizeof(double) * 4);  /* return result in-place */

    return (prev_total_error < tol) ? 1 : 0;
}

float Object_Vector(float pitch, float roll, float magnitude, uint16_t* TargetX, uint16_t* TargetY, uint16_t* TargetZ, uint16_t* VectX, uint16_t* VectY, uint16_t* VectZ) {
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
    float x = sin_pitch * cos_roll; // x = sin(θ) * cos(φ)
    float y = sin_pitch * sin_roll; // y = sin(θ) * sin(φ)
    float z = cos_pitch;            // z = cos(θ)
    
    /* Multiplying the Vector by the magnitude from the distance sensor */
    *VectX = magnitude * x;
    *VectY = magnitude * y;
    *VectZ = magnitude * z; 

    /* Grab Arm Position */
    double Arm_Theta[4] = {ArmRA[1], ArmPA[1], ArmYA[1], ElbowPA[1]};

    Pos_Fwrd_Kin(Arm_Theta[4], Arm_Pos[3]);

    /* Determine Destination Vector */
    *TargetX = Arm_Pos[0] + VectX;
    *TargetY = Arm_Pos[1] + VectY;
    *TargetZ = Arm_Pos[2] + VectZ;
}

int32_t Hand_InvsKin(uint8_t CamX, uint8_t CamY, uint16_t A5, uint16_t A6, uint16_t* A5N, uint16_t* A6N) {
  // Iteration Method of Inverse Kinematics
  const float Kp = 0.5;  // Proportional Gain

  float joint5Limits[2] = {45, 225};
  float joint6Limits[2] = {60, 130};

  // Error from Center
  int ErrorX = 112 - CamX;
  int ErrorY = 112 - CamY;

  *A5N = A5 + (Kp * ErrorX);
  *A6N = A6 + (Kp * ErrorY);

  // Constrain to angles
  *A5N = constrain(*A5N, joint5Limits[0], joint5Limits[1]);
  *A6N = constrain(*A6N, joint6Limits[0], joint6Limits[1]);

  if (abs((ErrorX < 10)) && abs((ErrorY < 10))) {
    return 0;  // Finished Tracking
  } else {
    return 1;  // Still Searching
  }
}

float ApproachT(double A, double B) {
    double difference = B - A; /* Difference from W & H */
    double s = 1.0 / 35.0; /* Slope of the function */

    return (1.0 / (1.0 + exp(-k * difference))); /* Return sigmoid function */
}
