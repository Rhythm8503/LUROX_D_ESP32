/* 
    Developed by Taheemuddin Ahmed with the Supervision of Dr.Wafi Danesh
    Learning, Observation, Understanding, Reasoning, Execution, Dynamic Prosthetic Algorithm.
                          L.U.R.O.X. D 2025
    Arduino Core: V3.2.1
    ESP32-S3 Board
    LUROX D: Mark II Software

*/

const double   Max_Reach     = 550.0;     /* mm  */
const int Traj_Points = 50;
/* Trajectory modes */
#define MODE_TOP_DOWN   1
#define MODE_SIDE_SWIPE 2
#define STEP_DELAY_MS  20   /* dwell per micro-step so servos physically settle */

/* ---- read the arm's current commanded angles into theta[4] ---- */
void Get_Current_Angles(double theta[4]) {
    theta[0] = (double)ArmRA[1];
    theta[1] = (double)ArmPA[1];
    theta[2] = (double)ArmYA[1];
    theta[3] = (double)ElbowPA[1];
}

/* ---- command all four joints to a new pose (degrees) ---- */
void Move_Arm_Pose(const double theta[4]) {
    ArmRA[0]   = (int)lround(theta[0]);
    ArmPA[0]   = (int)lround(theta[1]);
    ArmYA[0]   = (int)lround(theta[2]);
    ElbowPA[0] = (int)lround(theta[3]);
}


void Gen_Trajectory(const double p_start[3], const double p_target[3], int mode, double path[3][50]) {
    /* --- 1. control point --- */
    double p_ctrl[3] = {0.0, 0.0, 0.0};
    p_ctrl[1] = p_start[1];                       /* lock Y axis */

    if (mode == MODE_TOP_DOWN) {
        p_ctrl[0] = p_target[0];                  /* X = target */
        p_ctrl[2] = p_start[2];                   /* Z = start  */
    } else { /* SIDE_SWIPE */
        p_ctrl[0] = p_start[0];                   /* X = start  */
        p_ctrl[2] = p_target[2];                  /* Z = target */
    }

    /* --- 2. 50-point Bezier sweep --- */
    for (int i = 0; i < 50; ++i) {
        double t = (double)i / (double)(Traj_Points - 1);
        double u = 1.0 - t;
        double pt[3];
        for (int k = 0; k < 3; ++k)
            pt[k] = u*u * p_start[k] + 2.0*u*t * p_ctrl[k] + t*t * p_target[k];

        /* origin-aware reach clamp */
        double r = sqrt(pt[0]*pt[0] + pt[1]*pt[1] + pt[2]*pt[2]);
        if (r > Max_Reach) {
            double s = Max_Reach / r;
            pt[0] *= s; pt[1] *= s; pt[2] *= s;
        }
        for (int k = 0; k < 3; ++k)
            path[k][i] = pt[k];
    }
}

void Solve_Trajectory(const double theta_init[4], const double path[3][50], double theta_traj[4][50], double *avg_iters) {
    double seed[4];
    memcpy(seed, theta_init, sizeof(seed));
    long total_iters_unused = 0;  /* (iteration count not surfaced here) */
    (void)total_iters_unused;

    double sum_iters = 0.0;

    for (int i = 0; i < Traj_Points; ++i) {
        double p_des[3] = { path[0][i], path[1][i], path[2][i] };
        double th_out[4];
        Invrs_Kin(p_des, seed, th_out);     /* 10° filter inside */
        for (int k = 0; k < 4; ++k) {
            theta_traj[k][i] = th_out[k];
            seed[k] = th_out[k];                   /* chain seed forward */
        }
        sum_iters += 1.0;                         /* per-point counter */
    }
    if (avg_iters) *avg_iters = sum_iters / (double)Traj_Points;
}

float Move_Trajectory(const double theta_init[4], const double path[3][50]) {
    double seed[4];
    memcpy(seed, theta_init, sizeof(seed));

    for (int i = 0; i < Traj_Points; ++i) {
        double p_des[3] = { path[0][i], path[1][i], path[2][i] };

        /* solve this micro-step within ±10° of the current pose */
        double th_out[4];
        float err = Invrs_Kin(p_des, seed, th_out);
        (void)err;

        /* command the arm to the solved pose — point to point */
        Move_Arm_Pose(th_out);

        /* dwell so the servos reach the pose before the next step */
        delay(STEP_DELAY_MS);

        /* chain the solved pose forward as the next seed */
        memcpy(seed, th_out, sizeof(seed));
    }
    return 1.0f;
}

float Run_Trajectory(const double p_target[3], int mode) {
    /* sanity: reject unreachable targets */
    double r = sqrt(p_target[0]*p_target[0] +
                    p_target[1]*p_target[1] +
                    p_target[2]*p_target[2]);
    if (r > Max_Reach) return -1.0f;     /* caller should re-prompt */

    /* seed from the arm's real current pose */
    double theta_now[4];
    Get_Current_Angles(theta_now);

    double p_start[3];
    double R_dummy[3][3];
    Pos_Fwrd_Kin(theta_now, p_start, R_dummy);

    /* generate the Bezier path */
    static double path[3][Traj_Points];
    Gen_Trajectory(p_start, p_target, mode, path);

    /* drive the arm through the path, point to point */
    Move_Trajectory(theta_now, path);
    return 1.0f;
}