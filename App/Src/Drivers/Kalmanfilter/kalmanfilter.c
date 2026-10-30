/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilter.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Mar-2024 20:01:41
 */

/* Include Files */
#include "Drivers/Kalmanfilter/kalmanfilter.h"
#include "Drivers/Kalmanfilter/kalmanfilter_data.h"
#include "Drivers/Kalmanfilter/kalmanfilter_initialize.h"
#include <string.h>

/* Variable Definitions */
static double x_est[4];

static bool x_est_not_empty;

static double x_prd[4];

/* Function Definitions */
/*
 * %% define persistent variables
 *
 * Arguments    : double z
 *                double u
 *                double Q11
 *                double Q22
 *                double Q33
 *                double R
 *                double P11
 *                double margin
 *                double dt
 *                double input
 *                double input2
 *                double y[4]
 * Return Type  : void
 */
void kalmanfilter(double z, double u, double Q11, double Q22, double Q33,
                  double R, double P11, double margin, double dt, double input,
                  double input2, double y[4])
{
  static double p_est[16];
  static double p_prd[16];
  static const signed char b_a[4] = {1, 0, 0, 0};
  static const signed char iv[4] = {1, 0, 0, 0};
  double Q[16];
  double B[4];
  double A_tmp;
  double a;
  double b;
  double x_est_tmp;
  int b_A_tmp;
  int c_A_tmp;
  int i;
  int i1;
  if (!isInitialized_kalmanfilter) {
    kalmanfilter_initialize();
  }
  /*     %% initialize variables */
  if (!x_est_not_empty) {
    x_est_not_empty = true;
    /*  x_est=[x,y,Vx,Vy,Ax,Ay]' */
    memset(&p_est[0], 0, 16U * sizeof(double));
    p_est[0] = P11;
    p_est[5] = 0.0;
    p_est[10] = 0.0;
    p_est[15] = 0.0;
    memset(&p_prd[0], 0, 16U * sizeof(double));
    p_prd[0] = P11;
    p_prd[5] = 0.0;
    p_prd[10] = 0.0;
    p_prd[15] = 0.0;
  }
  /*     %% initialize constants */
  /*  Initialize measurement matrix */
  memset(&Q[0], 0, 16U * sizeof(double));
  Q[0] = Q11;
  Q[5] = Q22;
  Q[10] = Q33;
  Q[15] = 5.0E-6;
  /*  initialize process noise covariance */
  u -= 9.81;
  /*     %% check if timestep is too small (btw if accelerometer values are
   * invalid we're fucked) */
  if (dt > 0.0001) {
    double A[16];
    double b_A[16];
    /*  Initialize state transition matrix */
    A[0] = 1.0;
    A[4] = dt;
    A_tmp = dt * dt;
    A[8] = 0.5 * A_tmp;
    A[12] = -0.5 * A_tmp;
    A[1] = 0.0;
    A[5] = 1.0;
    A[9] = dt;
    A[13] = -dt;
    A[2] = 0.0;
    A[6] = 0.0;
    A[10] = input2;
    A[14] = 0.0;
    A[3] = 0.0;
    A[7] = 0.0;
    A[11] = 1.0;
    A[15] = 0.0;
    /*  [x]             */
    /*  [v]  */
    /*  [bias_a] */
    /*  Predicted state and covariance */
    B[0] = 0.0 * u;
    B[1] = 0.0 * u;
    B[2] = input * u;
    B[3] = 0.0 * u;
    for (i = 0; i < 4; i++) {
      x_est_tmp = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        b_A_tmp = i1 << 2;
        c_A_tmp = i + b_A_tmp;
        x_est_tmp += A[c_A_tmp] * x_est[i1];
        b_A[c_A_tmp] =
            ((A[i] * p_est[b_A_tmp] + A[i + 4] * p_est[b_A_tmp + 1]) +
             A[i + 8] * p_est[b_A_tmp + 2]) +
            A[i + 12] * p_est[b_A_tmp + 3];
      }
      x_prd[i] = x_est_tmp + B[i];
      x_est_tmp = b_A[i];
      A_tmp = b_A[i + 4];
      a = b_A[i + 8];
      b = b_A[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        b_A_tmp = i + (i1 << 2);
        p_prd[b_A_tmp] =
            (((x_est_tmp * A[i1] + A_tmp * A[i1 + 4]) + a * A[i1 + 8]) +
             b * A[i1 + 12]) +
            Q[b_A_tmp];
      }
    }
  }
  if ((z < x_est[0] + margin) && (z > x_est[0] - margin)) {
    double b_x_est_tmp;
    /*  Estimation */
    A_tmp = 0.0;
    /*  Estimated state and covariance */
    a = 0.0;
    for (i = 0; i < 4; i++) {
      x_est_tmp = ((p_prd[i] + 0.0 * p_prd[i + 4]) + 0.0 * p_prd[i + 8]) +
                  0.0 * p_prd[i + 12];
      B[i] = x_est_tmp;
      b_A_tmp = iv[i];
      A_tmp += x_est_tmp * (double)b_A_tmp;
      a += (double)b_A_tmp * x_prd[i];
    }
    A_tmp += R;
    b = z - a;
    a = B[0] / A_tmp;
    x_est[0] = x_prd[0] + a * b;
    x_est_tmp = B[1] / A_tmp;
    x_est[1] = x_prd[1] + x_est_tmp * b;
    b_x_est_tmp = B[2] / A_tmp;
    x_est[2] = x_prd[2] + b_x_est_tmp * b;
    A_tmp = B[3] / A_tmp;
    x_est[3] = x_prd[3] + A_tmp * b;
    for (i = 0; i < 4; i++) {
      b_A_tmp = b_a[i];
      c_A_tmp = i << 2;
      Q[c_A_tmp] = a * (double)b_A_tmp;
      Q[c_A_tmp + 1] = x_est_tmp * (double)b_A_tmp;
      Q[c_A_tmp + 2] = b_x_est_tmp * (double)b_A_tmp;
      Q[c_A_tmp + 3] = A_tmp * (double)b_A_tmp;
    }
    for (i = 0; i < 4; i++) {
      x_est_tmp = Q[i];
      A_tmp = Q[i + 4];
      a = Q[i + 8];
      b = Q[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        b_A_tmp = i1 << 2;
        c_A_tmp = i + b_A_tmp;
        p_est[c_A_tmp] =
            p_prd[c_A_tmp] -
            (((x_est_tmp * p_prd[b_A_tmp] + A_tmp * p_prd[b_A_tmp + 1]) +
              a * p_prd[b_A_tmp + 2]) +
             b * p_prd[b_A_tmp + 3]);
      }
    }
    /*  Compute the estimated measurements */
  } else {
    x_est[0] = x_prd[0];
    x_est[1] = x_prd[1];
    x_est[2] = x_prd[2];
    x_est[3] = x_prd[3];
    memcpy(&p_est[0], &p_prd[0], 16U * sizeof(double));
  }
  y[0] = x_est[0];
  y[1] = x_est[1];
  y[2] = x_est[2];
  y[3] = x_est[3];
}

/*
 * %% define persistent variables
 *
 * Arguments    : void
 * Return Type  : void
 */
void kalmanfilter_init(void)
{
  x_est_not_empty = false;
  x_est[0] = 0.0;
  x_prd[0] = 0.0;
  x_est[1] = 0.0;
  x_prd[1] = 0.0;
  x_est[2] = 0.0;
  x_prd[2] = 0.0;
  x_est[3] = 0.0;
  x_prd[3] = 0.0;
}

/*
 * File trailer for kalmanfilter.c
 *
 * [EOF]
 */
