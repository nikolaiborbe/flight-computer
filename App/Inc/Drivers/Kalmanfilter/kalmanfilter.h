/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilter.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Mar-2024 20:01:41
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void kalmanfilter(
    double z, double u, double Q11, double Q22, double Q33, double R, double P11, double margin,
    double dt, double input, double input2, double y[4]
);

void kalmanfilter_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for kalmanfilter.h
 *
 * [EOF]
 */
