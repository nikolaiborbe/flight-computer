/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanfilter_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Mar-2024 20:01:41
 */

/* Include Files */
#include "Drivers/Kalmanfilter/kalmanfilter_initialize.h"
#include "Drivers/Kalmanfilter/kalmanfilter.h"
#include "Drivers/Kalmanfilter/kalmanfilter_data.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanfilter_initialize(void)
{
  kalmanfilter_init();
  isInitialized_kalmanfilter = true;
}

/*
 * File trailer for kalmanfilter_initialize.c
 *
 * [EOF]
 */
