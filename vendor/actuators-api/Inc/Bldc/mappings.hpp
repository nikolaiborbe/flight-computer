#pragma once

struct SigmoidParameters {
  double a = 1.0;
  double b = 0.5;
};

/**
  * @brief Sigmoid curve!
  *
  * @param x Input value between x0 and x1.
  * @param x0 The lower bound of the input range.
  * @param x1 The upper bound of the input range.
  * @param y0 The output value corresponding to x0.
  * @param y1 The output value corresponding to x1.
  * @param parameters The steepness and midpoint of the curve.
  * @return const float
  */
double sigmoid_curve(double x, double x0, double x1, double y0, double y1, SigmoidParameters parameters);

/**
 * @brief Inverse of a sigmoid curve.
 */
double logit_curve(double y, double y0, double y1, double x0, double x1, SigmoidParameters parameters);
