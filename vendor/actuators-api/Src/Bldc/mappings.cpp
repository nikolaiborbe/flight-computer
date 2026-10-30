#include "mappings.hpp"
#include <algorithm>
#include <cmath>

double sigmoid_curve(double x, double x0, double x1, double y0, double y1, SigmoidParameters parameters) {
  // Sigmoid function parameters a and b control the shape of the curve
  double t = (x - x0) / (x1 - x0);  // Normalize x to [0, 1]
  return y0 + (y1 - y0) / (1.0f + std::exp(-parameters.a * (t - parameters.b)));
}


double logit_curve(double y, double y0, double y1, double x0, double x1, SigmoidParameters parameters) {
  double s = (y - y0) / (y1 - y0);
  s = std::clamp(s, 1e-9, 1.0 - 1e-9);
  double t = parameters.b - (1.0 / parameters.a) * std::log((1.0 / s) - 1.0);
  return x0 + t * (x1 - x0);
}
