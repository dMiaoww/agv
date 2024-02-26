#include "move_base/common.h"

namespace motionplanner {

double DiffRad(double a, double b) {
  double diff = std::fmod(b - a + M_PI, 2 * M_PI);
  if (diff < 0) return diff + M_PI;
  return diff - M_PI;
}

double AbsDiffRad(double a, double b) {
  double diff = std::fmod(b - a + M_PI, 2 * M_PI);
  if (diff < 0) return std::abs(diff + M_PI);
  return std::abs(diff - M_PI);
}

double NormalizeRad(double rad) {
  double diff = std::fmod(rad + M_PI, 2 * M_PI);
  if (diff < 0) diff += 2 * M_PI;
  return diff - M_PI;
}

double shortest_angular_distance(double from, double to) {
  double a = normalize_angle_positive(to - from);
  if (a > M_PI) a -= 2.0 * M_PI;
  return a;
}

double normalize_angle_positive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

}  // namespace motionplanner