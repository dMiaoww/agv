#pragma once
#include <cmath>
#include <glog/logging.h>
#include <vector>

#include "common_data.h"

class BezierCurve {
public:
  static std::vector<Pose> get(float T, Pose p0, Pose p1, Pose p2, Pose p3) {
    std::vector<Pose> vec;

    for (int i = 0; i <= T; ++i) {
      // Calculate the t parameter (from 0.0 -> 1.0)
      float t = (float)i / T;

      // Calculate blending functions
      float a = pow((1 - t), 3);
      float b = 3 * t * pow((1 - t), 2);
      float c = 3 * pow(t, 2) * (1 - t);
      float d = pow(t, 3);

      // Calculate the x,y and theta of the curve at the parametric point t
      Pose curvePoint;
      curvePoint.x = a * p0.x + b * p1.x + c * p2.x + d * p3.x;
      curvePoint.y = a * p0.y + b * p1.y + c * p2.y + d * p3.y;
      
      // Calculate derivative at the point
      float dx = -3 * pow((1 - t), 2) * p0.x + (3 * pow((1 - t), 2) - 6 * t * (1 - t)) * p1.x + (6 * t * (1 - t) - 3 * t * t) * p2.x + 3 * t * t * p3.x;
      float dy = -3 * pow((1 - t), 2) * p0.y + (3 * pow((1 - t), 2) - 6 * t * (1 - t)) * p1.y + (6 * t * (1 - t) - 3 * t * t) * p2.y + 3 * t * t * p3.y;

      // theta as the direction of the tangent
      curvePoint.theta = atan2(dy, dx);

      vec.push_back(curvePoint);
    }
    return vec;
  }
};