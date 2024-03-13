#pragma once
#include "common_data.h"

// OA是A在世界坐标系，AB是B在A坐标系，返回B在世界坐标系
Pose inline Transform2World(const Pose &OA, const Pose &AB) {
  Pose out = AB;
  double sint = sin(OA.theta);
  double cost = cos(OA.theta);
  out.x = OA.x + AB.x * (cost) + AB.y * (-sint);
  out.y = OA.y + AB.x * (sint) + AB.y * (cost);
  out.theta += OA.theta;

  return out;
};