#pragma once
#include "common_data.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <sstream>
#include <string>
#include <thread>

class Steer {
public:
  double real_angle;
  double real_vd;

  Steer() : real_angle(0), real_vd(0) {}
  Steer(double a, double v) : real_angle(a), real_vd(v) {}

  virtual ~Steer(){};

  virtual void SetSpeed(double a, double v) = 0;

  virtual bool AngleReach() = 0;

private:
  virtual void update() = 0;
};