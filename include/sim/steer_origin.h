#pragma once
#include "common_data.h"
#include "steer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <sstream>
#include <string>
#include <thread>

class SteerOrigin : public Steer{
public:
  SteerOrigin() : Steer(){
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&SteerOrigin::update, this));
  }
  SteerOrigin(double a, double v) : Steer(a, v) {
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&SteerOrigin::update, this));
  }

  virtual ~SteerOrigin(){};

  virtual void SetSpeed(double a, double v) {
    if (a > m_max_angle)
      a = m_max_angle;
    if (a < -m_max_angle)
      a = -m_max_angle;
    m_set_angle = a;
    m_set_vd = v;
  }

private:
  virtual void update() {
    while (true) {
      if (m_set_angle > real_angle)
        real_angle =
            std::min(m_set_angle, real_angle + m_max_angle_vel / 100.0);
      else
        real_angle =
            std::max(m_set_angle, real_angle - m_max_angle_vel / 100.0);
      if (m_set_vd > real_vd)
        real_vd = std::min(m_set_vd, real_vd + m_max_a / 100.0);
      else
        real_vd = std::max(m_set_vd, real_vd - m_max_a / 100.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  double m_set_vd;
  double m_set_angle;
  std::thread run_;

  double m_max_angle_vel = 0.1 * 2 * M_PI; // 弧度每秒
  double m_max_angle = M_PI / 2;
  double m_max_a = 1; // 加速度 1m / s
};