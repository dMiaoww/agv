#pragma once
#include "common_data.h"
#include "position_pid.h"
#include "steer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <glog/logging.h>
#include <iomanip>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

class SteerDDSU : public Steer {
public:
  SteerDDSU(std::string name) : Steer() {
    m_set_angle = 0;
    m_set_vd = 0;
    ptr_pid = std::make_shared<PositionPID>(25, 0, 0, update_time);
    run_ = std::thread(std::bind(&SteerDDSU::update, this));
    name_ = name;
  }
  SteerDDSU(double a, double v) : Steer(a, v) {
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&SteerDDSU::update, this));
  }

  virtual ~SteerDDSU(){};

  virtual void SetSpeed(double a, double v) {
    if (a > m_max_angle)
      a = m_max_angle;
    if (a < -m_max_angle)
      a = -m_max_angle;
    m_set_angle = a;
    m_set_vd = v;
  }

  virtual bool AngleReach() {
    if (fabs(m_set_angle - real_angle) < 0.01) {
      LOG(INFO) << m_set_angle << " " << real_angle;
      return true;
    } else
      return false;
  }

private:
  virtual void update() {
    std::ofstream file;
    file.open("DDSU" + name_);
    while (true) {
      // 先根据设定值计算两个轮子的速度，
      updateVel();
      // 在根据轮子速度更新角度
      updateTheta();

      std::this_thread::sleep_for(
          std::chrono::microseconds(int(1000*1000 * update_time)));
      file << m_set_angle << " " << m_set_vd << " " << real_angle << " "
           << real_vd << "\n";
    }
    file.close();
  }

  void updateVel() {
    double u = ptr_pid->getOutput(m_set_angle, real_angle);
    m_vl = m_set_vd - u * update_time;
    m_vr = m_set_vd + u * update_time;
  }

  void updateTheta() {
    real_vd = m_set_vd;
    real_angle += (m_vr - m_vl) / b * update_time;
  }

private:
  double m_set_vd;
  double m_set_angle;
  std::thread run_;
  std::string name_;

  // double m_max_angle_vel = 0.1 * 2 * M_PI; // 弧度每秒
  double m_max_angle = 140.0/180.0*M_PI;;
  double m_max_a = 1; // 加速度 1m / s
  double m_vl = 0;    // 左轮速度
  double m_vr = 0;    // 右轮速度
  double update_time = 0.005;  // 积分时间从0.01-0.001就会出现震荡
  double b = 0.125; // DDSU 中轮子到中点的距离

  std::shared_ptr<PositionPID> ptr_pid;
};