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

class SteerDDSULag : public Steer {
public:
  SteerDDSULag(std::string name) : Steer() {
    m_run = true;
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&SteerDDSULag::update, this));
    name_ = name;
    file.open("DDSU" + name_);
  }
  SteerDDSULag(double a, double v) : Steer(a, v) {
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&SteerDDSULag::update, this));
  }

  virtual ~SteerDDSULag(){
    m_run = false;
    if(run_.joinable()) run_.join();
    file.close();
  };

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
    double dt = 0.01;
    while (m_run) {
      vec_v.push_back(m_set_vd);
      vec_angle.push_back(m_set_angle);
      std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000)));
      if(vec_v.size() > int(v_wait_time/dt)) {
        real_vd = vec_v[0];
        vec_v.erase(vec_v.begin());
      }
      if(vec_angle.size() > int(a_wait_time/dt)) {
        real_angle = vec_angle[0];
        vec_angle.erase(vec_angle.begin());
      }
      file << m_set_angle << " " << m_set_vd << " " << real_angle << " "
           << real_vd << "\n";
    }
  }



private:
  double m_set_vd;
  double m_set_angle;
  std::thread run_;
  std::string name_;

  // double m_max_angle_vel = 0.1 * 2 * M_PI; // 弧度每秒
  double m_max_angle = 140.0/180.0*M_PI;;
  double a_wait_time = 1.5; // 延迟时间
  double v_wait_time = 0.5; // 延迟时间

  std::vector<double> vec_v;
  std::vector<double> vec_angle;

  std::ofstream file;
  std::shared_ptr<PositionPID> ptr_pid;

  bool m_run;
};