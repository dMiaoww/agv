#pragma once
#include "common_data.h"
#include "steer.h"
#include "steer_origin.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class CarOmni4 {
  using SteerPtr = std::shared_ptr<Steer>;

public:
  CarOmni4(Pose r, double L = 0.3, double D = 0.3) {
    m_pos = r;
    m_real_vel = Pose(0, 0, 0);
    m_set_vel = Pose(0, 0, 0);
    m_L = L;
    m_D = D;
    for (int i = 0; i < 4; i++) {
      steers.push_back(std::make_shared<SteerOrigin>());
    }
    vector_D.push_back(m_D), vector_D.push_back(m_D), vector_D.push_back(-m_D),
        vector_D.push_back(-m_D);

    vector_L.push_back(m_L), vector_L.push_back(-m_L), vector_L.push_back(-m_L),
        vector_L.push_back(m_L);

    run_ = std::thread(std::bind(&CarOmni4::updateCAR, this));
  }

  // flag 真表示只设置舵轮角度
  void SetSpeed(const Pose &cmd, bool flag = false) {
    m_set_vel = cmd;
    m_flag = flag;
  }

  Pose getPose() { return m_pos; }

  std::string getState() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << m_pos << "\tvel:" << m_real_vel
       << "\n";
    for (int i = 0; i < steers.size(); i++) {
      ss << "a:" << steers[i]->real_angle << "\tvd:" << steers[i]->real_vd
         << "\n";
    }
    return ss.str();
  }

private:
  void inverse_transform(const Pose &cmd) {
    // double a1 = atan2(cmd.y + cmd.theta * m_L, cmd.x + cmd.theta * m_D);
    // double a2 = atan2(cmd.y - cmd.theta * m_L, cmd.x - cmd.theta * m_D);
    // double v1 = (cmd.x + cmd.theta * m_D) / cos(a1);
    // double v2 = (cmd.x - cmd.theta * m_D) / cos(a2);
    // s1->SetSpeed(a1, v1);
    // s2->SetSpeed(a2, v2);
    // LOG(INFO) << cmd;
    for (int i = 0; i < steers.size(); i++) {
      double a = atan2(cmd.y + cmd.theta * vector_L[i],
                       cmd.x + cmd.theta * vector_D[i]);
      double v = (cmd.x + cmd.theta * vector_D[i]) / cos(a);
      if (a > M_PI / 2) {
        a -= M_PI;
        v *= -1;
      } else if (a < -M_PI / 2) {
        a += M_PI;
        v *= -1;
      }
      if (m_flag)
        v = 0;
      steers[i]->SetSpeed(a, v);
      // LOG(INFO) << a << " " <<  v;
    }
    return;
  }

  Pose forward_transform() {
    Pose vel;
    // cmd.x = 0.5 * (s1->real_vd * cos(s1->real_angle) +
    //                s2->real_vd * cos(s2->real_angle));
    // cmd.y = 0.5 * (s1->real_vd * sin(s1->real_angle) +
    //                s2->real_vd * sin(s2->real_angle));
    // cmd.theta = (s1->real_vd * cos(s1->real_angle) -
    //              s2->real_vd * cos(s2->real_angle)) /
    //             (2 * m_D);
    for (int i = 0; i < steers.size(); i++) {
      double &v = steers[i]->real_vd;
      double &a = steers[i]->real_angle;
      vel.x += v * cos(a) / steers.size();
      vel.y += v * sin(a) / steers.size();
      vel.theta += v * cos(a) / vector_D[i] / steers.size();
    }
    return vel;
  }

  void updateCAR() {
    while (true) {
      // 发送速度给舵轮
      inverse_transform(m_set_vel);
      // 计算底盘速度
      m_real_vel = forward_transform();

      double dt = 0.03;
      std::random_device rd;  // 随机数生成器
      std::mt19937 gen(rd()); // 使用 Mersenne Twister 算法生成随机数
      std::uniform_real_distribution<> distrib(
          0.0, 1.0); // 定义均匀分布的取值范围，包括0.0，不包括1.0
      // 生成一个0到1（不包含1）的随机浮点数
      double randomNumber = 0.04 * distrib(gen) - 0.02;

      m_pos.x += m_real_vel.x * cos(m_pos.theta) * dt -
                 m_real_vel.y * sin(m_pos.theta) * dt;
      m_pos.y += m_real_vel.x * sin(m_pos.theta) * dt +
                 m_real_vel.y * cos(m_pos.theta) * dt;
      m_pos.theta += m_real_vel.theta * dt;
      // LOG(INFO) << m_real_vel;

      std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }
  }

private:
  Pose m_pos;
  Pose m_real_vel;
  Pose m_set_vel;
  std::vector<SteerPtr> steers;
  std::vector<double> vector_D;
  std::vector<double> vector_L;
  double m_D;
  double m_L;
  bool m_flag;

  std::thread run_;
};