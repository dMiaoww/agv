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

class CarOmni {
  using SteerPtr = std::shared_ptr<Steer>;

public:
  CarOmni(Pose r, double L = 0.4, double D = 0.2) {
    m_pos = r;
    m_real_vel = Pose(0, 0, 0);
    m_set_vel = Pose(0, 0, 0);
    m_L = L;
    m_D = D;
    m_s_left = std::make_shared<SteerOrigin>();
    m_s_right = std::make_shared<SteerOrigin>();

    run_ = std::thread(std::bind(&CarOmni::updateCAR, this));
  }
  void SetSpeed(const Pose &cmd) { m_set_vel = cmd; }

  Pose getPose() { return m_pos; }

  std::string getState() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << m_pos << "\tvel:" << m_real_vel
       << "\na1: " << m_s_left->real_angle << "\ta2: " << m_s_right->real_angle
       << "\nv1: " << m_s_left->real_vd << "\tv2: " << m_s_right->real_vd;
    return ss.str();
  }

private:
  void inverse_transform(const Pose &cmd, const SteerPtr s1,
                         const SteerPtr s2) {
    double a1 = atan2(cmd.y + cmd.theta * m_L, cmd.x + cmd.theta * m_D);
    double a2 = atan2(cmd.y - cmd.theta * m_L, cmd.x - cmd.theta * m_D);
    double v1 = (cmd.x + cmd.theta * m_D) / cos(a1);
    double v2 = (cmd.x - cmd.theta * m_D) / cos(a2);
    s1->SetSpeed(a1, v1);
    s2->SetSpeed(a2, v2);
    return;
  }

  Pose forward_transform(const SteerPtr s1, const SteerPtr s2) {
    Pose cmd;
    cmd.x = 0.5 * (s1->real_vd * cos(s1->real_angle) +
                   s2->real_vd * cos(s2->real_angle));
    cmd.y = 0.5 * (s1->real_vd * sin(s1->real_angle) +
                   s2->real_vd * sin(s2->real_angle));
    cmd.theta = (s1->real_vd * cos(s1->real_angle) -
                 s2->real_vd * cos(s2->real_angle)) /
                (2 * m_D);
    return cmd;
  }

  void updateCAR() {
    while (true) {
      // 发送速度给舵轮
      inverse_transform(m_set_vel, m_s_left, m_s_right);
      // 计算底盘速度
      m_real_vel = forward_transform(m_s_left, m_s_right);

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
  SteerPtr m_s_left;
  SteerPtr m_s_right;
  double m_D;
  double m_L;

  std::thread run_;
};