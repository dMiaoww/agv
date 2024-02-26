#pragma once
#include "common_data.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <sstream>
#include <string>
#include <thread>
#include <iomanip>
class Steer {
public:
  double real_angle;
  double real_vd;
  Steer() {
    real_angle = 0;
    real_vd = 0;
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&Steer::update, this));
  }
  Steer(double a, double v) : real_angle(a), real_vd(v) {
    m_set_angle = 0;
    m_set_vd = 0;
    run_ = std::thread(std::bind(&Steer::update, this));
  }

  void SetSpeed(double a, double v) {
    if (a > m_max_angle)
      a = m_max_angle;
    if (a < -m_max_angle)
      a = -m_max_angle;
    m_set_angle = a;
    m_set_vd = v;
  }

private:
  void update() {
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
  double m_max_angle_vel = 1 * 2 * M_PI; // 弧度每秒
  double m_max_angle = M_PI / 2;
  double m_max_a = 1; // 加速度 1m / s

  double m_set_angle;
  double m_set_vd;

  std::thread run_;
};

class SimOmni {

public:
  SimOmni(Pose r, double L = 0.4, double D = 0.2) {
    m_pos = r;
    m_real_vel = Pose(0, 0, 0);
    m_set_vel = Pose(0, 0, 0);
    m_L = L;
    m_D = D;

    run_ = std::thread(std::bind(&SimOmni::updateCAR, this));
  }
  void SetSpeed(const Pose &cmd) { m_set_vel = cmd; }

  Pose getPose() { return m_pos; }

  std::string getState() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << m_pos << "\tvel:" << m_real_vel
       << "\ta1: " << m_s_left.real_angle << "\ta2: " << m_s_right.real_angle
       << "\tv1: " << m_s_left.real_vd << "\tv2: " << m_s_right.real_vd;
    return ss.str();
  }

private:
  void inverse_transform(const Pose &cmd, Steer &s1, Steer &s2) {
    double a1 = atan2(cmd.y + cmd.theta * m_L, cmd.x + cmd.theta * m_D);
    double a2 = atan2(cmd.y - cmd.theta * m_L, cmd.x - cmd.theta * m_D);
    double v1 = (cmd.x + cmd.theta * m_D) / cos(a1);
    double v2 = (cmd.x - cmd.theta * m_D) / cos(a2);
    s1.SetSpeed(a1, v1);
    s2.SetSpeed(a2, v2);
    return;
  }

  Pose forward_transform(const Steer &s1, const Steer &s2) {
    Pose cmd;
    cmd.x = 0.5 *
            (s1.real_vd * cos(s1.real_angle) + s2.real_vd * cos(s2.real_angle));
    cmd.y = 0.5 *
            (s1.real_vd * sin(s1.real_angle) + s2.real_vd * sin(s2.real_angle));
    cmd.theta =
        (s1.real_vd * cos(s1.real_angle) - s2.real_vd * cos(s2.real_angle)) /
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
      LOG(INFO) << m_real_vel;

      std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt*1000)));
    }
  }

private:
  Pose m_pos;
  Pose m_real_vel;
  Pose m_set_vel;
  Steer m_s_left;
  Steer m_s_right;
  double m_D;
  double m_L;

  std::thread run_;
};