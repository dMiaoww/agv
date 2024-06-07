#pragma once
#include "common_data.h"
#include "steer.h"
#include "steer_origin.h"
#include "transform.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <glog/logging.h>
#include <iomanip>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

class CarSingleSteer {
  using SteerPtr = std::shared_ptr<Steer>;

public:
  CarSingleSteer(Pose r, double L = 1) {
    m_pos = r;
    m_real_vel = Pose(0, 0, 0);
    m_set_vel = Pose(0, 0, 0);
    m_L = L;
    m_D = 0;
    m_s_front = std::make_shared<SteerOrigin>("1");

    run_ = std::thread(std::bind(&CarSingleSteer::updateCAR, this));
  }
  // flag 真表示只设置舵轮角度
  void SetSpeed(const Pose &cmd, bool flag = false) {
    m_set_vel = cmd;
    m_flag = flag;
    if (flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(
          1000)); // 必须先等待一段时间，让更新线程将参数传递给电机

      while (true) {
        bool is_reach = m_s_front->AngleReach();
        if (is_reach)
          break;
        // LOG(INFO) << m_real_vel << " " << m_set_vel;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  Pose getPose() { return m_pos; }

  std::string getState() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << m_pos << "\tvel:" << m_real_vel
       << "\na1: " << m_s_front->real_angle << "\nv1: " << m_s_front->real_vd;
    return ss.str();
  }

  void getSteerPose(Pose &steerPose) {
    // 根据L和D和车体坐标计算舵轮位置
    Pose p;
    p.x = m_L;
    p.y = 0;
    p.theta = m_s_front->real_angle;
    steerPose = Transform2World(m_pos, p);
  }

private:
  void inverse_transform(const Pose &cmd, const SteerPtr s1) {
    double a1 = atan2(cmd.theta * m_L, cmd.x);
    double v1 = cmd.x / cos(a1);

    s1->SetSpeed(a1, v1);
    return;
  }

  Pose forward_transform(const SteerPtr s1) {
    Pose cmd;
    cmd.x = s1->real_vd * cos(s1->real_angle);
    cmd.y = 0;
    cmd.theta = s1->real_vd * sin(s1->real_angle) / m_L; // 这里是角速度

    return cmd;
  }

  void updateCAR() {
    while (true) {
      // 发送速度给舵轮
      // inverse_transform(m_set_vel, m_s_front);
      m_s_front->SetSpeed(m_set_vel.theta, m_set_vel.x);
      // 计算底盘速度
      // m_real_vel = forward_transform(m_s_front);

      double dt = 0.03;
      std::random_device rd;  // 随机数生成器
      std::mt19937 gen(rd()); // 使用 Mersenne Twister 算法生成随机数
      std::uniform_real_distribution<> distrib(
          0.0, 1.0); // 定义均匀分布的取值范围，包括0.0，不包括1.0
      // 生成一个0到1（不包含1）的随机浮点数
      double randomNumber = 0.04 * distrib(gen) - 0.02;

      // m_pos.x += m_real_vel.x * cos(m_pos.theta) * dt -
      //            m_real_vel.y * sin(m_pos.theta) * dt;
      // m_pos.y += m_real_vel.x * sin(m_pos.theta) * dt +
      //            m_real_vel.y * cos(m_pos.theta) * dt;
      // m_pos.theta += m_real_vel.theta * dt;


      m_pos.x += m_s_front->real_vd * cos(m_pos.theta) * dt ;
      m_pos.y += m_s_front->real_vd * sin(m_pos.theta) * dt ;
      m_pos.theta += m_s_front->real_vd / m_L * tan(m_s_front->real_angle) * dt ;
      
      // LOG(INFO) << m_real_vel;

      std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }
  }

private:
  Pose m_pos;
  Pose m_real_vel;
  Pose m_set_vel;
  SteerPtr m_s_front;
  double m_D;
  double m_L;
  bool m_flag;

  std::thread run_;
};