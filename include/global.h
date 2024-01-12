#pragma once

#include <cmath>
#include <mutex>
#include <sys/stat.h>
#include <unordered_map>
#include <utility>

#include "common_data.h"
#include "lqr_w.h"

enum class JOBSTATE { free, send_task, pre_working, following, finish };

struct AGVstatus {
  float m_x = 0, m_y = 0, m_theta = 0, m_v = 0, m_w = 0;
  JOBSTATE job_state = JOBSTATE::free;

  AGVstatus() {}
  AGVstatus(float x, float y, float theta, float v, float w)
      : m_x(x), m_y(y), m_theta(theta), m_v(v), m_w(w) {}
};

class Global {
public:
  static void set_agv_status(int id, AGVstatus &&status) {
    std::unique_lock<std::mutex> lock(global_mutex);
    allagvs[id] = std::move(status);
  }

  static void set_agv_job_state(int id, JOBSTATE s) {
    std::unique_lock<std::mutex> lock(global_mutex);
    allagvs[id].job_state = s;
  }

  static void get_agv_state(std::unordered_map<int, AGVstatus> &agv) {
    std::unique_lock<std::mutex> lock(global_mutex);
    agv = allagvs;
  }

  static Pose get_agv_pose(const int id) {
    std::unique_lock<std::mutex> lock(global_mutex);
    return Pose(allagvs[id].m_x, allagvs[id].m_y, allagvs[id].m_theta);
  }

  // 从队列中删掉
  static void remove(const int id) {
    std::unique_lock<std::mutex> lock(global_mutex);
    if (allagvs.find(id) != allagvs.end()) {
      allagvs.erase(allagvs.find(id));
    }
  }

  // 根据速度推算位置
  static void update_state(const int id, const Pose info, const double v,
                           const double w, const double time) {
    // 应该还要加控制算法
    motionplanner::LqrW lqr_w;
    motionplanner::ControlStatus status;

    status.curr_pose =
        Pose(allagvs[id].m_x, allagvs[id].m_y, allagvs[id].m_theta);

    status.target_pose.x = info.x;
    status.target_pose.y = info.y;
    status.target_pose.theta = info.theta;
    status.target_vx = v;
    status.target_vyaw = w;

    status.curr_vx = v; // v 为上一时刻的控制速度
    status.curr_vyaw = w;

    double max_w = 0.6;
    double alpha = 1;
    double acc = 0.3;

    motionplanner::LqrW lqrw;
    double ww = lqrw.UpdateControl(status, 0.05);
    double vv = status.target_vx;

    AGVstatus &state = allagvs[id];
    state.m_x += vv * cos(state.m_theta) * time;
    state.m_y += vv * sin(state.m_theta) * time;
    state.m_theta += ww * time;
  }

private:
  static std::mutex global_mutex;
  static std::unordered_map<int, AGVstatus> allagvs;
};
