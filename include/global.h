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
  Pose pos;
  JOBSTATE job_state = JOBSTATE::free;

  AGVstatus() {}
  AGVstatus(float x, float y, float theta){
    pos.x = 0, pos.y = 0, pos.theta = 0;
  }
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
    return allagvs[id].pos;
  }

  // 从队列中删掉
  static void remove(const int id) {
    std::unique_lock<std::mutex> lock(global_mutex);
    if (allagvs.find(id) != allagvs.end()) {
      allagvs.erase(allagvs.find(id));
    }
  }

  // // 根据速度推算位置
  static void set_pose(const int id, const Pose& info) {
    std::unique_lock<std::mutex> lock(global_mutex);
    allagvs[id].pos = info;
  }

private:
  static std::mutex global_mutex;
  static std::unordered_map<int, AGVstatus> allagvs;
};
