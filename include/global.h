#pragma once

#include <cmath>
#include <glog/logging.h>
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
  AGVstatus(const Pose& p){
    pos.x = p.x, pos.y = p.y, pos.theta = p.theta;
  }
};

class Global {
public:
  static void set_agv_pos(int id, const Pose &pose) {
    std::unique_lock<std::mutex> lock(global_mutex);
    if(allagvs.find(id) != allagvs.end()){
      allagvs[id].pos = pose;
      return;
    }
    allagvs[id] = AGVstatus(pose);
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
    if (allagvs.find(id) != allagvs.end()) {
      return allagvs[id].pos;
    }
    else {
      return Pose(0,0,0);
    }
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

  static int get_size() {
    std::unique_lock<std::mutex> lock(global_mutex);
    return allagvs.size();
  }

private:
  static std::mutex global_mutex;
  static std::unordered_map<int, AGVstatus> allagvs;
};
