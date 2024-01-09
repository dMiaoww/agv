#pragma once

#include <mutex>
#include <sys/stat.h>
#include <unordered_map>
#include <utility>

enum class JOBSTATE {
  free, send_task, pre_working, following, finish
};

struct AGVstatus {
  float m_x = 0, m_y = 0, m_theta = 0, m_v = 0, m_w = 0;
  JOBSTATE job_state = JOBSTATE::free;
  
  AGVstatus(){}
  AGVstatus(float x, float y, float theta, float v, float w) : m_x(x), m_y(y), m_theta(theta), m_v(v), m_w(w) { }
};

class Global {
public:

  static void set_agv_status(int id, AGVstatus&& status) {
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

  // 从队列中删掉
  static void remove(const int id) {
    std::unique_lock<std::mutex> lock(global_mutex);
    if(allagvs.find(id) != allagvs.end()) {
      allagvs.erase(allagvs.find(id));
    }
  }

private:
  static std::mutex global_mutex;
  static std::unordered_map<int, AGVstatus> allagvs;
};
