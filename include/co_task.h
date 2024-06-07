#pragma  once 

#include "common_data.h"
#include "tcp_server.h"
#include "track/lqr_tracker.h"

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <glog/logging.h>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include "global.h"

  enum class CoTaskState{
    init,
    preworking,
    prework_finish,
    following,
    error_pause,
    finish
  };

class CoTask {
public:
  CoTask(const std::vector<std::pair<int, Pose>>& agvs, TcpServer* server, std::vector<Pose>* traj, Pose* vl){
    agvs_config = agvs;
    m_server = server;
    m_state = CoTaskState::init;
    m_traj = traj;
    m_vl = vl;
  }


  // 获得组件的id
  std::vector<int> getIds(){
    std::vector<int> ids;
    for(const auto it : agvs_config){
      ids.push_back(it.first);
    }
    return ids;
  }

  // 根据组件的位置，计算虚拟大车的坐标
  Pose CalcVirtualCenter(){
    std::unordered_map<int, AGVstatus> agv_real;
    Global::get_agv_state(agv_real);
    int size = agvs_config.size();
    Pose p;
    for(int i = 0; i < size; ++i) {
      int agv_id = agvs_config[i].first;
      // 车体坐标系下的位置
      const double cx = agvs_config[i].second.x; 
      const double cy = agvs_config[i].second.y; 
      
      // 世界坐标系下的位置
      double wx = agv_real.at(agv_id).pos.x;
      double wy = agv_real.at(agv_id).pos.y;
      double wt = agv_real.at(agv_id).pos.theta;

      // 世界坐标系下的偏移量
      double dx = cx * cos(wt) - cy * sin(wt);
      double dy = cx * sin(wt) + cy * cos(wt);
      
      p.x += (wx - dx) / size;
      p.y += (wy - dy) / size;
      p.theta += wt / size;
    }
    return p;
  }

  void StartPrework(Pose start);

  bool StartFollow();

  void Pause(bool pause);

private:
  void FollowThread();

  // 计算组件位置
  void CalcComponentPos(const Pose& center, std::vector<std::pair<int, Pose>>& agvs_res);

private:
  std::vector<std::pair<int, Pose>> agvs_config;
  CoTaskState m_state;
  std::vector<Pose>* m_traj;
  Pose* m_vl;
  TcpServer* m_server;
  motionplanner::LqrTracker tracker;

  std::thread m_follow_thread;
  std::atomic<bool> m_thread_run;
  std::atomic<bool> m_pause;

  std::mutex mtx;
  std::condition_variable cv; 
};