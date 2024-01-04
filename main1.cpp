#include "MsgStrctCC.h"
#include "co_task.h"
#include "common_data.h"
#include "global.h"
#include "tcp_server.h"
#include "common_data.h"
#include <chrono>
#include <thread>
#include <utility>



int main(int argc, char **argv) {
  TcpServer server(3333);
  server.start();


  Pose start(7,-10,0);

  // 5 号车的相对位置（2，2，0）， 6 号车的相对位置（-2，-2，0）
  CoTask task;
  std::vector<std::pair<int, Pose>> agvs;
  agvs.push_back(std::make_pair(10, Pose(2,2,0)));
  // agvs.push_back(std::make_pair(6, Pose(-2,-2,0)));
  task.Init(agvs);

  while(true) {
    std::unordered_map<int, AGVstatus> agv;
    Global::get_agv_state(agv);
    if(agv.size() == 1) break;
  }  // 等待两个车都连上

  std::vector<std::pair<int, Pose>> agvs_res;
  task.Update(start, agvs_res);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // 设置工作模式
    for(const auto &it : agvs_res) {
    MSG_CC::SetWorkModeCommand cmd;
    cmd.m_mode = SetWorkModeCommand::AutoMode;
    cmd.m_agvId = (int)it.first;
    server.send(it.first, (char *) &cmd, sizeof(cmd));
    LOG(INFO) << "send agv: " << it.first << " auto_mode";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // 先让小车到位
  for(const auto &it : agvs_res) {
    MSG_CC::MoveJob job;
    job.m_jobId = 2024;
    job.m_agvId = (int)it.first;
    job.m_jobtype = MSG_CC::MoveJob::MoveToPoint;
    job.m_x = 1000*it.second.x; job.m_y = 1000*it.second.y; job.m_theta = it.second.theta;

    server.send(it.first, (char *) &job, sizeof(job));
    LOG(INFO) << "send agv: " << it.first << " pos: " << it.second;
    Global::set_agv_job_state(it.first, JOBSTATE::working);
  }

  // 等待小车的状态都变成完成
  while(true) {
    std::unordered_map<int, AGVstatus> agv;
    Global::get_agv_state(agv);
    bool finish = true;
    for(auto it : agv) {
      if(it.second.job_state != JOBSTATE::finish) finish = false;
    }
    if(finish) break;
  }
  LOG(INFO) << "pre job finished";

  // 然后给小车发命令
  while(true) ;
  
  

  return 0;
}