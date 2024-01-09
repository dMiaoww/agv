#include "MsgStrctCC.h"
#include "co_task.h"
#include "common_data.h"
#include "global.h"
#include "tcp_server.h"
#include "tracker.h"

#include <chrono>
#include <thread>
#include <utility>
#include <vector>

int main(int argc, char **argv) {
  TcpServer server(3333);
  server.start();

  Pose start(7, -10, 0);

  // 10 号车的相对位置（2，2，0）， 6 号车的相对位置（-2，-2，0）
  CoTask task;
  std::vector<std::pair<int, Pose>> agvs;
  agvs.push_back(std::make_pair(10, Pose(2, 2, 0)));
  // agvs.push_back(std::make_pair(6, Pose(-2,-2,0)));
  task.Init(agvs);

  while (true) {
    std::unordered_map<int, AGVstatus> agv;
    Global::get_agv_state(agv);
    if (agv.size() == 1)
      break;
  } // 等待两个车都连上

  std::vector<std::pair<int, Pose>> agvs_res;
  task.CalcComponentPos(start, agvs_res);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // 设置工作模式
  for (const auto &it : agvs_res) {
    MSG_CC::SetWorkModeCommand cmd;
    cmd.m_mode = SetWorkModeCommand::AutoMode;
    cmd.m_agvId = (int)it.first;
    server.send(it.first, (char *)&cmd, sizeof(cmd));
    LOG(INFO) << "send agv: " << it.first << " auto_mode";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // 先让小车到位
  for (const auto &it : agvs_res) {
    MSG_CC::MoveJob job;
    job.m_jobId = 2024;
    job.m_agvId = (int)it.first;
    job.m_jobtype = MSG_CC::MoveJob::MoveToPoint;
    job.m_x = 1000 * it.second.x;
    job.m_y = 1000 * it.second.y;
    job.m_theta = it.second.theta;

    server.send(it.first, (char *)&job, sizeof(job));
    LOG(INFO) << "send agv: " << it.first << " pos: " << it.second;
    Global::set_agv_job_state(it.first, JOBSTATE::send_task);
  }

  // 等待小车的状态都变成完成
  while (true) {
    std::unordered_map<int, AGVstatus> agv;
    Global::get_agv_state(agv);
    bool finish = true;
    for (auto it : agv) {
      if (it.second.job_state != JOBSTATE::finish) 
        finish = false;
    }
    if (finish)
      break;
  }
  LOG(INFO) << "pre job finished";

  // 生成路线
  std::vector<Pose> traj;
  for (double x = start.x; x < (start.x + 2); x += 0.05) {
    traj.push_back(Pose(x, start.y, start.theta));
  }

  // 然后给小车发命令
  Tracker track;
  int index = 0;
  double v = 0;
  while (true) {
    // 计算虚拟大车的速度和参考点
    Pose VirtualCenter = task.CalcVirtualCenter();
    track.GetTrackParam(traj, VirtualCenter, index, v);
    
    // 发送给每个小车
    std::vector<std::pair<int, Pose>> agvs_res;
    task.CalcComponentPos(start, agvs_res);
    for (const auto &it : agvs_res) {
      MSG_CC::FollowPoint job;
      job.m_jobId = 2024;
      job.m_agvId = (int)it.first;
      job.m_x = it.second.x;
      job.m_y = it.second.y;
      job.m_theta = it.second.theta;
      job.m_v = v;
      job.m_w = 0;
      job.m_time = std::chrono::steady_clock::now().time_since_epoch().count();

      server.send(it.first, (char *)&job, sizeof(job));
      LOG(INFO) << "send agv: " << it.first << " pos: " << it.second;
      Global::set_agv_job_state(it.first, JOBSTATE::following);
    }

    if (v == 0) {
      break;
    }
  }

  return 0;
}