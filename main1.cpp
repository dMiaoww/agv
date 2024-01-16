#include "MainWindow.h"
#include "MsgStrctCC.h"
#include "co_task.h"
#include "common_data.h"
#include "curve.h"
#include "global.h"
#include "lqr_tracker.h"
#include "tcp_server.h"


#include <chrono>
#include <cmath>
#include <cstdlib>
#include <glog/logging.h>
#include <thread>
#include <utility>
#include <vector>

#define TEST 1

TcpServer server(3333);
// 设置任务起点
Pose start(0, 0, 0);
CoTask task_handler;

void setWorkMode() {
  // 把组件AGV发送到起点
  std::vector<std::pair<int, Pose>> agvs_res;
  task_handler.CalcComponentPos(start, agvs_res);

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
}

int main(int argc, char **argv) {
  GLog_set glog_set(argv[0]);
  server.start();

  // 10 号车的相对位置（2，2，0）， 6 号车的相对位置（-2，-2，0）
  // 设置哪些车组成虚拟大车
  std::vector<std::pair<int, Pose>> agvs;
  agvs.push_back(std::make_pair(10, Pose(0, 2, 0)));
  agvs.push_back(std::make_pair(6, Pose(0, -2, 0)));
  task_handler.Init(agvs);

  // 生成虚拟大车的路线
  std::vector<Pose> traj = BezierCurve::get(1000, Pose(0, 0, 0), Pose(5, 0, 0),
                                            Pose(10, 5, 0), Pose(10, 10, 0));

    // std::vector<Pose> traj = BezierCurve::get(1000, Pose(0, 0, 0), Pose(5, 0, 0),
    //                                         Pose(10, 0, 0), Pose(15, 0, 0));

  std::vector<double> traj_s;
  traj_s.resize(traj.size());  // 分配空间并且塞满了占位元素，所以不能push_back  
                                         // 如果是reserve，就是只分配内存，不创建对象
  double s = 0;
  for(int i = 1; i < traj.size(); i++){
    s += std::hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y);
    traj_s[i] = s;
  }


  // setWorkMode();  // 完成准备工作，就是先让小车到位
  Global::set_agv_status(10, AGVstatus(0, 2, 0, 0, 0));
  Global::set_agv_status(6, AGVstatus(0, -2, 0, 0, 0));

  // 绘图
  MainWindow main_window(&task_handler, &traj);


  // 然后给小车发命令
  motionplanner::LqrTracker tracker;
  int index = 0;
  double v = 0;
  double w = 0;
  while (true) {
    // 计算虚拟大车的速度
    Pose VirtualCenter = task_handler.CalcVirtualCenter();
    LOG(INFO) << VirtualCenter;
    auto state = tracker.Track(traj, 0.5, traj_s, 0, traj.size(), VirtualCenter, v, w, &v, &w, &index);
    // TODO：根据速度计算下一时刻会到达的地方


    // 发送给每个小车
    std::vector<std::pair<int, Pose>> agvs_res;
    task_handler.CalcComponentPos(traj[index], agvs_res);
    LOG(INFO) << index << " " << traj[index];

    for (const auto &it : agvs_res) {
      MSG_CC::FollowPoint job;
      job.m_jobId = 2024;
      job.m_agvId = it.first;
      job.m_x = it.second.x;
      job.m_y = it.second.y;
      job.m_theta = it.second.theta;
      // if(it.first == 10)
      //   job.m_v = std::hypot(v - w * 2, w*2);
      // else
      //   job.m_v = std::hypot(v + w * 2, w*2);
      // job.m_w = w;
      job.m_time = std::chrono::steady_clock::now().time_since_epoch().count();

      server.send(it.first, (char *)&job, sizeof(job));
      LOG(INFO) << "send agv: " << it.first << " pos: " << it.second
                << " v: " << job.m_v;
      Global::set_agv_job_state(it.first, JOBSTATE::following);

      // 小车的速度和角速度计算位置
      Global::update_state(it.first, it.second, 0.05);

      //Global::set_agv_status(it.first, AGVstatus(job.m_x+0.01, job.m_y, job.m_theta, job.m_v, job.m_w));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (state != motionplanner::LqrTracker::kTracking) {
      break;
    }
  }


  LOG(INFO) << "end";
  return 0;
}