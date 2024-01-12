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
#include <glog/logging.h>
#include <thread>
#include <utility>
#include <vector>

#define TEST 1

TcpServer server(3333);
// 设置任务起点
Pose start(0, 0, 0);
CoTask task_handler;



int main(int argc, char **argv) {
  GLog_set glog_set(argv[0]);
  server.start();

  // 10 号车的相对位置（2，2，0）， 6 号车的相对位置（-2，-2，0）
  // 设置哪些车组成虚拟大车
  std::vector<std::pair<int, Pose>> agvs;
  agvs.push_back(std::make_pair(10, Pose(2, 2, 0)));
  // agvs.push_back(std::make_pair(6, Pose(-2, -2, 0)));
  task_handler.Init(agvs);

  // 生成小车的路线
  std::vector<Pose> traj = BezierCurve::get(1000, Pose(0, 0, 0), Pose(5, 0, 0),
                                            Pose(10, 3, 0), Pose(15, 6, 0));

  std::vector<double> traj_s;
  traj_s.resize(traj.size());  // 分配空间并且塞满了占位元素，所以不能push_back  
                                         // 如果是reserve，就是只分配内存，不创建对象
  double s = 0;
  for(int i = 1; i < traj.size(); i++){
    s += std::hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y);
    traj_s[i] = s;
  }


  // setWorkMode();  // 完成准备工作，就是先让小车到位
  // Global::set_agv_status(10, AGVstatus(2, 2, 0, 0, 0));
  // Global::set_agv_status(6, AGVstatus(-2, -2, 0, 0, 0));

  // 绘图
  MainWindow main_window(&task_handler, &traj);


  // 然后给小车发命令
  motionplanner::LqrTracker tracker;
  int index = 0;
  double v = 0;
  double w = 0;
  while (true) {
    // 计算小车的速度和参考点
    Pose VirtualCenter = Global::get_agv_pose(10);
    LOG(INFO) << VirtualCenter;
    tracker.Track(traj, 0.5, traj_s, 0, traj.size(), VirtualCenter, v, w, &v, &w, &index);

    

      LOG(INFO) << "send agv: " << 10 << " pos: " << traj[index]
                << " v: " << v;
      Global::set_agv_job_state(10, JOBSTATE::following);

      // 小车的速度和角速度计算位置
      Global::update_state(10, traj[index], v, w, 0.05);

      // Global::set_agv_status(it.first, AGVstatus(job.m_x, job.m_y, job.m_theta, job.m_v, job.m_w));
    

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (v == 0) {
      break;
    }
  }

  while (true) {
  }

  return 0;
}