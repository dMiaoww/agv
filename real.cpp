#include "MainWindow.h"
#include "MsgStrctCC.h"
#include "agv_model.h"
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
#include <sys/wait.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#define TEST 0



int main(int argc, char **argv) {
  GLog_set glog_set(argv[0]);

  TcpServer server(3333);
  server.start();
  
  // 设置任务起点
  Pose start(0, -7, 0);
  
  // 设置哪些车组成虚拟大车
  std::vector<std::pair<int, Pose>> agvs = {{7, Pose(0, -0.5, 0)},
                                            {18, Pose(0, 0.5, 0)}};
  
  // 生成虚拟大车的路线
  std::vector<Pose> traj = BezierCurve::get(
      1000, Pose(0, -7, 0), Pose(2, -7, 0), Pose(4, -8, 0), Pose(6, -9, 0));

  // std::vector<Pose> traj = BezierCurve::get(1000, Pose(0, -5, 0), Pose(2, -5,
  // 0),
  //                                           Pose(4, -5, 0), Pose(6, -5, 0));
  Pose vl = start;
  CoTask task_handler(agvs, &server, &traj, &vl);

  server.setCb_interrupt([&task_handler](){
    task_handler.Pause(true);
  });

 
  // 绘图
  MainWindow main_window(&task_handler, &traj, &vl);
  main_window.setCb_followStartClick(
      [&task_handler]() { task_handler.StartFollow(); });
  
  
  while(Global::get_size() < 2);
  
  
  task_handler.StartPrework(start);


  while (true) {
  }

  LOG(INFO) << "end";
  return 0;
}