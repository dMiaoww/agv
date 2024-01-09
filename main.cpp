// #include "MsgStrctCC.h"
// #include "co_task.h"
// #include "common_data.h"
// #include "global.h"
// #include "tcp_server.h"
// #include "common_data.h"
// #include <utility>



// int main(int argc, char **argv) {
//   TcpServer server(3333);
//   server.start();


//   Pose start(5,5,0);

//   // 5 号车的相对位置（2，2，0）， 6 号车的相对位置（-2，-2，0）
//   CoTask task;
//   std::vector<std::pair<int, Pose>> agvs;
//   agvs.push_back(std::make_pair(5, Pose(2,2,0)));
//   agvs.push_back(std::make_pair(6, Pose(-2,-2,0)));
//   task.Init(agvs);

//   while(true) {
//     std::unordered_map<int, AGVstatus> agv;
//     Global::get_agv_state(agv);
//     if(agv.size() == 2) break;
//   }  // 等待两个车都连上

//   std::vector<std::pair<int, Pose>> agvs_res;
//   task.CalcComponentPos(start, agvs_res);

//   // 先让小车到位
//   for(const auto &it : agvs_res) {
//     MSG_CC::MoveJob job;
//     job.m_jobId = 2024;
//     job.m_agvId = it.first;
//     job.m_head = MSG_CC::MoveJob::MoveToPoint;
//     job.m_x = it.second.x; job.m_y = it.second.y; job.m_theta = it.second.theta;

//     server.send(it.first, (char *) &job, sizeof(job));
//     LOG(INFO) << "send agv: " << it.first << "pos: " << it.second;
//     Global::set_agv_job_state(it.first, JOBSTATE::working);
//   }

//   // 等待小车的状态都变成完成
//   while(true) {
//     std::unordered_map<int, AGVstatus> agv;
//     Global::get_agv_state(agv);
//     bool finish = true;
//     for(auto it : agv) {
//       if(it.second.job_state != JOBSTATE::finish) finish = false;
//     }
//     if(finish) break;
//   }

//   // 然后给小车发命令

  

//   return 0;
// }