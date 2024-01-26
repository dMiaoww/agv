#include "co_task.h"
#include "MsgStrctCC.h"
#include "common_data.h"
#include "lqr_tracker.h"

#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

void CoTask::FollowThread() {
  // 先检查小车是否到位
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
  m_state = CoTaskState::prework_finish;

  // 等待 follow 开始
  std::unique_lock<std::mutex> ulock(mtx);
  cv.wait(ulock);

  LOG(INFO) << "start following";
  m_state = CoTaskState::following;
  std::vector<double> traj_s;
  traj_s.resize(
      m_traj->size()); // 分配空间并且塞满了占位元素，所以不能push_back
                       // 如果是reserve，就是只分配内存，不创建对象
  double s = 0;
  for (int i = 1; i < m_traj->size(); i++) {
    s += std::hypot((*m_traj)[i].x - (*m_traj)[i - 1].x,
                    (*m_traj)[i].y - (*m_traj)[i - 1].y);
    traj_s[i] = s;
  }
  tracker.Init();
  Pose vl = (*m_traj)[0];
  while (m_thread_run) {
    if (m_pause)
      continue;

    auto state = tracker.Track(*m_traj, 0.3, traj_s, vl);
    if (state == motionplanner::LqrTracker::kSuccessful) {
      // 发送任务完成消息
      m_state = CoTaskState::finish;
      FollowJobFinish msg;
      msg.m_jobId = 2024;
      for (auto it : agvs_config) {
        msg.m_agvId = it.first;
        m_server->send(it.first, (char *)&msg, sizeof(msg));
      }
      m_thread_run = false;
      return;
    }

    // 将下一时刻会到达的地方，发送给每个小车
    std::vector<std::pair<int, Pose>> agvs_res;
    CalcComponentPos(vl, agvs_res);
    for (const auto &it : agvs_res) {
      MSG_CC::FollowPoint job;
      job.m_jobId = 2024;
      job.m_agvId = it.first;
      job.m_x = it.second.x;
      job.m_y = it.second.y;
      job.m_theta = it.second.theta;
      m_server->send(it.first, (char *)&job, sizeof(job));
      LOG(INFO) << "send agv: " << it.first << " pos: " << it.second;
      Global::set_agv_job_state(it.first, JOBSTATE::running);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool CoTask::StartFollow() {
  // 必须在 prework_finish 状态才可以
  cv.notify_all();
  return true;
}

void CoTask::Pause(bool pause) {
  m_pause = pause;
  StopAgvCommand msg;
  if (m_pause) {
    msg.m_mode = MSG_CC::StopAgvCommand::StopMode;
  } else {
    msg.m_mode = MSG_CC::StopAgvCommand::RestartMode;
  }
  tracker.Init();
  msg.m_jobId = 2024;
  for (auto it : agvs_config) {
    msg.m_agvId = it.first;
    m_server->send(it.first, (char *)&msg, sizeof(msg));
  }
}

void CoTask::StartPrework(Pose start) {
  std::vector<std::pair<int, Pose>> agvs_res;
  CalcComponentPos(start, agvs_res);
  // 设置工作模式
  for (const auto &it : agvs_res) {
    MSG_CC::SetWorkModeCommand cmd;
    cmd.m_mode = SetWorkModeCommand::AutoMode;
    cmd.m_agvId = (int)it.first;
    m_server->send(it.first, (char *)&cmd, sizeof(cmd));
    LOG(INFO) << "send agv: " << it.first << " auto_mode";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  for (const auto &it : agvs_res) {
    MSG_CC::MoveJob job;
    job.m_jobId = 2024;
    job.m_agvId = (int)it.first;
    job.m_jobtype = MSG_CC::MoveJob::MoveToPoint;
    job.m_x = 1000 * it.second.x;
    job.m_y = 1000 * it.second.y;
    job.m_theta = it.second.theta;

    m_server->send(it.first, (char *)&job, sizeof(job));
    LOG(INFO) << "send agv: " << it.first << " pos: " << it.second;
    Global::set_agv_job_state(it.first, JOBSTATE::send_task);
  }

  m_state = CoTaskState::preworking;
  m_follow_thread = std::thread(std::bind(&CoTask::FollowThread, this));
}

void CoTask::CalcComponentPos(const Pose &center,
                              std::vector<std::pair<int, Pose>> &agvs_res) {
  agvs_res.clear();
  for (const auto &it : agvs_config) {
    Pose pose;

    double sint = sin(center.theta);
    double cost = cos(center.theta);
    pose.x = center.x + it.second.x * (cost) + it.second.y * (-sint);
    pose.y = center.y + it.second.x * (sint) + it.second.y * (cost);
    pose.theta = center.theta;

    agvs_res.push_back(std::make_pair(it.first, pose));
  }
}