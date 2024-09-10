#include "track/pf3_tracker.h"

#include "glog_set.h"
#include "common_data.h"
// #include "move_base/move_base_v1.h"
// #include "move_base/controller/lqr_nsteer.h"
#include "track/controller/lqr_steer.h"
#include "track/controller/pure_pursuit.h"
#include "track/otg_filter.h"

#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include <sstream>
// #include "yaml-cpp/yaml.h"

namespace motionplanner
{

  Pf3Tracker::Pf3Tracker() : Tracker()
  {
    state_ = RobotState::free;
    m_begin = 0;
    m_otg.reset();
   
    
    pe = 0;
    pth_e = 0;
  }

  Pf3Tracker::~Pf3Tracker() = default;

  // MoveCmd Pf3Tracker::GetCmd2(const std::vector<Pose> &traj, const double v_ref,
  //                            const Pose &robot, const MoveCmd &last)
  // {
  //   MoveCmd cmd;
  //   ControlStatus status;

  //   status.curr_pose = robot;
  //   status.target_pose = traj.at(0);

  //   cmd.vx = v_ref;

  //   double max_w = max_w_;
  //   double alpha = alpha_;

  //   PurePursuit controller;
  //   // cmd.w = lqr_steer.UpdateControl(status, lqr_param_); // 注意这里算出来的是角度
  //   cmd.w = controller.UpdateControl(status, lqr_param_);


  //   cmd.vx = std::max({-max_v_, last.vx - acc_ * dt_, cmd.vx});
  //   cmd.vx = std::min({max_v_, last.vx + acc_ * dt_, cmd.vx});
  //   // cmd.vy = std::max({-max_v_, last.vy - acc_ * dt_, cmd.vy});
  //   // cmd.vy = std::min({max_v_, last.vy + acc_ * dt_, cmd.vy});
  //   // cmd.w = std::max({-max_w, last.w - alpha * dt_, cmd.w});
  //   // cmd.w = std::min({max_w, last.w + alpha * dt_, cmd.w});
  //   LOG(INFO) << "[pure_pursuit] vx:" << cmd.vx << " vy:" << cmd.vy << " angle:" << cmd.w;

  //   return cmd;
  // }

    MoveCmd Pf3Tracker::GetCmd(const std::vector<Pose> &traj, const double v_ref,
                             const Pose &robot, const MoveCmd &last)
  {
    MoveCmd cmd;
    ControlStatus status;
    status.k = calculateCurvature(traj, 0); // 曲率
    if(v_ref < 0) status.k *= -1;
    status.curr_pose = robot;
    status.target_pose = traj.at(0);
    status.target_vx = v_ref;
    // 参考角速度应该用参考轨迹变化量，与真实位置无关，避免了控制时朝向摆动的问题
    double dis = getDistance(traj.at(0), traj.at(1)) + 1e-6;
    status.target_vyaw =
        shortest_angular_distance(traj.at(0).theta, traj.at(1).theta) / (1e-6 + dis *
                                                                                    status.target_vx);

    status.curr_vx = last.vx; // v 为上一时刻的控制速度
    status.curr_vyaw = last.w;

    cmd.vx = status.target_vx;

    double max_w = max_w_;
    double alpha = alpha_;

    LqrSteer lqr_steer;
    // cmd.w = lqr_steer.UpdateControl(status, lqr_param_); // 注意这里算出来的是角度
    cmd.w = lqr_steer.UpdateControl2(status, lqr_param_, pe, pth_e);
    // LOG(INFO) << "[lqr] vx:" << cmd.vx << " vy:" << cmd.vy << " w:" << cmd.w;
    // if(cmd.w > 0 ) cmd.w *= 0.95;
    // else if(cmd.w < 0) cmd.w *= 1.025;

    cmd.vx = std::max({-max_v_, last.vx - acc_ * dt_, cmd.vx});
    cmd.vx = std::min({max_v_, last.vx + acc_ * dt_, cmd.vx});
    // cmd.vy = std::max({-max_v_, last.vy - acc_ * dt_, cmd.vy});
    // cmd.vy = std::min({max_v_, last.vy + acc_ * dt_, cmd.vy});
    // cmd.w = std::max({-max_w, last.w - alpha * dt_, cmd.w});
    // cmd.w = std::min({max_w, last.w + alpha * dt_, cmd.w});
    LOG(INFO) << "[lqr] vx:" << cmd.vx << " vy:" << cmd.vy << " angle:" << cmd.w;

    return cmd;
  }

  void Pf3Tracker::SetAlgoParam()
  {
    lqr_param_.max_iteration = 100;
    lqr_param_.t = dt_;
    lqr_param_.L = 1.004;

    // 测试使用，从文件中读取控制参数
    std::ifstream infile("pf3_param.txt");
    std::string line, token;
    getline(infile, line);
    
    std::stringstream ssline(line);
    getline(ssline, token, ' ');
    lqr_param_.w_x = 1;
    getline(ssline, token, ' ');
    lqr_param_.w_y = 1;
    getline(ssline, token, ' ');
    lqr_param_.w_yaw = 0.5;
    getline(ssline, token, ' ');
    lqr_param_.w_vyaw = 0.1;


    LOG(INFO) << "lqr param(t, w_x, w_y, w_yaw, vx, vyaw): " << lqr_param_.t
              << " " << lqr_param_.w_x << " " << lqr_param_.w_x << " "
              << lqr_param_.w_yaw << " " << lqr_param_.w_vx << " "
              << lqr_param_.w_vyaw;
    Init();
  }

  void Pf3Tracker::SetMotionParam(double frequency, int p, double max_v,
                             double min_v, double max_w, double min_w,
                             double acc, double stop_acc, double alpha, double pos_accuracy,
                             double theta_accuracy) {
  dt_ = 1.0 / frequency;
  p_ = p;
  max_v_ = 1.5;
  min_v_ = 0;
  max_w_ = max_w;
  min_w_ = min_w;
  acc_ = acc;
  stop_acc_ = stop_acc;
  alpha_ = alpha;
  pos_accuracy_ = pos_accuracy;
  theta_accuracy_ = theta_accuracy;

  LOG(INFO) << "track param, dt:" << dt_ << ", p:" << p_;
  LOG(INFO) << "track param, max_v:" << max_v_ << ", min_v:" << min_v_;
  LOG(INFO) << "track param, max_w:" << max_w_ << ", min_w:" << min_w_;
  LOG(INFO) << "track param, acc:" << acc_ << ", alpha:" << alpha_;
}

  Tracker::State Pf3Tracker::Track(const std::vector<Pose> &traj,
                                   const double v_max,
                                   const std::vector<double> &traj_s,
                                   size_t begin_i, size_t end_i, Pose &robot,
                                   const MoveCmd &last_cmd, MoveCmd &now_cmd,
                                   size_t *n_idx, bool backward)
  {
    now_cmd = MoveCmd(0, 0, 0);
    if (*n_idx == 0 || *n_idx >= end_i)
      Init();

    // robot at start point
    if (state_ == RobotState::free)
    {
      // 往前遍历，找到最近的点
      m_begin = getNearestId(traj, robot, m_begin, end_i);
      double theta =
          DiffRad(robot.theta, traj.at(m_begin).theta);
      double dis = getDistance(robot, traj.at(m_begin));
      if (dis > 0.5 )
      {
        LOG(ERROR) << "wrong start point, dis: " << dis << " delta_theta: " << theta  
                   << " robot: " << robot.x << " " << robot.y << " " << robot.theta
                   << " begin: " << begin_i << " traj[" << m_begin
                   << "]:" << traj.at(m_begin).x << " " << traj.at(m_begin).y << " " << traj.at(m_begin).theta;
        Init();
        now_cmd = MoveCmd(0, 0, 0);
        *n_idx = begin_i;
        return Tracker::State::kFailed;
      }

      // 距离远的时候固定找10厘米之外的点
      if (getDistance(robot, traj.at(end_i - 1)) >= 0.1)
      {
        m_begin = getNextId(traj, robot, m_begin, end_i, 0.1);
      }
      if (m_begin == 0)
        m_begin++; // 短路径会进入这个分支
      if (m_begin >= end_i)
        return State::kFailed;
      LOG(INFO) << "m_begin: " << m_begin << " end_i(traj size): " << end_i
                << " begin_i: " << begin_i;

      state_ = RobotState::move;
      LOG(INFO) << "robot change state free to move";
    }

    if (state_ == RobotState::move)
    {
      // for(const auto& it : vec_cmd) {
      //   robot = predict(robot, it.vx, it.w);
      // }
      

      int min_id = getNearestId(traj, robot, m_begin, end_i);
      double min_dis = getMinDistanceToLine(robot, traj.at(min_id));
      LOG(INFO) << "m_begin: " << m_begin << " min_id: " << min_id
                << " min_dis: " << min_dis;

      // robot at stop point
      double tt = fabs(DiffRad(atan2(traj.at(end_i - 1).y - robot.y,
                             traj.at(end_i - 1).x - robot.x), robot.theta));
      if ((min_id + 5) > end_i && ((!backward && tt > M_PI_2) ||(backward && tt < M_PI_2)))
      {
        Init();
        *n_idx = end_i - 1;
        LOG(INFO) << "0.The robot has reached the target point.";
        return Tracker::State::kSuccessful;
      }
      else
      {
        // 1、小车脱轨
        if (fabs(min_dis) > 0.3)
        {
          LOG(ERROR) << "robot far away from traj, min_traj(x,y,theta): "
                     << traj.at(min_id).x << " " << traj.at(min_id).y << " "
                     << traj.at(min_id).theta << " agv(x,y,theta): " << robot.x
                     << " " << robot.y << " " << robot.theta;
          now_cmd = MoveCmd(0, 0, 0);
          *n_idx = min_id;
          Init();
          return Tracker::kFailed;
        }

        // 3、正常
        double cmax = fabs(calculateCurvature(traj, min_id));
        double ratio = 0.1;
        double v_curv = ratio * pow(cmax, -0.8); // 曲率 -> 速度
        for (int i = min_id + 1; i < traj_s.size(); i++)
        {
          double c2 = fabs(calculateCurvature(traj, i));
          cmax = std::max(cmax, c2);
          double v = ratio * pow(c2, -0.8);
          double s = traj_s[i] - traj_s[min_id];
          if (v < last_cmd.vx)
          {
            double dec_s = 0.5 * (last_cmd.vx * last_cmd.vx - v * v) / acc_ +
                           1; // 计算减速距离
            if (dec_s > s)
            {
              // 减速距离大于实际距离，需要减速
              v_curv = std::min(v_curv, v);
            }
          }
          if (s > 0.5 * last_cmd.vx * last_cmd.vx / acc_ + 0.5)
            break;
        }


        double dis_to_end = traj_s[traj_s.size() - 1] - traj_s[min_id];
          m_otg_lim.vMax = std::min({v_max, v_curv});
          OtgFilter::VelParam target = {dis_to_end, 0, 0, 0};
          m_otg.qk.d = 0;
          // m_otg.qk.v = last_cmd.vx;
          m_otg.runCycleS1(3000 * dt_, m_otg_lim, target);
          double v_ref = backward ? -m_otg.qk.v : m_otg.qk.v;
          LOG(INFO) << "dis_to_end: " << dis_to_end << " lim_v: " << m_otg_lim.vMax
                    << " v: " << m_otg.qk.v
                    << " a: " << (v_ref - last_cmd.vx) / dt_;

          LOG(INFO) << "backward:" << backward << " ref: " << v_ref;


        // double v_curv = 0.15 * pow(cmax, -0.7);
        int next_index;
        LOG(INFO) << "v_cuve: " << v_curv;
        if(true || fabs(v_ref) > 0.3 || dis_to_end < 0.5) {
          // if(true) {
          next_index = min_id;
          if (next_index < *n_idx)
            next_index = *n_idx;

          std::vector<Pose> traj_ref;
          if (next_index + p_ < end_i)
          {
            traj_ref.assign(std::next(traj.begin(), next_index),
                            std::next(traj.begin(), next_index + p_));
          }
          else
          {
            traj_ref.assign(std::next(traj.begin(), next_index),
                            std::next(traj.begin(), end_i - 1));
            traj_ref.insert(traj_ref.end(), next_index + p_ - end_i + 1,
                            traj[end_i - 2]);
          }
        
          now_cmd = GetCmd(traj_ref, v_ref, robot, last_cmd);
        } else if(true) {
          // pp
          double pre = 0.5 + fabs(v_ref);
          if(pre > dis_to_end) pre = dis_to_end;
          lqr_param_.w_vx = pre;
          next_index = getNextId(traj, robot, min_id, end_i, pre);
          ControlStatus status;

          status.curr_pose = robot;
          status.target_pose = traj.at(next_index);

          now_cmd.vx = v_ref;

          PurePursuit controller;
          // cmd.w = lqr_steer.UpdateControl(status, lqr_param_); // 注意这里算出来的是角度
          now_cmd.w = controller.UpdateControl(status, lqr_param_);


          now_cmd.vx = std::max({-max_v_, last_cmd.vx - acc_ * dt_, now_cmd.vx});
          now_cmd.vx = std::min({max_v_, last_cmd.vx + acc_ * dt_, now_cmd.vx});
          LOG(INFO) << "[pure_pursuit] vx:" << now_cmd.vx << " vy:" << now_cmd.vy << " angle:" << now_cmd.w;
          
        } else {
          // stanly
          // 1. calculate front wheel position
          Pose front;
          front.theta = robot.theta;
          front.x = robot.x + 1.004 * cos(front.theta);
          front.y = robot.y + 1.004 * sin(front.theta);
          next_index = getNextId(traj, robot, min_id, end_i, 0);
          double dx = front.x - traj[next_index].x;
          double dy = front.y - traj[next_index].y;
          // double e = dx*cos(front.theta+M_PI_2) + dy*sin(front.theta+M_PI_2);

          double e = -getMinDistanceToLine(front, traj[next_index]);
          
          double k = 0.5;
          double theta_e = NormalizeRad(traj[next_index].theta-front.theta);
          double theta_d = atan2(k*e, v_ref);

          now_cmd.vx = v_ref;
          now_cmd.w = theta_e + theta_d;
          LOG(INFO) << "[stanly] vx:" << now_cmd.vx << " angle:" << now_cmd.w << " theta_e:" << theta_e << " e:" << e;

        }

        *n_idx = min_id;
        m_begin = min_id;



        LOG(INFO) << "next_i:" << next_index
                  << ", traj(x_y_theta):" << traj.at(next_index).x << " "
                  << traj.at(next_index).y << " " << traj.at(next_index).theta
                  << " robot(x_y_theta)" << robot.x << " " << robot.y << " "
                  << robot.theta << " end_i:" << end_i << " v_w:" << now_cmd.vx
                  << " " << now_cmd.w;
        return Tracker::State::kTracking;
      }
    }
    return Tracker::State::kTracking;
  }


  Pose Pf3Tracker::predict(const Pose& robot, double v, double angle) {
    double vx = v*cos(angle);
    double w = v*sin(angle)/lqr_param_.L;
    Pose res;
    res.x = robot.x + v*cos(angle)*dt_;
    res.y = robot.y + v*sin(angle)*dt_;
    res.theta = robot.theta + w*dt_;
    return res;
  }

} // namespace motionplanner