// 全向移动的车型,使用舵轮实现

#include "move_base/omni_steer_tracker.h"
#include "move_base/common.h"
#include "move_base/controller/lqr_nsteer.h"
#include "move_base/otg_filter.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <glog/logging.h>
#include <math.h>

namespace motionplanner {

OmniSteerTracker::OmniSteerTracker() : Tracker() {
  state_ = RobotState::free;
  m_begin = 0;
  m_otg.reset();
  // agv_type_ = global_agv_configure_.agv_type_configure.agv_type;
}

OmniSteerTracker::~OmniSteerTracker() = default;

// 距离的绝对值
double OmniSteerTracker::getDistance(const Pose &p1, const Pose &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/** robot  p2     p3:    return < 0
    p2     robot  p3:    return 0~1
    p2     p3     robot: return > 1
**/
double OmniSteerTracker::getDot(const Pose &robot, const Pose &p2,
                                const Pose &p3) {
  Pose p21{robot.x - p2.x, robot.y - p2.y, 0};
  Pose p23{p3.x - p2.x, p3.y - p2.y, 0};

  double dot =
      (p21.x * p23.x + p21.y * p23.y) / (p23.x * p23.x + p23.y * p23.y + 1e-9);
  return dot;
}

// 找到轨迹上最靠近小车、且小车并未到达的点
int OmniSteerTracker::getNearestId(const std::vector<Pose> &traj,
                                   const Pose &robot, int begin_i, int end_i) {
  double min_dist = 10000;
  int min_id = begin_i;
  for (int i = begin_i; i < end_i; i++) {
    double dd = getDistance(robot, traj.at(i));
    if (dd <= min_dist + 1e-3) {
      min_dist = dd;
      min_id = i;
    }
  }
  // robot已经越过最近点且最近点不是终点
  if ((min_id + 1) < end_i &&
      getDot(robot, traj.at(min_id), traj.at(min_id + 1)) > 0) {
    min_id++;
  }
  return min_id;
}

// 计算 point 到 line 的距离 左正右负
double OmniSteerTracker::getMinDistanceToLine(const Pose &point,
                                              const Pose &line) {
  double x1 = line.x;
  double y1 = line.y;
  double x2 = line.x + cos(line.theta);
  double y2 = line.y + sin(line.theta);

  return (y1 - y2) * point.x - (x1 - x2) * point.y + (x1 * y2 - x2 * y1);
}

double OmniSteerTracker::calculateCurvature(const std::vector<Pose> &traj,
                                            const int &i) {
  if ((i + 3) > traj.size())
    return 0;

  double curvature = 0.0;

  motionplanner::Point A(traj.at(i).x, traj.at(i).y);
  motionplanner::Point B(traj.at(i + 1).x, traj.at(i + 1).y);
  motionplanner::Point C(traj.at(i + 2).x, traj.at(i + 2).y);
  motionplanner::Point D = {(B.x + C.x) / 2, (B.y + C.y) / 2}; // BC的中点
  motionplanner::Point E = {(A.x + B.x) / 2, (A.y + B.y) / 2}; // AB的中点

  double slope1 = (C.y - B.y) / (C.x - B.x); // BC的斜率
  double slope2 = (B.y - A.y) / (B.x - A.x); // AB的斜率

  double slope_per1 = -1 / slope1; // BC的垂直线斜率
  double slope_per2 = -1 / slope2; // AB的垂直线斜率

  double x = (slope_per1 * D.x - slope_per2 * E.x + E.y - D.y) /
             (slope_per1 - slope_per2);    // 圆心的x坐标
  double y = slope_per1 * (x - D.x) + D.y; // 圆心的y坐标

  Point O = {x, y}; // 圆心

  double radius = std::hypot(O.x - B.x, O.y - B.y); // 半径

  curvature = 1 / radius; // 曲率

  return curvature;
}

// [begin_i, end_i) 区间上离小车距离最接近 preview_dist 的点
int OmniSteerTracker::getNextId(const std::vector<Pose> &traj,
                                const Pose &robot, int begin_i, int end_i,
                                double preview_dist) {
  int next_id = begin_i;
  double dist = 10000;

  for (int i = begin_i; i < end_i; i++) {
    double now_dist = fabs(getDistance(robot, traj.at(i)) - preview_dist);
    if (now_dist <= dist + 1e-3) {
      next_id = i;
      dist = now_dist;
    } else {
      break;
    }
  }
  return next_id; // next_id is always valid
}

int OmniSteerTracker::sgn(double x) { return (x >= 0) ? 1 : -1; }

Pose OmniSteerTracker::predictPose(const Pose &robot, const MoveCmd &cmd,
                                   double dt) {
  Pose res;
  double th = robot.theta;
  res.x = robot.x + dt * cmd.vx * cos(th) - dt * cmd.vy * sin(th);
  res.y = robot.y + dt * cmd.vx * sin(th) + dt * cmd.vy * cos(th);
  res.theta = th + dt * cmd.w;
  return res;
}

MoveCmd OmniSteerTracker::getRotateCmd(double curr_theta, double target_theta,
                                       double w1) {
  MoveCmd cmd;

  cmd.w = 1 * shortest_angular_distance(curr_theta, target_theta);

  cmd.w = std::max({-max_w_, w1 - alpha_ * dt_, cmd.w});
  cmd.w = std::min({max_w_, w1 + alpha_ * dt_, cmd.w});

  // 防止终点处的速度太慢，限制最小角速度
  if (fabs(cmd.w) < min_w_) {
    cmd.w = sgn(cmd.w) * min_w_;
  }

  LOG(INFO) << "cmdrotate(w): " << cmd.w << " robot_pose " << curr_theta
            << " target_theta " << target_theta;

  return cmd;
}

MoveCmd OmniSteerTracker::GetCmd(const std::vector<Pose> &traj,
                                 const double v_max, const Pose &robot,
                                 const MoveCmd &last) {
  MoveCmd cmd;
  ControlStatus status;

  status.curr_pose = robot;
  status.target_pose = traj[0];
  status.target_vx = v_max;
  status.target_vy = 0;
  // 参考角速度应该用参考轨迹变化量，与真实位置无关，避免了控制时朝向摆动的问题
  double dis = getDistance(traj[0], traj[1]) + 1e-6;
  status.target_vyaw = shortest_angular_distance(traj[0].theta, traj[1].theta) /
                       dis * v_max;

  status.curr_vx = last.vx; // v 为上一时刻的控制速度
  status.curr_vy = last.vy;
  status.curr_vyaw = last.w;

  double max_w = max_w_;
  double alpha = alpha_;

  LqrNSteer lqr_nsteer;
  cmd = lqr_nsteer.UpdateControl(status, lqr_param_);

  cmd.vx = std::max({0.0, last.vx - acc_ * dt_, cmd.vx});
  cmd.vx = std::min({max_v_, last.vx + acc_ * dt_, cmd.vx});
  cmd.vy = std::max({0.0, last.vy - acc_ * dt_, cmd.vy});
  cmd.vy = std::min({max_v_, last.vy + acc_ * dt_, cmd.vy});
  cmd.w = std::max({-max_w, last.w - alpha * dt_, cmd.w});
  cmd.w = std::min({max_w, last.w + alpha * dt_, cmd.w});

  LOG(INFO) << "[lqr] vx:" << cmd.vx << " vy:" << cmd.vy << " w:" << cmd.w;
  return cmd;
}

void OmniSteerTracker::SetAlgoParam() {
  lqr_param_.t = dt_;
  Init();
}

// 【begin_i, end_i), 左毕右开
Tracker::State OmniSteerTracker::Track(const std::vector<Pose> &traj,
                                       const double v_max,
                                       const std::vector<double> &traj_s,
                                       size_t begin_i, size_t end_i,
                                       Pose &robot, const MoveCmd &last_cmd,
                                       MoveCmd &now_cmd, size_t *n_idx) {
  // 新路径
  if (*n_idx == 0 || *n_idx >= end_i)
    Init();

  // robot at start point
  if (state_ == RobotState::free) {
    // 起点距离当前位置太远
    m_begin = getNearestId(traj, robot, begin_i, end_i);
    if (getDistance(robot, traj.at(m_begin)) > 0.2) {
      LOG(ERROR) << "wrong start point, "
                 << "robot: " << robot.x << " " << robot.y
                 << " begin: " << begin_i << " traj[" << m_begin
                 << "]:" << traj.at(m_begin).x << " " << traj.at(m_begin).y;
      Init();
      *n_idx = begin_i;
      return Tracker::State::kFailed;
    }

    // 距离远的时候固定找10厘米之外的点
    m_begin = getNextId(traj, robot, m_begin, end_i - 1, 0.1);

    // if (m_begin == 0)
    //   m_begin++; // 长度小于10厘米的短路径会进入这个分支
    // if (m_begin >= end_i)
    //   return State::kFailed;
    LOG(INFO) << "m_begin: " << m_begin << " end_i(traj size): " << end_i
              << " begin_i: " << begin_i;

    if (shortest_angular_distance(robot.theta, traj[0].theta) > 0.1) {
      state_ = RobotState::rotate1;
    }else{
      state_ = RobotState::move;
    }
  }

  if (state_ == RobotState::rotate1) {
    if (getDistance(robot, traj.at(end_i - 1)) <= 0.05)
      m_begin = end_i - 1;
    double theta =
        shortest_angular_distance(robot.theta, traj.at(m_begin).theta);
    // 旋转完成
    if (fabs(theta) < theta_accuracy_) {
      if (m_begin != (end_i - 1)) {
        state_ = RobotState::move;
        m_otg.reset();
        LOG(INFO) << "robot change state rotate1 to move";
      } else {
        Init();
        *n_idx = end_i - 1;
        LOG(INFO) << "0.The robot has reached the target point";
        return Tracker::State::kSuccessful;
      }
    }
    // 旋转中
    else {
      now_cmd = getRotateCmd(robot.theta, traj.at(m_begin).theta, last_cmd.w);
      *n_idx = m_begin;
      return Tracker::State::kTracking;
    }
  }

  if (state_ == RobotState::move) {
    robot = predictPose(robot, cmd2, dt_);
    robot = predictPose(robot, cmd1, dt_);
    robot = predictPose(robot, last_cmd, dt_);

    int min_id = getNearestId(traj, robot, m_begin, end_i);
    LOG(INFO) << "m_begin: " << m_begin << " min_id: " << min_id
              << " min_dis: " << getMinDistanceToLine(robot, traj.at(min_id));

    // robot at stop point
    if ((min_id + 5) > end_i &&
        getDistance(robot, traj.at(end_i - 1)) <= 0.05) {
      state_ = RobotState::rotate2;
      LOG(INFO) << "robot change state move to rotate2";
    } else {
      // 1、小车脱轨
      if (fabs(getMinDistanceToLine(traj.at(min_id), robot)) > 0.15) {
        LOG(ERROR) << "robot far away from traj, min_traj(x,y,theta): "
                   << traj.at(min_id).x << " " << traj.at(min_id).y << " "
                   << traj.at(min_id).theta << " agv(x,y,theta): " << robot.x
                   << " " << robot.y << " " << robot.theta;
        *n_idx = min_id;
        Init();
        return Tracker::kFailed;
      }

      // 2、小车冲出终点
      if ((min_id + 5) > end_i &&
          fabs(shortest_angular_distance(atan2(traj.at(end_i - 1).y - robot.y,
                                               traj.at(end_i - 1).x - robot.x),
                                         robot.theta)) > M_PI / 2.0 &&
          getDistance(robot, traj.at(end_i - 1)) > 0.05) {
        LOG(ERROR) << "robot over traj end " << traj.at(end_i - 1).x << " "
                   << traj.at(end_i - 1).y << " " << traj.at(end_i - 1).theta
                   << " agv(x,y): " << robot.x << " " << robot.y << " "
                   << robot.theta
                   << " dis to end: " << getDistance(robot, traj.at(end_i - 1));
        *n_idx = end_i - 1;
        Init();
        return Tracker::kFailed;
      }

      // 3、正常
      double v0 = std::hypot(last_cmd.vx, last_cmd.vy);
      double cmax = calculateCurvature(traj, min_id);
      double v_curv = 0.25 * pow(cmax, -0.8); // 曲率 -> 速度
      for (int i = min_id + 1; i < traj_s.size(); i++) {
        double c2 = calculateCurvature(traj, i);
        cmax = std::max(cmax, c2);
        double v = 0.25 * pow(c2, -0.8);
        double s = traj_s[i] - traj_s[min_id];
        if (v < v0) {
          double dec_s = 0.5 * (v0 * v0 - v * v) / acc_ + 0.5; // 计算减速距离
          if (dec_s > s) {
            // 减速距离大于实际距离，需要减速
            v_curv = std::min(v_curv, v);
          }
        }
        if (s > 0.5 * v0 * v0 / acc_ + 0.5)
          break;
      }
      
      // double v_curv = 0.15 * pow(cmax, -0.7);

      double pre = 0;
      double latErr = fabs(getMinDistanceToLine(traj.at(min_id), robot));
      double thetaErr =
          fabs(shortest_angular_distance(robot.theta, traj.at(min_id).theta));
      double err = latErr + 0.5 * thetaErr;
      // if ( err > 0.05) {
      //   pre = std::max({2*err, pre});   // 作用： 避免看的太近超调走S型
      //   pre = std::min(pre, 0.06*pow(cmax, -0.5));   // 作用：
      //   避免转弯的时候看的太远
      // }
      double v_error = 0.07 * pow(err, -1.075); // 跟踪误差 -> 速度

      int next_index = getNextId(traj, robot, min_id, end_i, pre);
      if (next_index < *n_idx)
        next_index = *n_idx;
      if (next_index == end_i - 1)
        --next_index; // end_i-1 的角度和之前角度差距很大，不能作为参考点

      double v_ref = std::min({v_error, v_curv, v_max});

      std::vector<Pose> traj_ref;
      if (next_index + p_ < end_i) {
        traj_ref.assign(std::next(traj.begin(), next_index),
                        std::next(traj.begin(), next_index + p_));
      } else {
        traj_ref.assign(std::next(traj.begin(), next_index),
                        std::next(traj.begin(), end_i - 1));
        traj_ref.insert(traj_ref.end(), next_index + p_ - end_i + 1,
                        traj[end_i - 2]);
      }
      // LOG(INFO) << "predist: " << pre << " cmax: " << cmax
      //           << " v_curv: " << v_curv << " v_error: " << v_error
      //           << " error: " << err;

      m_otg_lim.aMax = 0.33 * acc_;
      if (v_ref < 0.001) {
        m_otg_lim.aMax = 0.33 * stop_acc_;
      }

      double dis_to_end = traj_s[traj_s.size() - 1] - traj_s[next_index];
      m_otg_lim.vMax = v_ref;
      OtgFilter::VelParam target = {dis_to_end, 0, 0, 0};
      m_otg.qk.d = 0;
      m_otg.qk.v = v0;
      m_otg.runCycleS1(3000 * dt_, m_otg_lim, target);

      // LOG(INFO) << "dis_to_end: " << dis_to_end << " lim_v: " << m_otg_lim.vMax
      //           << " v: " << m_otg.qk.v << " a: " << (m_otg.qk.v - v0) / dt_;

      now_cmd = GetCmd(traj_ref, m_otg.qk.v, robot, last_cmd);
      *n_idx = min_id;
      m_begin = min_id;

      cmd2 = cmd1;
      cmd1 = last_cmd;

      LOG(INFO) << "next_i:" << next_index
                << ", traj:" << traj.at(next_index).x << " "
                << traj.at(next_index).y << " " << traj.at(next_index).theta
                << " robot:" << robot.x << " " << robot.y << " "
                << robot.theta << ",end_i:" << end_i
                << ",vx_xy_w:" << now_cmd.vx << " " << now_cmd.vy << " "
                << now_cmd.w;
      return Tracker::State::kTracking;
    }
  }

  if (state_ == RobotState::rotate2) {
    double theta =
        shortest_angular_distance(robot.theta, traj.at(end_i - 1).theta);
    double dis = getDistance(robot, traj.at(end_i - 1));
    if (fabs(theta) < theta_accuracy_) {
      Init();
      *n_idx = end_i - 1;

      LOG(INFO) << "0.The robot has reached the target point, dis: " << dis;
      return Tracker::State::kSuccessful;
    }
    now_cmd = getRotateCmd(robot.theta, traj.at(m_begin).theta, last_cmd.w);
    *n_idx = end_i - 1;
    LOG(INFO) << "rotate2. dis: " << dis;
    return Tracker::State::kTracking;
  }
}

} // namespace motionplanner