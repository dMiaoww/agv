#include "move_base/pf3_tracker.h"

#include "glog_set.h"
#include "move_base/common.h"
#include "move_base/controller/lqr_nsteer.h"
#include "move_base/controller/lqr_steer.h"
// #include "move_base/move_base_v1.h"
#include "move_base/otg_filter.h"
#include <algorithm>
#include <cstdlib>
#include <glog/logging.h>
#include <math.h>
// #include "yaml-cpp/yaml.h"

namespace motionplanner {

Pf3Tracker::Pf3Tracker() : Tracker() {
  state_ = RobotState::free;
  e = 0;
  th_e = 0;
  m_begin = 0;
  m_otg.reset();
  // agv_type_ = global_agv_configure_.agv_type_configure.agv_type;
}

Pf3Tracker::~Pf3Tracker() = default;

// 距离的绝对值
double Pf3Tracker::getDistance(const Pose &p1, const Pose &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/** robot  p2     p3:    return < 0
    p2     robot  p3:    return 0~1
    p2     p3     robot: return > 1
**/
double Pf3Tracker::getDot(const Pose &robot, const Pose &p2, const Pose &p3) {
  Pose p21{robot.x - p2.x, robot.y - p2.y, 0};
  Pose p23{p3.x - p2.x, p3.y - p2.y, 0};

  double dot =
      (p21.x * p23.x + p21.y * p23.y) / (p23.x * p23.x + p23.y * p23.y + 1e-9);
  return dot;
}

// 找到轨迹上最靠近小车、且小车并未到达的点
int Pf3Tracker::getNearestId(const std::vector<Pose> &traj, const Pose &robot,
                             int begin_i, int end_i) {
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
double Pf3Tracker::getMinDistanceToLine(const Pose &point, const Pose &line) {
  double x1 = line.x;
  double y1 = line.y;
  double x2 = line.x + cos(line.theta);
  double y2 = line.y + sin(line.theta);

  return (y1 - y2) * point.x - (x1 - x2) * point.y + (x1 * y2 - x2 * y1);
}

double Pf3Tracker::calculateCurvature(const std::vector<Pose> &traj,
                                      const int &i) {
  if ((i + 3) > traj.size())
    return 0;

  // double x1 = traj.at(i).x;
  // double y1 = traj.at(i).y;
  // double x2 = traj.at(i+1).x;
  // double y2 = traj.at(i+1).y;
  // double x3 = traj.at(i+2).x;
  // double y3 = traj.at(i+2).y;

  // double x1_prime = (x2 - x1);
  // double y1_prime = (y2 - y1);
  // double x2_prime = (x3 - x2);
  // double y2_prime = (y3 - y2);

  // double numerator = std::abs((x1_prime * y2_prime) - (y1_prime *
  // x2_prime))+1e-6; double denominator = std::pow((x1_prime * x1_prime +
  // y1_prime * y1_prime), 1.5);

  // return numerator / (denominator+0.000001);

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
int Pf3Tracker::getNextId(const std::vector<Pose> &traj, const Pose &robot,
                          int begin_i, int end_i, double preview_dist) {
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

int Pf3Tracker::sgn(double x) { return (x >= 0) ? 1 : -1; }

MoveCmd Pf3Tracker::GetCmd(const std::vector<Pose> &traj, const double v_ref,
                           const Pose &robot, const MoveCmd &last) {
  
  MoveCmd cmd;
  ControlStatus status;
  status.k = calculateCurvature(traj, 0); // 曲率
  status.curr_pose = robot;
  status.target_pose = traj.at(0);
  status.target_vx = v_ref;
  // 参考角速度应该用参考轨迹变化量，与真实位置无关，避免了控制时朝向摆动的问题
  double dis = getDistance(traj.at(0), traj.at(1)) + 1e-6;
  status.target_vyaw =
      shortest_angular_distance(traj.at(0).theta, traj.at(1).theta) /
      (1e-6 + dis * status.target_vx);

  status.curr_vx = last.vx; // v 为上一时刻的控制速度
  status.curr_vyaw = last.w;

  cmd.vx = status.target_vx;

  double max_w = max_w_;
  double alpha = alpha_;

  LqrSteer lqr_steer;
  cmd.w = lqr_steer.UpdateControl(status, lqr_param_); // 注意这里算出来的是角度
  // LOG(INFO) << "[lqr] vx:" << cmd.vx << " vy:" << cmd.vy << " w:" << cmd.w;

  cmd.vx = std::max({-max_v_, last.vx - acc_ * dt_, cmd.vx});
  cmd.vx = std::min({max_v_, last.vx + acc_ * dt_, cmd.vx});
  // cmd.vy = std::max({-max_v_, last.vy - acc_ * dt_, cmd.vy});
  // cmd.vy = std::min({max_v_, last.vy + acc_ * dt_, cmd.vy});
  // cmd.w = std::max({-max_w, last.w - alpha * dt_, cmd.w});
  // cmd.w = std::min({max_w, last.w + alpha * dt_, cmd.w});
  LOG(INFO) << "[lqr] vx:" << cmd.vx  << " angle:" << cmd.w;

  return cmd;
}

void Pf3Tracker::SetAlgoParam() {
  lqr_param_.max_iteration = 100;
  lqr_param_.t = dt_;
  lqr_param_.L = 1.004;
  LOG(INFO) << "lqr param(t, w_x, w_y, w_yaw, vx, vyaw): " << lqr_param_.t
            << " " << lqr_param_.w_x << " " << lqr_param_.w_x << " "
            << lqr_param_.w_yaw << " " << lqr_param_.w_vx << " "
            << lqr_param_.w_vyaw;
  Init();
}

Tracker::State Pf3Tracker::Track(const std::vector<Pose> &traj,
                                 const double v_max,
                                 const std::vector<double> &traj_s,
                                 size_t begin_i, size_t end_i, Pose &robot,
                                 const MoveCmd &last_cmd, MoveCmd &now_cmd,
                                 size_t *n_idx, bool is_backward) {
  now_cmd = MoveCmd(0, 0, 0);
  if (*n_idx == 0 || *n_idx >= end_i)
    Init();

  // robot at start point
  if (state_ == RobotState::free) {
    // 起点距离当前位置太远
    m_begin = getNearestId(traj, robot, begin_i, end_i);
    double theta =
        shortest_angular_distance(robot.theta, traj.at(m_begin).theta);
    if (getDistance(robot, traj.at(m_begin)) > 0.2 ||
        fabs(theta) > 30.0 / 180.0 * 3.14) {
      LOG(ERROR) << "wrong start point, "
                 << "robot: " << robot.x << " " << robot.y << " " << robot.theta
                 << " begin: " << begin_i << " traj[" << m_begin
                 << "]:" << traj.at(m_begin).x << " " << traj.at(m_begin).y
                 << " " << traj.at(m_begin).theta;
      Init();
      now_cmd = MoveCmd(0, 0, 0);
      *n_idx = begin_i;
      return Tracker::State::kFailed;
    }

    // 距离远的时候固定找10厘米之外的点
    if (getDistance(robot, traj.at(end_i - 1)) >= 0.1) {
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

  if (state_ == RobotState::move) {
    // robot.x += 0.05 * v2 * cos(robot.theta);
    // robot.y += 0.05 * v2 * sin(robot.theta);
    // robot.theta += 0.05 * w2;
    // robot.x += 0.05 * v1 * cos(robot.theta);
    // robot.y += 0.05 * v1 * sin(robot.theta);
    // robot.theta += 0.05 * w1;
    // robot.x += 0.05 * last_cmd.vx * cos(robot.theta);
    // robot.y += 0.05 * last_cmd.vx * sin(robot.theta);
    // robot.theta += 0.05 * last_cmd.w;

    int min_id = getNearestId(traj, robot, m_begin, end_i);
    LOG(INFO) << "m_begin: " << m_begin << " min_id: " << min_id
              << " min_dis: " << getMinDistanceToLine(robot, traj.at(min_id));

    // robot at stop point
    if ((min_id + 5) > end_i &&
        getDistance(robot, traj.at(end_i - 1)) <= 0.05) {
      Init();
      *n_idx = end_i - 1;
      LOG(INFO) << "0.The robot has reached the target point.";
      return Tracker::State::kSuccessful;
    } else {
      // 1、小车脱轨
      if (fabs(getMinDistanceToLine(traj.at(min_id), robot)) > 0.5) {
        LOG(ERROR) << "robot far away from traj, min_traj(x,y,theta): "
                   << traj.at(min_id).x << " " << traj.at(min_id).y << " "
                   << traj.at(min_id).theta << " agv(x,y,theta): " << robot.x
                   << " " << robot.y << " " << robot.theta;
        now_cmd = MoveCmd(0, 0, 0);
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
        now_cmd = MoveCmd(0, 0, 0);
        *n_idx = end_i - 1;
        Init();
        return Tracker::kFailed;
      }

      // 3、正常
      double cmax = calculateCurvature(traj, min_id);
      double v_curv = 0.25 * pow(cmax, -0.8); // 曲率 -> 速度
      for (int i = min_id + 1; i < traj_s.size(); i++) {
        double c2 = calculateCurvature(traj, i);
        cmax = std::max(cmax, c2);
        double v = 0.25 * pow(c2, -0.8);
        double s = traj_s[i] - traj_s[min_id];
        if (v < last_cmd.vx) {
          double dec_s = 0.5 * (last_cmd.vx * last_cmd.vx - v * v) / acc_ +
                         0.5; // 计算减速距离
          if (dec_s > s) {
            // 减速距离大于实际距离，需要减速
            v_curv = std::min(v_curv, v);
          }
        }
        if (s > 0.5 * last_cmd.vx * last_cmd.vx / acc_ + 0.5)
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
      double v_error = 0.07 * pow(err + 0.05, -1.075); // 跟踪误差 -> 速度

      int next_index = getNextId(traj, robot, min_id, end_i, pre);
      if (next_index < *n_idx)
        next_index = *n_idx;
      if (next_index == end_i - 1)
        --next_index; // end_i-1 的角度和之前角度差距很大，不能作为参考点

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
      LOG(INFO) << "predist: " << pre << " cmax: " << cmax
                << " v_curv: " << v_curv << " v_error: " << v_error
                << " error: " << err;

      m_otg_lim.aMax = 0.33 * acc_;
      if (v_max < 0.001) {
        m_otg_lim.aMax = 0.33 * stop_acc_;
      }

      double dis_to_end = traj_s[traj_s.size() - 1] - traj_s[next_index];
      m_otg_lim.vMax = std::min({v_max, v_curv, v_error});
      OtgFilter::VelParam target = {dis_to_end, 0, 0, 0};
      m_otg.qk.d = 0;
      // m_otg.qk.v = last_cmd.vx;
      m_otg.runCycleS1(3000 * dt_, m_otg_lim, target);
      LOG(INFO) << "dis_to_end: " << dis_to_end << " lim_v: " << m_otg_lim.vMax
                << " v: " << m_otg.qk.v
                << " a: " << (m_otg.qk.v - last_cmd.vx) / dt_;

      double v_ref = m_otg.qk.v;
      // if (is_backward)
      //   v_ref *= -1;

      now_cmd = GetCmd(traj_ref, v_ref, robot, last_cmd);

      *n_idx = min_id;
      m_begin = min_id;

      v1 = last_cmd.vx;
      w1 = last_cmd.w;
      v2 = v1;
      w2 = w1;

      LOG(INFO) << "next_i:" << next_index
                << ", traj(x_y_theta):" << traj.at(next_index).x << " "
                << traj.at(next_index).y << " " << traj.at(next_index).theta
                << " robot(x_y_theta)" << robot.x << " " << robot.y << " "
                << robot.theta << " end_i:" << end_i << " v_w:" << now_cmd.vx
                << " " << now_cmd.vy;
      return Tracker::State::kTracking;
    }
  }
}
}
