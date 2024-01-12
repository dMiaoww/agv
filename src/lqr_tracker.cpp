#include "lqr_tracker.h"

#include <algorithm>
#include <cstdlib>
#include <glog/logging.h>
#include <math.h>

// #include "yaml-cpp/yaml.h"

namespace motionplanner {

LqrTracker::LqrTracker() {
  state_ = RobotState::free;
  m_begin = 0;
  m_otg.reset();
}

LqrTracker::~LqrTracker() = default;

// 距离的绝对值
double LqrTracker::getDistance(const Pose &p1, const Pose &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/** robot  p2     p3:    return < 0
    p2     robot  p3:    return 0~1
    p2     p3     robot: return > 1
**/
double LqrTracker::getDot(const Pose &robot, const Pose &p2, const Pose &p3) {
  Pose p21{robot.x - p2.x, robot.y - p2.y, 0};
  Pose p23{p3.x - p2.x, p3.y - p2.y, 0};

  double dot =
      (p21.x * p23.x + p21.y * p23.y) / (p23.x * p23.x + p23.y * p23.y + 1e-9);
  return dot;
}

// 找到轨迹上最靠近小车、且小车并未到达的点
int LqrTracker::getNearestId(const std::vector<Pose> &traj, const Pose &robot,
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
double LqrTracker::getMinDistanceToLine(const Pose &point, const Pose &line) {
  double x1 = line.x;
  double y1 = line.y;
  double x2 = line.x + cos(line.theta);
  double y2 = line.y + sin(line.theta);

  return (y1 - y2) * point.x - (x1 - x2) * point.y + (x1 * y2 - x2 * y1);
}

double LqrTracker::calculateCurvature(const std::vector<Pose> &traj,
                                      const int &i) {
  if ((i + 3) > traj.size())
    return 0;

  double x1 = traj.at(i).x;
  double y1 = traj.at(i).y;
  double x2 = traj.at(i + 1).x;
  double y2 = traj.at(i + 1).y;
  double x3 = traj.at(i + 2).x;
  double y3 = traj.at(i + 2).y;

  double x1_prime = (x2 - x1);
  double y1_prime = (y2 - y1);
  double x2_prime = (x3 - x2);
  double y2_prime = (y3 - y2);

  double numerator =
      std::abs((x1_prime * y2_prime) - (y1_prime * x2_prime)) + 1e-6;
  double denominator =
      std::pow((x1_prime * x1_prime + y1_prime * y1_prime), 1.5);

  return numerator / (denominator + 0.000001);
}

// [begin_i, end_i) 区间上离小车距离最接近 preview_dist 的点
int LqrTracker::getNextId(const std::vector<Pose> &traj, const Pose &robot,
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

int LqrTracker::sgn(double x) { return (x >= 0) ? 1 : -1; }

double LqrTracker::shortest_angular_distance(double from, double to) {
  double t = to - from;
  if (t > M_PI) {
    t -= 2 * M_PI;
  } else if (t < -M_PI) {
    t += 2 * M_PI;
  }
  return t;
}

// std::pair<double, double> LqrTracker::getRotateCmd(double curr_theta,
//                                                    double target_theta,
//                                                    double v, double w) {
//   std::pair<double, double> cmd;
//   cmd.first = 0;

//   cmd.second = 1 * shortest_angular_distance(curr_theta, target_theta);

//   cmd.second = std::max({-max_w_, w - alpha_ * dt_, cmd.second});
//   cmd.second = std::min({max_w_, w + alpha_ * dt_, cmd.second});

//   // 防止终点处的速度太慢，限制最小角速度
//   if (fabs(cmd.second) < min_w_) {
//     cmd.second = sgn(cmd.second) * min_w_;
//   }

//   LOG(INFO) << "cmdrotate(w): " << cmd.second << " robot_pose " << curr_theta
//              << " target_theta " << target_theta;

//   return cmd;
// }

// std::pair<double, double> LqrTracker::GetCmd(const std::vector<Pose> &traj,
//                                              const std::vector<double>
//                                              &traj_v, double x, double y,
//                                              double theta, double v, double
//                                              w) {
//   std::pair<double, double> cmd = std::make_pair(0, 0);

//   ControlStatus status;

//   status.curr_pose.x = x;
//   status.curr_pose.y = y;
//   status.curr_pose.theta = theta;

//   status.target_pose = traj.at(0);
//   status.target_vx = traj_v.at(0);
//   //
//   参考角速度应该用参考轨迹变化量，与真实位置无关，避免了控制时朝向摆动的问题
//   double dis = getDistance(traj.at(0), traj.at(1))+1e-6;
//   status.target_vyaw =
//       shortest_angular_distance(traj.at(0).theta, traj.at(1).theta) / dis *
//       status.target_vx;

//   status.curr_vx = v;  // v 为上一时刻的控制速度
//   status.curr_vyaw = w;

//   cmd.first = status.target_vx;

//   double max_w = max_w_;
//   double alpha = alpha_;

//   LqrW lqrw;
//   double ww = lqrw.UpdateControl(status, dt_);
//   cmd.second = ww;

//   // cmd.first = std::max({0.01, v - acc_ * dt_, cmd.first});
//   // cmd.first = std::min({max_v_, v + acc_ * dt_, cmd.first});
//   cmd.second = std::max({-max_w, w - alpha * dt_, cmd.second});
//   cmd.second = std::min({max_w, w + alpha * dt_, cmd.second});

//   LOG(INFO) << "[lqr]" << "v:"<< cmd.first << " w:" << cmd.second
//       << " target_vyaw:" << status.target_vyaw << " dyaw:" <<
//       shortest_angular_distance(status.target_pose.theta, theta);
//   return cmd;
// }

void LqrTracker::SetAlgoParam() { Init(); }

LqrTracker::State LqrTracker::Track(const std::vector<Pose> &traj,
                                    const double vmax,
                                    const std::vector<double> &traj_s,
                                    size_t begin_i, size_t end_i, Pose &robot,
                                    double v0, double w0, double *v, double *w,
                                    int *n_idx) {
  static Pose last_robot;
  *v = 0;
  *w = 0;
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
      *v = 0;
      *w = 0;
      *n_idx = begin_i;
      return State::kFailed;
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

    state_ = RobotState::rotate1;
    LOG(INFO) << "robot change state free to rotate1";
  }

  if (state_ == RobotState::rotate1) {
    if (getDistance(robot, traj.at(end_i - 1)) <= 0.05)
      m_begin = end_i - 1;
    double theta =
        shortest_angular_distance(robot.theta, traj.at(m_begin).theta);
    // 旋转完成
    if (fabs(theta) < 0.01) {
      if (m_begin != (end_i - 1)) {
        state_ = RobotState::move;
        last_robot = robot;
        m_otg.reset();
        LOG(INFO) << "robot change state rotate1 to move";
      } else {
        Init();
        *n_idx = end_i - 1;
        *v = 0;
        *w = 0;
        LOG(INFO) << "0.The robot has reached the target point";
        return State::kSuccessful;
      }
    }
    // 旋转中
    else {
      // auto cmd = getRotateCmd(robot.theta, traj.at(m_begin).theta, v0, w0);
      // *v = cmd.first;
      // *w = cmd.second;
      // *n_idx = m_begin;
      // return State::kTracking;
    }
  }

  if (state_ == RobotState::move) {
    // robot.x += 0.05 * v2 * cos(robot.theta);
    // robot.y += 0.05 * v2 * sin(robot.theta);
    // robot.theta += 0.05 * w2;
    // robot.x += 0.05 * v1 * cos(robot.theta);
    // robot.y += 0.05 * v1 * sin(robot.theta);
    // robot.theta += 0.05 * w1;
    // robot.x += 0.05 * v0 * cos(robot.theta);
    // robot.y += 0.05 * v0 * sin(robot.theta);
    // robot.theta += 0.05 * w0;

    double dtheta = shortest_angular_distance(last_robot.theta, robot.theta);
    if (sgn(w0) != sgn(dtheta)) {
      LOG(INFO) << "w0: " << w0 << " dtheta: " << dtheta;
    }

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
        *v = 0;
        *w = 0;
        *n_idx = min_id;
        Init();
        return kFailed;
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
        *v = 0;
        *w = 0;
        *n_idx = end_i - 1;
        Init();
        return kFailed;
      }

      // 3、正常
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
      // double latErr = fabs(getMinDistanceToLine(traj.at(min_id), robot));
      // double thetaErr =
      //     fabs(shortest_angular_distance(robot.theta, traj.at(min_id).theta));
      double err = 0;
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

      std::vector<Pose> traj_ref;
      std::vector<double> traj_ref_v;
      traj_ref_v.emplace_back(std::min({v_error, v_curv, vmax}));
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

      m_otg_lim.aMax = acc_ ;
      if (vmax < 0.001) {
        m_otg_lim.aMax = acc_;
      }

      double dis_to_end = traj_s.back() - traj_s[next_index];
      LOG(INFO) << traj_s.back() << " " <<  traj_s[next_index] << " next_index " << next_index;
      m_otg_lim.vMax = traj_ref_v[0];
      OtgFilter::VelParam target = {dis_to_end, 0, 0, 0};
      m_otg.qk.v = v0;
      m_otg.qk.d = 0;
      m_otg.runCycleS1(1000 * dt_, m_otg_lim, target);
      traj_ref_v[0] = m_otg.qk.v;
      LOG(INFO) << " lim_v: " << m_otg_lim.vMax
                << " v: " << m_otg.qk.v << " a: " << (m_otg.qk.v - v0) / dt_;

      // std::pair<double, double> cmd =
      //     GetCmd(traj_ref, traj_ref_v, robot.x, robot.y, robot.theta, v0,
      //     w0);

      *v = m_otg.qk.v;

      double dis = getDistance(traj.at(0), traj.at(1)) + 1e-6;
      *w = shortest_angular_distance(traj.at(0).theta, traj.at(1).theta) / dis *
           *v;

      *n_idx = min_id;
      m_begin = min_id;

      v1 = v0;
      w1 = w0;
      v2 = v1;
      w2 = w1;

      LOG(INFO) << "next_i:" << next_index
                << ", traj(x_y_theta):" << traj.at(next_index).x << " "
                << traj.at(next_index).y << " " << traj.at(next_index).theta
                << " robot(x_y_theta)" << robot.x << " " << robot.y << " "
                << robot.theta << ",end_i:" << end_i << ",v_w:" << *v << " "
                << *w;
      last_robot = robot;
      return State::kTracking;
    }
  }
}

} // namespace motionplanner