#include "track/lqr_tracker.h"

#include <algorithm>
#include <cstdlib>
#include <glog/logging.h>
#include <math.h>

// #include "yaml-cpp/yaml.h"

namespace motionplanner {

LqrTracker::LqrTracker() {
  state_ = RobotState::free;
  index = 0;
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

double LqrTracker::angle_dis(double from, double to) {
  double t = to - from;
  if (t > M_PI) {
    t -= 2 * M_PI;
  } else if (t < -M_PI) {
    t += 2 * M_PI;
  }
  return t;
}

std::pair<double, double> LqrTracker::getRotateCmd(double curr_theta,
                                                   double target_theta,
                                                   double v, double w) {
  std::pair<double, double> cmd;
  cmd.first = 0;

  cmd.second = 1 * angle_dis(curr_theta, target_theta);

  cmd.second = std::max({-max_w_, w - alpha_ * dt_, cmd.second});
  cmd.second = std::min({max_w_, w + alpha_ * dt_, cmd.second});

  // 防止终点处的速度太慢，限制最小角速度
  if (fabs(cmd.second) < min_w_) {
    cmd.second = sgn(cmd.second) * min_w_;
  }

  LOG(INFO) << "cmdrotate(w): " << cmd.second << " robot_pose " << curr_theta
            << " target_theta " << target_theta;

  return cmd;
}

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
                                    Pose &robot) {
  // robot at start point
  if (state_ == RobotState::free) {
    if (index == 0)
      index++;
    state_ = RobotState::rotate1;
    LOG(INFO) << "robot change state free to rotate1";
  }

  if (state_ == RobotState::rotate1) {
    double theta = angle_dis(robot.theta, traj.at(index).theta);
    // 旋转完成
    if (fabs(theta) < 0.01) {
      state_ = RobotState::move;
      m_otg.reset();
      LOG(INFO) << "robot change state rotate1 to move";
    }
    // 旋转中
    else {
      auto cmd = getRotateCmd(robot.theta, traj.at(index).theta, v1, w1);
      // 更新坐标
      w1 = cmd.second;
      robot.theta += cmd.second * dt_;
      return State::kTracking;
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

    int index = getNearestId(traj, robot, index,
                             traj.size()); // 这里得到index是虚拟车当前的位置

    // robot at stop point
    if ((index + 5) > traj.size() && getDistance(robot, traj.back()) <= 0.02) {
      state_ = RobotState::rotate2;
      LOG(INFO) << "robot change state move to rotate2";
    } else {

      // 3、正常
      double cmax = calculateCurvature(traj, index);
      double v_curv = 0.25 * pow(cmax, -0.8); // 曲率 -> 速度
      for (int i = index + 1; i < traj_s.size(); i++) {
        double c2 = calculateCurvature(traj, i);
        cmax = std::max(cmax, c2);
        double v = 0.25 * pow(c2, -0.8);
        double s = traj_s[i] - traj_s[index];
        if (v < v1) {
          double dec_s = 0.5 * (v1 * v1 - v * v) / acc_ + 0.5; // 计算减速距离
          if (dec_s > s) {
            // 减速距离大于实际距离，需要减速
            v_curv = std::min(v_curv, v);
          }
        }
        if (s > 0.5 * v1 * v1 / acc_ + 0.5)
          break;
      }

      m_otg_lim.aMax = acc_;
      if (vmax < 0.001) {
        m_otg_lim.aMax = acc_;
      }
      double dis_to_end = traj_s.back() - traj_s[index];
      m_otg_lim.vMax = std::min(vmax, v_curv);
      OtgFilter::VelParam target = {dis_to_end, 0, 0, 0};
      m_otg.qk.v = v1;
      m_otg.qk.d = 0;
      m_otg.runCycleS1(1000 * dt_, m_otg_lim, target);
      LOG(INFO) << " lim_v: " << m_otg_lim.vMax << " v: " << m_otg.qk.v
                << " a: " << (m_otg.qk.v - v1) / dt_;

      double v0 = m_otg.qk.v;
      double w0;
      if (index < traj.size() - 1) {
        double dis = getDistance(traj.at(index), traj.at(index + 1)) + 1e-6;
        w0 = angle_dis(traj.at(index).theta, traj.at(index + 1).theta) /
                dis * v0;
      } else {
        w0 = 0;
      }

      double time_dis = v0 * dt_;
      double next_index = getNextId(traj, traj[index], index, traj.size(),
                                    time_dis); // 这里是下一时刻虚拟车到达的位置

      index = next_index; // 0.05 秒以后 agv 会到达的位置
      robot = traj[next_index];

      v1 = v0;
      w1 = w0;
      v2 = v1;
      w2 = w1;
      return State::kTracking;
    }
  }

  if (state_ == RobotState::rotate2) {
    double theta =
        angle_dis(robot.theta, traj.back().theta);
    double dis  = getDistance(robot, traj.back());
    if (fabs(theta) < 0.01) {
      Init();
      LOG(INFO) << "0.The robot has reached the target point, dis: " << dis;
      return State::kSuccessful;
    }
    auto cmd = getRotateCmd(robot.theta, traj.back().theta, v1, w1);
    w1 = cmd.second;
    robot.theta += cmd.second * dt_;
    return State::kTracking;
  }
}

} // namespace motionplanner