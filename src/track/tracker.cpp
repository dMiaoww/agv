#include "track/tracker.h"

#include "glog_set.h"
#include "common_data.h"


namespace motionplanner {

Tracker::Tracker(/* args */)
    : dt_(0.01),
      p_(3),
      max_v_(1),
      min_v_(0),
      max_w_(0.5),
      min_w_(-0.5),
      acc_(0.2),
      alpha_(0.2),
      pos_accuracy_(0.05),
      theta_accuracy_(0.1),
      end_flag_(false),
      printlog_(true) {}

Tracker::~Tracker() = default;

void Tracker::SetMotionParam(double frequency, int p, double max_v,
                             double min_v, double max_w, double min_w,
                             double acc, double stop_acc, double alpha, double pos_accuracy,
                             double theta_accuracy) {
  dt_ = 1.0 / frequency;
  p_ = p;
  max_v_ = max_v;
  min_v_ = min_v;
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

Tracker::State Tracker::Track(const std::vector<Pose> &traj, const double v_max,
                      const std::vector<double> &traj_s, size_t begin_i,
                      size_t end_i, Pose& robot, const MoveCmd& last, MoveCmd& now, size_t *next_i, bool backward) {
  // GetIndex(traj, begin_i, end_i, robot, next_i);
  // if (*next_i == end_i) {
  //   *next_i = 0;
  //   *v = 0;
  //   *w = 0;
  //   return Tracker::State::kSuccessful;
  // } else if (*next_i > end_i) {
  //   *next_i = 0;
  //   *v = 0;
  //   *w = 0;
  //   return Tracker::State::kFailed;
  // }

  // size_t next_index = *next_i;
  // std::vector<Pose> traj_ref;
  // std::vector<double> traj_ref_v;
  // if (next_index + p_ <= end_i) {
  //   traj_ref.assign(std::next(traj.begin(), next_index),
  //                   std::next(traj.begin(), next_index + p_));

  //   traj_ref_v.assign(std::next(traj_v.begin(), next_index),
  //                     std::next(traj_v.begin(), next_index + p_));
  // } else {
  //   traj_ref.assign(std::next(traj.begin(), next_index),
  //                   std::next(traj.begin(), end_i));
  //   traj_ref.insert(traj_ref.end(), next_index + p_ - end_i, traj[end_i - 1]);

  //   traj_ref_v.assign(std::next(traj_v.begin(), next_index),
  //                     std::next(traj_v.begin(), end_i));
  //   traj_ref_v.insert(traj_ref_v.end(), next_index + p_ - end_i,
  //                     traj_v[end_i - 1]);
  // }

  // std::pair<double, double> cmd =
  //     GetCmd(traj_ref, traj_ref_v, robot.x, robot.y, robot.theta, v0, w0);

  // // for (size_t k = 0; k < traj_ref.size(); k++) {
  // //   LOG(INFO) << "k:" << k << "," << traj_ref[k].x << " " << traj_ref[k].y
  // //             << " " << traj_ref[k].theta;
  // // }
  // *v = cmd.first;
  // *w = cmd.second;

  // double d_dis = 0;
  // if (next_index != 0) {
  //   d_dis = std::abs((robot.x - traj[next_index - 1].x) *
  //                        (traj[next_index].y - traj[next_index - 1].y) -
  //                    (traj[next_index].x - traj[next_index - 1].x) *
  //                        (robot.y - traj[next_index - 1].y)) /
  //           std::hypot((traj[next_index].x - traj[next_index - 1].x),
  //                      (traj[next_index].y - traj[next_index - 1].y));
  // }

  // LOG(INFO) << "next_i:" << next_index << ", x_y:" << traj[next_index].x << " "
  //           << traj[next_index].y << ",end_i:" << end_i << ",v_w:" << *v << " "
  //           << *w << ", error:" << d_dis;

  return Tracker::State::kTracking;
}

void Tracker::GetIndex(const std::vector<Pose> &traj, size_t begin_i,
                       size_t end_i, const Pose &robot, size_t *next_i) {
  if (end_i > traj.size() || begin_i >= end_i) {
    LOG(ERROR) << "Index error, begin:" << begin_i << ", end:" << end_i
               << ", traj size:" << traj.size();

    *next_i = SIZE_MAX;
    return;
  }
  LOG(INFO) << "get index, begin_i:" << begin_i << ",end_i:" << end_i
            << ",begin pos:" << traj.at(begin_i).x << "," << traj.at(begin_i).y
            << ",end pos:" << traj.at(end_i - 1).x << ","
            << traj.at(end_i - 1).y << "," << traj.at(end_i - 1).theta
            << ",robot pos:" << robot.x << "," << robot.y << "," << robot.theta;
  size_t index = SIZE_MAX;

  bool flag = false;
  Pose curr_to_robot;
  Pose curr_to_next;

  for (size_t i = begin_i; (i + 1) < end_i; ++i) {
    const Pose &curr_point = traj[i];
    const Pose &next_point = traj[i + 1];

    curr_to_robot.x = robot.x - curr_point.x;
    curr_to_robot.y = robot.y - curr_point.y;
    curr_to_robot.theta = 0;

    curr_to_next.x = next_point.x - curr_point.x;
    curr_to_next.y = next_point.y - curr_point.y;
    curr_to_next.theta = 0;

    double dot =
        (curr_to_robot.x * curr_to_next.x + curr_to_robot.y * curr_to_next.y) /
        (curr_to_next.x * curr_to_next.x + curr_to_next.y * curr_to_next.y);

    if (dot < 0) {
      flag = false;
      if (0.25 > std::hypot(curr_to_robot.x, curr_to_robot.y)) {
        index = i;
        LOG(WARNING) << "dis:" << std::hypot(curr_to_robot.x, curr_to_robot.y)
                     << ",i:" << index;
        LOG(WARNING) << "curr_point:" << curr_point.x << "," << curr_point.y;
        LOG(WARNING) << "next_point:" << next_point.x << "," << next_point.y;
        LOG(WARNING) << "robot:" << robot.x << "," << robot.y;
        break;
      } else {
        continue;
      }
    } else if (dot < 1) {
      if (i + 2 == end_i) {
        index = SIZE_MAX - 1;
      }

      if (flag) {
        double dis = std::abs(curr_to_robot.x * curr_to_next.y -
                              curr_to_next.x * curr_to_robot.y) /
                     std::hypot(curr_to_next.x, curr_to_next.y);

        const Pose &last_c = traj[i - 1];
        const Pose &last_n = traj[i];

        Pose c2robot(robot.x - last_c.x, robot.y - last_c.y, 0);
        Pose c2n(last_n.x - last_c.x, last_n.y - last_c.y, 0);
        double last_dis = std::abs(c2robot.x * c2n.y - c2n.x * c2robot.y) /
                          std::hypot(c2n.x, c2n.y);
        if (last_dis < dis) {
          index = i;
          LOG(WARNING) << "last_dis:" << last_dis << ",dis:" << dis
                       << ",i:" << index;
          LOG(WARNING) << "last_c:" << last_c.x << "," << last_c.y;
          LOG(WARNING) << "last_n:" << last_n.x << "," << last_n.y;
          LOG(WARNING) << "robot:" << robot.x << "," << robot.y;
          break;
        }
      }
      flag = true;
    } else {
      if (i + 2 == end_i) {
        index = SIZE_MAX - 2;
      }
      flag = false;
      continue;
    }
  }

  // double dis = std::hypot(robot.x - traj.at(end_i - 1).x,
  //                         robot.y - traj.at(end_i - 1).y);
  // double rad = AbsDiffRad(robot.theta, traj.at(end_i - 1).theta);
  // if (dis < 2 * pos_accuracy_) {
  //   end_flag_ = true;
  // } else {
  //   end_flag_ = false;
  // }

  if (index < end_i) {
    *next_i = index;
  } else if (index == SIZE_MAX - 2) {
    double dis = std::hypot(robot.x - traj.at(end_i - 1).x,
                            robot.y - traj.at(end_i - 1).y);
    double rad = AbsDiffRad(robot.theta, traj.at(end_i - 1).theta);

    if ((dis < pos_accuracy_) && (rad < theta_accuracy_)) {
      *next_i = end_i;
      LOG(INFO) << "1.The robot has reached the target point, robot x_y_theta:"
                << robot.x << " " << robot.y << " " << robot.theta
                << ", target_x_y_theta:" << traj.at(end_i - 1).x << " "
                << traj.at(end_i - 1).y << " " << traj.at(end_i - 1).theta;
    } else if (dis < 2 * pos_accuracy_) {
      *next_i = end_i - 1;
    } else {
      *next_i = SIZE_MAX;
      LOG(ERROR) << "The robot over traj end , robot x_y:" << robot.x << " "
                 << robot.y << ", traj end point x_y:" << traj.at(end_i - 1).x
                 << " " << traj.at(end_i - 1).y;
    }
  } else if (index == SIZE_MAX - 1) {
    double dis = std::hypot(robot.x - traj.at(end_i - 1).x,
                            robot.y - traj.at(end_i - 1).y);
    double rad = AbsDiffRad(robot.theta, traj.at(end_i - 1).theta);

    if ((dis < pos_accuracy_) && (rad < theta_accuracy_)) {
      *next_i = end_i;
      LOG(INFO) << "0.The robot has reached the target point, robot x_y_theta:"
                << robot.x << " " << robot.y << " " << robot.theta
                << ", target_x_y_theta:" << traj.at(end_i - 1).x << " "
                << traj.at(end_i - 1).y << " " << traj.at(end_i - 1).theta;
    } else {
      *next_i = end_i - 1;
    }
  } else if (index == SIZE_MAX) {
    double dis =
        std::hypot(robot.x - traj.at(begin_i).x, robot.y - traj.at(begin_i).y);
    double rad = AbsDiffRad(robot.theta, traj.at(begin_i).theta);

    if ((dis < pos_accuracy_) && (rad < theta_accuracy_)) {
      *next_i = end_i;
      LOG(INFO) << "-1.The robot has reached the target point, robot x_y_theta:"
                << robot.x << " " << robot.y << " " << robot.theta
                << ", target_x_y_theta:" << traj.at(end_i - 1).x << " "
                << traj.at(end_i - 1).y << " " << traj.at(end_i - 1).theta;
    } else if (dis < 0.25) {
      *next_i = begin_i;
      LOG(WARNING)
          << "The trajectory has only one point, traj begin x_y_theta : "
          << traj.at(begin_i).x << " " << traj.at(begin_i).y << " "
          << traj.at(begin_i).theta;
    } else {
      *next_i = SIZE_MAX;
      LOG(ERROR) << "The robot dont reach the traj begin, robot x_y:" << robot.x
                 << " " << robot.y
                 << ", traj begin point x_y:" << traj.at(begin_i).x << " "
                 << traj.at(begin_i).y;
    }
  } else {
    *next_i = end_i + 1;
    LOG(ERROR) << "CODE ERROR! index:" << index << ", start_end_i:" << begin_i
               << " " << end_i;
  }
}

Tracker::State Tracker::TrackStop(double& w){
  double dw = alpha_ * dt_;
  if (w > dw) {
    w = w - dw;
  } else if (w < -dw) {
    w = w + dw;
  } else {
    w = 0;
  }
  return Tracker::State::kTracking;
}

// 距离的绝对值
double Tracker::getDistance(const Pose &p1, const Pose &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/** robot  p2     p3:    return < 0
    p2     robot  p3:    return 0~1
    p2     p3     robot: return > 1
**/
double Tracker::getDot(const Pose &robot, const Pose &p2, const Pose &p3) {
  Pose p21{robot.x - p2.x, robot.y - p2.y, 0};
  Pose p23{p3.x - p2.x, p3.y - p2.y, 0};

  double dot =
      (p21.x * p23.x + p21.y * p23.y) / (p23.x * p23.x + p23.y * p23.y + 1e-9);
  return dot;
}

// 找到轨迹上最靠近小车、且小车并未到达的点
int Tracker::getNearestId(const std::vector<Pose> &traj, const Pose &robot,
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
double Tracker::getMinDistanceToLine(const Pose &point, const Pose &line) {
  double x1 = line.x;
  double y1 = line.y;
  double x2 = line.x + cos(line.theta);
  double y2 = line.y + sin(line.theta);

  return (y1 - y2) * point.x - (x1 - x2) * point.y + (x1 * y2 - x2 * y1);
}

double Tracker::calculateCurvature(const std::vector<Pose> &traj,
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

  motionplanner::Point v1 = {B.x-A.x, B.y-A.y};
  motionplanner::Point v2 = {C.x-B.x, C.y-B.y};
  auto cross = v1.x*v2.y - v1.y*v2.x;
  if(cross < 0) curvature *= -1;

  return curvature;
}

// [begin_i, end_i) 区间上离小车距离最接近 preview_dist 的点
int Tracker::getNextId(const std::vector<Pose> &traj, const Pose &robot,
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

int Tracker::sgn(double x) { return (x >= 0) ? 1 : -1; }

}  // namespace motionplanner
