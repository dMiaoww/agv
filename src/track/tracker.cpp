#include "track/tracker.h"

#include "glog_set.h"

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
                      size_t end_i, Pose& robot, const MoveCmd& last, MoveCmd& now, size_t *next_i, bool is_backward) {
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

Tracker::State Tracker::TrackStop(double w0, double *w, double *v){
  double dw = alpha_ * dt_; *v = 0;
  if (w0 > dw) {
    *w = w0 - dw;
  } else if (w0 < -dw) {
    *w = w0 + dw;
  } else {
    *w = 0;
  }
  return Tracker::State::kTracking;
}

}  // namespace motionplanner