#pragma once
#include <cmath>
#pragma once

#include <math.h>
#include <vector>

#include "common_data.h"
#include "otg_filter.h"

class Tracker {
public:
  Tracker() {
    otg_limit.aMax = 0.3;
    otg_limit.vMax = 0.5;
    otg_limit.jMax = 1;
  }

  // 输入是路径和上一次的参考点索引，输出是当前的参考点索引和速度
  void GetTrackParam(const std::vector<Pose> &traj, const Pose robot,
                     int &index, double &v) {
    for (;index < traj.size(); ++index) {
      // 选合适的index
      if (fabs(angleB(robot, traj.at(index))) < M_PI_2) {
        break;
      } 
    }

    double dis_to_end =
        std::hypot(traj.back().x - robot.x, traj.back().y - robot.y);

    if (index == traj.size()) {
      if (dis_to_end < 0.05) {

        LOG(INFO) << "[success] Reach end.";
        v = 0;
        return;
      } else if (dis_to_end > 0.05 &&
                 fabs(angleB(robot, traj.back())) > M_PI_2) {
        LOG(ERROR) << "[fail] Over end.";
        v = 0;
        return;
      }
    }

    double a_1 = 0;
    double v_1 = 0;
    target_vel.d = dis_to_end;
    target_vel.j = 1;
    target_vel.a = 0;
    target_vel.v = 0;

    otg.runCycleS1(50, otg_limit, target_vel);
    v = otg.qk.v;
  }

private:
  double angleB(const Pose &p1, const Pose &p2) {
    double d = std::atan2(p2.y - p1.y, p2.x - p1.x) - p1.theta;
    if (d > M_PI * 2) {
      d -= M_PI * 2;
    }
    if (d < M_PI * 2) {
      d += M_PI * 2;
    }
    return d;
  }

private:
  motionplanner::OtgFilter otg;
  motionplanner::OtgFilter::LimitParam otg_limit;
  motionplanner::OtgFilter::VelParam target_vel;
};