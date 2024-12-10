#include "track/controller/lqr_w.h"
#include <iostream>
#include <math.h>
#include "curve.h"


using namespace motionplanner;
// using namespace Track;
double shortest_angular_distance(double from, double to) {
  double t = to - from;
  if (t > M_PI) {
    t -= 2 * M_PI;
  } else if (t < -M_PI) {
    t += 2 * M_PI;
  }
  return t;
}
// 距离的绝对值
double getDistance(const Pose &p1, const Pose &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}
double getDot(const Pose &robot, const Pose &p2, const Pose &p3) {
  Pose p21{robot.x - p2.x, robot.y - p2.y, 0};
  Pose p23{p3.x - p2.x, p3.y - p2.y, 0};

  double dot =
      (p21.x * p23.x + p21.y * p23.y) / (p23.x * p23.x + p23.y * p23.y + 1e-9);
  return dot;
}
// 找到轨迹上最靠近小车、且小车并未到达的点
int getNearestId(const std::vector<Pose> &traj, const Pose &robot,
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

int main() {
    Pose robot(-0.376885, -1.06056, 1.57);
    Pose target(-0.275, -0.55, 1.57);
    Pose p2((target.x+robot.x)/2, robot.y, 0);
    Pose p3(target.x, (target.y+robot.y)/2, 0);
    auto traj = BezierCurve::get(50, robot, p2, p3, target);

    while(fabs(robot.y-target.y) > 0.005) {

        // now_cmd.w = ;

    auto t = getNearestId(traj, robot, 1, traj.size());


        ControlStatus status;

  status.curr_pose = robot;
  status.target_pose = target;
  std::cout << "target "<< traj[t].x << " "<< traj[t].y << " "<< traj[t].theta << std::endl;

  status.target_vx = 0.1;
  status.target_vyaw = 0;

  status.curr_vx = 0.1; // v 为上一时刻的控制速度

      double dis = getDistance(traj[t], traj[t+1]) + 1e-6;
    status.target_vyaw =
       0;
  status.curr_vyaw = 0;


ControlParam lqr_param_;
lqr_param_.max_iteration = 100;
  lqr_param_.t = 0.05;
  lqr_param_.w_x = 1;
  lqr_param_.w_y = 1;
  lqr_param_.w_yaw = 0.05;
  lqr_param_.w_vx = 0.1;
//   lqr_param_.w_vyaw = 0.01;



        motionplanner::LqrW lqrw;
        double w = lqrw.UpdateControl(status, 0.1);
        if(w > 0.6) w = 0.6;
        if(w < -0.6) w = -0.6;

        // update pos
        robot.x += 0.1*0.05*cos(robot.theta);
        robot.y += 0.1*0.05*sin(robot.theta);
        robot.theta += w*0.05;
        if(robot.theta > M_PI) robot.theta -= 2*M_PI;

        std::cout << " "<< w << " "<< robot.x << " "<< robot.y << " "<< robot.theta << std::endl;
    }
        std::cout << robot.x << " "<< robot.y << " "<< robot.theta << std::endl;

}