#include "common_data.h"
#include "track/controller/pure_pursuit.h"


#include "glog_set.h"
#include <cmath>
#include <tuple>
#include <iostream>

namespace motionplanner {




double PurePursuit::UpdateControl(const ControlStatus &status, const ControlParam &param) {
  double dx = status.target_pose.x - status.curr_pose.x;
  double dy = status.target_pose.y - status.curr_pose.y;
  double alpha = atan2(dy, dx) - status.curr_pose.theta;
  double delta = atan2(2*param.L*sin(alpha)/param.w_vx, 1.0);
  std::cout << alpha << " " << delta << std::endl; 
  return delta;
}




}  // namespace motionplanner
