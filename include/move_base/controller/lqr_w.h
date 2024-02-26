#ifndef LQRW_H
#define LQRW_H

#include "controller.h"


namespace motionplanner {

/*
可用于二轮差速、四轮差速底盘模型，vl = vx-w*L/2, vr =
vx-w*L/2，L是左右轮之间的距离 四轮差速中，前后轮速度严格相等
*/
class LqrW {
public:
  double UpdateControl(const ControlStatus &status,
                           const ControlParam &param);

private:
  Eigen::MatrixXd Solve(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                        const int32_t &max_iteration);

  void GetJacobi(const double &yaw, const double &vx, const double &vyaw,
                 const double &t, Eigen::MatrixXd &A, Eigen::MatrixXd &B);

}; // end class
} // namespace motionplanner

#endif