#ifndef LQRSTEER_H
#define LQRSTEER_H


#include "controller.h"


namespace motionplanner {

/*
单舵轮
*/
class LqrSteer {
public:
  // 返回转角
  double UpdateControl(const ControlStatus &status, const ControlParam &param);

  // 返回转角，使用横向误差和航向误差
  double UpdateControl2(const ControlStatus &status, const ControlParam &param,
                        double &pe, double &pth_e);

private:
  Eigen::MatrixXd Solve(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);

  void GetJacobi(const ControlStatus &status, const ControlParam &param,
                 double angle_r, Eigen::MatrixXd &A, Eigen::MatrixXd &B);

  Eigen::MatrixXd Motion2(const ControlStatus &status,
                          const ControlParam &param);
};
} // namespace motionplanner

#endif