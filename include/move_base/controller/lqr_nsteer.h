#ifndef LQRNSTEER_H
#define LQRNSTEER_H

#include "move_base/controller/controller.h"

namespace motionplanner {

/*
N舵轮：
    车头
舵轮1  舵轮2
舵轮3  舵轮4

*/
class LqrNSteer {
public:
  struct SteerCmd {
    double a1;
    double a2;
    double a3;
    double a4;
    double v1;
    double v2;
    double v3;
    double v4;
  };

public:
  // 对于右上、左下分布的舵轮，L 和 D 都是正的
  // 两种不同的计算参考速度的方式：平移+旋转、只有平移
  MoveCmd UpdateControl(const ControlStatus &status,
                         const ControlParam &param);

private:
  Eigen::MatrixXd Solve(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);

  void GetJacobi(const ControlStatus &status, const ControlParam &param,
                 Eigen::MatrixXd &A, Eigen::MatrixXd &B);

  SteerCmd inverse_transform(const double vx, const double vy, const double w,
                             const ControlParam &param);
};
} // namespace motionplanner

#endif