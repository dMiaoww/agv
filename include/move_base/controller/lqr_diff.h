#ifndef LQRDIFF_H_
#define LQRDIFF_H_

#include <Eigen/Eigen>
#include <utility>
#include <stdint.h>

#include "controller.h"

namespace motionplanner{
class LqrDiff 
{
public:
  LqrDiff();
  ~LqrDiff();

  std::pair<double, double> UpdateControl(const ControlStatus &status, const ControlParam &param);

private:
  Eigen::MatrixXd Solve(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, 
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const int32_t &max_iteration);
  
  void GetJacobi(const double &yaw, const double &vx, const double &vyaw, const double &t, Eigen::MatrixXd &A, Eigen::MatrixXd &B);
};

}// end namespace motionplanner

#endif // LQR_DIFF_H_