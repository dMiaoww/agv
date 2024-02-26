#include "move_base/controller/lqr_w.h"

#include "glog_set.h"

namespace motionplanner {

double LqrW::UpdateControl(const ControlStatus &status, const ControlParam &param) {
  Eigen::MatrixXd Q(3, 3);
  Eigen::MatrixXd R(1, 1);
  Q << 1, 0, 0, 
      0, 1, 0, 
      0, 0, 0.1;
  R << 0.1;

  Eigen::Vector3d xwan;
  // LOG(INFO) << "curr_pose(x, y, theta)" << status.curr_pose.x << "  "
  //           << status.curr_pose.y << "  " << status.curr_pose.theta;
  // LOG(INFO) << "target_pose(x, y, theta)" << status.target_pose.x << "  "
  //           << status.target_pose.y << "  " << status.target_pose.theta;

  xwan << status.curr_pose.x - status.target_pose.x,
      status.curr_pose.y - status.target_pose.y,
      shortest_angular_distance(status.target_pose.theta,
                                status.curr_pose.theta);

  Eigen::MatrixXd A, B, K;
  GetJacobi(status.target_pose.theta, status.target_vx, 0, param.t,
            A, B);
  K = Solve(A, B, Q, R, param.max_iteration);

  double res = status.target_vyaw - (K * xwan)(0, 0);
  LOG(INFO) << "xwan(x, y, theta)" << xwan[0] << " " << xwan[1] << " "
            << xwan[2] << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2);
  // LOG(INFO) << "res " << res.first << "  " << res.second;
  return res;
}


Eigen::MatrixXd LqrW::Solve(const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &B,
                                const Eigen::MatrixXd &Q,
                                const Eigen::MatrixXd &R,
                                const int32_t &max_iteration) {
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd P = Q, K;
  int32_t num_iteration = 0;
  double error_t = 1e-2;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_iteration && diff > error_t) {
    K = (BT * P * B + R).inverse() * BT * P * A;
    Eigen::MatrixXd P_next = AT * P * (A - B * K) + Q;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }
  return (BT * P * B + R).inverse() * BT * P * A;
}

void LqrW::GetJacobi(const double &yaw, const double &vx,
                         const double &vyaw, const double &t,
                         Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  A.resize(3, 3);
  B.resize(3, 1);
  A << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  B << 0, 0, 0;
  A(0, 2) = -vx * sin(yaw) * t;
  A(1, 2) = vx * cos(yaw) * t;
  
  B(2, 0) = t;
}

} // end namespace