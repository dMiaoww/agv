#include "lqr_w.h"

#include "glog_set.h"

namespace motionplanner {

double shortest_angular_distance(double from, double to) {
  double t = to - from;
  if (t > M_PI) {
    t -= 2 * M_PI;
  } else if (t < -M_PI) {
    t += 2 * M_PI;
  }
  return t;
}

double LqrW::UpdateControl(const ControlStatus &status, const double dt) {
  Eigen::MatrixXd Q(3, 3);
  Eigen::MatrixXd R(1, 1);
  Q << 20, 0, 0, 
      0, 20, 0, 
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
  GetJacobi(status.target_pose.theta, status.target_vx, 0, dt,
            A, B);
  K = Solve(A, B, Q, R);

  double res = status.target_vyaw - (K * xwan)(0, 0);
  std::cout << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2) << std::endl;
 
  // LOG(INFO) << "res " << res.first << "  " << res.second;
  return res;
}


Eigen::MatrixXd LqrW::Solve(const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &B,
                                const Eigen::MatrixXd &Q,
                                const Eigen::MatrixXd &R) {
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd P = Q, K;
  int32_t num_iteration = 0;
  double error_t = 1e-2;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < 100 && diff > error_t) {
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