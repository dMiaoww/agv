#include "move_base/common.h"
#include "move_base/controller/lqr_steer.h"
 
#include "glog_set.h"
#include <cmath>
#include <tuple>
 
namespace motionplanner {
 
double LqrSteer::UpdateControl(const ControlStatus &status, const ControlParam &param) {
  Eigen::MatrixXd Q(3, 3);
  Eigen::MatrixXd R(1, 1);
  Q << 1, 0, 0, 
      0, 1, 0, 
      0, 0, 0.1;
  R << 1;
 
  double angle_r = atan2(param.L * status.k , param.L);
 
  Eigen::MatrixXd A, B, K;
  GetJacobi(status, param, angle_r, A, B);
  K = Solve(A, B, Q, R);
 
  Eigen::Vector3d xwan;
  xwan << status.curr_pose.x - status.target_pose.x,
          status.curr_pose.y - status.target_pose.y,
          NormalizeRad(status.curr_pose.theta - status.target_pose.theta);
  auto u = angle_r - (K * xwan)(0,0);
  if(std::isnan(u)) u = 0;
  LOG(INFO) << "xwan(x, y, theta)" << xwan[0] << " " << xwan[1] << " "
            << xwan[2] << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2);
  
  return u;
}
 
 
double LqrSteer::UpdateControl2(const ControlStatus &status, const ControlParam &param, 
                      double &pe, double &pth_e) {
 
  double angle_r = atan2(param.L * status.k , param.L);
 
  Eigen::MatrixXd K = Motion2(status, param);
 
  Eigen::Vector4d xwan;
  double dx = status.target_pose.x - status.curr_pose.x;
  double dy = status.target_pose.y - status.curr_pose.y;
  double e = std::hypot(dx, dy);
  double th_e = NormalizeRad(status.curr_pose.theta - status.target_pose.theta);
  double temp_angle = NormalizeRad(status.target_pose.theta - atan2(dy, dx));
  if(temp_angle < 0)  e *= -1;
  
  xwan << e, (e - pe) / param.t, th_e, (th_e - pth_e) / param.t;
 
  auto u = angle_r - (K * xwan)(0,0);
  LOG(INFO) << "xwan(x, y, theta)" << xwan[0] << " " << xwan[1] << " "
            << xwan[2] << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2);
  pe = e;
  pth_e = th_e;  
  return u;
}
 
 
Eigen::MatrixXd LqrSteer::Solve(const Eigen::MatrixXd &A,
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
 
void LqrSteer::GetJacobi(const ControlStatus &status, const ControlParam &param,
                double angle_r, Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  double vx = status.target_vx;
  double yaw = status.target_pose.theta;
  double t = param.t;
  
  A.resize(3, 3);
  A << 1, 0, 0, 
       0, 1, 0, 
       0, 0, 1;
  A(0, 2) = -vx * sin(yaw) * t;
  A(1, 2) =  vx * cos(yaw) * t;
  
  B.resize(3, 1);
  B << 0, 0, vx * t / param.L * (1 + pow(tan(angle_r), 2));
}
 
Eigen::MatrixXd LqrSteer::Motion2(const ControlStatus &status, const ControlParam &param) {
  Eigen::MatrixXd Q(4, 4);
  Eigen::MatrixXd R(1, 1);
  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0 ,0, 1;
  R << 1;  
 
  Eigen::MatrixXd A, B;
  A.Zero(4, 4);
  A(0, 0) = 1.0;
  A(0, 1) = param.t;
  A(1, 2) = status.target_vx;
  A(2, 2) = 1.0;
  A(2, 3) = param.t;
 
  B.Zero(4, 1);
  B(3, 0) = status.target_vx / param.L;
 
  Eigen::MatrixXd K = Solve(A, B, Q, R);
  return K;
}
 
 
}  // namespace motionplanner