#include "track/common.h"
#include "track/controller/lqr_steer.h"


#include <cmath>
#include <tuple>
#include <iostream>

namespace motionplanner {

double LqrSteer::UpdateControl(const ControlStatus &status, const ControlParam &param) {
  Eigen::MatrixXd Q(3, 3);
  Eigen::MatrixXd R(1, 1);
  Q << param.w_x, 0, 0, 
      0, param.w_y, 0, 
      0, 0, param.w_yaw;
  R << param.w_vyaw;

  double angle_r = atan2(param.L * status.k , 1);

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
            << xwan[2] << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2) << " angle: " << u;
  return u;
}


double LqrSteer::UpdateControl2(const ControlStatus &status, const ControlParam &param, 
                      double &pe, double &pth_e) {

  double angle_r = atan2(param.L * status.k , 1);
  if(std::isnan(angle_r)) angle_r = 0;

  Eigen::MatrixXd K = Motion2(status, param);

  Eigen::Vector4d xwan;
  double dx = status.target_pose.x - status.curr_pose.x;
  double dy = status.target_pose.y - status.curr_pose.y;
  double e = std::hypot(dx, dy);
  double th_e = NormalizeRad(status.curr_pose.theta - status.target_pose.theta);
  // LOG(INFO) << "cu: " << status.curr_pose.theta << "tar: " << status.target_pose.theta;
  double temp_angle = NormalizeRad(status.target_pose.theta - atan2(dy, dx));
  if(temp_angle < 0)  e *= -1;
  
  if(pe == 0 && pth_e == 0) {
    pe = e; pth_e = th_e;
  }
  xwan << e, (e - pe) / param.t, th_e, (th_e - pth_e) / param.t;

  auto u = angle_r - NormalizeRad((K * xwan)(0,0));
  if(std::isnan(u)) u = 0;
  LOG(INFO) << "angle_ff: " << angle_r << " angle_fb: " << angle_r - u << " angle: " << u;
  pe = e;
  pth_e = th_e;  
  return u;
}


// Eigen::MatrixXd LqrSteer::Solve(const Eigen::MatrixXd &A,
//                                 const Eigen::MatrixXd &B,
//                                 const Eigen::MatrixXd &Q,
//                                 const Eigen::MatrixXd &R) {
//   Eigen::MatrixXd AT = A.transpose();
//   Eigen::MatrixXd BT = B.transpose();
//   Eigen::MatrixXd P = Q, K;
//   int32_t num_iteration = 0;
//   double error_t = 1e-2;
//   double diff = std::numeric_limits<double>::max();
//   while (num_iteration++ < 150 && diff > error_t) {
//     K = (BT * P * B + R).inverse() * BT * P * A;
//     Eigen::MatrixXd P_next = AT * P * (A - B * K) + Q;
//     diff = fabs((P_next - P).maxCoeff());
//     P = P_next;
//   }
//   return (BT * P * B + R).inverse() * BT * P * A;
// }


Eigen::MatrixXd LqrSteer::Solve(const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &B,
                                const Eigen::MatrixXd &Q,
                                const Eigen::MatrixXd &R) {
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd X = Q, Xn;
  int32_t num_iteration = 150;
  double eps = 1e-2;
  double diff = std::numeric_limits<double>::max();
  for(int i = 0; i < num_iteration; i++) {
    Xn = AT*X*A - AT*X*B*(R+BT*X*B).inverse()*BT*X*A+Q;
    diff = fabs((Xn - X).maxCoeff());
    if(diff < eps) break;
    X = Xn;
  }
  return (R+BT*X*B).inverse() * (BT * X * A);
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
  Q << 0.1, 0, 0, 0,
       0, 0.1, 0, 0,
       0, 0, 0.1, 0,
       0, 0 ,0, 0.1;
  // R << 1 + status.target_vx*2;
  R << 1;  

  Eigen::MatrixXd A, B;
  A.resize(4, 4);
  A << 1, param.t, 0, 0, 
      0, 0 ,status.target_vx ,0, 
      0 ,0 ,1 ,param.t,
      0 ,0 ,0 ,0;
  // std::cout << "A:\n" << A << "\n";
  
  B.resize(4, 1);
  B << 0 ,0 , 0, status.target_vx / param.L;
  // std::cout << "B:\n" << B<< "\n";
  
  Eigen::MatrixXd K = Solve(A, B, Q, R);
  // std::cout << "Q:\n" << Q<< "\n";
  // std::cout << "R\n" << R<< "\n";
  // std::cout << "K:\n" << K<< "\n";
  return K;
}


}  // namespace motionplanner
