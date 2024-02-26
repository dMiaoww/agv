#include "move_base/common.h"
#include "move_base/controller/lqr_nsteer.h"

#include "glog_set.h"
#include <cmath>
#include <tuple>
#include <utility>

namespace motionplanner {


  MoveCmd LqrNSteer::UpdateControl(const ControlStatus &status, const ControlParam &param) {
  Eigen::MatrixXd Q, R;
  Q.setIdentity(3,3);
  R.setIdentity(3,3);

  Eigen::MatrixXd A, B, K;
  GetJacobi(status, param, A, B);
  K = Solve(A, B, Q, R);

  Eigen::Vector3d ur;
  ur << status.target_vx, status.target_vy, status.target_vyaw;

  Eigen::Vector3d xwan;
  xwan << status.curr_pose.x - status.target_pose.x,
          status.curr_pose.y - status.target_pose.y,
          NormalizeRad(status.curr_pose.theta - status.target_pose.theta);
  Eigen::MatrixXd u = ur - (K * xwan);
  // LOG(INFO) << "xwan(x, y, theta)" << xwan[0] << " " << xwan[1] << " "
  //           << xwan[2] << " k:" << K(0,0) << " " << K(0,1) << " " << K(0,2);
  // return inverse_transform(u(0,0), u(1,0), u(2,0), param);
  return MoveCmd(u(0,0), u(1,0), u(2,0));
}


Eigen::MatrixXd LqrNSteer::Solve(const Eigen::MatrixXd &A,
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

void LqrNSteer::GetJacobi(const ControlStatus &status, const ControlParam &param,
                Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  double vx = status.target_vx;
  double vy = status.target_vy;
  double yaw = status.target_pose.theta;
  double t = param.t;
  
  A.setIdentity(3,3);
  A(0, 2) = -vx * sin(yaw) * t  -vy * cos(yaw) * t;
  A(1, 2) =  vx * cos(yaw) * t  -vy * sin(yaw) * t;
  
  B.resize(3, 3);
  B << cos(yaw)*t, -sin(yaw)*t, 0,
       sin(yaw)*t, cos(yaw)*t, 0,
       0, 0, t;
}

LqrNSteer::SteerCmd LqrNSteer::inverse_transform(const double vx, const double vy, const double w, const ControlParam &param) {
  return SteerCmd();
  // LqrNSteer::SteerCmd cmd;
  // cmd.a1 = atan2(vy + w * param.L, vx - w * param.D);
  // cmd.v1 = std::hypot(vy + w * param.L, vx - w * param.D);

  // cmd.a2 = atan2(vy + w * param.L, vx + w * param.D);
  // cmd.v2 = std::hypot(vy + w * param.L, vx + w * param.D);

  // cmd.a3 = atan2(vy - w * param.L, vx - w * param.D);
  // cmd.v3 = std::hypot(vy - w * param.L, vx - w * param.D);

  // cmd.a4 = atan2(vy - w * param.L, vx + w * param.D);
  // cmd.v4 = std::hypot(vy - w * param.L, vx + w * param.D);
  // return cmd;
}

}  // namespace motionplanner

