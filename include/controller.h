#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Eigen/Eigen>
#include <stdint.h>

#include "common_data.h"

namespace motionplanner{

 struct ControlStatus{
  Pose target_pose;
  Pose curr_pose;
  double curr_vx;
  double curr_vy;
  double curr_vyaw;
  double target_vx;
  double target_vy;
  double target_vyaw;
  double k; // 曲率 用于计算参考转角
};

struct ControlParam{
  double t;    // 控制周期，second 为单位
  int32_t max_iteration; //迭代次数
  double w_x;  // 权重，初始化 QR 矩阵
  double w_y;
  double w_yaw;
  double w_vx;
  double w_vyaw;
  double L; // 舵轮中的轴距；双舵轮时为一半
  double D; // 双舵轮中，舵轮的偏移量，右正左负（存疑）
};

}// end namespace motionplanner

#endif // CONTROLLER_H_