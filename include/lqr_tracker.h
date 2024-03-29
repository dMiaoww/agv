#ifndef LQRTRACKER_H_
#define LQRTRACKER_H_

#include <Eigen/Eigen>
#include <vector>

#include "common_data.h"
#include "otg_filter.h"

namespace motionplanner {

class LqrTracker {
public:
  enum class RobotState { free, rotate1, rotate2, move };
  enum State { kTracking = 0, kSuccessful, kFailed };

public:
  LqrTracker();
  ~LqrTracker();

  std::pair<double, double> GetCmd(const std::vector<Pose> &traj,
                                   const std::vector<double> &traj_v, double x,
                                   double y, double theta, double v, double w);

  std::pair<double, double> getRotateCmd(double curr_theta, double target_theta,
                                         double v, double w);

  // read param from config file track.yaml
  void SetAlgoParam();

  // 这里会对 vitrual_robot 的位置进行更新
  State Track(const std::vector<Pose> &traj, const double vmax,
              const std::vector<double> &traj_s, Pose& robot);  

  // State TrackStop(double w0, double *w, double *v) {
  //   Init();
  //   return TrackStop(w0, v, w);
  // }

  void Init() {
    state_ = RobotState::free;
    m_otg.reset();
    m_otg_lim.aMax = 0.1;
    m_otg_lim.jMax = 1;
    v1 = 0;
    w1 = 0;
    v2 = 0;
    w2 = 0;
    index = 0;
  }

private:
  // 返回距离的绝对值
  double getDistance(const Pose &p1, const Pose &p2);

  double getDot(const Pose &robot, const Pose &p2, const Pose &p3);

  // 找到轨迹上最靠近小车、且小车并未到达的点
  int getNearestId(const std::vector<Pose> &traj, const Pose &robot,
                   int begin_i, int end_i);

  // 计算 point 到 line 的距离 左正右负
  double getMinDistanceToLine(const Pose &point, const Pose &line);

  double calculateCurvature(const std::vector<Pose> &traj, const int &i);

  // [begin_i, end_i) 区间上离小车距离最接近 preview_dist 的点
  int getNextId(const std::vector<Pose> &traj, const Pose &robot, int begin_i,
                int end_i, double preview_dist);

  int sgn(double x);

  double angle_dis(double from, double to);

private:
  // ControlParam lqr_param_;
  RobotState state_;
  size_t index;
  OtgFilter m_otg;

  double v1, w1;
  double v2, w2;

  double acc_ = 0.1;
  double dt_ = 0.05;
  double max_w_ = 0.3;
  double alpha_ = 0.2;
  double min_w_ = 0.02;

  // AGVTYPE agv_type_;
public:
  OtgFilter::LimitParam m_otg_lim;
};
} // namespace motionplanner

#endif // LQR_TRACKER_H_