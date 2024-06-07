#ifndef OmniSteerTRACKER_H_
#define OmniSteerTRACKER_H_

#include <Eigen/Eigen>
#include <vector>

#include "global_variable/global_variable.h"
#include "common.h"
#include "controller.h"
#include "otg_filter.h"
#include "tracker.h"

namespace motionplanner {

class OmniSteerTracker : public Tracker {
public:
  enum class RobotState {
    free,
    rotate1,
    rotate2,
    move,
    setAngle // 设置舵轮角度
  };

public:
  OmniSteerTracker();
  virtual ~OmniSteerTracker();

  MoveCmd GetCmd(const std::vector<Pose> &traj, const double v_max,
                 const Pose &robot, const MoveCmd &last) override;

  // w1: 上一次的旋转角速度
  MoveCmd getRotateCmd(double curr_theta, double target_theta, double w1);

  void SetAlgoParam() override;

  virtual Tracker::State
  Track(const std::vector<Pose> &traj, const double v_max,
        const std::vector<double> &traj_s, size_t begin_i, size_t end_i,
        Pose &robot, const MoveCmd &last_cmd, MoveCmd &now_cmd, size_t *n_idx) override;

  virtual State TrackStop(double w0, double *w, double *v) override {
    Init();
    return Tracker::TrackStop(w0, w, v);
  }

  virtual void Init() override {
    state_ = RobotState::free;
    m_otg.reset();
    m_otg_lim.aMax = 0.33 * acc_;
    m_otg_lim.jMax = 0.33;
    cmd1 = MoveCmd(0, 0, 0);
    cmd2 = MoveCmd(0, 0, 0);
    m_begin = 0;
    v = 0;
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

  Pose predictPose(const Pose&robot, const MoveCmd& cmd, double dt);

private:
  ControlParam lqr_param_;
  RobotState state_;
  size_t m_begin;
  OtgFilter m_otg;
  double v;

  MoveCmd cmd1;
  MoveCmd cmd2;

  // AGVTYPE agv_type_;

public:
  OtgFilter::LimitParam m_otg_lim;
};
} // namespace motionplanner

#endif