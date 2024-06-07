#ifndef Pf3Tracker_H_
#define Pf3Tracker_H_
 
#include <Eigen/Eigen>
#include <vector>
 
#include "global_variable/global_variable.h"
#include "move_base/common.h"
#include "move_base/controller/controller.h"
#include "move_base/otg_filter.h"
#include "move_base/tracker.h"
 
namespace motionplanner {
 
class Pf3Tracker : public Tracker {
public:
  enum class RobotState { free,  move };
 
public:
  Pf3Tracker();
  virtual ~Pf3Tracker();
 
  MoveCmd GetCmd(const std::vector<Pose> &traj, const double v_max, const Pose& robot,
                 const MoveCmd& last) override;
 
 
  // read param from config file track.yaml
  void SetAlgoParam() override;
 
  virtual Tracker::State Track(const std::vector<Pose> &traj,
                               const double v_max,
                               const std::vector<double> &traj_s,
                               size_t begin_i, size_t end_i, Pose &robot,
                               const MoveCmd& last, MoveCmd& now,
                               size_t *n_idx, bool is_backward) override;
 
  virtual State TrackStop(double w0, double *w, double *v) override {
    Init();
    return Tracker::TrackStop(w0, v, w);
  }
 
  virtual void Init() override {
    state_ = RobotState::free;
    m_otg.reset();
    m_otg_lim.aMax = 0.25 * alpha_;
    m_otg_lim.jMax = 0.33;
    v1 = 0;
    w1 = 0;
    v2 = 0;
    w2 = 0;
    m_begin = 0;
    e = 0;
    th_e = 0;
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
 
private:
  ControlParam lqr_param_;
  RobotState state_;
  size_t m_begin;
  OtgFilter m_otg;
 
  double v1, w1;
  double v2, w2;
  double e, th_e;
 
  // AGVTYPE agv_type_;
 
public:
  OtgFilter::LimitParam m_otg_lim;
};
} // namespace motionplanner
 
#endif // LQR_TRACKER_H_