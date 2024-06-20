#ifndef Pf3Tracker_H_
#define Pf3Tracker_H_

#include <Eigen/Eigen>
#include <vector>

#include "common.h"



#include "track/controller/controller.h"
#include "track/otg_filter.h"
#include "track/tracker.h"

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
                               size_t *n_idx, bool backward = false) override;

  virtual State TrackStop(double &w) override {
    Init();
    return Tracker::TrackStop(w);
  }

  virtual void Init() override {
    state_ = RobotState::free;
    m_otg.reset();
    m_otg_lim.aMax = 0.33 * acc_;
    m_otg_lim.jMax = 0.33;
    vec_cmd.clear();
    m_begin = 0;
    pe = 0;
    pth_e = 0;
  }

  void SetMotionParam(double frequency, int p, double max_v,
                             double min_v, double max_w, double min_w,
                             double acc, double stop_acc, double alpha, double pos_accuracy,
                             double theta_accuracy);

private:

  // 预测下一时刻的位置
  Pose predict(const Pose& robot, double v, double angle);

private:
  ControlParam lqr_param_;
  RobotState state_;
  size_t m_begin;
  OtgFilter m_otg;

  std::vector<MoveCmd> vec_cmd;
  double pe, pth_e;



public:
  OtgFilter::LimitParam m_otg_lim;
};
} // namespace motionplanner

#endif // LQR_TRACKER_H_
