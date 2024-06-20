#ifndef TRACKER_H_
#define TRACKER_H_

#include <Eigen/Eigen>
#include <vector>

#include "track/common.h"
#include "common_data.h"

namespace motionplanner {

class Tracker {
 protected:
  double dt_;
  // prediction horizon
  int p_;
  double max_v_;
  double min_v_;
  double max_w_;
  double min_w_;
  double acc_;
  double stop_acc_;
  double alpha_;
  double pos_accuracy_;
  double theta_accuracy_;
  bool end_flag_;

  bool printlog_;

  void GetIndex(const std::vector<Pose> &traj, size_t begin_i, size_t end_i,
                const Pose &robot, size_t *next_i);

  // bool IsOverlap(const motionplanner::Pose &a, const motionplanner::Pose &b) {
  //   return std::hypot(a.x - b.x, a.y - b.y) < pos_accuracy_ &&
  //          std::abs(a.theta - b.theta);
  // }

  /// @brief
  /// @param traj reference traj
  /// @param v_max always > 0
  /// @param robot real robot pos
  /// @param last last control cmd
  /// @return MoveCmd
  virtual MoveCmd GetCmd(const std::vector<Pose> &traj, const double v_ref,
                                           const Pose& robot, const MoveCmd& last) = 0;

 public:
  Tracker(/* args */);
  virtual ~Tracker();
  enum State { kTracking = 0, kSuccessful, kFailed, kAllSuccessful};

  virtual void SetMotionParam(double frequency, int p, double max_v, double min_v,
                      double max_w, double min_w, double acc, double stop_acc,
                      double alpha, double pos_accuracy, double theta_accuracy);

  virtual void SetAlgoParam() = 0;

  virtual State Track(const std::vector<Pose> &traj, const double v_max,
                      const std::vector<double> &traj_s, size_t begin_i,
                      size_t end_i, Pose& robot, const MoveCmd& last, MoveCmd& now, size_t *next_i, bool backward = false);

  // 速度为 0 时将角速度慢慢下降为 0
  virtual State TrackStop(double &w);

  virtual void Init() { return; }

    // 返回距离的绝对值
  virtual double getDistance(const Pose &p1, const Pose &p2);

  virtual double getDot(const Pose &robot, const Pose &p2, const Pose &p3);

  // 找到轨迹上最靠近小车、且小车并未到达的点
  virtual int getNearestId(const std::vector<Pose> &traj, const Pose &robot,
                   int begin_i, int end_i);

  // 计算 point 到 line 的距离 左正右负
  virtual double getMinDistanceToLine(const Pose &point, const Pose &line);

  virtual double calculateCurvature(const std::vector<Pose> &traj, const int &i);

  // [begin_i, end_i) 区间上离小车距离最接近 preview_dist 的点
  virtual int getNextId(const std::vector<Pose> &traj, const Pose &robot, int begin_i,
                int end_i, double preview_dist);

  virtual int sgn(double x);
};
}  // namespace motionplanner

#endif  // TRACKER_H_
