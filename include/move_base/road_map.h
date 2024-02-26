#ifndef ROAD_MAP_H_
#define ROAD_MAP_H_
#include <map>
#include <mutex>

#include "global_variable/global_variable.h"
#include "move_base/common.h"

struct Node {
  Node() : parent(), g(0), h(0){};
  ~Node() {}
  double g;
  double h;
  double f() const { return (g + h); }
  std::map<std::string, Node>::iterator parent;
  motionplanner::Pose pos;
};

struct CompareNode {
  bool operator()(std::map<std::string, Node>::iterator parent_node,
                  std::map<std::string, Node>::iterator child_node) const {
    return (parent_node->second.f() > child_node->second.f());
  }
};

enum Dir {
  kHead = 0,
  kTail,
  kIgnore
};

class RoadMap {
 public:
  RoadMap();

  ~RoadMap();

  bool Init(int order = 3);

  double Heuristic(int sx, int sy, int gx, int gy) const {
    return std::hypot(gx - sx, gy - sy);
  }

  bool GetPath(const motionplanner::Pose &start,
               const motionplanner::Pose &goal,
               const std::vector<std::pair<std::string, std::string>> &discard,
               std::vector<motionplanner::Pose> *path,
               std::vector<std::vector<motionplanner::Point>> *path_ctrl);

  bool Search(const std::string &start, const std::string &goal,
              const std::vector<std::pair<std::string, std::string>> &discard);

  bool Backtrack(std::map<std::string, Node>::iterator it,
                 std::vector<motionplanner::Pose> *path,
                 std::vector<std::vector<motionplanner::Point>> *path_ctrl);

  motionplanner::Point DeCasteljau(std::vector<motionplanner::Point> &&point,
                                   double t) const;

  void GetBeizer(const std::vector<std::vector<motionplanner::Point>> &point,
                 std::vector<motionplanner::Pose> *bezier, double *s);

  bool GetPath(const std::vector<std::string> &node,
               std::vector<std::vector<motionplanner::Pose>> *path);

  bool Split(const std::vector<std::string> &node,
             std::vector<std::vector<std::string>> *seg_node,
             std::vector<Dir> *dir);

  bool GetCurve(const std::vector<std::string> &node,
                std::vector<motionplanner::Pose> *curve,
                std::vector<double> *s);

 private:
  std::map<std::string, Node> node_;
  std::map<std::string, std::map<std::string, std::pair<float, Dir>>>
      net_;
  std::map<std::string,
           std::map<std::string, std::vector<motionplanner::Point>>>
      curve_;
  std::vector<std::map<std::string, Node>::iterator> queue_;
  double start_dir_;
  double goal_dir_;
  float max_step_;  // m
};
#endif  // ROAD_MAP_H_