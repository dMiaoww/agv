#ifndef COMMON_H_
#define COMMON_H_

#include <chrono>
#include <cmath>
#include <string>

#include "glog_set.h"

namespace motionplanner {

class Elapsed {
public:
  Elapsed() : log_(), time_point_(std::chrono::steady_clock::now()) {}
  explicit Elapsed(std::string str) : log_(std::move(str)) {
    time_point_ = std::chrono::steady_clock::now();
  }
  // ms
  inline int64_t GetMilliseconds() {
    return (std::chrono::steady_clock::now() - time_point_).count() / 1000000;
  }
  // s
  inline int64_t GetSeconds() {
    return (std::chrono::steady_clock::now() - time_point_).count() /
           1000000000;
  }
  // s, time stamp
  static inline int64_t GetTimeStamp() {
    return std::chrono::steady_clock::now().time_since_epoch().count() /
           1000000000;
  }
  ~Elapsed() {
    // LOG(INFO) << log_ << " spend time :"
    //           << (std::chrono::steady_clock::now() - time_point_).count() /
    //                  1000000
    //           << "ms.";
    LOG(INFO) << log_ << " spend time :"
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - time_point_)
                     .count()
              << "ms.";
  }

  void Reset() { time_point_ = std::chrono::steady_clock::now(); }

private:
  std::string log_;
  std::chrono::steady_clock::time_point time_point_;
};

struct Point {
  inline Point() : x(0.0f), y(0.0f) {}

  inline Point(double x, double y) {
    this->x = x;
    this->y = y;
  }

  inline Point(const Point &p) {
    x = p.x;
    y = p.y;
  }

  Point &operator=(const Point &p) {
    if (this != &p) {
      this->x = p.x;
      this->y = p.y;
    }
    return *this;
  }

  Point operator+(const Point &p1) {
    Point p;
    p.x = x + p1.x;
    p.y = y + p1.y;

    return p;
  }

  Point operator-(const Point &p1) {
    Point p;
    p.x = x - p1.x;
    p.y = y - p1.y;

    return p;
  }

  Point operator*(double radio) {
    Point p;
    p.x = x * radio;
    p.y = y * radio;

    return p;
  }

  Point operator/(double radio) {
    Point p;
    p.x = x / radio;
    p.y = y / radio;

    return p;
  }

  double Norm() { return sqrt(x * x + y * y); }

  ~Point() = default;

  double x;
  double y;
};

// struct Pose {
//   inline Pose() : x(0.0f), y(0.0f), theta(0.0f) {}

//   inline Pose(double x, double y, double theta) {
//     this->x = x;
//     this->y = y;
//     this->theta = theta;
//   }

//   inline Pose(const Pose &p) {
//     x = p.x;
//     y = p.y;
//     theta = p.theta;
//   }

//   Pose &operator=(const Pose &p) {
//     if (this != &p) {
//       this->x = p.x;
//       this->y = p.y;
//       this->theta = p.theta;
//     }
//     return *this;
//   }

//   double Length() { return std::sqrt(x * x + y * y); }

//   ~Pose() = default;

//   double x;
//   double y;
//   double theta;
// };

struct MoveCmd {
  double vx;
  double vy;
  double w;

  MoveCmd(double a, double b, double c) : vx(a), vy(b), w(c) {}
  MoveCmd():vx(0), vy(0), w(0){}
};

double DiffRad(double a, double b);

double AbsDiffRad(double a, double b);

double NormalizeRad(double rad);

double shortest_angular_distance(double from, double to);

double normalize_angle_positive(double angle);

} // namespace motionplanner

#endif