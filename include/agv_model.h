#include "common_data.h"
#include "lqr_w.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

// 模拟 AGV 的运动情况
class AGV_Model {

public:
  AGV_Model(int id, double x, double y, double theta) {
    m_id = id;
    now.x = x, now.y = y, now.theta = theta;
    target_1 = now;
    v = 0, w = 0;
    m_run_thread =
        std::thread(std::bind(&AGV_Model::run, this)); // 根据 v w 更新坐标
  }

  void setTarget(Pose p) {
    // if(p.theta != target_1.theta){
    //   p.theta = atan2(p.y - target_1.y, p.x - target_1.x);
    // }
    
    // 根据相邻两次参考点计算速度
    double v_ref, w_ref;
    w_ref = (p.theta - now.theta) / t;
    v_ref = std::hypot((p.x - now.x) / t, (p.y - now.y) / t);

    motionplanner::ControlStatus status;
    status.curr_pose = now;
    status.target_pose = p;
    status.target_vx = v_ref;
    status.target_vyaw = w_ref;
    
    motionplanner::LqrW lqrw;
    double ww = lqrw.UpdateControl(status, 0.05);
    double vv = status.target_vx;

    double max_w = 0.6;
    double alpha = 1;
    double acc = 0.3;

    std::unique_lock<std::mutex> lock(m_mutex);
    this->v = vv;
    this->w = ww;
    target_1 = p;
    LOG(INFO) << m_id << " " << v;
  }

  void Stop(){
    this->v = 0;
    this->w = 0;
  }

  Pose getPose() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return now;
  }

private:
  void run() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(t * 1000)));
      std::unique_lock<std::mutex> lock(m_mutex);
      Pose nn;
      if (w == 0) {
        nn.x = now.x + v * t * cos(now.theta);
        nn.y = now.y + v * t * sin(now.theta);
        nn.theta = now.theta;
      } else {
        nn.theta = now.theta + w * t;
        nn.x = now.x + v / w * t * (sin(nn.theta) - sin(now.theta));
        nn.y = now.y - v / w * t * (cos(nn.theta) - cos(now.theta));
      }
      now = nn;
      LOG(INFO) << m_id << " " << now << " v: " << v;
    }
  }

private:
  double v;
  double w;
  Pose now;      // 当前的真实位姿
  Pose target_1; // 上一时刻的参考点
  double t = 0.05;
  int m_id;

  std::mutex m_mutex;
  std::thread m_run_thread; // 更新坐标
};