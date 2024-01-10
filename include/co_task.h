#pragma  once 

#include "common_data.h"
#include <glog/logging.h>
#include <utility>
#include "global.h"


class CoTask {
public:
  void Init(const std::vector<std::pair<int, Pose>>& agvs){
    agvs_config = agvs;
  }

  // 传入位置
  void CalcComponentPos(const Pose& center, std::vector<std::pair<int, Pose>>& agvs_res) {
    agvs_res.clear();
    for(const auto& it : agvs_config) {
      Pose pose;

      // 根据 center 和相对坐标计算实际坐标。假设相对角度为0,先不考虑旋转，只平移
      double sint = sin(it.second.theta);
      double cost = cos(it.second.theta);
      pose.x = center.x + it.second.x * (-cost) + it.second.y * (sint);  
      pose.y = center.y + it.second.x * (sint) + it.second.y * (cost);  
      pose.theta = center.theta;

      agvs_res.push_back(std::make_pair(it.first, pose));
    }
  }

  // 根据组件的位置，计算虚拟大车的坐标
  Pose CalcVirtualCenter(){
    std::unordered_map<int, AGVstatus> agv_real;
    Global::get_agv_state(agv_real);
    int size = agvs_config.size();
    Pose p;
    for(int i = 0; i < size; ++i) {
      int agv_id = agvs_config[i].first;
      // 车体坐标系下的位置
      const double cx = agvs_config[i].second.x; 
      const double cy = agvs_config[i].second.y; 
      
      // 世界坐标系下的位置
      double wx = agv_real.at(agv_id).m_x;
      double wy = agv_real.at(agv_id).m_y;
      double wt = agv_real.at(agv_id).m_theta;

      // 世界坐标系下的偏移量
      double dx = cx * cos(wt) - cy * sin(wt);
      double dy = cx * sin(wt) + cy * cos(wt);
      
      p.x += (wx - dx) / size;
      p.y += (wy - dy) / size;
      p.theta += wt / size;
    }
    return p;
  }


private:
  std::vector<std::pair<int, Pose>> agvs_config;
};