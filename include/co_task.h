#pragma  once 

#include "common_data.h"
#include <cmath>
#include <glog/logging.h>
#include <utility>
#include <vector>
#include "global.h"


class CoTask {
public:
  void Init(const std::vector<std::pair<int, Pose>>& agvs){
    agvs_config = agvs;
  }

  // 传入位置，这里不计算角度，只计算 x y
  void CalcComponentPos(const Pose& center, std::vector<std::pair<int, Pose>>& agvs_res) {
    agvs_res.clear();
    for(const auto& it : agvs_config) {
      Pose pose;

      double sint = sin(center.theta);
      double cost = cos(center.theta);
      pose.x = center.x + it.second.x * (cost) + it.second.y * (-sint);  
      pose.y = center.y + it.second.x * (sint) + it.second.y * (cost);  
      pose.theta = center.theta;
      
      agvs_res.push_back(std::make_pair(it.first, pose));
    }
  }

  // 获得组件的id
  std::vector<int> getIds(){
    std::vector<int> ids;
    for(const auto it : agvs_config){
      ids.push_back(it.first);
    }
    return ids;
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
      double wx = agv_real.at(agv_id).pos.x;
      double wy = agv_real.at(agv_id).pos.y;
      double wt = agv_real.at(agv_id).pos.theta;

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