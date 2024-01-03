#pragma  once 

#include "common_data.h"
#include <utility>
class CoTask {
public:
  void Init(const std::vector<std::pair<int, Pose>>& agvs){
    agvs_config = agvs;
  }

  // 传入位置
  void Update(const Pose& center, std::vector<std::pair<int, Pose>>& agvs_res) {
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


private:
  std::vector<std::pair<int, Pose>> agvs_config;
};