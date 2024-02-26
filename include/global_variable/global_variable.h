#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "common_data.h"
//#include "amcl/pf/pf.h"
#include <atomic>
// #include "st_location_node/message.h"

#ifdef __cplusplus
extern "C" {
#endif
//声明全局变量
extern Pose global_pose_;                                 //全局坐标系下的位姿
extern std::mutex global_pose_mutex_;

extern std::vector<Pose> global_particlecloud_;           //粒子分布
extern std::mutex global_particlecloud_mutex_;

extern Car_status global_robot_status_;                   //agv的实时动态信息
extern std::vector <Car_status> global_robot_status_buff_;//缓存最近的agv动态信息
extern std::mutex global_status_mutex_;
extern std::mutex global_status_buff_mutex_;

extern map_t *global_map_;                                //地图数据 
//extern std::string global_map_file_name_;               //地图文件名
extern mapNameStru global_map_name_stru_;                 //地图文件名
extern std::mutex global_map_mutex_;                    //修改地图文件的锁

extern std::vector<laserData> global_laser_points_;       //激光点数据
extern laserScan global_laser_scan_;                      //激光点数据 距离形式
extern std::vector<laserData> global_base_laser_points_;  //激光点数据 基于车体坐标系
extern laserScan global_base_laser_scan_;                 //激光点数据 距离形式 基于车体坐标系
extern std::mutex global_laser_points_mutex_;

extern std::vector<laserData> global_cam_points_;
extern std::mutex global_cam_points_mutex_;

extern std::vector<laserData> global_laser12_points_[2];         //两个激光点数据 基于激光自身坐标系
extern std::mutex global_laser12_points_mutex_;

// extern std::unordered_map<std::string, sensor_msgs::LaserScan> laser_message_map_;   // 
extern std::mutex laser_message_map_mutex_;

// extern MapParser *global_map_parse_;                      //解析地图类
extern std::mutex global_map_parse_mutex_;                //解析地图类的锁

// extern agvConfigure global_agv_configure_;                //agv配置
extern std::mutex global_agv_configure_mutex_;

extern std::mutex global_agv_work_mode_mutex_;
extern SetModeCommand::agvWorkModeEnum global_agv_work_mode_;                //agv的工作模式

extern std::vector<laserData> global_test_points_;        //测试用的激光点数据
extern std::mutex global_test_points_mutex_;

extern std::vector<laserData> global_test_points1_;       //测试用的激光点数据
extern std::mutex global_test_points_mutex1_;

extern std::vector<Point> global_test_line_points;        //需要显示的直线
extern std::mutex global_test_line_points_mutex_;

extern std::vector<Point> global_path_line_points;        //需要显示的规划好的全局路径
extern std::mutex global_path_line_points_mutex_;

extern std::vector<Point> global_obstacles_line_points;        //需要显示的障碍物
extern std::mutex global_obstacles_line_points_mutex_;

extern std::vector<Point> global_cost_map_rect_points;        //需要显示的局部地图矩形
extern std::mutex global_cost_map_rect_points_mutex_;

extern std::vector<CostPoint> global_cost_map_points;           //需要显示的局部地图代价
extern std::mutex global_cost_map_points_mutex_;

extern std::vector<AgvNeighberInfo> global_agv_neighbers_info;           //周围其他车辆信息
extern std::mutex global_agv_neighbers_info_mutex_;

//extern pf_sample_set_t* global_partical_set;                  //定位的粒子

extern safeRegionGroup global_region_group_[12];                //如果激光也做安全激光，这里面存储的就是安全激光区域的边界点

extern std::atomic_bool soft_emergency_stop_;                   //软急停

extern std::atomic_bool local_trajectory_builder_failed_;       //

#ifdef __cplusplus
}
#endif
#endif
