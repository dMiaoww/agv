#ifndef COMMON_DATA_H
#define COMMON_DATA_H

#include <iostream>
#include <signal.h>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <cmath>
#include <climits>
#include <cstring>

#include <math.h>
#include <cmath>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <set>




#include "glog_set.h"
// #include "configurationStruct.h"

#include "MsgStrctAgv.h"
#include "MsgStrctCC.h"
#include "Eigen/Core"
// #include "ceres/ceres.h"

#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>

using namespace MSG_CC;
using namespace MSG_AGV;

#ifdef __cplusplus
extern "C" {
#endif

#define LASER_NUMBER 2

// #define byte unsigned char
#define MAGNETIC_IO_NUMBER   16

enum class MessageSource
{
  _MessageFromCC = 0x00,
  _MessageFromDeploy = 0x01,
  _MessageFromMes = 0x02,
  _MessageFromAGV = 0x03,
  _MessageFromMQTT = 0x04,
  _MessageFromUnknow = 0x05
};

struct Job_message
{
  BaseStruct* message;        //消息内容
  MessageSource source;       //0：从CC发过来的消息  1：从deploy发过来的消息  2：从MES发过来的消息
};

//点信息
struct Point
{
public: 
    inline Point() : x(0.0), y(0.0), curvature(0.0f) {}
    inline Point(double x, double y, double curvature = 0)
    {
      this->x = x;
      this->y = y;
      this->curvature = curvature;
    }
    inline Point(const Point& p)
    {
      x = p.x;
      y = p.y;
      curvature = p.curvature;
    }

    Point& operator=(const Point& p)
    {
      if (this != &p)
      {
        this->x = p.x;
        this->y = p.y;
        this->curvature = p.curvature;
      }
      return *this;
    }

public: 
    double x;
    double y;
    double curvature;

};

struct CostPoint
{
public: 
    inline CostPoint() : x(0.0), y(0.0), value(0) {}
    inline CostPoint(double x, double y, char value)
    {
      this->x = x;
      this->y = y;
      this->value = value;
    }
    inline CostPoint(double x, double y)
    {
      this->x = x;
      this->y = y;
      this->value = 0.0;
    }
    inline CostPoint(const CostPoint& p)
    {
      x = p.x;
      y = p.y;
      value = p.value;
    }

    CostPoint& operator=(const CostPoint& p)
    {
      if (this != &p)
      {
        this->x = p.x;
        this->y = p.y;
        this->value = p.value;
      }
      return *this;
    }

public: 
    double x;
    double y;
    char value;
};

struct AgvNeighberInfo
{
public: 
    inline AgvNeighberInfo() : x(0.0), y(0.0), theta(0.0f) {}
    inline AgvNeighberInfo(double x, double y, double theta)
    {
      this->x = x;
      this->y = y;
      this->theta = theta;
    }
    inline AgvNeighberInfo(const AgvNeighberInfo& p)
    {
      x = p.x;
      y = p.y;
      theta = p.theta;

      footprints.reserve(p.footprints.size());
      for(int i = 0; i < p.footprints.size(); ++i)
      {
        footprints.emplace_back(p.footprints[i]);
      }
      circlePrints.reserve(p.circlePrints.size());
      for(int i = 0; i < p.circlePrints.size(); ++i)
      {
        circlePrints.emplace_back(p.circlePrints[i]);
      }
    }

    AgvNeighberInfo &operator=(const AgvNeighberInfo &p)
    {
      if (this != &p)
      {
        this->x = p.x;
        this->y = p.y;
        this->theta = p.theta;
        footprints.reserve(p.footprints.size());
        for (int i = 0; i < p.footprints.size(); i++)
        {
          this->footprints.emplace_back(p.footprints[i]);
        }
        circlePrints.reserve(p.circlePrints.size());
        for (int i = 0; i < p.circlePrints.size(); i++)
        {
          this->circlePrints.emplace_back(p.circlePrints[i]);
        }
      }
      return *this;
    }

  public: 
    double x;
    double y;
    double theta;

    std::vector<Point> footprints;
    std::vector<Point> circlePrints;
};

struct Polygon
{
public: 
    Polygon(){}
    ~Polygon(){}

    inline Polygon(const Polygon& poly)
    {
      for(int i = 0; i < poly.points.size(); i++)
          points.push_back(poly.points[i]);
    }

    Polygon& operator=(const Polygon& poly)
    {
      if (this != &poly)  
      {
        for (int i = 0; i < poly.points.size(); i++)
            this->points.push_back(poly.points[i]);
      }
      return *this;
    }

public: 
    std::vector<Point> points;
};

//位姿信息
struct Pose
{
public: 
    inline Pose() : x(0.0), y(0.0), theta(0.0) {}
    inline Pose(double x, double y, double theta) 
    {
      this->x = x;
      this->y = y;
      this->theta = theta;
    }
    inline Pose(const Pose& pose)
    {
      x = pose.x;
      y = pose.y;
      theta = pose.theta;
    } 

    Pose& operator=(const Pose& pose)
    {
      if (this != &pose)  
      {
        this->x = pose.x;
        this->y = pose.y;
        this->theta = pose.theta;
      }
      return *this;
    }

public: 
    double x;
    double y;
	  double theta;
};

inline std::ostream& operator<<(std::ostream& os, const Pose& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.theta << ")";
}

struct VetexNode // 结点位置
{
    int Id;
    double x;
    double y;
    double theta;
    std::string tagname;

    VetexNode(): Id(0), x(0.0), y(0.0), theta(0.0),tagname("") { }

    VetexNode(int id, double xx, double yy, double Theta, std::string Tagname):
      Id(id), x(xx), y(yy), theta(Theta),tagname(Tagname) { }

    inline VetexNode(const VetexNode& node) {
      Id = node.Id;
      x = node.x;
      y = node.y;
      theta = node.theta;
      tagname = node.tagname;
    } 

    VetexNode& operator=(const VetexNode& node) {
      if (this != &node)  
      {
        this->Id = node.Id;
        this->x  = node.x;
        this->y  = node.y;
        this->theta = node.theta;
        this->tagname = node.tagname;
      }
      return *this;
    }
};

struct VetexNode_property
{
  int num_in_path;
  double bezier_d1;
  int bezier_d1_num;
  double bezier_d2;
  int bezier_d2_num;
  double delta_angle;
};

struct Path_property
{
public:
  double max_v;
  double max_acc;

  // 占用的资源
  std::vector<std::string> area_name;     // 所要占用的限制区域
  std::string goal_name;                  // 所要占用的目标点

  // 特殊功能区域
  std::string door_name;    // 门区域名称，需要降低速度
  char ignore_SafeLaser;    // 是否关闭安全激光，0：不关闭    1：关闭
  double allow_speed;       // 允许的最大速度，0：表示不在限制区域
  bool forbiddenAvoid;

public:
  Path_property()
  {
    max_v = 1.5;
    max_acc = 1.0;
    //area_name = "";
    door_name = "";
    ignore_SafeLaser = 0;
    allow_speed = 0;
    forbiddenAvoid = false;
  }
  inline Path_property(const Path_property &p)
  {
    max_v = p.max_v;
    max_acc = p.max_acc;
    area_name = p.area_name;
    door_name = p.door_name;
    ignore_SafeLaser = p.ignore_SafeLaser;
    allow_speed = p.allow_speed;
    forbiddenAvoid = p.forbiddenAvoid;
  }

  Path_property &operator=(const Path_property &p)
  {
    if (this != &p)
    {
      this->max_v = p.max_v;
      this->max_acc = p.max_acc;
      this->area_name = p.area_name;
      this->door_name = p.door_name;
      this->ignore_SafeLaser = p.ignore_SafeLaser;
      this->allow_speed = p.allow_speed;
      this->forbiddenAvoid = p.forbiddenAvoid;
    }
    return *this;
  }
};



struct PathPoint
{
  double x;
  double y;
  double theta;
  double curvature;    //曲率
  double v_max;        //最大速度

  PathPoint()
  {
      x = 0.0;
      y = 0.0;
      theta = 0.0;
      curvature = 0.0;
  }

  PathPoint(const PathPoint& point)
  {
      x = point.x;
      y = point.y;
      theta = point.theta;
      curvature = point.curvature;
      v_max = point.v_max;
  }
  
  PathPoint(VetexNode& node)
  {
      x = node.x;
      y = node.y;
      theta = node.theta;
      curvature = 0.0;
  }

  PathPoint& operator=(const PathPoint& node)
  {
      x = node.x;
      y = node.y;
      theta = node.theta;
      curvature = node.curvature;
      v_max = node.v_max;
      return *this;
  }
};

//激光传感器配置
struct SickTimConfig
{  
  std::string hostname;
  std::string  port;
  int timelimit;
  int device_number;
  double range_min;
  double range_max;
  double time_increment;
  double min_ang;
  double max_ang;
  bool intensity;
  int skip;
  double time_offset;
  bool auto_reboot;
  //SickTimConfig()
  //{
  //  memset(this,0,sizeof(SickTimConfig)); 
  //}
};

//一帧激光数据
struct laserScan
{
  int64_t time_stamp;
  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;
  std::vector<double> ranges;
  //std::vector<double> angle_ranges;
  std::vector<int>    intensities;
  //laserScan()
  //{
  //  memset(this,0,sizeof(laserScan)); 
  //}
};

//一个激光点的信息
struct laserData
{
  double x;
  double y;
  double intensity;
};

// Description for a single map cell.
struct map_cell_t
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;
  //map_cell_t()
  //{
  //  memset(this,0,sizeof(map_cell_t)); 
  //}
};

// Description for a map
struct map_t
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;  
  // Map scale (m/cell)
  double scale;
  // Map dimensions (number of cells)
  size_t size_x, size_y;  
  // The map data, stored as a grid
  map_cell_t *cells;
  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;  
  //laser point number;
  size_t point_num;           //障碍点的个数
  Point *occ_points;          //存储所有的障碍点的坐标
  //std::string map_file_name;  //地图文件的名字
  //map_t()
  //{
  //  memset(this,0,sizeof(map_t)); 
  //}
};

//agv的车辆状态
struct Car_status
{
  int64_t odom_time_stamp;
  double w1;
  double w2;
  double w3;
  double w4;
  double vx;  //x方向速度
  double vy;  //y方向速度
  double w;   //角速度
  double x;   //odom坐标系下的位姿
  double y;
  double theta;
  double battery_voltage;          //电池电压
  double battery_current;          //电池电流
  double battery_level;            //电池电量
  bool is_connected;               //是否与充电桩连接成功
  int battery_state;               //充电/放电状态                     0x4F为充电完成  1    0x43为充电进行中  2  0x00为放电状态   3
  bool door_opened;                //门是否打开
  bool allow_leave;                //是否可以离开
  bool laser_state;                //激光的连接状态
  AgvWorkMode agv_work_mode;       //agv的工作模式：手动、半自动、全自动
  slamSwitch slam_switch;          //打开或关闭slam模式

  unsigned char io_data[TCP_IO_NUMBER_CC];       //IO数据
  unsigned char meg_data[MAGNETIC_IO_NUMBER];
  double analog_data[TCP_ANALOG_NUMBER_CC];      //模拟量
  bool emergencyStop_data;                       //急停按钮
  bool safeLaserOpen;
  bool bumperStrip_data;                         //防撞条 
  bool payload;                                  //载货状态   

  typedef enum
  {
    SAFE_Free = 0,
    SAFE_Warning2,
    SAFE_Warning1,
    SAFE_Stop,
    } StateSAFE;
  StateSAFE state_safe;

  typedef enum
  {
    SPEED_Fast = 1,
    SPEED_Medium,
    SPEED_Slow,
    SPEED_Creep,
    SPEED_Back,
    SPEED_Rotate,
    SPEED_Left,
    SPEED_Right,
    SPEED_CLOSE,
    SPEED_VVFast,
    SPEED_VFast,
    SPEED_Door
    } StateSpeed;
  StateSpeed state_speed;

  enum class EnumChargerState {
    Free, Charging, CommError, InnerError
  };

  enum class EnumMotorState{
    Normal,  CommError, InnerError
  };

  enum class EnumBatteryState {
    Chargefinish = 1,
    Charging = 2,
    Discharge = 3,
    CommError,    // 通信异常
    InnerError,  // 电池内部故障
  };

  Car_status()
  {
    w1 = w2 = w3 = w4 = vx = vy = w = x = y = theta = battery_voltage = battery_current = battery_level = 0.0;
    is_connected = false;
    battery_state = 3;
    agv_work_mode = AgvWorkMode::_ManualControlMode;
    slam_switch = slamSwitch::_OpenSlamSwitchEnum_off;
    memset(io_data, '\0', TCP_IO_NUMBER_CC);
    memset(meg_data, '\0', MAGNETIC_IO_NUMBER);
    memset(analog_data, '\0', TCP_ANALOG_NUMBER_CC * sizeof(double));
    emergencyStop_data = false;
    bumperStrip_data = false;
    safeLaserOpen = false;
    door_opened = false;
    allow_leave = true;
    laser_state = true;
    state_safe = SAFE_Free;
    state_speed = SPEED_Creep;
    payload = false;
  }

  Car_status &operator=(const Car_status &p)
  {
    this->odom_time_stamp = p.odom_time_stamp;
    this->w1 = p.w1;
    this->w2 = p.w2;
    this->w3 = p.w3;
    this->w4 = p.w4;
    this->vx = p.vx;
    this->vy = p.vy;

    this->w = p.w;
    this->x = p.x;
    this->y = p.y;
    this->theta = p.theta;
    this->battery_voltage = p.battery_voltage;
    this->battery_current = p.battery_current;
    this->battery_level = p.battery_level;
    this->is_connected = p.is_connected;
    this->battery_state = p.battery_state;
    this->door_opened = p.door_opened;
    this->allow_leave = p.allow_leave;

    this->emergencyStop_data = p.emergencyStop_data;
    this->safeLaserOpen = p.safeLaserOpen;
    this->bumperStrip_data = p.bumperStrip_data;
    this->laser_state = p.laser_state;
    this->agv_work_mode = p.agv_work_mode;
    this->slam_switch = p.slam_switch;

    this->state_safe = p.state_safe;
    this->state_speed = p.state_speed;
    this->payload = p.payload;

    memcpy(this->io_data, p.io_data, TCP_IO_NUMBER_CC * sizeof(unsigned char));
    memcpy(this->meg_data, p.meg_data, MAGNETIC_IO_NUMBER * sizeof(unsigned char));
    memcpy(this->analog_data, p.analog_data, TCP_ANALOG_NUMBER_CC * sizeof(double));
    return *this;
  }

  Car_status(const Car_status& p) {
      odom_time_stamp = p.odom_time_stamp;
			w1 = p.w1;
      w2 = p.w2;
      w3 = p.w3;
      w4 = p.w4;
      vx = p.vx;
      vy = p.vy;

      w = p.w;
      x = p.x;
      y = p.y;
      theta = p.theta;
      battery_voltage = p.battery_voltage;
      battery_current = p.battery_current;
      battery_level = p.battery_level;
      is_connected = p.is_connected;
      battery_state = p.battery_state;
      door_opened = p.door_opened;
      allow_leave = p.allow_leave;
      laser_state = p.laser_state;
      agv_work_mode = p.agv_work_mode;
      slam_switch = p.slam_switch;

      emergencyStop_data = p.emergencyStop_data;
      safeLaserOpen = p.safeLaserOpen;
      bumperStrip_data = p.bumperStrip_data;
      state_safe = p.state_safe;
      state_speed = p.state_speed;
      payload = p.payload;

      memcpy(io_data, p.io_data, TCP_IO_NUMBER_CC * sizeof(unsigned char));
      memcpy(meg_data, p.meg_data, MAGNETIC_IO_NUMBER * sizeof(unsigned char));
      memcpy(analog_data, p.analog_data, TCP_ANALOG_NUMBER_CC * sizeof(double));
		}

};

struct Twist
{
  double linear_x;  
  double linear_y; 
  double linear_z; 
  double angular_x;
  double angular_y;
  double angular_z;

  inline Twist()
  {
    linear_x = 0.0;
    linear_y = 0.0;
    linear_z = 0.0;
    angular_x = 0.0;
    angular_y = 0.0;
    angular_z = 0.0;
  }

  inline Twist(const Twist &p)
  {
    linear_x = p.linear_x;
    linear_y = p.linear_y;
    linear_z = p.linear_z;
    angular_x = p.angular_x;
    angular_y = p.angular_y;
    angular_z = p.angular_z;
  }

  Twist &operator=(const Twist &p)
  {
    if (this != &p)
    {
      this->linear_x = p.linear_x;
      this->linear_y = p.linear_y;
      this->linear_z = p.linear_z;
      this->angular_x = p.angular_x;
      this->angular_y = p.angular_y;
      this->angular_z = p.angular_z;
    }
    return *this;
  }

};

struct MagneticMsg
{
  size_t forward_back;	//0:前进    1:后退
  size_t tryCount;
  //MagneticMsg()
  //{
  //  memset(this,0,sizeof(MagneticMsg)); 
  //}
};

struct TemplateGuideMsg
{
  //模板信息
  double L[100], d[2], m[2], D, theta;

  //agv信息
  double agvWidth, wheelCenter2Front, virtualReaderX, virtualReaderY;
  double lidarX[100];
  double lidarY[100];
  double lidarTheta[100];
  bool inverse[100];
  double allowedLaserMinAngle[100];
  double allowedLaserMaxAngle[100];
  int lidarNum;
  bool templateOnLeft;
  bool reflectorOnLeft;
  std::string templateType;

  //运动控制
  double maxRotateYawRate;//纯旋转最大角速度
  double rotateAngleIncAcc;//原地旋转角加速度
  double rotateAngleDecAcc;//原地旋转角减速度

  double maxYawrate;//最大角速度
  double maxSpeed;//最大速度
  double speedIncAcc;//最大加速度
  double speedDecAcc;//最大减速度
  double angleIncAcc;//跟踪直线时最大角加速度
  double angleDecAcc;//跟踪直线时最大角减速度

  //连续1字型模板
  int stationNum;
  int destStation;//从1开始计算
};

struct IpPortText
{
  IpPortText() = default;
  IpPortText(std::string Ip, int Port, std::string Send_text, std::string Response_text, int Time_out): 
    ip(Ip), port(Port), SendText(Send_text), ResponseText(Response_text), TimeOut(Time_out) {}
  std::string ip;
  int port;
  std::string SendText;
  std::string ResponseText;
  int TimeOut;
  //IpPortText()
  //{
  //  memset(this,0,sizeof(IpPortText)); 
  //}
};

struct AccurateDrive
{
  int control_type;
  int forward_back;  //0前进
  std::string goalName;
  int taskType;
  //AccurateDrive()
  //{
  //  memset(this,0,sizeof(AccurateDrive)); 
  //}
};



struct timeSetMsg
{
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
};

struct SetIoMsg
{
  size_t index;   // 第几个IO
  size_t value;
  //SetIoMsg()
  //{
  //  memset(this,0,sizeof(SetIoMsg)); 
  //}
};

struct mapNameStru
{
  std::string mapName;
  std::string directory;
  std::string beforeMap;
  std::string afterMap;
  int floorNum;
  //mapNameStru()
  //{
  //  memset(this,0,sizeof(mapNameStru)); 
  //}
};

struct mapPose
{
  std::string mapName;
  std::string locationGoal;
  //mapPose()
  //{
  //  memset(this,0,sizeof(mapPose)); 
  //}
};

struct anomalyText
{
  int error_type;
  int suberror_type;
  size_t interrupt;
  std::string workplace_name;
  std::string subTask_name;
  //anomalyText()
  //{
  //  memset(this,0,sizeof(anomalyText)); 
  //}
};

namespace ErrorData
{
  //搜集错误的结构体
  struct Error_unit
  {
    int module_type;
    int error_grade;
    int error_type;        
    int error_value;
    std::string workplace_name;
    std::string subTask_name;
  };
}

struct LightType
{
  enum LightTypeEnum {
    Empty      = 0x00,           //初始化
    Free       = 0x01,           //空闲---白灯闪烁
    Move       = 0x02,           //行走中---绿色
    TurnLeft   = 0x03,           //左转---左侧黄灯闪烁
    TurnRight  = 0x04,           //右转---右侧黄灯闪烁
    Waiting    = 0x05,           //等待---黄，双闪
    Subtasking = 0x06,           //子任务中---蓝灯闪烁
    Charging   = 0x07,           //充电中---绿灯闪烁
    SafeClosed = 0x08,           //安全激光关闭---红黄交替
    Stop       = 0x09,           //短时间停障碍或急停---红色
    StopLong   = 0x0A,           //长时间急障碍---红色快闪
    Errored    = 0x0B            //故障---红灯慢闪
  };

  inline bool operator==(const LightType &light_type)
  {
      return (m_light_type == light_type.m_light_type);
  }

  LightType& operator=(const LightType &light_type)
  {
    if (this != &light_type)
    {
      this->m_light_type = light_type.m_light_type;
    }
    return *this;
  }

  unsigned char m_light_type;
};

//                    stop         warning2      warning1
//区域1：直行-高速 <区域组1防区1> <区域组1防区2> <区域组1防区3> 1 1
//区域2：直行-中速 <区域组2防区1> <区域组2防区2> <区域组2防区3> 1 2
//区域3：直行-低速 <区域组3防区1> <区域组3防区2> <区域组3防区3> 1 3

//区域4：左转-高速 <区域组4防区1> <区域组4防区2> <区域组4防区3> 2 1
//区域5：左转-中速 <区域组5防区1> <区域组5防区2> <区域组5防区3> 2 2
//区域6：左转-低速 <区域组6防区1> <区域组6防区2> <区域组6防区3> 2 3

//区域7：右转-高速 <区域组7防区1> <区域组7防区2> <区域组7防区3> 3 1
//区域8：右转-中速 <区域组8防区1> <区域组8防区2> <区域组8防区3> 3 2
//区域9：右转-低速 <区域组9防区1> <区域组9防区2> <区域组9防区3> 3 3

//区域10：原地旋转-高速 <区域组10防区1> <区域组10防区2> <区域组10防区3> 4 1
//区域11：原地旋转-中速 <区域组11防区1> <区域组11防区2> <区域组11防区3> 4 2
//区域12：原地旋转-低速 <区域组12防区1> <区域组12防区2> <区域组12防区3> 4 3

struct safeRegionGroup
{
  std::vector<Point> warning1_sector;
  std::vector<Point> warning2_sector;
  std::vector<Point> stop_sector;
};

#ifdef __cplusplus
}
#endif

// 定時器
class StCTimer {
public:
	template<class F>
	StCTimer(F func) : m_func(func) {
    m_bexit.store(false);
		m_bimmediately_run.store(false);
  } 

	virtual ~StCTimer() {}

	// 启动函数
	void start(unsigned int imsec, bool bimmediately_run = false) {

		// 间隔时间为0或默认无效值，直接返回
		if (imsec == 0 || imsec == static_cast<unsigned int>(-1)) {
			return;
		}

		m_bexit.store(false);
		m_imsec = imsec;
		m_bimmediately_run.store(bimmediately_run);
		//m_thread = std::thread(std::bind(&StCTimer::Run, this));
		m_thread = std::thread(&StCTimer::Run, this);
	}

	// 结束
	void stop() {
		m_bexit.store(true);
		// 唤醒线程
		m_cond.notify_all();	

		if (m_thread.joinable()) {
			m_thread.join();
		}
	}

	void setExit(bool b_exit) {
		m_bexit.store(b_exit);
	}
private:
	void Run() {
		// 立即执行判断
		if (m_bimmediately_run.load()) {
			if (m_func) {
				m_func();
			}
		}

		while (!m_bexit.load()) {
			{
				// 锁放在花括号内部，减小锁的粒度
				std::unique_lock<std::mutex> locker(m_mutex);

				// 如果是被唤醒的，需要判断是不是虚假唤醒
				// wait_for是等待第三个参数满足条件，当不满足时，超时后继续往下执行
				m_cond.wait_for(locker, std::chrono::milliseconds(m_imsec), 
						[this]() { return m_bexit.load();});
			}

			// 再次判断退出条件
			if (m_bexit.load()) {
				return;
			}
      
			// 函数执行
			if (m_func) {
				m_func();
			}
		}
	}

private:
	// 私有数据部分

	// 退出标识，标志是否退出定时线程循环
	std::atomic_bool m_bexit;
	// 是否立即执行标识，标识新建的定时线程是否立即执行一次任务，而不需要等待一个间隔时间才开始执行第一次任务
	std::atomic_bool m_bimmediately_run;	
	// 间隔时间
	unsigned int m_imsec = 1000;					
	// 执行函数，具体的定时执行业务操作
	std::function<void()> m_func;
	// 线程变量
	std::thread m_thread;
	// 互斥锁
	std::mutex m_mutex;
	// 条件变量，结合互斥锁，在线程不执行任务时，睡眠一段时间，在退出调用时，可以唤醒线程完成退出
	std::condition_variable m_cond;					
};

inline double smoothVelocity(double current_v, double last_v, double acceleration, double deceleration) {
	double fabs_current_v = fabs(current_v);
  double fabs_last_v =  fabs(last_v);
  double acc = acceleration;

  if (fabs_current_v < fabs_last_v) {
    acc = deceleration;
  }

  if (fabs_current_v > fabs_last_v + acc) {
    fabs_current_v = fabs_last_v + acc;
  }

  if (fabs_current_v < fabs_last_v - acc) {
    fabs_current_v = fabs_last_v - acc;
  }

  double direction = current_v;
  if (fabs(current_v) < 0.0000001) {
    direction = last_v;
  }
  // if (fabs(current_v) < 0.0000001) {

  //     if (last_v < 0)
  //   return -fabs_current_v;

  //   return fabs_current_v;
  // }

  if (direction < 0)
    return -fabs_current_v;

  return fabs_current_v;
}

namespace common {
namespace math {

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
// 角度归一化到[-pi; pi]范围内
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}



} // namespace math
} // namespace common

// 计时器
class TicToc {
public:
  TicToc() {
    tic();
  }

  void reset() {
    tic();
  }

  void tic() {
    start = std::chrono::steady_clock::now();
  }

  int toc() {
    end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double> >
					(end - start).count() * 1000; // ms
  }
private:
  std::chrono::steady_clock::time_point start, end;
};



#endif 