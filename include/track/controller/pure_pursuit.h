#ifndef PurePursuit_H
#define PurePursuit_H


#include "controller.h"


namespace motionplanner {

/*
单舵轮
*/
class PurePursuit {
public:
  // 返回转角
  double UpdateControl(const ControlStatus &status, const ControlParam &param);

}; // namespace motionplanner

}
#endif