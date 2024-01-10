#include "global.h"

std::unordered_map<int, AGVstatus> Global::allagvs = {
  {10, AGVstatus()},
  {5, AGVstatus()},
  {6, AGVstatus()},
};


std::mutex Global::global_mutex;