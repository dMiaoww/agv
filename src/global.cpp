#include "global.h"

std::unordered_map<int, AGVstatus> Global::allagvs = {
  // {10, AGVstatus()},
  // {7, AGVstatus()},
  // {6, AGVstatus()},
};


std::mutex Global::global_mutex;