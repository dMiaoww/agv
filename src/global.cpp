#include "global.h"

std::unordered_map<int, AGVstatus> Global::allagvs;
std::mutex Global::global_mutex;