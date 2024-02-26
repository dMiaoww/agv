

add_library(move_base
src/common.cpp 
src/omni_steer_tracker.cpp
src/otg_filter.cpp
# src/road_map.cpp
src/tracker.cpp
src/controller/lqr_diff.cpp
src/controller/lqr_nsteer.cpp
src/controller/lqr_steer.cpp
src/controller/lqr_w.cpp
)

target_link_libraries(move_base glog)