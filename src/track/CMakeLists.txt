aux_source_directory(. src)
aux_source_directory(controller CONTROL_SRC)

add_library(track ${src} ${CONTROL_SRC})

add_executable(testMain   testMain.cpp controller/lqr_w.cpp)
target_link_directories(testMain SYSTEM PUBLIC track glog)