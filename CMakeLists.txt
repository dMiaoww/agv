cmake_minimum_required(VERSION 3.5)
project(agv_server)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include("cmake/FindGlog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

include_directories(include)

add_subdirectory(src)

# add_executable(main main.cpp)
# target_link_libraries(main glog pthread src)

add_executable(main1 main1.cpp)
target_link_libraries(main1 glog pthread src)