cmake_minimum_required(VERSION 3.5)
project(agv_server)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

include("cmake/FindGlog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

include_directories(include)
include_directories(include/track/controller)

# 添加一个变量，保存所有的ImGui源文件
set(IMGUI_SOURCES 
    ${CMAKE_SOURCE_DIR}/depend/imgui/imgui.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/imgui_demo.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/imgui_widgets.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/depend/imgui/backends/imgui_impl_opengl3.cpp
)

# 添加include路径
include_directories(
    ${CMAKE_SOURCE_DIR}/depend
    ${CMAKE_SOURCE_DIR}/depend/imgui
    ${CMAKE_SOURCE_DIR}/depend/imgui/backends
    # ... # 其他的包含文件路径
)

# 链接其他 OpenGL 和 GLFW 等库
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# 创建一个静态库目标
add_library(imgui ${IMGUI_SOURCES})

add_subdirectory(src)


add_executable(real real.cpp)
target_link_libraries(real glog pthread src imgui OpenGL::GL glfw dl track)


add_executable(sim_omni sim_omni.cpp)
target_link_libraries(sim_omni glog pthread src imgui OpenGL::GL glfw dl track)

add_executable(sim_ss sim_single_steer.cpp)
target_link_libraries(sim_ss glog pthread src imgui OpenGL::GL glfw dl track)

add_executable(sim_lag sim_lag.cpp)
target_link_libraries(sim_lag glog pthread src imgui OpenGL::GL glfw dl track)

add_executable(paint test_paint.cpp)
target_link_libraries(paint glog pthread src imgui OpenGL::GL glfw dl track)