#pragma once

#include "common_data.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>

#include "co_task.h"
#include "math.h"

class MainWindow {
public:
  MainWindow(CoTask *task_handler, std::vector<Pose>* traj, Pose* vl);

  ~MainWindow();

  void Init();

  void FreshWindow();

  void setCb_followStartClick(std::function<void()> func){
    m_followStartClick = func;
  }

private:
  void DrawCar(const double xx, const double yy, const double ttheta,
               const float w, const float h, const ImU32 col);
  
  void DrawTraj(const std::vector<Pose>&, const ImU32 col);

  void DrawCoordinateSystem();

  void DrawGrid();

private:
  GLFWwindow *window;
  int window_height;
  int window_ox;  // 世界坐标系的左下角坐标
  int window_oy;

  const int ratio;  // 比例

  CoTask *m_task_handler;
  std::vector<Pose>* m_traj;

  std::thread fresh_thread;

  Pose* m_vl;

  std::function<void()> m_followStartClick;
};