#pragma once

#include "common_data.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <chrono>
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

private:
  void DrawCar(const double xx, const double yy, const double ttheta,
               const float w, const float h, const ImU32 col);
  
  void DrawTraj(const std::vector<Pose>&, const ImU32 col);

private:
  GLFWwindow *window;
  int window_height;
  int window_ox;
  int window_oy;

  CoTask *m_task_handler;
  std::vector<Pose>* m_traj;

  std::thread fresh_thread;

  Pose* m_vl;
};