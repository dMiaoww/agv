#include "MainWindow.h"
#include "common_data.h"
#include "global.h"
#include "imgui.h"
#include <cmath>
#include <functional>
#include <glog/logging.h>
#include <thread>

MainWindow::MainWindow(CoTask *task_handler) {
  m_task_handler = task_handler;
  window_ox = -5;
  window_oy = -5;

  // 初始化GLFW
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = glfwCreateWindow(800, 600, "MainWindow", NULL, NULL);
  glfwMakeContextCurrent(window);

  // 初始化ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

  fresh_thread = std::thread(std::bind(&MainWindow::FreshWindow, this));
}

MainWindow::~MainWindow() {
  if (fresh_thread.joinable()) {
    fresh_thread.join();
  }
}

void MainWindow::DrawCar(const double xx, const double yy, const double ttheta,
                         const float w, const float h, const ImU32 col) {
  ImDrawList *draw_list = ImGui::GetWindowDrawList();

  // 获取当前窗口的位置和大小
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_size = ImGui::GetWindowSize();

  float rect_w = w, rect_h = h; // 矩形的宽度和高度
  // 方块的位置
  float x = window_pos.x + (xx - window_ox) * 10;
  float y = window_pos.y + window_size.y - (yy - window_oy) * 10;
  float theta = -ttheta; // 旋转角度（度）

  // 计算角度
  const float radian = theta;

  // 计算矩形的四个顶点
  ImVec2 rect[4];
  rect[0] =
      ImVec2(x + rect_w / 2.0f * cos(radian) - rect_h / 2.0f * sin(radian),
             y + rect_w / 2.0f * sin(radian) + rect_h / 2.0f * cos(radian));
  rect[1] =
      ImVec2(x - rect_w / 2.0f * cos(radian) - rect_h / 2.0f * sin(radian),
             y - rect_w / 2.0f * sin(radian) + rect_h / 2.0f * cos(radian));
  rect[2] =
      ImVec2(x - rect_w / 2.0f * cos(radian) + rect_h / 2.0f * sin(radian),
             y - rect_w / 2.0f * sin(radian) - rect_h / 2.0f * cos(radian));
  rect[3] =
      ImVec2(x + rect_w / 2.0f * cos(radian) + rect_h / 2.0f * sin(radian),
             y + rect_w / 2.0f * sin(radian) - rect_h / 2.0f * cos(radian));

  draw_list->PushClipRectFullScreen();
  draw_list->AddQuadFilled(rect[0], rect[1], rect[2], rect[3], col);
  draw_list->PopClipRect();
}

void MainWindow::FreshWindow() {
  glfwMakeContextCurrent(window);
  while (!glfwWindowShouldClose(window)) {
    // 清除前一帧的输入数据
    glfwPollEvents();
    // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // 开始新一帧的 ImGui 渲染
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (ImGui::Begin("My window")) {
      // 虚拟大车
      Pose BIG = m_task_handler->CalcVirtualCenter();
      ImGui::Text("BIG (%.2f, y: %.2f, theta: %.2f)", BIG.x, BIG.y,
                  BIG.theta / M_PI * 180.0);
      DrawCar(BIG.x, BIG.y, BIG.theta, 60, 55, IM_COL32(255, 0, 0, 255));

      // 组件小车
      Pose agv_10 = Global::get_agv_pose(10);
      ImGui::Text("10  (%.2f, y: %.2f, theta: %.2f)", agv_10.x, agv_10.y,
                  agv_10.theta / M_PI * 180.0);
      DrawCar(agv_10.x, agv_10.y, agv_10.theta, 20, 15,
              IM_COL32(0, 255, 0, 255));
    }
    ImGui::End();

    // 渲染ImGui
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
}