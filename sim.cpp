#include "common_data.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "sim_omni.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
// #include "move_base/common.h"
#include "curve.h"
#include "glog_set.h"
#include "move_base/common.h"
#include "move_base/omni_steer_tracker.h"

// using namespace motionplanner;

int window_ox = -5;
int window_oy = -15;
const int ratio = 50;

Pose start(0, -7, 0);
// int index = 0;
auto traj = BezierCurve::get(1000, Pose(0, -7, 0), Pose(2, -7, 0),
                             Pose(4, -8, 0), Pose(6, -9, 0));
motionplanner::Tracker *tracker_;
SimOmni *agv;

void DrawTraj(ImDrawList *draw_list, std::vector<Pose> &traj) {
  // 获取当前窗口的位置和大小
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_size = ImGui::GetWindowSize();
  for (int i = 0; i < traj.size() - 1; ++i) {
    ImVec2 p1, p2;
    // LOG(INFO) << traj[i+1];
    p1.x = window_pos.x + (traj[i].x - window_ox) * ratio;
    p1.y = window_pos.y + window_size.y - (traj[i].y - window_oy) * ratio;

    p2.x = window_pos.x + (traj[i + 1].x - window_ox) * ratio;
    p2.y = window_pos.y + window_size.y - (traj[i + 1].y - window_oy) * ratio;

    draw_list->AddLine(p1, p2, IM_COL32(255, 255, 255, 255), 2.0f);
  }
}

void DrawCar(ImDrawList *draw_list, Pose robot) {
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_size = ImGui::GetWindowSize();
  float rect_w = 1 * ratio, rect_h = 0.6 * ratio; // 矩形的宽度和高度
  // 方块的位置
  float x = window_pos.x + (robot.x - window_ox) * ratio;
  float y = window_pos.y + window_size.y - (robot.y - window_oy) * ratio;
  float theta = -robot.theta; // 旋转角度（度）
  // 计算角度
  const float radian = theta;
  ImVec2 center;
  center.x = x;
  center.y = y;
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
  draw_list->AddQuadFilled(rect[0], rect[1], rect[2], rect[3],
                           IM_COL32(255, 255, 0, 255));
  draw_list->AddCircle(center, 2, IM_COL32(0, 0, 255, 255));
  draw_list->PopClipRect();
}

int main(int argc, char **argv) {
  GLog_set glog_set(argv[0]);

  tracker_ = new motionplanner::OmniSteerTracker();
  tracker_->Init();
  tracker_->SetMotionParam(20, 3, 1, 0, 0.6, 0, 0.3, 0.5, 0.5, 0.02, 0.03);
  tracker_->SetAlgoParam();

  agv = new SimOmni(start);

  std::vector<double> traj_s;
  traj_s.push_back(0);
  double s = 0;
  for (int i = 1; i < traj.size(); i++) {
    s += std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
    traj_s.push_back(s);
  }

  // std::for_each(traj_s.begin(), traj_s.end(), [](double num) {
  //       std::cout << num << ' ';
  //   });

  size_t next_i = 0;
  motionplanner::MoveCmd last_cmd(0, 0, 0); // 上一次的控制速度

  // 初始化GLFW
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow *window = glfwCreateWindow(800, 600, "MainWindow", NULL, NULL);
  glfwMakeContextCurrent(window);

  // 初始化ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");
  glfwMakeContextCurrent(window);

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  auto state = motionplanner::Tracker::State::kTracking;
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
      std::chrono::high_resolution_clock::time_point t2 =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> time_span = t2 - t1;
      // 虚拟大车
      ImGui::Text(agv->getState().c_str());
      ImGui::Text("%lf", time_span.count() / 1000.0);
      // 绘制路径

      if (state == motionplanner::Tracker::State::kTracking) {
        motionplanner::MoveCmd now_cmd;
        Pose robot = agv->getPose();
        state = tracker_->Track(traj, 0.1, traj_s, next_i, traj.size(), robot,
                                last_cmd, now_cmd, &next_i);
        Pose cmd;
        cmd.x = now_cmd.vx, cmd.y = now_cmd.vy, cmd.theta = now_cmd.w;
        agv->SetSpeed(cmd);
        last_cmd = now_cmd;
      } else {
        agv->SetSpeed(Pose(0, 0, 0));
      }

      // ImVec2 p1, p2;
      // p1.x = window_pos.x + (robot.x - window_ox) * ratio;
      // p1.y = window_pos.y + window_size.y - (robot.y - window_oy) *
      // ratio; p2.x =
      //     window_pos.x + (robot.x + 1 * cos(robot.theta) - window_ox) *
      //     ratio;
      // p2.y = window_pos.y + window_size.y -
      //        (robot.y + 1 * sin(robot.theta) - window_oy) * ratio;
      // draw_list->AddLine(p1, p2, IM_COL32(255, 255, 0, 255), 2.0f);

      // 画车
      DrawCar(ImGui::GetWindowDrawList(), agv->getPose());
      DrawTraj(ImGui::GetWindowDrawList(), traj);

      // 绘制坐标系(grid)
      // DrawCoordinateSystem();
      // DrawGrid();
    }
    ImGui::End();

    // 渲染ImGui
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}