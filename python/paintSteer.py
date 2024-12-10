import time
from collections import deque
import matplotlib.pyplot as plt
import math
from random import random


L = 1
class SteeringWheelSimulator:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.speed_history = []  # 保存最近5秒的速度记录（0.5秒间隔）
        self.angle_history = []  # 保存最近20秒的角度记录（2秒间隔）
        self.current_speed = 0
        self.current_angle = 0
        self.x = x
        self.y = y
        self.yaw = yaw
        self.dt = 0.05

    def update(self, speed, angle):
        self.speed_history.append(speed)
        self.angle_history.append(angle)

        self.get_delayed_angle()
        self.get_delayed_speed()

        v = self.current_speed
        angle = self.current_angle
        self.x += v * math.cos(self.yaw) * self.dt +  0.0*random()-0.0
        self.y += v * math.sin(self.yaw) * self.dt +  0.0*random()-0.0
        self.yaw += self.yaw + v / L * math.tan(angle) * self.dt


    def get_delayed_speed(self):
        if len(self.speed_history) >= 2 :  # t_delay = 2*0.05
            self.current_speed = self.speed_history[len(self.speed_history)-2]
        # return self.current_speed  # 如果没有找到 0.1 秒前的记录，返回当前速度

    def get_delayed_angle(self):
        current_time = time.time()
        dt = self.dt
        T_lag = 0.3
        T_delay = 0.3
        if len(self.speed_history) >= int(T_delay/dt) :  # t_delay = 4*0.05
            self.current_angle = (self.angle_history[len(self.angle_history)-2]*dt + T_lag*self.current_angle) / (T_lag+dt)
        else:
            self.current_angle = ( T_lag*self.current_angle) / (T_lag+dt)
        if self.current_angle >= 1:
            self.current_angle = 1
        if self.current_angle <= - 1.:
            self.current_angle = - 1
  


# 仿真5秒钟并生成图像
def simulate_and_plot():
    simulator = SteeringWheelSimulator()
    times = []
    speeds = []
    angles = []
    delayed_speeds = []
    delayed_angles = []

    t = 0
    while t < 5:
        speed = 1  # 示例速度
        angle = math.sin(t)  # 示例角度
        rs = simulator.get_delayed_speed()
        ra = simulator.get_delayed_angle()

        simulator.update(speed, angle)

        times.append(t)
        speeds.append(speed)
        angles.append(angle)
        delayed_speeds.append(rs)
        delayed_angles.append(ra)
        t += 0.05

        # time.sleep(0.05)  # 模拟时间间隔

    # 绘制输入输出图像
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    ax1.plot(times, speeds, label='输入速度')
    ax1.plot(times, delayed_speeds, label='0.5秒前的速度')
    ax1.set_ylabel('速度')
    ax1.legend()

    ax2.plot(times, angles, label='输入角度')
    ax2.plot(times, delayed_angles, label='2秒前的角度')
    ax2.set_ylabel('角度')
    ax2.set_xlabel('时间 (秒)')
    ax2.legend()

    plt.show()

simulate_and_plot()
