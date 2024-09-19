import time
from collections import deque
import matplotlib.pyplot as plt
import math

class SteeringWheelSimulator:
    def __init__(self):
        self.speed_history = deque(maxlen=10)  # 保存最近5秒的速度记录（0.5秒间隔）
        self.angle_history = deque(maxlen=40)  # 保存最近20秒的角度记录（2秒间隔）
        self.current_speed = 0
        self.current_angle = 0

    def update(self, speed, angle):
        self.speed_history.appendleft((time.time(), speed))
        self.angle_history.appendleft((time.time(), angle))
        

    def get_delayed_speed(self):
        current_time = time.time()
        for timestamp, speed in self.speed_history:
            if current_time - timestamp >= 0.5:
                self.current_speed = speed
                break
        return self.current_speed  # 如果没有找到0.5秒前的记录，返回当前速度

    def get_delayed_angle(self):
        current_time = time.time()
        for timestamp, angle in self.angle_history:
            if current_time - timestamp >= 2:
                self.current_angle = angle
                break
        return self.current_angle  # 如果没有找到2秒前的记录，返回当前角度

# 仿真5秒钟并生成图像
def simulate_and_plot():
    simulator = SteeringWheelSimulator()
    times = []
    speeds = []
    angles = []
    delayed_speeds = []
    delayed_angles = []

    start_time = time.time()
    while time.time() - start_time < 5:
        current_time = time.time() - start_time
        speed = current_time  # 示例速度
        angle = math.sin(current_time)  # 示例角度
        rs = simulator.get_delayed_speed()
        ra = simulator.get_delayed_angle()

        simulator.update(speed, angle)

        times.append(current_time)
        speeds.append(speed)
        angles.append(angle)
        delayed_speeds.append(rs)
        delayed_angles.append(ra)

        time.sleep(0.1)  # 模拟时间间隔

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
