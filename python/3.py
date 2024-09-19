import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def first_order_inertia(T, K, dt, input_signal, previous_output):
    # return (K * input_signal + T * previous_output) / (T + dt)
    return (K)*input_signal + (1-K)*previous_output


# 参数设置
T = 1  # 时间常数
K = 0.02  # 增益
time = np.linspace(0, 10, 1000)
# input_signal = np.ones(len(time))
input_signal = np.sin(time)
setpoint = 1.0  # 目标值

# 创建PID控制器
pid = PIDController(Kp=3, Ki=0, Kd=0)

# 模拟惯性环节和PID控制
output_signal = np.zeros(len(time))
control_signal = np.zeros(len(time))
for i in range(1, len(time)):
    dt = time[i] - time[i-1]
    c = pid.update(input_signal[i], output_signal[i-1], dt)
    control_signal[i] =  c + input_signal[i]
    # control_signal = input_signal[i]
    output_signal[i] = first_order_inertia(T, K, dt, control_signal[i], output_signal[i-1])

# 绘制输入和输出信号
plt.figure()
plt.plot(time, input_signal, label='Input Signal')
plt.plot(time, control_signal, label='Control Signal')
plt.plot(time, output_signal, label='Output Signal with PID Control')
plt.xlabel('Time')
plt.ylabel('Signal')
plt.title('First Order Inertia with PID Control Simulation')
plt.legend()
plt.show()
