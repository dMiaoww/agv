import numpy as np
import matplotlib.pyplot as plt

def first_order_inertia(T, Delay, t, input_signal):
    output_signal = np.zeros(len(input_signal))
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        N = Delay/dt
        if i <= N:
            output_signal[i] = (dt * 0 + T * output_signal[i-1]) / (T + dt)  # T 是时间常数
        else: 
            output_signal[i] = (dt * input_signal[int(i-N)] + T * output_signal[i-1]) / (T + dt)  # T 是时间常数
    return output_signal

# 参数设置
T = 0.2  # 迟滞时间常数
Delay = 0.2 # 纯滞后时间
time = np.linspace(0, 10, 1000) # dt = 0.1
input_signal = np.sin(time)

# 模拟惯性环节
output_signal = first_order_inertia(T, Delay, time, input_signal)

# 绘制输入和输出信号
plt.figure()
plt.plot(time, input_signal, label='Input Signal')
plt.plot(time, output_signal, label='Output Signal')
plt.xlabel('Time')
plt.ylabel('Signal')
plt.title('First Order Inertia Simulation')
plt.legend()
plt.show()
