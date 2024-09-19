import numpy as np
import matplotlib.pyplot as plt

def first_order_inertia(T, K, t, input_signal):
    output_signal = np.zeros(len(input_signal))
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        output_signal[i] = (K * input_signal[i] + T * output_signal[i-1]) / (T + dt)
    return output_signal

# 参数设置
T = 5.0  # 时间常数
K = 0.03  # 增益
time = np.linspace(0, 10, 1000)
input_signal = np.sin(time)

# 模拟惯性环节
output_signal = first_order_inertia(T, K, time, input_signal)

# 绘制输入和输出信号
plt.figure()
plt.plot(time, input_signal, label='Input Signal')
plt.plot(time, output_signal, label='Output Signal')
plt.xlabel('Time')
plt.ylabel('Signal')
plt.title('First Order Inertia Simulation')
plt.legend()
plt.show()
