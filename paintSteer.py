import matplotlib.pyplot as plt

# 四个文件名
# filenames = ["steer0", "steer1", "steer2", "steer3"]
filenames = ["DDSU0", "DDSU1", "DDSU2", "DDSU3"]

plt.figure(figsize=[15, 20])  # 设置画布尺寸

for i, filename in enumerate(filenames):
    # 初始化列表
    m_set_angle = []
    m_set_vd = []
    real_angle = []
    real_vd = []

    # 读取数据
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.split()))
            if len(values) == 4:
                s_angle, s_vd, r_angle, r_vd = values
            else:
                continue

            m_set_angle.append(s_angle)
            m_set_vd.append(s_vd)
            real_angle.append(r_angle)
            real_vd.append(r_vd)

    # 绘制角度对比图
    plt.subplot(4, 2, i*2+1)
    plt.plot(m_set_angle, label='Set Angle')
    plt.plot(real_angle, label='Real Angle')
    plt.title('Angle Comparison for file ' + str(i+1))
    plt.legend()

    # 绘制速度对比图
    plt.subplot(4, 2, i*2+2)
    plt.plot(m_set_vd, label='Set VD')
    plt.plot(real_vd, label='Real VD')
    plt.title('VD Comparison for file ' + str(i+1))
    plt.legend()

plt.tight_layout()  # 调整各subplot之间的距离
plt.show()
