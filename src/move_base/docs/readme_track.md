# 1 外部调用 `MoveBase::Track`
有三种状态：
- `State::kSlowStop` : slow stop, by traffic signals from CC, or obstacles
- `State::kEmgStop`: emergency stop
- `State::kTrack`: nothing happens
这个状态通过 `MoveBase::Process` 管理

`State::kTrack` 状态下:
- 遍历前方路径点的区域限速，如果需要减速设置 `traj_v[0] = lim_v`
- 调用 `tracker_->Track` 计算控制量
- `serial_->sendcmd` 发送给底盘，由底盘实现速度分解（当前是单片机实现的）


# 2 内部实现 `lqr_tracker::Track`
每一段路径分成三部分，
1. 起点旋转 `rotate1`，会判断起点和小车当前位置的距离，避免出现小车被人工挪走但是路径没有更新的情况
2. 沿路径行驶 `move`
3. 终点旋转 `rotate2`，不进行检测，到位就结束

## 2.1 沿路径行驶过程
这里包括了索引点选取、速度规划和控制量计算
1. 推算n个节拍后小车位置（考虑电机的延迟）
2. `getNearestId`，找到最近且前方的点
3. 小车脱轨？ 越过终点？ 到达终点进入 `rotate2` 状态？ 如果都不是则进入下一步
4. 根据曲率和当前跟踪误差计算速度限制，跟踪误差已经很大了就减速；对于索引点，根据当前误差和速度前瞻一段距离`getNextId`？（这些tricks并非完全使用）
5. 根据速度限制和到终点的距离计算纵向速度 `OtgFilter::runCycleS1`，这个函数是加速度连续的，所以如果在加速过程中进行减速的话，会看到加速度从正值慢慢变成负值，因此会有一小段时间速度上升
```
m_otg_lim.aMax = 0.33 * acc_;  // 这里*0.33和后面dt*3000是因为使用1的话，终点处会以一个较大的速度停止
if(traj_v[0] < 0.001) {
  m_otg_lim.aMax = 0.33 * stop_acc_;
}
      
double dis_to_end = traj_s[traj_s.size()-1] - traj_s[next_index];
m_otg_lim.vMax = traj_ref_v[0];
OtgFilter::VelParam target = {dis_to_end, 0, 0 ,0};
m_otg.qk.d = 0;
m_otg.qk.v = v0;
m_otg.runCycleS1(3000*dt_, m_otg_lim, target); 
traj_ref_v[0] = m_otg.qk.v;
```
6. `GetCmd`计算实际控制量，具体算法在后文描述

## 2.2 旋转控制量计算 `getRotateCmd` 
$w = k_p(\theta_{target} - \theta_{curr})$

# 3 行走控制器
- 控制量包括两部分，前馈和反馈 $u = u_{feedforward} + u_{feedback}$
- 前馈就是无误差状态下需要的控制量，也就是跟真实状态无关，只通过参考路径计算
- 反馈是根据真实状态和参考量（目标量）之间的误差计算得到

得到状态空间$X_{t+1} = AX_t + Bu$，再定义两个权重矩阵$Q R$，经过计算可以得到反馈矩阵 K，
$$
u_{feedback} = - K*X
$$
`lqr_w::Solve` 函数是求矩阵K的计算过程

# 4 非完整性模型
只控制角速度。
AB矩阵中如果有跟状态量有关的值，都使用参考量带入。
下标 r 表示参考量
## 4.1 差速轮
$$
A = \left[ \begin{matrix} 
1 & 0 & -vdt*sin\theta    \\
0 & 1 & vdt*cos\theta    \\
0 & 0 & 1    \\
\end{matrix} \right]
B = \left[ \begin{matrix} 
0   \\  0   \\  dt 
\end{matrix} \right]
X = \left[ \begin{matrix} 
x-x_r   \\  y-y_r   \\  \theta - \theta_r 
\end{matrix} \right]
$$
## 4.2 单舵轮（居中）
$$
w = \frac{v_d sin\alpha}{L}  \\
v_x = v_d cos\alpha            \\
$$
从`w`和`vx`计算电机转角和驱动速度：
$$
\alpha = arctan{\frac{wL}{v_x}} \\
v_d = v_x/cos\alpha
$$
理论上差速轮和单舵轮底盘可以套用相同的控制算法，计算出速度和角速度后再根据以上公式计算出转角，~~但是由于逆向求解是非线性的，所以这样得到的转角变化幅度很大，模拟效果不好~~

以下是另外两种模型：
## 4.3 单舵轮状态方程2
$$
A = \left[ \begin{matrix} 
1 & dt & 0 & 0  \\
0 & 0  & v & 0  \\
0 & 0  & 1 & dt \\
0 & 0 & 0 & 0
\end{matrix} \right] 
B = \left[ \begin{matrix} 
0   \\  0   \\  0  \\  v/L 
\end{matrix} \right] 
X = \left[ \begin{matrix} 
e   \\  (e-pe)/dt   \\  th_e  \\  (th_e-pth_e)/dt 
\end{matrix} \right] 
$$
Q = np.eye(4)
R = np.eye(1)
`L`表示轴距，`p`指代上一时刻的值

## 4.4 单舵轮状态方程3
$$
A = \left[ \begin{matrix} 
1 & 0 & -vdt*sin\theta   \\
0 & 1  & vdt*cos\theta   \\
0 & 0  & 1
\end{matrix} \right] 
B = \left[ \begin{matrix} 
0   \\  0   \\  \frac{vdt(1+tan^2 \alpha_r)}{L}
\end{matrix} \right] 
X = \left[ \begin{matrix} 
x-x_r   \\  y-y_r   \\  \theta - \theta_r 
\end{matrix} \right] 
$$
Q = [1...1....0.1]
R = 1 
# 5 完整性模型
$$
\left[ \begin{matrix} 
x_{t+1} \\ 
y_{t+1} \\ 
\theta_{t+1} 
\end{matrix} \right] = \left[ \begin{matrix} 
x_{t} + v_x dt cos\theta - v_y dt sin\theta  \\ 
y_{t} + v_x dt sin\theta + v_y dt cos\theta \\ 
\theta_{t}+w dt 
\end{matrix} \right]
$$
换种形式：
$$
\left[ \begin{matrix}  \dot{x} \\ \dot{y} \\ \dot{\theta} \end{matrix} \right] = \left[ \begin{matrix} v_x cos\theta - v_y sin\theta  \\ v_x sin\theta + v_y cos\theta \\  w  \end{matrix} \right]
$$
引入参考状态，根据泰勒公式进行展开：
$$
\left[ \begin{matrix}  \dot{x} - \dot{x_r} \\ \dot{y}- \dot{y_r} \\ \dot{\theta} - \dot{\theta_r} \end{matrix} \right] = \left[ \begin{matrix} 
0 & 0 & -v_xsin\theta - v_y cos\theta  \\ 
0 & 0 & v_x cos\theta - v_y sin\theta  \\  
0 & 0 &0  \end{matrix} \right] 
\left[ \begin{matrix}  x - x_r \\ y- y_r \\ \theta - \theta_r \end{matrix} \right]+ \left[ \begin{matrix} cos\theta_r & -sin\theta_r & 0  \\ sin\theta_r & cos\theta_r & 0 \\  0 & 0 & 1  \end{matrix} \right] \left[ \begin{matrix}  v_x - v_{xr} \\ v_y - v_{yr} \\ w - w_{r} \end{matrix} \right]
$$
离散化，
$$
\dot{x} = (1+AT)X+BTu
$$

# 6 多舵轮速度分解
单个舵轮的速度矢量 = 车体中心的平移矢量 + 相对于中心的转动速度
如果舵轮在车体右上角，舵轮与车体中心的横向距离是 D，纵向距离是 L，那么：
$$
v_{dx} = v_x + wD   \\
v_{dy} = v_y + wL 
$$
这种方法可以推导到双舵轮、四舵轮等等，具体实现在`lqr_nsteer::inverse_transform`中

# 7 差速驱动单元
一组差速轮组成一个舵轮的情况
计算出转角后 PID 计算实际的电机速度，v固定, 控制 w，（待测试）