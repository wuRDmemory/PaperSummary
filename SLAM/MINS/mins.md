# MINS: Efficient and Robust Multisensor-aided Inertial Navigation System

---

## 写在前面

MINS为黄国权老师rpng组对于openvins的升级版，相比于openvins：

- 整个框架对于传感器的集成度更高了，不仅仅是vio系统，同时兼容了各种传感器



参考：

- https://copland.udel.edu/~ghuang/papers/tr_wheel-vio.pdf 关于odom部分的公式推导

---

## 符号注解

下面是会用到的符号整理

- 坐标系表示：{$I$}表示IMU坐标系， {$C$}表示Camera坐标系，{$O$}表示Wheel Odom坐标系，{$L$}表示Lidar坐标系， {$G$}表示GNSS坐标系，{$E$}表示ENU坐标系

- 状态量表示：
  $$
  \begin{aligned}
  \mathrm{x_k}&=(\mathrm{x_{I_k}, x_{H_k}}) \\
  \mathrm{x_{I_k}}&=(\mathrm{{}^{I_k}_{E}R, {}^{E}p_{I_k},{}^{E}v_{I_k}, b_g, b_a}) \\
  \mathrm{x_{H_k}}&=(\mathrm{{}^{I_{k-1}}_{E}R, {}^{E}p_{I_{k-1}},\dots, {}^{I_{k-h}}_{E}R, {}^{E}p_{I_{k-h}}})
  \end{aligned}
  $$

- odom部分符号表示：

  - 轮子转速符号：$w_{l_k}, w_{r_k}$；
  - 轮径和轮间距：$X_{OI} = \{r_{l}, r_{r}, b\}$；



---

## 方法细节

### Wheel Encoder Update

左右轮速的符号表示如下，其中$w_{m}$为观测变量，$w_{l_k}$表示为轮速真值，$n_{wl}$为噪声值
$$
\begin{equation}
\begin{aligned}
w_{ml_k}&=w_{l_k}+n_{wl} \\
w_{mr_k}&=w_{r_k}+n_{wr}
\end{aligned}
\end{equation}
$$
随后使用差分轮速的公式得到线速度和角速度，其中
$$
\begin{equation}
\begin{aligned}
{}^{O_k}w&=(w_{rk}r_r - w_{lk}r_l)/b \\
{}^{O_k}v&=(w_{rk}r_r + w_{lk}r_l)/2
\end{aligned}
\end{equation}
$$
差分机器人的轮速运动模型微分公式如下：
$$
\begin{equation}
\begin{bmatrix}
{}^{O_{\tau}}_{O_{k-1}}\dot \theta \\
{}^{O_{k-1}}\dot x_{O_{\tau}} \\
{}^{O_{k-1}}\dot y_{O_{\tau}}
\end{bmatrix} :=
\begin{bmatrix}
-{}^{O_{\tau}} w \\
{}^{O_{\tau}}v \mathrm{cos}({}_{O_{\tau}}^{O_{k-1}}\theta) \\
{}^{O_{\tau}}v \mathrm{sin}({}_{O_{\tau}}^{O_{k-1}}\theta)
\end{bmatrix} =
\begin{bmatrix}
-{}^{O_{\tau}} w \\
{}^{O_{\tau}}v \mathrm{cos}({}^{O_{\tau}}_{O_{k-1}}\theta) \\
-{}^{O_{\tau}}v \mathrm{sin}({}^{O_{\tau}}_{O_{k-1}}\theta)
\end{bmatrix}
\end{equation}
$$
其中重点注意相对位姿中参考系问题，论文中的原文为 **we follow global-to-local orientation representation**：

1. 角度微分的参考系是$O_{\tau}$，因此角速度其实是机体角速度的负值；
2. 左公式中使用的角度因为参考系为k-1，因此无需取负号，右公式中角度的参考系变为$\tau$，因此需要取符号；

因为我们经常用的是local-to-global表示，因此需要把其中所有的角度表示中的上下角标反向，各中的角度前加负号即可。这里后面的公式笔者使用local-to-global的表示：
$$
\begin{equation}
\begin{aligned}
Z_{O} &=: 
\begin{bmatrix}
{}^{O_{k-1}}_{O_{k}}\theta \\ 
{}^{O_{k-1}}\mathbf{d}_{O_k}
\end{bmatrix} = 
\begin{bmatrix}
\int_{t_{k-1}}^{t_{k}}{}^{O_t}wdt \\
\int_{t_{k-1}}^{t_{k}} {}^{O_{t}}v \mathrm{cos}({}^{O_{k-1}}_{O_{t}}\theta) dt \\
\int_{t_{k-1}}^{t_{k}} {}^{O_{t}}v \mathrm{sin}({}^{O_{t-1}}_{O_{t}}\theta) dt
\end{bmatrix} \\
&=:\mathbf{g_{O}}(\{w_l, w_r\}_{k-1:k}, \mathrm{x_{OI}})
\end{aligned}
\end{equation}
$$

观测方程就比较明确了，其中$\Lambda=\{e_1, e_2\}$
$$
\begin{equation}
\mathrm{h_{O}(x_k)}:=
\begin{bmatrix}
{}^{O_{k-1}}_{O_{k}} \hat\theta \\ 
{}^{O_{k-1}}\mathbf{\hat d}_{O_k}
\end{bmatrix}=:
\begin{bmatrix}
\mathrm{e_3}^T\mathrm{Log({}^{O_{k-1}}_{E}R {}_{O_{k}}^{E}R)} \\
\Lambda {}^{O_{k-1}}_{E}R(\mathrm{{}^{E}p_{O_k} - {}^{E}p_{O_{k-1}}})
\end{bmatrix}
\end{equation}
$$

误差观测方程如下：
$$
\begin{equation}
\begin{aligned}
\mathbf{g_{O}}(\{w_l, w_r\}_{k-1:k}, \mathrm{x_{OI}})+\underbrace{\mathbf{G_O}}_{\frac{\partial{\mathrm{\tilde x}_{OI}}}{\partial{\mathrm{\tilde x}_{k}}}} \mathrm{\tilde x_{k}} + \mathbf{G_n} n_{w} &= \mathrm{h_O(x_k)} + \underbrace{\mathbf{H_O}}_{\frac{\partial{h_O(\mathrm{x_k})}}{\partial{\mathrm{\tilde x_k}}}}\mathrm{\tilde x_k}+\mathbf{H_n}n_{\mathrm{ \tilde x}} \\
\tilde z_{O} =\mathbf{g_{O}}(\{w_l, w_r\}_{k-1:k}, \mathrm{x_{OI}}) - \mathrm{h_{O}(x_k)} &= \mathbf{H_O} \mathrm{\tilde x_k} - \mathbf{G_O} \mathrm{\tilde x_k} + \mathbf{H_n} n_{\mathrm{\tilde x}} - \mathbf{G_n} n_{w} 
\end{aligned}
\end{equation}
$$



其中观测矩阵推导如下(这部分主要参考Visual-Inertial-Wheel Odometry with Online Calibration)：

#### 1. 对 $\mathbf{g_{O}}(\{w_l, w_r\}_{k-1:k}, \mathrm{x_{OI}})$ 进行推导

首先整个推导过程中如果直接对最终的状态变量( $ \mathrm{x}_{OI}=\left[ r_l, r_r, b \right] $ )进行求导的话会非常复杂，因此我们先对变量 $\left[{}^{O_k} \theta, {}^{O_{k}}\mathrm{x}, {}^{O_{k}}\mathrm{y}\right]$ 和 $ \left[ {}^{O_{\tau}}w, {}^{O_{\tau}}v \right] $ 进行误差状态的递推，之后利用链式法则进行微分方程的整理。

1. 首先推导状态变量$ \left[ {}^{O_{\tau}}w, {}^{O_{\tau}}v \right] $和$ \mathrm{x}_{OI} $ 的关系：

  $$
  \begin{equation}
  \begin{aligned}
  {}^{O} \tilde w &= \begin{bmatrix} -\frac{w_{ml}}{b} & \frac{w_{mr}}{b} & -\frac{w_{mr}r_r - w_{ml}r_l}{b^2} \end{bmatrix} \begin{bmatrix} \tilde r_{l} \\ \tilde r_{r} \\ \tilde b \end{bmatrix} + \begin{bmatrix} \frac{r_l}{b} & -\frac{r_r}{b} \end{bmatrix} \begin{bmatrix} n_{wl} \\ n_{wr} \end{bmatrix} \\
  &= H_{w\mathbf{x_{OI}}}\mathrm{\tilde x_{CI}} + H_{w\mathbf{n}_{OI}}\mathbf{n}_w \\

  {}^{O} \tilde v &= \begin{bmatrix} \frac{w_{ml}}{2} & \frac{w_{mr}}{2} & 0 \end{bmatrix} \begin{bmatrix} \tilde r_{l} \\ \tilde r_{r} \\ \tilde b \end{bmatrix} + \begin{bmatrix} -\frac{r_l}{2} & -\frac{r_r}{2} \end{bmatrix} \begin{bmatrix} n_{wl} \\ n_{wr} \end{bmatrix} \\
  &= H_{v\mathbf{x_{OI}}}\mathrm{\tilde x_{CI}} + H_{w\mathbf{n}_{OI}}\mathbf{n}_w

  \end{aligned}
  \end{equation}
  $$

2. 整体的递推公式可以简化如下，请读者依旧注意这里旋转坐标系的表示问题和原文不同

  $$
  \begin{equation}
  \begin{aligned}
  {}^{O_k}_{O_{\tau+1}} \tilde \theta &= {}^{O_k}_{O_{\tau}} \tilde \theta + \mathbf{H_{1, \tau}} \mathbf{\tilde x_{OI}} + \mathbf{H_{1, \tau}} \mathbf{n_{w, \tau}} \\
  {}^{O_k}\mathbf{\tilde x}_{O_{\tau+1}} &= {}^{O_k}\mathbf{\tilde x}_{O_{\tau}} + \mathbf{H_{3, \tau}} {}^{O_k}_{O_{\tau}} \tilde \theta + \mathbf{H_{4, \tau}}  \mathbf{\tilde x_{OI}} + \mathbf{H_{5, \tau}} \mathbf{n_{w, \tau}} \\
  {}^{O_k}\mathbf{\tilde y}_{O_{\tau+1}} &= {}^{O_k}\mathbf{\tilde y}_{O_{\tau}} + \mathbf{H_{6, \tau}} {}^{O_k}_{O_{\tau}} \tilde \theta + \mathbf{H_{7, \tau}}  \mathbf{\tilde x_{OI}} + \mathbf{H_{8, \tau}} \mathbf{n_{w, \tau}} \\
  \end{aligned}
  \end{equation}
  $$

  上述公式可以写为状态传递方程的矩阵形式，如下: \
  $$
  \begin{equation}
  \begin{aligned}
  \boldsymbol{\Phi}_{t r, \tau} & =\left[\begin{array}{ccc}
  1 & 0 & 0 \\
  \mathbf{H}_{3, \tau} & 1 & 0 \\
  \mathbf{H}_{6, \tau} & 0 & 1
  \end{array}\right], \quad \boldsymbol{\Phi}_{W I, \tau}=\left[\begin{array}{l}
  \mathbf{H}_{1, \tau} \\
  \mathbf{H}_{4, \tau} \\
  \mathbf{H}_{7, \tau}
  \end{array}\right], \quad \boldsymbol{\Phi}_{n, \tau}=\left[\begin{array}{l}
  \mathbf{H}_{2, \tau} \\
  \mathbf{H}_{5, \tau} \\
  \mathbf{H}_{8, \tau}
  \end{array}\right]
  \end{aligned}
  \end{equation}
  $$

  有了误差状态的传递方程，则观测的协方差的更新公式如下：
  $$
  \begin{equation}
  \mathbf{P}_{m, \tau+1} =\boldsymbol{\Phi}_{t r, \tau} \mathbf{P}_{m, \tau} \boldsymbol{\Phi}_{t r, \tau}^{\top}+\boldsymbol{\Phi}_{n, \tau} \mathbf{Q}_\tau \boldsymbol{\Phi}_{n, \tau}^{\top}
  \end{equation}
  $$

  如公式（6）（8）所示，整个观测因为和状态变量中的 $\mathbf{x}_{OI}$ 相关，因此可以借鉴预积分的思想，在迭代的过程中不断的对传递矩阵进行更新，公式如下：
  $$
  \begin{equation}
  \frac{\partial \mathbf{g}_{\tau+1}}{\partial \tilde{\mathbf{x}}_{W I}} =\boldsymbol{\Phi}_{t r, \tau} \frac{\partial \mathbf{g}_\tau}{\partial \tilde{\mathbf{x}}_{W I}}+\boldsymbol{\Phi}_{W I, \tau}
  \end{equation}
  $$

3. 对其中的各个变量进行细致推导 \
根据公式（3）或者公式（4）可以得到观测 $O_{\tau}$ 到 $O_{\tau+1}$ 的递推公式：\
$$
\begin{equation}
\begin{aligned}
{}_{O_\tau+1}^{O_k}\theta &= {}_{O_\tau}^{O_k}\theta + {}^{O_{\tau}} \omega \Delta t \\
{}^{O_k}\mathrm{x}_{{}_{O_\tau+1}} &= {}^{O_k}\mathrm{x}_{{}_{O_\tau}} + \frac{{}^{O_{\tau}}v(\mathcal{sin}({}_{O_{\tau+1}}^{O_k}\theta)-\mathcal{sin}({}_{O_\tau}^{O_k}\theta))}{{}^{O_{\tau}}w} \\
{}^{O_k}\mathrm{y}_{{}_{O_{\tau+1}}} &= {}^{O_k}\mathrm{y}_{{}_{O_\tau}} + \frac{{}^{O_{\tau}}v(-\mathcal{cos}({}_{O_{\tau+1}}^{O_k}\theta)+\mathcal{cos}({}_{O_\tau}^{O_k}\theta))}{{}^{O_{\tau}}w} \\
\end{aligned}
\end{equation}
$$

可以看到 $\theta$ 的误差状态传递方程是比较简单的，这里不做推导。

这里仅对 ${}^{O_{k}}\mathrm{x}_{O_{\tau+1}}$ 的误差状态传递方程进行推导，设：
$$
\begin{equation}
f\mathrm{({}^{O_k}\theta, {}^{O_{\tau}}w, {}^{O_{\tau}}v)} = \frac{{}^{O_{\tau}}v(\mathcal{sin}({}_{O_\tau+1}^{O_k}\theta)-\mathcal{sin}({}_{O_\tau}^{O_k}\theta))}{{}^{O_{\tau}}w}
\end{equation}
$$

