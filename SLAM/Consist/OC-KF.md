# OC-EKF(Observability-constrained EKF)

---

[toc]

&nbsp;

----

## Reference

1. Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight. 开源S-MSCKF的论文；
2. Observability-constrained Vision-aided Inertial Navigation. OC-EKF的论文；
3. https://zhuanlan.zhihu.com/p/304889273. 关于FEJ的总结；
4. High-Precision, Consistent EKF-based Visual-Inertial Odometry. MSCKF2.0 论文；
5. https://blog.csdn.net/wubaobao1993/article/details/109299097. 关于MSCKF1.0预测部分的总结；

&nbsp;

---

## Notation

依旧先来说清楚文中的Notation是如何表示的：

1. $l$ 节点的理想值表示为$\mathrm{x}_{l}$；在 k 时刻的估计值表示为$\mathrm{\hat{x}}_{l}^{(k)}$；其误差状态表示为$\mathrm{\tilde{x}}_{l}$；
2. $l$ 节点的估计值在不同时刻的关系为：$\mathbf{\hat{x}}^{(k+n)}_{l}={}^{(k+n)}\mathbf{\tilde{x}}^{(k)}_{l}+\mathbf{\hat{x}}^{(k)}_{l}$，特别的，对于旋转有：${}^{l}_{G}\mathbf{\hat{q}}^{(k+n)}\approx (\mathbf{I}-\lfloor {}^{(k+n)}\theta^{(k)}_{l}\rfloor_{\times}){}^{l}_{G}\mathbf{\hat{q}}^{(k)}$；

&nbsp;

----

## IMU状态传递方程

INS系统的重中之重，还是先来推导IMU的状态传递方程。为了和参考【3】的推导保持一致，这里也仅仅对旋转、位移和速度进行分析，忽略bias的部分。

&nbsp;

### MSCKF2.0的传递方程

在参考【3】中，作者从理论意义上推导了IMU的传递方程，推得的状态转移矩阵如下：
$$
\begin{bmatrix}
{}^{I_{l+1}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l+1}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l+1}}^{(l)}
\end{bmatrix} = 
\begin{bmatrix} 
{}^{I_{l+1}}_{I_{l}}R^{(l)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}_{l}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}_{l}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}^{I_{l}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l}}^{(l)}
\end{bmatrix}
+
\begin{bmatrix}
{}^{I_{l+1}}_{I_{l}}\tilde{\theta}^{(l)} \\
({}^{I_l}_{G}R^{(l)})^{T}\left[\tilde{\mathrm{y}}^{(l)}_{l}\right] \\
({}^{I_l}_{G}R^{(l)})^{T}\left[\tilde{\mathrm{s}}^{(l)}_{l}\right]
\end{bmatrix} \tag{1}
$$
其中$-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}_{l}\right]_{\times}$和$-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}_{l}\right]_{\times}$可以理解为是旋转误差作用于位移和速度的杆臂，具有一定的物理意义；

> 其实在实际的MSCKF2.0中，传递方程并不是公式（1）所示的形式，具体可见参考【4】

&nbsp;

### MSCKF1.0的传递方程

在MSCKF1.0的理论中，IMU的误差状态传递方程主要由运动方程求得，如下：
$$
\dot{\tilde{\mathbf{X}}}_{\mathrm{IMU}}=\mathbf{F} \tilde{\mathbf{X}}_{\mathrm{IMU}}+\mathbf{G} \mathbf{n}_{\mathrm{IMU}}  \tag{2}
$$

其中：
$$
\mathbf{F}=\left[\begin{array}{ccc}
-\lfloor\hat{\boldsymbol{\omega}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}_{G}^{I_l}R)^{T}\lfloor\hat{\mathbf{a_m}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
$$
由于此处的G对于能观性的分析没有实质性的用处，这里不作分析；

将微分方程转为离散的形式：
$$
\boldsymbol{\tilde{X}}\left(t_{l+1}\right)=\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \boldsymbol{\tilde{X}}\left(t_{l}\right)+\int_{t_{l}}^{t_{l+1}} \boldsymbol{\Phi}\left(t_{l+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{3}
$$
其中：
$$
\begin{cases}
\dot{\boldsymbol{\Phi}}\left(t_{l+1}, t_{l}\right) = \boldsymbol{F}(t)\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \\
\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right)=\exp(\int_{t_{l}}^{t_{l+1}} \boldsymbol{F}(t) \mathrm{d} t)  
\end{cases} \tag{4}
$$
&nbsp;

### OC-KF的传递方程

在OC-KF中，IMU部分的传递方程使用的是MSCKF1.0中的微分方程的形式，这里先来推导状态转移矩阵的闭式解。

由公式（4）的第一行可以列出如下公式，这里把时间跨度直接认为是 t，初始的时间为 t0：
$$
\begin{aligned}
\begin{bmatrix}
\dot{\Phi}_{11}(t) & \dot{\Phi}_{12}(t) & \dot{\Phi}_{13}(t) \\
\dot{\Phi}_{21}(t) & \dot{\Phi}_{22}(t) & \dot{\Phi}_{23}(t) \\
\dot{\Phi}_{31}(t) & \dot{\Phi}_{32}(t) & \dot{\Phi}_{33}(t) \\
\end{bmatrix}
&=\left[\begin{array}{ccc}
-\lfloor\hat{\boldsymbol{\omega}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}^{G}_{t}R)^{T}\lfloor\hat{\mathbf{a_m}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
\begin{bmatrix}
{\Phi}_{11}(t) & {\Phi}_{12}(t) & {\Phi}_{13}(t) \\
{\Phi}_{21}(t) & {\Phi}_{22}(t) & {\Phi}_{23}(t) \\
{\Phi}_{31}(t) & {\Phi}_{32}(t) & {\Phi}_{33}(t) \\
\end{bmatrix} \\
\begin{bmatrix}
{\Phi}_{11}(t_0) & {\Phi}_{12}(t_0) & {\Phi}_{13}(t_0) \\
{\Phi}_{21}(t_0) & {\Phi}_{22}(t_0) & {\Phi}_{23}(t_0) \\
{\Phi}_{31}(t_0) & {\Phi}_{32}(t_0) & {\Phi}_{33}(t_0) \\
\end{bmatrix} 
&= 
\begin{bmatrix}
\mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} \\
\end{bmatrix}
\end{aligned} \tag{5}
$$
公式（5）可以引出如下的几组公式：
$$
\begin{cases}
\dot{\Phi}_{11}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{11}(t) \\
\dot{\Phi}_{12}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{12}(t) \\ 
\dot{\Phi}_{13}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{13}(t) \\ 
\end{cases}  \quad
\begin{cases}
\dot{\Phi}_{21}(t)=\Phi_{31}(t) \\
\dot{\Phi}_{22}(t)=\Phi_{32}(t) \\ 
\dot{\Phi}_{23}(t)=\Phi_{33}(t) \\ 
\end{cases}  \quad
\begin{cases}
\dot{\Phi}_{31}(t)=-({}^{G}_{I_t}R)^{T}\lfloor\hat{\mathbf{a_m}}\rfloor_{\times} \Phi_{11}(t) \\
\dot{\Phi}_{32}(t)=-({}^{G}_{I_t}R)^{T}\lfloor\hat{\mathbf{a_m}}\rfloor_{\times} \Phi_{12}(t) \\ 
\dot{\Phi}_{33}(t)=-({}^{G}_{I_t}R)^{T}\lfloor\hat{\mathbf{a_m}}\rfloor_{\times} \Phi_{13}(t) \\ 
\end{cases} \tag{6}
$$
下面就是一组一组的展开一下：

&nbsp;

#### 第一组

容易看出，第一组的闭式解其实都是exp函数，所以：
$$
\begin{cases}
\Phi_{11}(t)=exp(-\lfloor \hat{\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{11}(t_0) \\
\Phi_{12}(t)=exp(-\lfloor \hat{\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{12}(t_0) \\ 
\Phi_{13}(t)=exp(-\lfloor \hat{\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{13}(t_0) \\ 
\end{cases}
$$
由于初始的状态转移矩阵为单位矩阵，所以$\Phi_{12}(t_0)$和$\Phi_{13}(t_0)$均为0，$\Phi_{11}(t_0)=\mathbf{I}$，于是：
$$
\begin{cases}
\Phi_{11}(t, t_{0})=exp(\int_{t_0}^{t}(-\lfloor\hat{\omega}(t)\rfloor_{\times})dt)={}_{t_0}^{t}R \\
\Phi_{12}(t, t_{0})=\mathbf{0} \\
\Phi_{13}(t, t_{0})=\mathbf{0} \\
\end{cases} \tag{7}
$$

&nbsp;

#### 第三组

由于第二组中的$\dot{\Phi}_{21}$与$\Phi_{31}$相关，所以这里先推导第三组的情况：
$$
\begin{cases}
\Phi_{31}(t)=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor\hat{\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
\Phi_{32}(t)= \mathbf{I} \times \Phi_{32}(t_0) = \mathbf{0} \\ 
\Phi_{33}(t)= \mathbf{I}\times \Phi_{33}(t_0) = \mathbf{I} \\ 
\end{cases} \tag{8A}
$$
重点分析首行元素：
$$
\begin{aligned}
\Phi_{31}(t)&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor\hat{\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {}_{G}^{t}R  ({}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g})\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} {}_{t}^{G}R {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt ({}^{t_0}_{G}R)^{T} \\
&=-\lfloor {}^{G}v_{t}-{}^{G}v_{t_0}+{}^{G}\mathbf{g}(t-t_0) \rfloor_{\times}({}^{t_0}_{G}R)^{T}
\end{aligned} \tag{8B}
$$
其中：

1. $\mathbf{a}_{m}$表示在机体坐标系中的测量值，夹杂重力；
2. ${}^{G}\mathbf{a}$表示在世界坐标系下的加速度，不夹杂重力；
3. 最后一行化简引入了${}^{G}v_{t}={}^{G}v_{t_0}+\int{}^{G}\mathbf{a}dt$，其中的加速度不夹杂重力；

所以公式（8A）可以重新写作：
$$
\begin{cases}
\Phi_{31}(t)= -\lfloor {}^{G}v_{t}-{}^{G}v_{t_0}+{}^{G}\mathbf{g}(t-t_0) \rfloor_{\times}({}^{t_0}_{G}R)^{T}\\
\Phi_{32}(t)= \mathbf{0} \\ 
\Phi_{33}(t)= \mathbf{I} \\ 
\end{cases} \tag{8}
$$
&nbsp;

#### 第二组

将第三组的结果带入到第二组中
$$
\begin{cases}
\Phi_{21}(t)= -\int_{t_0}^{t} \int_{t_0}^{t}\lfloor {}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt d\tau({}^{t_0}_{G}R)^{T}\\
\Phi_{22}(t)= \mathbf{I} \times \Phi_{22}(t_0) = \mathbf{I} \\ 
\Phi_{23}(t)= \Delta{t}\times \Phi_{22}(t_0) = \mathbf{I}\Delta{t} \\ 
\end{cases} \tag{9A}
$$
重点分析首行元素：
$$
\begin{aligned}
\Phi_{21}(t) &= -\int_{t_0}^{t} \int_{t_0}^{t}\lfloor {}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt d\tau({}^{t_0}_{G}R)^{T} \\
&=-\lfloor {}^{G}p_{t}-{}^{G}p_{t_0}-{}^{G}v_{t_0}(t-t_0)+\frac{1}{2}{}^{G}\mathbf{g}(t-t_0)^2 \rfloor_{\times}({}^{t_0}_{G}R)^{T}
\end{aligned} \tag{9B}
$$
所以公式（9A）可以重新写作：
$$
\begin{cases}
\Phi_{21}(t)= -\lfloor {}^{G}p_{t}-{}^{G}p_{t_0}-{}^{G}v_{t_0}(t-t_0)+\frac{1}{2}{}^{G}\mathbf{g}(t-t_0)^2 \rfloor_{\times}({}^{t_0}_{G}R)^{T} \\
\Phi_{22}(t)= \mathbf{I} \\ 
\Phi_{23}(t)= \mathbf{I}\Delta{t} \\ 
\end{cases} \tag{9}
$$
&nbsp;

##### 小结

综合公式（7）（8）（9）可得：
$$
\boldsymbol{\Phi}\left(t, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t}R & 0 & 0 \\
-\lfloor \mathbf{\hat{y}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor \mathbf{\hat{s}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{10}
$$

&nbsp;

需要注意的是这里的转移矩阵是从 t0 时刻开始进行递推的。

&nbsp;

----

### OC-KF相邻时刻的状态转移矩阵

如上一章节可知，t1 时刻和 t2 时刻的状态转移矩阵分别如下：

#### t1 时刻的状态转移矩阵

$$
\boldsymbol{\Phi}\left(t_1, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t_1}R & 0 & 0 \\
-\lfloor \mathbf{\hat{y}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_1 \\
-\lfloor \mathbf{\hat{s}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{11}
$$

&nbsp;

#### t2 时刻的状态转移矩阵

$$
\boldsymbol{\Phi}\left(t_2, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{\hat{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{\hat{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{12}
$$

&nbsp;

#### t1 到 t2 时刻的状态转移矩阵

结合公式（11）（12）易得：
$$
\begin{aligned}
\Phi(t_2, t_1)&=\Phi(t_2, t_0) (\Phi(t_1, t_0))^{-1} \\
&=\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{\hat{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{\hat{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}_{t_0}^{t_1}R^{T} & 0 & 0 \\
(\lfloor \mathbf{\hat{y}}^{(t_1)} - \mathbf{\hat{s}}^{(t_1)}\Delta{t}_1 \rfloor_{\times})({}^{t_1}_{G}R)^{T} & \mathbf{I} & -\mathbf{I}\Delta{t}_1 \\
\lfloor \mathbf{\hat{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{\hat{y}}^{(t_2)}-\mathbf{\hat{y}}^{(t_1)}+\mathbf{\hat{s}}^{(t_1)}\Delta{t}_1- \mathbf{\hat{s}}^{(t_1)}\Delta{t}_2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_2-\Delta{t}_1） \\
-\lfloor \mathbf{\hat{s}}^{(t_2)}-\mathbf{\hat{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\end{aligned}
$$
