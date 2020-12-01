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

INS系统的重中之重，还是先来推导IMU的状态传递方程。为了和参考【3】的推导保持一致，这里的状态变量也定为$\mathbf{X}=\left[{}^{I}_{G}\mathrm{q}\quad {}^{G}p_{I}\quad {}^{G}v_{I} \right]$，忽略零偏的部分。

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

> Notation：
>
> 以下均表示理想情况下的状态传递



由公式（4）的第一行可以列出如下公式，这里把时间跨度直接认为是 t，初始的时间为 t0：
$$
\begin{aligned}
\begin{bmatrix}
\dot{\Phi}_{11}(t) & \dot{\Phi}_{12}(t) & \dot{\Phi}_{13}(t) \\
\dot{\Phi}_{21}(t) & \dot{\Phi}_{22}(t) & \dot{\Phi}_{23}(t) \\
\dot{\Phi}_{31}(t) & \dot{\Phi}_{32}(t) & \dot{\Phi}_{33}(t) \\
\end{bmatrix}
&=\left[\begin{array}{ccc}
-\lfloor{\boldsymbol{\omega}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}^{G}_{t}R)^{T}\lfloor{\mathbf{a_m}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
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
\dot{\Phi}_{31}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{11}(t) \\
\dot{\Phi}_{32}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{12}(t) \\ 
\dot{\Phi}_{33}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{13}(t) \\ 
\end{cases} \tag{6}
$$
下面就是一组一组的展开一下：

&nbsp;

#### 第一组

容易看出，第一组的闭式解其实都是exp函数，所以：
$$
\begin{cases}
\Phi_{11}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{11}(t_0) \\
\Phi_{12}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{12}(t_0) \\ 
\Phi_{13}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{13}(t_0) \\ 
\end{cases}
$$
由于初始的状态转移矩阵为单位矩阵，所以$\Phi_{12}(t_0)$和$\Phi_{13}(t_0)$均为0，$\Phi_{11}(t_0)=\mathbf{I}$，于是：
$$
\begin{cases}
\Phi_{11}(t, t_{0})=exp(\int_{t_0}^{t}(-\lfloor {\omega}(t)\rfloor_{\times})dt)={}_{t_0}^{t}R \\
\Phi_{12}(t, t_{0})=\mathbf{0} \\
\Phi_{13}(t, t_{0})=\mathbf{0} \\
\end{cases} \tag{7}
$$

&nbsp;

#### 第三组

由于第二组中的$\dot{\Phi}_{21}$与$\Phi_{31}$相关，所以这里先推导第三组的情况：
$$
\begin{cases}
\Phi_{31}(t)=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
\Phi_{32}(t)= \mathbf{I} \times \Phi_{32}(t_0) = \mathbf{0} \\ 
\Phi_{33}(t)= \mathbf{I}\times \Phi_{33}(t_0) = \mathbf{I} \\ 
\end{cases} \tag{8A}
$$
重点分析首行元素：
$$
\begin{aligned}
\Phi_{31}(t)&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {}_{G}^{t}R  ({}^{G} {\mathbf{a}}(t)+{}^{G}\mathbf{g})\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} {}_{t}^{G}R {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt ({}^{t_0}_{G}R)^{T} \\
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
\Phi_{21}(t)= -\int_{t_0}^{t} \int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt d\tau({}^{t_0}_{G}R)^{T}\\
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
-\lfloor \mathbf{{y}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor \mathbf{{s}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
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
-\lfloor \mathbf{{y}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_1 \\
-\lfloor \mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{11}
$$

&nbsp;

#### t2 时刻的状态转移矩阵

$$
\boldsymbol{\Phi}\left(t_2, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
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
-\lfloor \mathbf{{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}_{t_0}^{t_1}R^{T} & 0 & 0 \\
(\lfloor \mathbf{{y}}^{(t_1)} - \mathbf{{s}}^{(t_1)}\Delta{t}_1 \rfloor_{\times})({}^{t_1}_{G}R)^{T} & \mathbf{I} & -\mathbf{I}\Delta{t}_1 \\
\lfloor \mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)}-\mathbf{{y}}^{(t_1)}+\mathbf{\hat{s}}^{(t_1)}\Delta{t}_1- \mathbf{\hat{s}}^{(t_1)}\Delta{t}_2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_2-\Delta{t}_1） \\
-\lfloor \mathbf{{s}}^{(t_2)}-\mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\end{aligned} \tag{13}
$$

&nbsp;

-----

## 视觉部分的观测方程

下面的所有的下角标 $l$ 表示id为 $l$ 的相机，不表示时间，这里暂时不涉及时间，可以认为是理想的观测模型。

为了分析能观性，这里还需要一个步骤就是观测模型，以单个观测点$P_{f_j}$为例，其观测模型为：
$$
\begin{aligned}
z_l&=\pi({}^{C_l}\mathrm{p}_{f_j})+n_{l} \\
{}^{C_l}\mathrm{p}_{f_j}&={}^{C}_{I}R {}^{G}_{l}R({}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l})+{}^{C}\mathrm{p}_I 
\end{aligned} \tag{14}
$$

所以观测模型为：
$$
\begin{aligned}
H_{(I_l|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\right]_{\times}({}_{G}^{I_l}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}}\end{bmatrix} \\ 
H_{(f_j|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\end{aligned} \tag{15}
$$
其中：
$$
J_{(f_j|l)}=\frac{1}{Z}\begin{bmatrix}1 & 0 & -\frac{X}{Z} \\ 0 & 1 & -\frac{Y}{Z} \end{bmatrix}
$$
&nbsp;

-----

## 理想情况下能观性的分析

OC-KF在做能观性分析的时候，不像参考【3】中所示的是在能观性矩阵中抽出一部分进行通用的分析，该方法采用递推的方式证明了在时刻 t，零空间应该是什么样的，且理想状态下是如何传播的（propagation），下面就两个部分进行分析：

### 在 t 时刻，系统零空间是如何的

假设系统从 t0 时刻开始，那么该时刻的零空间为：
$$
\mathbf{N}_{t_0}=\begin{bmatrix}
\begin{array}{c|c}
\mathbf{0} & {}^{t_0}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_0} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_0} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{array}
\end{bmatrix} \tag{16}
$$
根据能观性矩阵的定义，在 t 时刻，第$l$个节点对 $f_j$ 特征点的能观性矩阵的对应行：
$$
\mathcal{O}_{l}^{(t)}=\mathbf{H}_{f_j}\Phi(t, t_0) \tag{17}
$$
所以