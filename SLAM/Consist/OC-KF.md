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
\boldsymbol{\tilde{X}}\left(t_{k}\right)=\boldsymbol{\Phi}\left(t_{k}, t_{0}\right) \boldsymbol{\tilde{X}}\left(t_{0}\right)+\int_{t_{0}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{3}
$$
其中：
$$
\begin{cases}
\dot{\boldsymbol{\Phi}}\left(t, t_{0}\right) = \boldsymbol{F}(t)\boldsymbol{\Phi}\left(t, t_{0}\right) \\
\boldsymbol{\Phi}\left(t_{k}, t_{0}\right)=\exp(\int_{t_{0}}^{t_{k}} \boldsymbol{F}(t) \mathrm{d} t)  
\end{cases} \tag{4}
$$
&nbsp;

### OC-KF的传递方程

在OC-KF中，IMU部分的传递方程使用的是MSCKF1.0中的微分方程的形式，这里先来推导状态转移矩阵的闭式解。

由公式（4）的第一行可以看出：
$$
\begin{aligned}
\begin{bmatrix}
\dot{\Phi}_{11}(t) & \dot{\Phi}_{12}(t) & \dot{\Phi}_{13}(t) \\
\dot{\Phi}_{21}(t) & \dot{\Phi}_{22}(t) & \dot{\Phi}_{23}(t) \\
\dot{\Phi}_{31}(t) & \dot{\Phi}_{32}(t) & \dot{\Phi}_{33}(t) \\
\end{bmatrix}
&=\left[\begin{array}{ccc}
-\lfloor\hat{\boldsymbol{\omega}}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}^{G}_{I_t}R)^{T}\lfloor\hat{\mathbf{a_m}}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
\begin{bmatrix}
{\Phi}_{11}(t) & {\Phi}_{12}(t) & {\Phi}_{13}(t) \\
{\Phi}_{21}(t) & {\Phi}_{22}(t) & {\Phi}_{23}(t) \\
{\Phi}_{31}(t) & {\Phi}_{32}(t) & {\Phi}_{33}(t) \\
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
\Phi_{11}(t)={}_{}^{}
\end{cases}
$$
