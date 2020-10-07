# [周结1]——MSCKF论文笔记

## Reference

1. A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation. MSCKF1.0的论文；
2. Quaternion Kinematics for Error-State KF. 关于四元数以及ESKF的论文；
3. Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight. MSCKF-VIO对应的论文；
4. https://github.com/KumarRobotics/msckf_vio 双目MSCKF的工程；

> PS: MSCKF的工程算是我见过最贴近论文的工程了，基本上工程中的数学部分在论文里面都给了出来，所以对于MSCKF1.0来说，论文是一定要弄懂的。本篇将参考[1]和参考[3]放在一起进行梳理，因为两者确实很相似，不过在参考[3]中作者对能观性进行了一定程度的补偿。



----

## EKF的状态变量构建

### 坐标系的表示问题

论文中{G}为世界(global)坐标系，{I}为IMU坐标系(或者称为机体坐标系)。

### 位姿表示方式

关于位姿的表示了，在大多数的SLAM系统中，位姿表示一般为$T=\{R^{G}_{I}|t^{G}_{I}\}$，但是MSCKF(或者大部分的EKF-base的VO/VIO系统)中的位姿表示变为了$T=\{R^{I}_{G}|p^{G}_{I}\}$，论文中都表示为$T=\{^{I}_{G}q| ^{G}p_{I}\}$，然后**其中最关键的一点是：四元数的表示方法使用的不是传统的Hamilton表示方法，而是JPL表示方法**，这两种表示方法的区别和联系可以看参考[[2]](https://arxiv.org/abs/1711.02508)，简单的说就是大佬们为了数学运算的一些性质，自己又搞了一个表示方法出来，借用参考[2]中的区别图如下：

<img src="pictures/MSCKF1_1.png"/>

这里还有一个笔者认为比较重要的不同就是微分方程也变化了一些，区别如下：

1. Hamilton表示法中，四元数的微分方程表示为$\dot{\mathbf{q}}=\frac{1}{2}\mathbf{q}\otimes\Omega(^Lw) $
2. JPL表示法中，四元数的微分方程表示为$\dot{\mathbf{q}}=\frac{1}{2}\Omega(^Lw)\otimes \mathbf{q}$

其中$\Omega(^Lw)$均可以表示为一个纯四元数的右乘矩阵形式：
$$
\Omega(w)=\begin{bmatrix}-[w]_{\times} & w \\ -w^T & 0 \end{bmatrix}
$$
笔者认为改性质的主要原因就是因为JPL的旋转是左手坐标系的，与右手坐标系的旋转刚好是旋转轴一致，但是旋转方向相反。

&nbsp;

### IMU状态变量的表示方式

IMU状态变量一如既往的还是用5个变量表示，如下：
$$
\mathrm{X}_{IMU}=\begin{bmatrix} ^{L}_{G}\overline{q}^T & b_g^{T} & ^{G}v_{I}^{T} & b_a^{T} & ^{G}p_{I}^{T} \end{bmatrix}^{T}   \tag{1}
$$
IMU的error-state向量表示如下：
$$
\tilde{\mathrm{X}}_{IMU}=\begin{bmatrix} \delta{\theta_{I}^T} & \tilde{b}_g^T & ^{G}\tilde{v}_{I}^T & \tilde{b}_{a}^T & ^{G}\tilde{p}_{I}^{T} \end{bmatrix}^{T}   \tag{2}
$$
其中$\delta{\theta}$为旋转向量，由$\delta{\overline{q}}=\left[\frac{1}{2}\delta{\theta}^T, 1\right]^T$表示，唯一需要注意的就是一旦这样表示了，就意味着假设了旋转角度很小，因为是error-state，所以这个假设是十分成立的；

&nbsp;

### 带相机位姿的状态变量表示方式

k时刻的整个滤波器的位姿为：
$$
\mathrm{\widehat{X}_{k}}=\begin{bmatrix}\mathrm{\widehat{X}_{IMU}} & ^{C_1}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_1}^T & ... & ^{C_N}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_N}^T	\end{bmatrix}^T  \tag{3}
$$
对应的error-state为：
$$
\mathrm{\tilde{X}_{k}}=\begin{bmatrix}\mathrm{\tilde{X}_{IMU}} & \delta{\theta}_{C_1} & ^{G}\widehat{p}_{C_1}^T & ... & \delta{\theta}_{C_N} & ^{G}\tilde{p}_{C_N}^T	\end{bmatrix}^T  \tag{4}
$$
&nbsp;

### 状态传导（Propagation）

对于EKF-Base的方法而言，状态传导部分基本上依赖于IMU的运动方程，使用IMU的测量进行最新时刻的位姿的推导，并以此作为机体位姿转移到相机位姿上，在连续时域上，IMU的运动方程如下：
$$
\begin{aligned}
\begin{cases}
^{I}_{G}\dot{\bar{q}}(t) &=\frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}(t))_{G}^{I} \bar{q}(t) \\ 
\dot{\mathbf{b}}_{g}(t) &=\mathbf{n}_{w g}(t) \\
^{G}\dot{\mathbf{v}}(t) &= ^{G}\mathbf{a}(t) \\ 
\dot{\mathbf{b}}_{a}(t) &=\mathbf{n}_{w a}(t) \\
^{G}\dot{\mathbf{p}}(t) &= ^{G}\mathbf{v}(t)
\end{cases}
\end{aligned}   \tag{5}
$$
其中四元数为JPL表示法，$\Omega$为上面的纯四元数右乘矩阵形式。

公式（5）中的参数均为理想情况下的参数，实际过程中的运动方程为：
$$
\begin{aligned}
\begin{cases}
^{I}_{G}\dot{\widehat{\bar{q}}}(t) &=\frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\widehat\omega}(t))_{G}^{I} \bar{q}(t) \\ 
\dot{\mathbf{\widehat b}}_{g}(t) &=0 \\
^{G}\dot{\mathbf{\widehat v}}(t) &= \mathbf{C}_{\hat{q}}^{T} \hat{\mathbf{a}}-2\left\lfloor\boldsymbol{\omega}_{G} \times\right\rfloor^{G} \hat{\mathbf{v}}_{I}-\left\lfloor\boldsymbol{\omega}_{G} \times\right\rfloor^{2} G_{\hat{\mathbf{p}}_{I}}+{ }^{G} \mathbf{g} \\ 
\dot{\mathbf{\widehat b}}_{a}(t) &= 0 \\
^{G}\dot{\mathbf{p}}(t) &= ^{G}\mathbf{\widehat v}(t)
\end{cases}
\end{aligned}   \tag{6}
$$
其中：

1. $\widehat \omega(t)=\omega_{m}-\widehat b_{g}-R^{b}_{w}\omega_{G}$，表示实际过程中通过测量减去零偏的值，同时作者这里考虑了地球转动的影响；
2. $\widehat a=a_m-\widehat b_a$，表示实际过程中通过测量减去零偏的值，作者在计算速度的微分时，也考虑了地球转动对于测量值的影响；
3. 以上均在连续时域中进行的分析，对连续函数进行离散化时需要在位置上考虑加速度的影响，这样会更加的准确一些；

&nbsp;

#### 误差状态的微分方程

这部分请读者参考参考[2](https://arxiv.org/abs/1711.02508)第五章的内容，这里直接给出结论，如下：
$$
\dot{\tilde{\mathbf{X}}}_{\mathrm{IMU}}=\mathbf{F} \widetilde{\mathbf{X}}_{\mathrm{IMU}}+\mathbf{G} \mathbf{n}_{\mathrm{IMU}}  \tag{7}
$$
其中：

1. $$
   \mathbf{F}=\left[\begin{array}{ccccc}
   -\lfloor\hat{\boldsymbol{\omega}} \times\rfloor & -\mathbf{I}_{3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
   -\mathbf{C}_{\hat{q}}^{T}\lfloor\hat{\mathbf{a}} \times\rfloor & \mathbf{0}_{3 \times 3} & -2\left\lfloor\boldsymbol{\omega}_{G} \times\right\rfloor & \mathbf{-} \mathbf{C}_{\hat{q}}^{T} & -\left\lfloor\boldsymbol{\omega}_{G} \times\right. \\
   \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
   \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{I}_{3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3}
   \end{array}\right]
   $$
   
2. $$
   \mathbf{G}=\left[\begin{array}{cccc}
   -\mathbf{I}_{3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
   \mathbf{0}_{3 \times 3} & \mathbf{I}_{3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
   \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & -\mathbf{C}_{\hat{q}}^{T} & \mathbf{0}_{3 \times 3} \\
   \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{I}_{3} \\
   \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3}
   \end{array}\right]
   $$

稍微不同的一点是参考2中考虑了重力，而这里并没有考虑重力，同时这里多考虑了地球的自转影响。

&nbsp;

#### 状态传递矩阵的推导

ESKF花了很大的功夫在推导**误差状态**的传递方程，但是实际上对于EKF-Base的方法而言，最重要的还是**状态**的传递方程。李明扬大佬在MSCKF2.0的论文中为了保证状态传递矩阵的特性，对**状态**的传递方程进行了**理论上的推导（相对于数值推导）**，这个MSCKF2.0的理论推导放在后面去讲，本文还是按照MSCKF1.0的推导进行。

这里

根据误差状态的传递公式（7），稍作一下变换就可以得到（注意状态传递的时候就不需要白噪声项了）：
$$
\begin{aligned}
\dot{\tilde{\mathbf{X}}}_{\mathrm{t}}&=\dot{(\mathbf{X}_{t}-\widehat{\mathbf{X}}_{t})}=\dot{(\Phi(t, t0) \mathbf{X}_{t0}-\widehat{\Phi(t, t0)}\widehat{\mathbf{X}}_{t0})}\\
&\approx \dot{\Phi(t, t0)}(\mathbf{X}_{t0}-\widehat{\mathbf{X}_{t0}}) \\
&=\mathbf{F} \tilde{\mathbf{X}}_{\mathrm{t}} \\
&\approx \mathbf{F}\Phi(t, t0)(\mathbf{X}_{t0}-\widehat{\mathbf{X}_{t0}}) 
\end{aligned}  \tag{8}
$$

于是可以得到转移
