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
-\mathbf{C}_{\hat{q}}^{T}\lfloor\hat{\mathbf{a}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
$$
由于此处的G对于能观性的分析没有实质性的用处，这里不作分析；

将微分方程转为离散的形式：
$$
\boldsymbol{\tilde{X}}\left(t_{k+1}\right)=\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right) \boldsymbol{\tilde{X}}\left(t_{k}\right)+\int_{t_{k}}^{t_{k+1}} \boldsymbol{\Phi}\left(t_{k+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{3}
$$
其中：
$$
\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right)=\exp(\int_{t_{k}}^{t_{k+1}} \boldsymbol{F}(t) \mathrm{d} t)   \tag{4}
$$
&nbsp;

