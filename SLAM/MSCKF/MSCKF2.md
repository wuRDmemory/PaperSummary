# MSCKF（三）——更新部分

----

## Reference

1. A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation. MSCKF1.0的论文；
2. Quaternion Kinematics for Error-State KF. 关于四元数以及ESKF的论文；
3. Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight. S-MSCKF对应的论文；
4. https://github.com/KumarRobotics/msckf_vio S-MSCKF的工程；
5. https://zhuanlan.zhihu.com/p/76341809 知乎上大佬对MSCKF的总结，本文没有过多的程序方面的讲解，都是从理论上推导的，也是个人的一些解惑过程；
6. https://blog.csdn.net/wubaobao1993/article/details/109299097  笔者关于预测部分的总结；

&nbsp;

-----

## 参数的表示

首先重新回顾一下MSCKF的要估计的参数：
$$
\tilde{\mathrm{X}}_{IMU}=\begin{bmatrix} ^{I}_{G}\delta{\theta^T} & \tilde{b}_g^T & ^{G}\tilde{v}_{I}^T & \tilde{b}_{a}^T & ^{G}\tilde{p}_{I}^{T} & _{I}^{C}\delta{\theta}^T & ^{I}\tilde{p}_{C}^T \end{bmatrix}^{T}   \tag{1}
$$
整个过程中的参数向量为：
$$
\mathrm{\tilde{X}_{k}}=\begin{bmatrix}\mathrm{\tilde{X}_{IMU}} & \delta{\theta}_{C_1} & ^{G}\tilde{p}_{C_1}^T & ... & \delta{\theta}_{C_N} & ^{G}\tilde{p}_{C_N}^T	\end{bmatrix}^T  \tag{2}
$$

其次整个框架需要求解的变量其实还是机体的位姿信息：
$$
\mathrm{\widehat{X}_{k}}=\begin{bmatrix}\mathrm{\widehat{X}_{IMU}} & ^{C_1}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_1}^T & ... & ^{C_N}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_N}^T	\end{bmatrix}^T  \tag{3}
$$
&nbsp;

----

## MSCKF的预测部分

这里简单回顾MSCKF的预测部分，详细可以阅读参考[6]；

### IMU误差状态的传导

对于每一帧IMU数据，都可以根据误差状态的微分方程求得状态转移部分的递推方程的闭式解，如下：
$$
\begin{aligned}
\begin{cases}
\mathrm{\tilde{x}_{k+1}^{IMU}} &= \Phi(\mathrm{k+1},\mathrm{k})\mathrm{\tilde{x}_k^{IMU}} \\
\mathrm{P_{k+1|k}^{IMU}} &=\Phi(\mathrm{k+1},\mathrm{k})\mathrm{P_{k|k}^{IMU}}\Phi(\mathrm{k+1},\mathrm{k})^T + \Phi(\mathrm{k+1},\mathrm{k})\mathrm{G}\mathrm{N}\mathrm{G}^T\Phi(\mathrm{k+1},\mathrm{k})^T
\end{cases}
\end{aligned}  \tag{4}
$$
&nbsp;

### 新帧到达前的协方差矩阵更新

新帧未到来之前，由于没有任何新的观测可以影响相机位姿，因此整个协方差的更新过程如下：
$$
\mathrm{P}_{k+1|k}=\begin{bmatrix} \mathrm{P}^{IMU}_{k+1|k} & \Phi(k+1, k)\mathrm{P}^{IC}_{k|k} \\ (\Phi(k+1, k)\mathrm{P}^{IC}_{k|k})^T & \mathrm{P}^{CAM}_{k|k} \end{bmatrix}  \tag{5}
$$
&nbsp;

### 新帧到来时的协方差矩阵的扩展

新帧到来的时候，算法直接使用外参矩阵讲IMU的最新位姿作为其位姿，因此两者的误差状态满足：
$$
\mathrm{\tilde{X}^{CAM}_{k+1|k}}=\begin{bmatrix} {}_{G}^{C}\delta{\theta} \\ ^{G}\tilde{p}_{C} \end{bmatrix}=\begin{bmatrix}\mathrm{\hat{R}_{I}^{C}} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{I}_{3x3} & \mathbf{0}_{3x3} \\ 
-\mathrm{\hat{R}_{G}^{I}}^{T}([{}^{I}\mathrm{\hat{p}_{C}}]_{\times}) & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{I}_{3x3} & \mathbf{0}_{3x3} & \mathrm{\hat{R}_{G}^{I}}^{T} \end{bmatrix} \begin{bmatrix} ^{I}_{G}\delta{\theta} \\ \tilde{b}_g \\ ^{G}\tilde{v}_{I} \\ \tilde{b}_{a} \\ ^{G}\tilde{p}_{I} \\ _{I}^{C}\delta{\theta} \\ ^{I}p_{C} \end{bmatrix} = \mathbf{J}^{CAM}_{IMU} \mathrm{\tilde{X}^{IMU}_{k+1|k}} \tag{6}
$$
所以整个的协方差矩阵的扩展过程如下：
$$
\mathbf{P}_{k+1 \mid k} \leftarrow\left[\begin{array}{c}
\mathbf{I}_{6 N+21} \\
\mathbf{J}
\end{array}\right] \mathbf{P}_{k+1 \mid k}\left[\begin{array}{c}
\mathbf{I}_{6 N+21} \\
\mathbf{J}
\end{array}\right]^{T} \tag{7}
$$

&nbsp;

----

## MSCKF更新部分

该部分主要探讨新帧的观测是如何影响整个滤波的。

