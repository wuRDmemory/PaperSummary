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

### 何时进行更新

根据参考1中III-E节，MSCKF在两个事件上进行更新操作：

1. 当一个特征点被跟踪丢失的时候，此时特征点的信息不再变化，且此时进行三角化的时候，相机位姿的信息也均是优化多次的位姿，较为准确，**我们称之为情况1**；
2. 当窗口中的相机位姿达到上限的时候，此时势必要把一帧旧的相机位姿滑动出去，此时算法会将所有与该位姿相关的特征点用于更新，最大程度的再次利用这个位姿，**我们称之为情况2**；

在S-MSCKF中，上面的两个部分对应两个函数：

1. removeLostFeatures();
2. pruneCamStateBuffer();

后面会简单看一下实际工程中作者是如何做的。

&nbsp;

### 单个特征点的观测方程

在MSCKF中，如果一个点在k时刻被跟丢了，那么该特征点会用来更新整个看到它的相机位姿。

假设特征点$f_j$在世界坐标系下的位置为${}^{G}P_{f_j}$，于是对于相机$i$，可以通过位姿将其转换到相机坐标系下：
$$
{ }^{C_{i}} \mathbf{p}_{f_{j}}=\left[\begin{array}{c}
{ }^{C_{i}} X_{j} \\
{ }^{C_{i}} Y_{j} \\
{ }^{C_{i}} Z_{j}
\end{array}\right]=\mathbf{C}\left({ }_{G}^{C_{i}} \bar{q}\right)\left({ }^{G} \mathbf{p}_{f_{j}}-{ }^{G} \mathbf{p}_{C_{i}}\right) \tag{8}
$$
于是对应的观测就是：
$$
\mathbf{{z}}_{i}^{(j)}=\frac{1}{{}^{C_{i}}Z_{j}}\left[\begin{array}{c}
{}^{C_{i}} X_{j} \\
{}^{C_{i}} Y_{j}
\end{array}\right]+\mathbf{n}_{i}^{(j)}, \quad i \in \mathcal{S}_{j} \tag{9}
$$
但是到这个地方其实还并不是MSCKF的观测方程，因为MSCKF的估计变量为error-state，所以进一步定义误差：
$$
\begin{aligned}
r^{(j)}_{i}&=z^{(j)}_{i}-\hat{z}^{(j)}_{i}(\mathrm{{x}_i}) \\
&=z^{(j)}_{i}-\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i}+\mathrm{\tilde{x}_i}) \\
&=z^{(j)}_{i}-(\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i})+\mathrm{J^{(j)}_{\hat{x}_i}}\mathrm{\tilde{x}_i}+\mathrm{J^{(j)}_{{}^{G}\hat{p}_{f_j}}}\mathrm{{}^{G}\tilde{p}_{f_j}})  \\
&=\mathrm{H}^{(j)}_{\mathrm{\hat{x}}_i}\mathrm{\tilde{x}_i}+\mathrm{H}^{(j)}_{\hat{p}_{f_j}}\mathrm{{}^{G}\tilde{p}_{f_j}}+\mathbf{n}_{i}^{(j)}
\end{aligned}   \tag{10}
$$
上面的公式（10）就是我们需要的对于error-state的观测公式了；

&nbsp;

### 对于观测方程的化简

这一步主要是对观测方程进行变换，主要有两个目的：

1. 将特征点的优化从状态空间中去掉，这样整个状态空间中估计的就只有相机的位姿，大大减少待估计的变量数；
2. 由于MSCKF对特征点采用的是延迟初始化的策略，因此当该特征点被三角化的时候，其值已经足够精准，且如果是情况1的话，这个优化显得费力不讨好；

对于特征点$f_j$而言，假设所有观测到该特征点的相机为$\mathrm{x}_i, i \in [1, N]$，根据公式（10），那么一共有N个residual可以叠加在一起，于是有：
$$
\mathbf{r}^{(j)} \simeq \mathbf{H}_{\mathbf{X}}^{(j)} \widetilde{\mathbf{X}}+\mathbf{H}_{f}^{(j) G} \widetilde{\mathbf{p}}_{f_{j}}+\mathbf{n}^{(j)} \tag{11}
$$
其中：

1. residual的维度为2N x 1；
2. $\mathbf{H}^{(j)}_{\mathbf{X}}$维度为2N x 5M，其中M为滑动窗口中所有的相机数；
3. $\mathbf{H}^{(j)}_{\mathbf{f}}$维度为2N x 3；

容易看到，当N大于2时，$\mathbf{H}^{(j)}_{f}$的秩为3（这里不用过分纠结有没有可能是秩<=3，简单说，只要有重投影误差，那么秩就一定>=3），于是其左零空间的维度为（2N-3），假设为矩阵$\mathbf{A^T}$（左零一定加转置符号哈），其维度为（2N-3 x 2N）所以公式（11）两端均乘以矩阵$\mathbf{A^{T}}$得到：
$$
\begin{aligned}
\mathbf{r}^{(j)}_{o} = \mathbf{A^{T}}\mathbf{r}^{(j)} &\simeq \mathbf{A^{T}}\mathbf{H}_{\mathbf{X}}^{(j)} \widetilde{\mathbf{X}}+\mathbf{A^{T}}\mathbf{H}_{f}^{(j) G} \widetilde{\mathbf{p}}_{f_{j}}+\mathbf{A^{T}}\mathbf{n}^{(j)} \\
&= \mathbf{H}_{o}^{(j)} \widetilde{\mathbf{X}} + \mathbf{n}_{o}^{(j)}
\end{aligned}   \tag{12}
$$
其中：

1. 上式中所有矩阵或者向量的行数均为（2N-3）；

2. 噪声矩阵的协方差满足：
   $$
   E\left\{\mathbf{n}_{o}^{(j)} \mathbf{n}_{o}^{(j) T}\right\}=\sigma_{\mathrm{im}}^{2} \mathbf{A}^{T} \mathbf{A}=\sigma_{\mathrm{im}}^{2} \mathbf{I}_{2 M_{j}-3} \tag{13}
   $$
   

