# IMU-Model

这次来详细总结一下IMU的一些东西，主要包含：

- 运动模型
- 观测模型
- 误差状态方程



### IMU的测量模型

通常，我们将IMU的测量模型写作如下模型（注意不是观测模型）：
$$
w_m = w_t+b_w+n_w \\
a_m = a_t+b_a+n_a
$$
亦即：“测量值=真值+零偏+白噪声”，其中：

1. 零偏值是我们希望实时进行估计的，会在状态变量中进行估计；
2. 零偏值通常被建模为随机游走；



### 高斯白噪声

高斯白噪声其实是我们常见的噪声模型，一个典型的高斯白噪声满足：
$$
\begin{array}{c}{E[n(t)]=0} \\ {E\left[n\left(t_{1}\right) n\left(t_{2}\right)\right]=\sigma_{g}^{2} \delta\left(t_{1}-t_{2}\right)}\end{array}
$$
如果一个传感器的噪声仅仅包含高斯白噪声（或者白噪声）的话，那么基本上，使用平均滤波是能得到较好的有效值的，因为白噪声的期望值是0；

下一步把连续过程离散化，可以比较直观的得到：$n_d(k) = \sigma_{gd}w(k)$，其中$\sigma_d$是高斯白噪声的方差，而$w(k)$为标准的正态分布。

值得说明的一点是：$\sigma_{gd}$与连续空间的方差并不相同，而是等于$\sigma_g/\sqrt{\Delta t}$，直接去想的话，只能认为因为离散空间的积分与时间间隔相关，每次积分都要乘以$\Delta t$，所以最终的方差多了一个$\Delta t$，所以要除下来；但是这里还是给出数学上的推导：
$$
\begin{aligned} n_{d}[k] & \triangleq n\left(t_{0}+\Delta t\right) \simeq \frac{1}{\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} n(\tau) d t \\ E\left(n_{d}[k]^{2}\right) &=E\left(\frac{1}{\Delta t^{2}} \int_{t_{0}}^{t_{0}+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} n(\tau) n(t) d \tau d t\right) \\ &=E\left(\frac{\sigma_{g}^{2}}{\Delta t^{2}} \int_{t_{0}}^{t_{0}+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} \delta(t-\tau) d \tau d t\right) \\ &=E\left(\frac{\sigma_{g}^{2}}{\Delta t}\right) \end{aligned}
$$
一开始我比较不理解第一行的公式，但是后来仔细想想，其实一个时刻的高斯噪声值可以看做是某个时间段内所有值的平均。需要注意的是，高斯白噪声由概率密度描述的，在Kalibr的配置文件中，需要输入加计和陀螺仪的”噪声密度“值，对应$\sigma_{gd}$。



#### Bias的随机游走

随机游走过程本质上是离散的，大家可以想象一些折线图，每个拐点就是那一时刻的值；其在连续空间被建模为一个维纳过程，也就是导数为高斯白噪声的过程：
$$
\dot{b_g}=n(t)
$$
根据上面得到的高斯白噪声模型，随机游走的离散模型就比较简单了：
$$
\begin{align}
b_g(k+1) &=b_g(k)+\sigma_{gd}w(t)\Delta{t} \\
&=b_g(k)+\sigma_{g}\sqrt{\Delta t} w(t)
\end{align}
$$
在Kalibr的配置文件中，需要输入加计和陀螺仪的随机游走的方差值，也就是上面的$\sigma_g\sqrt{\Delta t}$。





