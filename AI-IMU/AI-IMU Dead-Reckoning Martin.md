# AI-IMU Dead-Reckoning Martin

## 论文出处
论文：[https://arxiv.org/abs/1904.06064](https://arxiv.org/abs/1904.06064) 
代码：[https://github.com/mbrossar/ai-imu-dr](https://github.com/mbrossar/ai-imu-dr)

&nbsp;
---
## 整体介绍
a. 本文主要提出了一种方法，能够仅仅使用IMU的航位推算得到较为精准的轨迹，如下图所示：

<img src="/home/ubuntu/Projects/PaperSummary/AI-IMU/imgs/20190530-164749.png" width=500/>

b. 该方法只适用于像汽车一样只能向前跑的系统，因为这样的系统在进行kalman滤波器的时候可以加入动力模型的辅助信息，在这里，辅助信息主要是指系统在侧向$y^{lat}$和与地面垂直的方向$y^{up}$上是没有移动的，文中称之为伪测量；

c. 论文的重要改进点在于，(2)中辅助信息的均值可以认为是0，但是协方差是否一直都是一个值呢？显然，在系统进行转弯的时候，水平方向的协方差要比直线方向的协方差是要大很多的，同理，在爬坡的时候，垂直地面方向的协方差也应该比直线的时候协方差是要大的；鉴于一般的滤波器都是人为设计协方差矩阵，同时设计好的协方差不能修改，因此文章希望使用CNN（并非RNN，作者说RNN不是很好）对观测的协方差进行动态的更新，进一步的，作者使用CNN也同时预测出来了上述的伪测量的值。

&nbsp;
---
## 系统实现
### IMU模型定义
一般情况下，IMU的模型建立为真值+零偏+高斯白噪声，如下：
$$
\begin{align}
w_{n}^{IMU} &= w_{n}+b_{n}^{w}+w_{n}^{w}  \\  a_{n}^{IMU} &=a_{n}+b_{n}^{a}+w_{n}^{a}
\end{align} \tag{1}
$$
其中零偏一般是一个随机游走的“准常量”（quasi-constant），因此对零偏进行建模，其模型如下：
$$
\begin{align}
b_{n+1}^{w} &= b_{n}^{w}+w_{n}^{b_w}  \\  
b_{n+1}^{a} &= b_{n}^{a}+w_{n}^{b_a}
\end{align}  \tag{2}
$$
其中$w_{n}^{w}, w_{n}^{a}, w_{n}^{b_w}, w_{n}^{b_a}$为高斯分布的白噪声

### IMU的运动模型
IMU的运动模型可以定义为如下形式：
$$
\begin{aligned} 
\mathbf{R}_{n+1}^{\mathrm{IMU}} &=\mathbf{R}_{n}^{\mathrm{IMU}} \exp \left(\left(\boldsymbol{\omega}_{n} d t\right)_{ \times}\right)  \\
\mathbf{v}_{n+1}^{\mathrm{IMU}} &=\mathbf{v}_{n}^{\mathrm{IMU}}+\left(\mathbf{R}_{n}^{\mathrm{IMU}} \mathbf{a}_{n}+\mathbf{g}\right) d t \\ 
\mathbf{p}_{n+1}^{\mathrm{IMU}} &=\mathbf{p}_{n}^{\mathrm{IMU}}+\mathbf{v}_{n}^{\mathrm{IMU}}dt
\end{aligned}  \tag{3}
$$
其中的所有变量都是真值形式，没有考虑到偏差等等的因素，如果把带有偏差的值运用到上面动态模型中，则误差会十分大，甚至比直接使用轮子的积分来的更差。

### KalmanFilter的建模
#### 1. 状态变量
状态变量选择为：
$$
\mathbf{x}_{n} :=\left(\mathbf{R}_{n}^{\mathrm{IMU}}, \mathbf{v}_{n}^{\mathrm{IMU}}, \mathbf{p}_{n}^{\mathrm{IMU}}, \mathbf{b}_{n}^{\omega}, \mathbf{b}_{n}^{\mathrm{a}}, \mathbf{R}_{n}^{\mathrm{c}}, \mathbf{p}_{n}^{\mathrm{c}}\right)
$$
其中系统的输入为IMU的测量值，$\mathbf{R}_{n}^{\mathrm{IMU}}，\mathbf{v}_{n}^{\mathrm{IMU}}，\mathbf{p}_{n}^{\mathrm{IMU}}$是载体在第n步（或者理解为t时刻）的姿态，速度和位置，其中的参考系都是世界坐标系，坐标系的信息如下图:

<img src="/home/ubuntu/Projects/PaperSummary/AI-IMU/imgs/1111.png" width=500>

可以看到每个变量都是从载体系到世界坐标系的转换，不要被变量上面的IMU所迷惑了；$\mathbf{b}_{n}^{\omega}, \mathbf{b}_{n}^{\mathrm{a}}$是IMU的零偏，$\mathbf{R}_{n}^{\mathrm{c}}, \mathbf{p}_{n}^{\mathrm{c}}$是IMU到载体系的外参矩阵，这里也做作为一个更新量进行了估计。

#### 2. 系统方程
同正常的kalman滤波步骤一样，我们首先建立预测方程（文章中把预测阶段称为传播propagate）和更新方程：
$$
\begin{align}
\mathbf{x}_{n+1} &=f\left(\mathbf{x}_{n}, \mathbf{u}_{n}\right)+\mathbf{w}_{n} \tag{4} \\
\mathbf{y}_{n} &=h\left(\mathbf{x}_{n}\right)+\mathbf{n}_{n} \tag{5}
\end{align} 
$$

其中正如上面所说，$y_n$是辅助变量，理想情况下，我们的设置值为[0, 0]，即$h\left(\mathbf{x}_{n}\right)\approx0$

#### 3. 伪测量的建立
这部分更类似于观测方程的建立，主要是把状态估计中的速度（载体在世界坐标系下的速度，即$^WV_n$）转移到载体系下
$$
\mathbf{v}_{n}^{\mathrm{c}}=\left[ \begin{array}{c}{v_{n}^{\mathrm{for}}} \\ {v_{n}^{\mathrm{lat}}} \\ {v_{n}^{\mathrm{up}}}\end{array}\right]=(\mathbf{R}_{n}^{\mathrm{c}})^T (\mathbf{R}_{n}^{\mathrm{IMU}})^T \mathbf{v}_{n}^{\mathrm{IMU}}+\left(\boldsymbol{\omega}_{n}\right) \times \mathbf{p}_{n}^{\mathrm{c}}
$$
对于汽车等只能向前的物体，上述的$v_n^{lat}, v_n^{up}$的值应该近似于0，因此伪量测为
$$
y_n=\left[\begin{array}{c}{y_n^{lat}} \\ {y_n^{up}}\end{array}\right]=\left[\begin{array}{c}{h^{lat}(x_n)+n_n^{lat}} \\ {h^{up}(x_n)+n_n^{up}}\end{array} \right]=\left[\begin{array}{c}{v_n^{lat}} \\ {v_n^{up}}\end{array} \right] + n_n
$$
其中$n_n$为噪声，这里算作一个松散的约束，这样的效果往往比强硬的把伪测量看做0要好的多。

### 系统的整体结构
整体结构如下图：

<img src="/home/ubuntu/Projects/PaperSummary/AI-IMU/imgs/20190531-131640.png" width=500>

整体而言，系统使用IEKF进行状态的估计，而使用神经网络根据IMU的测量值产生出伪量测及其协方差，整个算法中预测阶段的系统噪声的协方差$Q$是固定的，因为都是高斯白噪声。笔者还是比较看好这样的slam系统的，因为个人认为神经网络不是很适合slam这种几何学的问题，而作为辅助的话还是很看好的。

&nbsp;
---
## 网络的设计
从系统结构图上可以看到，系统使用的是IMU的量测值，其为基于时间的一组序列，作者选择其中N组数据进行卷积，之后产生对应的测量协方差：
$$
\mathbf{N}_{n+1}=\mathrm{CNN}\left(\left\{\omega_{i}^{\mathrm{IMU}}, \mathbf{a}_{i}^{\mathrm{IMU}}\right\}_{i=n-N}^{n}\right)
$$
作者说道设计中的三个动机如下：
- 尽量使用小的参数防止过拟合;
- 希望得到一个可解释的规则，例如转弯的时候，网络的输出要大一些;
- 没有使用RNN是因为RNN的训练更加的艰难;

网络最终的输出为一个2维向量$z_n=[z_n^{lat}, z_n^{up}]^T \in R^2$，随后作者引入一个超参数$\beta$用于控制协方差的扩张上下限，最终的协方差值取做：
$$
\mathbf{N}_{n+1}=\operatorname{diag}\left(\sigma_{\text { lat }}^{2} 10^{\beta \tanh \left(z_{n}^{\text { lat }}\right)}, \sigma_{\text { up }}^{2} 10^{\beta \tanh \left(z_{n}^{\text { up }}\right)}\right)
$$
因此协方差的动态变化范围为$10^{-\beta}～10^{\beta}$。

网络采用1维卷积的方式，共2层，第一层卷积核的大小为5，输出的深度为32，膨胀系数为1，第二层卷积核大小为5，输出深度为32，膨胀系数为3，最后还有一个全连接层将特征输出综合输出为2，取得滑动窗口的大小为N=15，最终一共产生的参数量为6210（这个参数量笔者计算的稍微不太对，按照stride=2来计算的话最终的结果是6304）。

在实现的一些初始参数这里就不过多解释了，需要注意的是，用于不同的机器人的时候，需要对这些参数稍加修改，具体还是参照论文中的部分。

## 网络的训练
网络的训练步骤就是：
- 选取数据；
- 计算滤波器的输出与真实轨迹之间的误差e，之后按照公式更新kalman滤波器的参数，包括协方差矩阵等等，对于量测方程中的量测协方差$R$，一般滤波器中该参数为人为设定，这里应该看做变量，之后根据链式法则将误差e传递回来，送给网络进行权重的调整；
- 更新网络的参数；

具体的一些超参数这里就不赘述了。

&nbsp;
---
## 最终的结果
论文最终对比了IMLS（以激光雷达数据为基础），双目ORB-SLAM2和纯IMU进行积分的方法，结果如下：

<img src="/home/ubuntu/Projects/PaperSummary/AI-IMU/imgs/20190531-150310.png" width=900>

结论还是很强势的，毕竟只是使用IMU的数据进行航位推算，最终能达到比ORB-SLAM2的效果还要还一点也确实很不错了。

除此之外，还记得作者设计网络时候的三个动机吗？作者也把其中第二个动机进行了验证，结果如下图：

<img src="/home/ubuntu/Projects/PaperSummary/AI-IMU/imgs/2019-05-31-15-08-18.png" width=500>

恩～果然在转弯的时候，方差出现了变化。

&nbsp;
---
## 总结
笔者最后把github上的代码下载下来运行了一下，几个测试数据集的效果还是不错的，是一个足以期待的方向。
