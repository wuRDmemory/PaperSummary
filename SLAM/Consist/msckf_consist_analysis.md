# The consistency of Visual-Inertial Odometry

---
[toc]
&nbsp;

---
## 写在前面
讲实话，笔者之前就听说过很多关于SLAM系统的能观性、或者说一致性的分析，最开始的切入点当然是大名鼎鼎的First-Estimate-Jacobian（FEJ）技术，且当时更多接触的是Graph-Base的方法，当时就直观感受而言，仅仅觉得FEJ固定了求解的方向，导致整个优化问题的信息矩阵在求解的时候是固定的，由此来解决由于信息矩阵的变化导致过度估计的问题。

但是上述仅仅是直观的一些想法，并没有理论做依据，之前也零星的看过黄老师关于FEJ的文章，但是公式太多，个人水平也很有限，所以不能很好的理解其中的精髓。

这次借着阅读整理MSCKF的机会，更深一步的理解一下如何分析系统的能观性以及FEJ到底在解决什么，参考更多的其实是李明扬大佬的一些文章和MSCKF2.0的工作。

&nbsp;

---

## Reference

1. Consistency of EKF-Based Visual-Inertial Odometry. 关于MSCKF一致性的分析，也是本文主要参考的论文；
2. Analysis and Improvement of the Consistency of Extended Kalman Filter based SLAM. 黄老师关于EKF一致性的分析；
3. Generalized Analysis and Improvement of the Consistency of EKF-based SLAM. 黄老师同年发表的一篇更长的关于一致性的文章，可以认为是参考2的扩充版；

&nbsp;

----

## IMU误差状态传播公式的回顾

简单的回顾一下MSCKF对于IMU误差状态（error-state）的传播过程：

1. 通过IMU运动方程得到误差状态的微分方程：
   $$
   \dot{\tilde{{X}}}=\mathbf{F}(\hat{{X}})\tilde{{X}}+\mathbf{G}n \tag{1}
   $$

2. 通过线性系统的离散化得到IMU误差状态递方程的闭式解：
   $$
   \boldsymbol{\tilde{X}}\left(t_{k+1}\right)=\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right) \boldsymbol{\tilde{X}}\left(t_{k}\right)+\int_{t_{k}}^{t_{k+1}} \boldsymbol{\Phi}\left(t_{k+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{2}
   $$
   其中$\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right)=\exp \int_{t_{k}}^{t_{k+1}} \boldsymbol{F}(t) \mathrm{d} t$
   

对于上述的推导过程，我们容易看到，整个推导过程建立在数值分析的层次，也就是闭式解本身是一个数值解，当然有的小伙伴会认为如果假设一段时间内微分量是不变的，直接乘以时间是不是就有理论意义？诚然，笔者认为这样的方法可以，但是不精确，不精确的解去分析整个系统的特性带来的结果必然也是不精确的；所以在最开始，我们需要从理论的角度来重新推导整个IMU误差状态的传导过程。

&nbsp;
---

## IMU误差状态传播公式的再推导

如上所述，本节主要是对误差状态传播公式的理论推导，这里主要分析三个参数的传播公式：
1. 旋转部分；
2. 速度部分；
3. 位移部分；

bias部分因为不涉及到能观性的问题，所以这里暂不引入，实际上，引入之后得到的结论也是一样的，后续会稍微提到一下。

>注：以下公式均在时间为${l}$的时刻。

&nbsp;

### 旋转部分的推导

主要依赖两个基本的公式：
$$
\begin{cases}
\begin{aligned}
{}^{I_{l+1}}_{G}R^{(l)}&={}^{I_{l+1}}_{I_{l}}R^{(l)} \quad{}^{I_{l}}_{G}R^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{3}
$$
将第二行的公式带入到第一行中可以得到：
$$
\begin{aligned}
{}^{I_{l+1}}_{G}R^{(l)}&={}^{I_{l+1}}_{I_{l}}R^{(l)} \quad{}^{I_{l}}_{G}R^{(l)} \\
(\mathbf{I}-\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{G}\hat{R}^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}\quad (\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)} \\
{}^{I_{l+1}}_{G}\hat{R}^{(l)}-(\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{G}\hat{R}^{(l)}&={}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}_{G}\hat{R}^{(l)} -(\left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}_{G}\hat{R}^{(l)} - {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}(\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}+\mathbf{O(2)} \\
\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times} &\approx \left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}+\underbrace{{}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}(\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{I_{l+1}}\hat{R}^{(l)}}_{\left[ {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}\tilde{\theta} \right]_{\times}} \\
\end{aligned} \tag{4}
$$
对公式（4）的最后一行使用vee操作可得：
$$
{}^{I_{l+1}}\tilde{\theta}={}^{I_{l+1}}_{I_{l}}\tilde{\theta} + {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}\tilde{\theta} \tag{5}
$$
&nbsp;

### 速度部分的推导

依赖三个基本公式：
$$
\begin{cases}
\begin{aligned}
{}^{G}v_{I_{l+1}}^{(l)} &= {}^{G}v^{(l)} + ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
{}^{G}v_{I_{l}}^{(l)}&={}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{6}
$$
把公式（6）中的第二行和第三行带入第一行之后得到：
$$
\begin{aligned}
{}^{G}v_{I_{l+1}}^{(l)} &= {}^{G}v^{(l)} + ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
{}^{G}\hat{v}_{I_{l+1}}^{(l)}+{}^{G}\tilde{v}_{I_{l+1}}^{(l)} &= {}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} + \{(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}\}^{T} \int_{t_{l}}^{t_{l+1}} \{(\mathbf{I}-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
&={}^{G}\hat{v}_{I_{l}}^{(l)} + ({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau + g \Delta{t}\\ 
&+{}^{G}\tilde{v}_{I_{l}}^{(l)} +\{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T}\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\int_{t_{l}}^{t_{l+1}} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau + \mathbf{O(2)} 
\end{aligned} \tag{7}
$$
此时我们将积分值进行替换，规则如下：
$$
\begin{aligned}
\mathrm{\hat{s}}^{(l)}&=\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau = {}^{I_{l}}_{G}R^{(l)}({}^{G}\hat{v}_{l+1}^{(l)}-{}^{G}\hat{v}_{l}^{(l)}-g \Delta{t}) \\
\mathrm{\tilde{s}}^{(l)} &= \int_{t_{l}}^{t_{l+1}} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau 
\end{aligned}  \tag{8}
$$
公式（8）中的等价关系由公式（6）的第一行推导出，将公式（8）带入到公式（7）中可以得到：
$$
{}^{G}\tilde{v}_{I_{l+1}}^{(l)}={}^{G}\tilde{v}_{I_{l}}^{(l)}-({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}\right]_{\times} {}^{I_{l}}\tilde{\theta}+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T} \tilde{\mathrm{s}}^{(l)} \tag{9}
$$
&nbsp;

### 位移部分的推导

与速度相同，这部分推导依赖四个基础公式：
$$
\begin{cases}
\begin{aligned}
{}^{G}p_{I_{l+1}}^{(l)} &= {}^{G}p_{I_l}^{(l)} + {}^{G}v_{I_l}^{(l)}\Delta{t}+ ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} \int_{t_{l}}^{s} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau d s + \frac{1}{2} g \Delta{t}^2 \\
{}^{G}p_{I_{l}}^{(l)}&={}^{G}\hat{p}_{I_{l}}^{(l)}+{}^{G}\tilde{p}_{I_{l}}^{(l)} \\
{}^{G}v_{I_{l}}^{(l)}&={}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{10}
$$
同样按照速度的推理，可以得到如下结论：
$$
{}^{G}\tilde{p}_{I_{l+1}}^{(l)}={}^{G}\tilde{p}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_l}^{(l)}\Delta{t}-({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}\right]_{\times} {}^{I_{l}}\tilde{\theta}+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T} \tilde{\mathrm{y}}^{(l)} \tag{11}
$$
其中的变量$\mathrm{y}$和速度中的一样，也是一个替换变量：
$$
\begin{aligned}
\mathrm{\hat{y}}^{(l)}&=\int_{t_{l}}^{t_{l+1}} \int_{t_l}^{s} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau d s = {}^{I_{l}}_{G}R^{(l)}({}^{G}\hat{p}_{l+1}^{(l)}-{}^{G}\hat{p}_{l}^{(l)}-{}^{G}v_{I_l}^{(l)}\Delta{t}-\frac{1}{2}g \Delta{t}^2) \\
\mathrm{\tilde{y}}^{(l)} &= \int_{t_{l}}^{t_{l+1}} \int_{t_l}^{s} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau d s 
\end{aligned}  \tag{12}
$$
&nbsp;

### 状态传递的传递方程

结合公式（5）（9）（12）可以写出在$l$时刻的误差状态的传递方程，对应于KF的预测部分；
$$
\begin{bmatrix}
{}^{I_{l+1}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l+1}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l+1}}^{(l)}
\end{bmatrix} = 
\begin{bmatrix} 
{}^{I_{l+1}}_{I_{l}}R^{(l)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_l}_{G}R)^{T}\left[\hat{\mathrm{y}}^{(l)}\right] & \mathbf{I} & \mathbf{I}\Delta{t} \\
-({}^{I_l}_{G}R)^{T}\left[\hat{\mathrm{s}}^{(l)}\right] & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}^{I_{l}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l}}^{(l)}
\end{bmatrix}
+
\begin{bmatrix}
{}^{I_{l+1}}_{I_{l}}\tilde{\theta}^{(l)} \\
({}^{I_l}_{G}R)^{T}\left[\tilde{\mathrm{y}}^{(l)}\right] \\
({}^{I_l}_{G}R)^{T}\left[\tilde{\mathrm{s}}^{(l)}\right]
\end{bmatrix} \tag{13}
$$
上式可以写作状态转移方程为：
$$
{}^{I_{l+1}}_{G}\tilde{\mathrm{x}}^{(l)}= \Phi(\mathrm{x}_{I_{l+1}}^{(l)}, \mathrm{x}_{I_l}^{(l)})^{(l)} {}^{I_{l}}_{G}\tilde{\mathrm{x}}^{(l)}+\mathrm{w}^{(l)} \tag{14}
$$

&nbsp;

---

