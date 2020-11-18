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

结合公式（5）（9）（12）可以写出在 $l$ 时刻的误差状态的传递方程，对应于KF的预测部分；
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
\end{bmatrix} \tag{13}
$$
上式可以写作状态转移方程为：
$$
{}^{I_{l+1}}_{G}\tilde{\mathrm{x}}^{(l)}= \Phi(\mathrm{x}_{I_{l+1}}^{(l)}, \mathrm{x}_{I_l}^{(l)}) {}^{I_{l}}_{G}\tilde{\mathrm{x}}^{(l)}+\mathrm{w}^{(l)} \tag{14}
$$

这里有必要引用原文中的一些介绍：以位移的误差状态量的递推公式为例，相当于在原先误差状态变量的基础上加上速度的变化，之后角速度的变化与一个杆臂$-({}^{I_l}_{G}R)^{T}\left[\hat{\mathrm{y}}^{(l)}\right]$相乘来影响位移的误差状态。所以对于公式（14）表示的状态转移过程而言，整个过程也具有一定的物理意义。

&nbsp;

---

## MSCKF的观测模型

下面的所有的下角标 $l$ 表示id为 $l$ 的相机，不表示时间，这里暂时不涉及时间，可以认为是理想的观测模型。

为了分析能观性，这里还需要一个步骤就是观测模型，以单个观测点$P_{f_j}$为例，其观测模型为：
$$
\begin{aligned}
z_l&=\pi({}^{C_l}\mathrm{p}_{f_j})+n_{l} \\
{}^{C_l}\mathrm{p}_{f_j}&={}^{C}_{I}R {}^{G}_{l}R({}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l})+{}^{C}\mathrm{p}_I 
\end{aligned} \tag{15}
$$

所以观测模型为：
$$
\begin{aligned}
H_{(I_l|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\right]_{\times}({}_{G}^{I_l}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}}\end{bmatrix} \\ 
H_{(f_j|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\end{aligned} \tag{16}
$$
其中：
$$
J_{(f_j|l)}=\frac{1}{Z}\begin{bmatrix}1 & 0 & -\frac{X}{Z} \\ 0 & 1 & -\frac{Y}{Z} \end{bmatrix}
$$
&nbsp;

----

## 能观性分析

能观性的分析主要依赖于能观性矩阵：
$$
\mathcal{O}=\begin{bmatrix} \mathrm{H}_k \\ \mathrm{H}_{k+1}\Phi_{k} \\ \vdots \\ \mathrm{H}_{k+m}\Phi_{k+m-1} \dots\Phi_{k} \end{bmatrix} \tag{17}
$$
&nbsp;

------

## 理想情况下的能观性矩阵

这里先进行理想情况下的能观性矩阵的推导，这个部分的所有变量均使用理想情况下的状态值（公式上来讲就是$\mathbf{X}_{I_l}^{(l-1)}=\mathbf{X}_{I_l}^{(l)}=\dots=\mathbf{X}_{I_l}^{l+m}$），也就是状态变量从预测出来的时候，就是真值了，后面一直都不变了（提前剧透了FEJ  -.-!!!，不过还是稍有不同，后面会解释）。

于是将公式（13）表示的状态转移矩阵与公式（16）表示的观测矩阵带入公式（17）就可以得到能观性矩阵，这里以 $l$ 时刻为例，有：
$$
\mathcal{O}_{l}=\mathrm{H}_{l}\Phi_{l-1}\Phi_{l-2}\dots\Phi_{k} \tag{18}
$$
这里整个推导十分麻烦，但是好在只要计算前两次乘法我们就能找到规律，下面分别对两次乘法进行展开：
$$
\begin{aligned}
\mathrm{H}_{l}\Phi_{l-1}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\left\{\underbrace{\begin{bmatrix} \left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix}\Phi_{l-1}}_{S0} \quad \dots \quad \mathbf{I} \quad \dots \quad \mathbf{0} \right\}
\end{aligned} \tag{19}
$$
可以看到，整个乘积的结果其实更加依赖的是IMU的观测部分与状态转移矩阵的乘积部分，也就是上式中的S0部分，下面单独展开：
$$
\begin{aligned}
S0&=
\begin{bmatrix} 
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{y}}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{s}}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_{l-1}}R)^{T}+({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{y}}_{l-1}\right]_{\times} &
-\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix} \\ 
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_{l-1}}R)^{T}+\left[ {}^{G}{p}_{l}-{}^{G}{p}_{l-1}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \right]_{\times}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1} \\
\end{bmatrix} \\
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2\right]_{\times}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix}
\end{aligned} \tag{20}
$$
其中第三行的展开中用到了反对称矩阵的性质：若矩阵R是特殊正交群SO(3)，则有$R[A]_{\times}R^T=[RA]_{\times}$的变换。

接着继续看公式（18）中与$\Phi_{l-2}$的乘积：
$$
\begin{aligned}
S1&=S0\Phi_{l-2}  \\
&=\begin{bmatrix}
\underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2\right]_{\times}}_{m_{l-1}}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l-1}}_{I_{l-2}}R & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-2}}_{G}R)^{T}\left[{\mathrm{y}}_{l-2}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-2} \\
-({}^{I_{l-2}}_{G}R)^{T}\left[{\mathrm{s}}_{l-2}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
(m_{l-1}+\left[{}^{G}{p}_{I_{l-1}}-{}^{G}{p}_{I_{{l-2}}}-{}^{G}v_{I_{l-2}}\Delta{t}_{l-2}-\frac{1}{2}g \Delta{t}_{l-2}^2\right]_{\times}+
\left[{}^{G}{v}_{l-1}-{}^{G}{v}_{l-2}-g \Delta{t}_{l-2}\right]_{\times}\Delta{t}_{l-1})({}_{G}^{I_{l-2}}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}(\Delta{t}_{l-2}+\Delta{t}_{l-1})
\end{bmatrix}^{T} \\ 
\end{aligned} \tag{21}
$$
把上式中的第一行拿出来分析：
$$
\begin{aligned}
S1_1
&={}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \\
&+{}^{G}{p}_{I_{l-1}}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{I_{l-2}}\Delta{t}_{l-2}-\frac{1}{2}g \Delta{t}_{l-2}^2 \\
&+({}^{G}{v}_{l-1}-{}^{G}{v}_{l-2}-g \Delta{t}_{l-2})\Delta{t}_{l-1} \\
&={}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}^2+2\Delta{t}_{t-1}\Delta{t}_{t-2}+\Delta{t}_{t-2}^2) \\
&={}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{t-2})^2 
\end{aligned}
$$
上式带入公式（21）可以得到：
$$
S1=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{t-2})^2 \right]_{\times}({}_{G}^{I_{l-2}}R)^{T} \\
-\mathbf{I} \\ -\mathbf{I}(\Delta{t}_{l-2}+\Delta{t}_{l-1})
\end{bmatrix}^{T}
$$
后面的情况均类似，所以由归纳法可以得到：
$$
\mathcal{O}_{l}=J_{(f_j|l)}{}^{C}_{I}R{}_{G}^{I_l}R
\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{k}}-{}^{G}v_{k}\Delta{t}-\frac{1}{2}g\Delta{t}^2 \right]_{\times}({}_{G}^{I_{k}}R)^{T}, -\mathbf{I},  -\mathbf{I}\Delta{t}}_{IMU} & \underbrace{\dots , \mathbf{I} , \dots , \mathbf{0}}_{feature} \end{bmatrix} \tag{22}
$$
其中$\Delta{t}=\Delta{t}_{l-1}+\dots+\Delta{t}_{k}$。

 容易看出（不是论文这么说真心凑不出啊），在理想情况下，能观性矩阵的零空间为：
$$
\mathbf{N}=\left[\begin{array}{cc}
\mathbf{0}_{3} & {}^{I_k}_{G}\mathbf{R} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{k}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{k}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_1}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_2}\right]_\times{\mathbf{g}} \\
\vdots & \vdots \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_N}\right]_\times{\mathbf{g}}
\end{array}\right] \tag{23}
$$
读者可以将公式（22）与公式（23）相乘，发现均为0，其中最后的g其实是为了凑两个g的叉乘而存在的。

&nbsp;

### 小结

在求解理想情况下的能观性矩阵时，主要利用了如下几点，这几点也是对于启发FEJ非常重要的几点：

1. 从公式（20）和公式（21）的旋转矩阵相乘可以看到，由于是理想情况，因此旋转矩阵可以直接消去 $l-1$ 时刻的影响：
   $$
   ({}^{I_{l-2}}_{G}R)^{T}=({}^{I_{l-1}}_{G}R)^{T}{}^{I_{l-1}}_{I_{l-2}}R \tag{24}
   $$

2. 由公式（21）及其附属推导可以看到，整个化简的关键在于位移和速度的量不变化；

&nbsp;

----

## 实际情况下的能观性矩阵

下面终于要分析一下实际的能观性矩阵的东西了。

假设程序运行到了$\alpha_{i+1}$时刻，那么对于第 $l$ 个位姿节点（因为MSCKF在维护一个窗口，窗口内的参数都是会更新的）能观性矩阵的组成部分来说：

1. 状态转移矩阵为$\Phi_{l-1}(\mathrm{\hat{x}}_{I_l}^{(l-1)}, \mathrm{\hat{x}}_{I_{l-1}}^{(l-1)})$，$\Phi_{l-2}(\mathrm{\hat{x}}_{I_{l-1}}^{(l-2)}, \mathrm{\hat{x}}_{I_{l-2}}^{(l-2)}$，$\dots$，$\Phi_{k}(\mathrm{\hat{x}}_{k+1}^{(k)}, \mathrm{\hat{x}}_{I_{k}}^{(k)})$，这部分因为是历史递推过来的，所以**不能**受到后来更新的影响；
2. 依旧对于特征点$P_{f_j} $ ，观测矩阵$\mathbf{H}_{l|\alpha_i}=\partial{\mathbf{e}^{(\alpha_i+1)}_{l}}/\partial{\mathrm{\hat{x}}_{l}^{(\alpha_i)}}$，此时公式中使用了 $l$ 节点最新的估计值，这里有读者可能比较疑惑为什么变量使用的是$\alpha_i$时刻的值，**因为在$\alpha_i+1$时刻，在窗口中的变量还没有被最新的观测更新**，所以在线性化的时候，值依旧是上一次被更新的值，也就是$\mathrm{\hat{x}}_{l}^{\alpha_i}$；

针对上面的两个点，这里写出他们的具体形式：

#### 状态转移矩阵

类比公式（13）可得：
$$
\Phi_{l-1}(\mathrm{\hat{x}}_{I_l}^{(l-1)}, \mathrm{\hat{x}}_{I_{l-1}}^{(l-1)})=
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R^{(l-1)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{s}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{25}
$$

#### 观测矩阵

类比公式（16），这里直接写出综合形式：
$$
\mathrm{H}_{l}^{(\alpha_i)}=J_{(f_j|l)} {}^{C}_{I}R {}_{G}^{I_l}R^{(\alpha_{i})}
\left\{\begin{bmatrix} \left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix} \quad \dots \quad \mathbf{I} \quad \dots \quad \mathbf{0} \right\} \tag{26}
$$
&nbsp;

#### 能观矩阵的构建

这里的推导就超级麻烦了，好在我们依旧可以从前两次乘法运算之后看到问题所在，因此类比于公式（19），先展开第一个乘积：
$$
\begin{aligned}
M0^{(\alpha_i)}&=\begin{bmatrix} \left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R^{(l-1)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{s}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T}({}^{I_{l}}_{I_{l-1}}R^{(l-1)})+({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1} 
\end{bmatrix}
\end{aligned} \tag{27}
$$
可以看到，重点是在上式中的第一个元素，将该部分单独展开：
$$
\begin{aligned}
M0^{(\alpha_i)}_{1}&=\left[\underbrace{\left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}}_{part1}\underbrace{({}_{G}^{I_l}R^{(\alpha_i)})^{T}({}^{I_{l}}_{G}R^{(l-1)})}_{D1}+\underbrace{\left[ {}^{G}\hat{p}_{l}^{(l-1)}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \right]_{\times}}_{part2}\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T} \\
&=\left[
part1 \underbrace{\left((\mathbf{I}-\lfloor{}^{(\alpha_i)}\theta_{(l-1)}\rfloor_{\times})({}^{I_{l}}_{G}R^{(l-1)})\right)^{T}({}^{I_{l}}_{G}R^{(l-1)})}_{D1}+part2
\right]
({}^{I_{l-1}}_{G}R^{(l-1)})^{T}  \\
&=\left[
\underbrace{part1+part2}_{D2}+part1\underbrace{\left(({}^{I_{l}}_{G}R^{(l-1)})^{T}\lfloor{}^{(\alpha_i)}\theta_{(l-1)}\rfloor_{\times}({}^{I_{l}}_{G}R^{(l-1)})\right)}_{D3}
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T}  \\
&=\left[
\lfloor
{}^{G}\mathrm{\hat{p}}_{f_j}\underbrace{-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}+{}^{G}\hat{p}_{l}^{(l-1)}}_{D4}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2
\rfloor_{\times} 
+ part1\times D3
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T} \\
&=\left[
\lfloor
{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2
\rfloor_{\times}+\lfloor-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}+{}^{G}\hat{p}_{l}^{(l-1)}\rfloor_{\times}+part1\times D3
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T}
\end{aligned}
$$

其中：

1. 第一行的化简将$\lfloor \hat{\mathrm{y}}^{(l-1)}_{l}\rfloor_{\times}$直接展开，且使用$\lfloor Ra \rfloor_{\times}=R\lfloor a \rfloor_{\times}R^{T}$的性质形成了part2部分；
2. 第三行的化简使用$\lfloor a \rfloor_{\times}^{T}=-\lfloor a \rfloor_{\times}$的性质，把负号消除形成D3部分；
3. 最后一行的化简，因为希望凑出一个类似于公式（20）最后一行的形式，所以这里直接将D4部分移了出来；

至此第一次乘积部分我们就完成了；下面来看第二次乘积：
$$
\begin{aligned}
M1^{(\alpha_i)}&=\begin{bmatrix} M0_{1}^{\alpha_i}({}^{I_{l-1}}_{G}R^{(l-1)})^{T} & -\mathbf{I} & \mathbf{I}\Delta{t}_{l-1}\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l-1}}_{G}R^{(l-2)}({}^{I_{l-2}}_{G}R^{(l-2)})^{T} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{y}}^{(l-2)}_{l-2}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-2} \\
-({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{s}}^{(l-2)}_{l-2}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\end{aligned}
$$
