# 一文总结4种SLAM中零空间的维护方法

----

[toc]

&nbsp;

----

## 写在前面

笔者最近在总结整个MSCKF的相关知识的时候，算是比较仔细的推导了与零空间相关的部分，放在之前笔者会认为这部分知识确实是比较难理解的部分，但是花了很多时间总结发现其实都还是有很强的相关性的，所以这里笔者算是总结一下这部分的内容，算是对近来知识的一个总结。

本文主要会涉及4种零空间的维护方法，分别是：

1. First-Estimate-Jacobian，该方法也是用途最广的维护能观性的方法，可以适用于EKF-base和Graph-base的SLAM系统中；
2. Observability-Constrainted 方法，该方法用途可能不多，但其有自身的优势在，且是开源的S-MSCKF中使用的方法；
3. DSO中关于零空间维护的方法，其中不仅使用了FEJ技术，同时对增量方程和求解的增量都做了去除零空间变量的影响；
4. VINS中关于零空间维护的方法，该方法主要是数值上的维护；

&nbsp;

------

## Reference

1. https://zhuanlan.zhihu.com/p/304889273 FEJ 的相关总结；
2. https://zhuanlan.zhihu.com/p/328940891 OC-KF的相关总结；
3. https://blog.csdn.net/wubaobao1993/article/details/105106301 DSO对于零空间维护的相关总结；
4. https://blog.csdn.net/wubaobao1993/article/details/108354488 VINS对于零空间维护的相关总结；

&nbsp;

----

## EKF-Base方法对零空间的维护

从前有两位年轻人都致力于EKF-Base（本文更多的是针对MSCKF方法）方法中能观性的分析，但是都遇到了很大的问题。于是两人约定一起到山上去请教一位老师傅。

老师傅：你二人既然都是研究能观性的分析，那么一定知道能观性分析的评价标准是能观性矩阵吧？
$$
\mathcal{O}=
\begin{bmatrix} \mathrm{H}_0 \\ \mathrm{H}_{1}\Phi_{0} \\ \vdots \\ \mathrm{H}_{t}\Phi_{t-1} \dots\Phi_{0} \end{bmatrix} \tag{1}
$$
两位年轻人：自然是知道的，对于线性系统而言，系统的能观性主要由能观性矩阵反应。

老师傅：嗯，很好，既然如此，我就给二位一人三个锦囊，希望你二人能在各自的研究方向上都能悟道。

两位年轻人接过锦囊，开心的下山了。

&nbsp;

### First-Estimate-Jacobian方法

第一位年轻人下山之后便回到了家，继续开始对于能观性的分析，他迫不及待的打开第一个锦囊，上面写着一行字和如下的公式：

#### 不依赖IMU的运动方程，从头推导理想情况下的状态传递方程

$$
\begin{aligned}
&\begin{cases}
{}^{I_{t+1}}_{G}R&={}^{I_{t+1}}_{I_{t}}R \quad{}^{I_{t}}_{G}R \\
{}^{I_{t}}_{G}R&=(\mathbf{I}-\left[ {}^{I_{t}}\tilde{\theta} \right]_{\times}){}^{I_{t}}_{G}\hat{R}
\end{cases}\\
&\begin{cases}
{}^{G}v_{I_{t+1}} &= {}^{G}v_{I_t} + ({}^{I_{t}}_{G}R)^{T} \int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
{}^{G}v_{I_{t}}&={}^{G}\hat{v}_{I_{t}}+{}^{G}\tilde{v}_{I_{t}} \\
{}^{I_{t}}_{G}R&=(\mathbf{I}-\left[ {}^{I_{t}}\tilde{\theta} \right]_{\times}){}^{I_{t}}_{G}\hat{R}
\end{cases} \\
&\begin{cases}
{}^{G}p_{I_{t+1}} &= {}^{G}p_{I_t} + {}^{G}v_{I_t}\Delta{t}+ ({}^{I_{t}}_{G}R)^{T} \int_{t_{l}}^{t_{l+1}} \int_{t_{l}}^{s} ({}^{I_{\tau}}_{I_{t}}R)^{T} \mathbf{a_m} d \tau d s + \frac{1}{2} g \Delta{t}^2 \\
{}^{G}p_{I_{t}}&={}^{G}\hat{p}_{I_{t}}+{}^{G}\tilde{p}_{I_{t}} \\
{}^{G}v_{I_{t}}&={}^{G}\hat{v}_{I_{t}}+{}^{G}\tilde{v}_{I_{t}} \\
{}^{I_{t}}_{G}R&=(\mathbf{I}-\left[ {}^{I_{t}}\tilde{\theta} \right]_{\times}){}^{I_{t}}_{G}\hat{R}
\end{cases}
\end{aligned}  \tag{2}
$$

年轻人反复看了多遍老师的建议，于是经过一段时日，年轻人终于按照提示推出了新的状态传递方程：
$$
\begin{bmatrix}
{}^{I_{t+1}}_{G}\tilde{\theta} \\
{}^{G}\tilde{p}_{I_{t+1}} \\
{}^{G}\tilde{v}_{I_{t+1}}
\end{bmatrix} = 
\begin{bmatrix} 
{}^{I_{t+1}}_{I_{t}}R & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t}\rfloor_{\times}({}^{I_t}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor{\mathrm{s}}_{t}\rfloor_{\times}({}^{I_t}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}^{I_{t}}_{G}\tilde{\theta} \\
{}^{G}\tilde{p}_{I_{t}} \\
{}^{G}\tilde{v}_{I_{t}}
\end{bmatrix}
+
\begin{bmatrix}
{}^{I_{t+1}}_{I_{t}}\tilde{\theta} \\
({}^{I_t}_{G}R)^{T}\left[\tilde{\mathrm{y}}_{l}\right] \\
({}^{I_t}_{G}R)^{T}\left[\tilde{\mathrm{s}}_{l}\right]
\end{bmatrix} \tag{3}
$$
其中

1. 所有的变量都是理想情况下的值，也就是并不涉及到时刻的概念（公式中的 t 和 t+1 更多的表示的递进关系，可以理解为当前的位姿经过IMU激励得到下一个位姿），更进一步的讲，某个变量在预测阶段被预测出来，那么由于预测阶段都是理想的，所以该预测值也是相当理想的，即真值；
2. $\mathrm{y}_{t}={}^{G}{p}_{t+1}-{}^{G}{p}_{t}-{}^{G}v_{t}\Delta{t}-\frac{1}{2}g \Delta{t}^2$；
3. $\mathrm{s}_{t}={}^{G}{v}_{t+1}-{}^{G}{v}_{t}-g\Delta{t}$；

年轻人觉得很开心，觉得新的状态传递方程一定可以帮助他解开心中的迷惑。但是研究良久，年轻人还是一无所获，于是他又拿出第二个锦囊，想看看老师又给了他怎样的启示，只见上面写道：

&nbsp;

#### 使用理想的观测矩阵，从前到后推导能观性矩阵中的某一行，总结出零空间应该是怎么样的

年轻人很聪明，他马上把注意力集中在理想情况下的过程，首先他写出了理想情况下的观测矩阵：
$$
\begin{aligned}
H_{(I_l|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\right]_{\times}({}_{G}^{I_l}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}}\end{bmatrix} \\ 
H_{(f_j|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\end{aligned} \tag{4}
$$
将公式（4）中的两部分和在一起可得：
$$
\mathbf{H}_{f_j|t}=J_{(f_j|t)} \quad {}^{C}_{I}R \quad {}_{G}^{I_t}R
\begin{bmatrix}\begin{array}{ccc|c}
\lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\rfloor_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3}
\end{array}\end{bmatrix} \tag{5}
$$

于是年轻人按照老师的建议，取出其中一行进行推导，如下：
$$
\mathcal{O}_{t}=\mathbf{H}_t\Phi(t,t-1)\dots\Phi(t_1,t_0) \tag{6}
$$
将公式（3）和公式（5）带入到公式（6）中可得：
$$
\mathcal{O}_{t|3\times3}=
\begin{bmatrix}
\lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_t}\rfloor_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{t}}_{G}R({}^{I_{t-1}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-1}\rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor{\mathrm{s}}_{t-1}\rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
\begin{bmatrix} 
{}^{I_{t-1}}_{G}R({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor{\mathrm{s}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\dots
\begin{bmatrix} 
{}^{I_{t_1}}_{G}R({}^{I_{t_0}}_{I_{G}}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t_0}\rfloor_{\times}({}^{I_{t_0}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor{\mathrm{s}}_{t_0}\rfloor_{\times}({}^{I_{t_0}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{7}
$$

公式（7）中因为该部分仅涉及到IMU状态部分的分析，因此省去了观测的部分，同时省去了观测矩阵前面的公共部分。

在推导过程中，年轻人发现按照这样乘法下去，在每一次乘法之后都能得到一个很相似的形式，以前面两次乘积为例可以得到：
$$
\begin{aligned}
&\begin{bmatrix}
\lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_t}\rfloor_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{t}}_{G}R({}^{I_{t-1}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-1}\rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_{t-1} \\
-\lfloor{\mathrm{s}}_{t-1}\rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{t-1}}_{G}R({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_{t-1} \\
-\lfloor{\mathrm{s}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&= 
\begin{bmatrix}
\lfloor {}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_t}-\mathrm{y}_{t-1}-\mathrm{s}_{t-1}\Delta{t} \rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}\Delta{t}_{t-1}
\end{bmatrix}^{T}\begin{bmatrix} 
{}^{I_{t-1}}_{G}R({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_{t-2} \\
-\lfloor{\mathrm{s}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
\lfloor {}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{t-1}}-{}^{G}\mathrm{v}_{t-1}\Delta{t}_{t-1}-\frac{1}{2}g\Delta{t}_{t-1}^{2} \rfloor_{\times}({}^{I_{t-1}}_{G}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}\Delta{t}_{t-1}
\end{bmatrix}^{T}
\begin{bmatrix} 
{}^{I_{t-1}}_{G}R({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{0} \\
-\lfloor{\mathrm{y}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_{t-2} \\
-\lfloor{\mathrm{s}}_{t-2}\rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
\lfloor {}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{t-2}}-{}^{G}\mathrm{v}_{t-2}\Delta{t}^{t-1}_{t-2}-\frac{1}{2}g(\Delta{t}^{t-1}_{t-2})^{2} \rfloor_{\times}({}^{I_{t-2}}_{G}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}\Delta{t}^{t-1}_{t-2}
\end{bmatrix}^{T}
\end{aligned}  \tag{8}
$$
年轻人很快便总结出了规律，于是在 t0 时刻，整个乘积变作（这里把特征部分的元素给添加回来了）：
$$
\mathcal{O}_{t|3\times3}=\begin{bmatrix}
\lfloor {}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{t_0}}-{}^{G}\mathrm{v}_{t_0}\Delta{t}^{t-1}_{t_0}-\frac{1}{2}g(\Delta{t}^{t-1}_{t_0})^{2} \rfloor_{\times}({}^{I_{t_0}}_{G}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}\Delta{t}^{t-1}_{t_0} \\ \hline
\mathbf{I}
\end{bmatrix}^{T} \tag{9}
$$
所以对应的零空间其实就是：
$$
\mathbf{N}_{t|3\times3}=
\begin{bmatrix}
\mathbf{0}_{3} & {}^{I_{t_0}}_{G}\mathbf{R} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t_0}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t_0}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{10}
$$
除此之外，年轻人也在推导过程中总结出了一个很重要的规律：**之所以乘积最后能满足公式（10）所示的零空间，其比较重要的原因在于在乘积过程中，所有的位姿和速度都用的是一个值（在理想情况下，都使用的是理想值），这保证了相同项之间可以互相抵消。**

于是年轻人回想自己的推导过程，因为在自己的系统中，整个系统维护了一个滑动窗口，该窗口内记录了一段时间内的位姿数据，该数据会因为新观测的到来而被更新，导致后续在构建观测矩阵时一直使用的是最新的值，**也就是同样的位姿和速度，使用了两个时刻的值**，这样的方法使得推导过程中增加好很多因为使用不同时刻值的扰动项！形式如下：

