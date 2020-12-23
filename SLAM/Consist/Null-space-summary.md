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
5. Consistency of EKF-Based Visual-Inertial Odometry. MSCKF能观性的分析论文；

&nbsp;

----

## EKF-Base方法对零空间的维护

> Notation:
>
> 1. 在以下推导中，所有的旋转均使用JPL表示法；
> 2. 在以下推导中，变量均选择为旋转，位置和速度，即$\mathrm{x}=\begin{bmatrix}{}^{\ell}_{G}q & {}^{G}p_{\ell} & {}^{G}v_{\ell}\end{bmatrix}^{T}$；

从前有两位年轻人都致力于EKF-Base（本文更多的是针对MSCKF方法）方法中能观性的分析，但是都遇到了很大的问题。于是两人约定一起到山上去请教一位德高望重的老师。

老师：你二人既然都是研究能观性的分析，那么一定知道能观性分析的评价标准是能观性矩阵吧？
$$
\mathcal{O}=
\begin{bmatrix} \mathrm{H}_0 \\ \mathrm{H}_{1}\Phi_{0} \\ \vdots \\ \mathrm{H}_{t}\Phi_{t-1} \dots\Phi_{0} \end{bmatrix} \tag{1}
$$
两位年轻人：自然是知道的，对于线性系统而言，系统的能观性主要由能观性矩阵反应。

老师：嗯，很好，既然如此，我就给二位一人三个锦囊，希望你二人能在各自的研究方向上都能悟道。

两位年轻人接过锦囊，开心的下山了。

&nbsp;

### First-Estimate-Jacobian方法

第一位年轻人下山之后便回到了家，继续开始对于能观性的分析，他迫不及待的打开第一个锦囊，上面写着一行字和如下的公式：

#### 锦囊1. 不依赖IMU的运动方程，从头推导理想情况下的状态传递方程

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

#### 锦囊2. 使用理想的观测矩阵，从前到后推导能观性矩阵中的某一行，总结出零空间应该是怎么样的

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
\mathcal{O}_{t}=\begin{bmatrix}
\lfloor {}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{t_0}}-{}^{G}\mathrm{v}_{t_0}\Delta{t}^{t-1}_{t_0}-\frac{1}{2}g(\Delta{t}^{t-1}_{t_0})^{2} \rfloor_{\times}({}^{I_{t_0}}_{G}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}\Delta{t}^{t-1}_{t_0} \\ \hline
\mathbf{I}
\end{bmatrix}^{T} \tag{9}
$$
所以对应的零空间其实就是：
$$
\mathbf{N}_{t}=
\begin{bmatrix}
\mathbf{0}_{3} & {}^{I_{t_0}}_{G}\mathbf{R} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t_0}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t_0}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{10}
$$
除此之外，年轻人也在推导过程中总结出了一个很重要的规律：**之所以乘积最后能满足公式（10）所示的零空间，其比较重要的原因在于在乘积过程中，所有的位姿和速度都用的是一个值（在理想情况下，都使用的是理想值），这保证了相同项之间可以互相抵消。**

> 例如在$\Phi(t,t-1)_{11}={}^{I_{t}}_{G}R({}^{I_{t-1}}_{G}R)^{T}$和$\Phi(t-1,t-2)_{11}={}^{I_{t-1}}_{G}R({}^{I_{t-2}}_{G}R)^{T}$，其中当${}^{I_{t-1}}_{G}R$使用同样的值时才可以抵消，不至于引入其他项。

于是年轻人回想自己的推导过程，因为在自己的系统中，**整个系统维护了一个滑动窗口，该窗口内记录了一段时间内的位姿数据，该数据会因为新观测而被更新，导致后续在构建观测矩阵时一直使用的是最新的值，也就是同样的位姿和速度，使用了两个时刻的值**，这样的方法使得推导过程中增加好很多因为使用不同时刻值的扰动项！例如在时刻$\alpha_{i+1}$ ，则对于第 $\ell$ 个节点（假设该节点的预测时刻也为 $\ell$ 时刻）来说形式如下：

$$
\mathcal{O}_{\ell}^{(\alpha_{i+1})}=\mathbf{M}_{\ell}^{(t)}
\begin{bmatrix}\begin{array}{ccc|c}
\mathbf{\Gamma}_{\ell}^{(\alpha_{i+1})}+\Delta \mathbf{\Gamma}_{\ell}^{(\alpha_{i+1})} &-\mathbf{I}_{3} & -\Delta t_{\ell} \mathbf{I}_{3} & \mathbf{I}_{3}
\end{array}\end{bmatrix} \tag{11}
$$

其中：

1. $\Gamma$ 的形式应与公式（9）中的理想情况相似：
   $$
   \boldsymbol{\Gamma}_{\ell}^{(\alpha_{i+1})}=\left\lfloor^{G} \hat{\mathbf{p}}_{f_{i}}-{ }^{G} \hat{\mathbf{p}}_{t_0}^{(t_0)}-{ }^{G} \hat{\mathbf{v}}_{t_0}^{(t_0)} \Delta t_{\ell}-\frac{1}{2}{ }^{G} \mathbf{g} \Delta t_{\ell}^{2} \times\right\rfloor ({}^{t_0}_{G}\hat{\mathbf{R}}^{(t_0)})^{T}
   $$

2. $\Delta{\Gamma}$ 是由于不同时刻线性化造成的扰动项，形式如下：
   $$
   \begin{aligned}
   \Delta \boldsymbol{\Gamma}_{\ell}^{(\alpha_{i+1})}=&\left(\left\lfloor^{G} \hat{\mathbf{p}}_{f_{j}}-{ }^{G} \hat{\mathbf{p}}_{\ell}^{(\alpha_{i})} \right\rfloor_{\times} \overline{\mathbf{E}}_{\mathbf{q}}+\overline{\mathbf{E}}_{\mathbf{p}}+\sum_{j=t_1}^{\ell-1}\left(\sum_{s=t_1}^{j} \mathbf{E}_{\mathbf{v}}^{s} \Delta t+\mathbf{E}_{\mathbf{p}}^{j}+\right.\right.\\
   &\left.\left.\sum_{s=t_1} \boldsymbol{\Phi}_{\mathbf{v q}}\left(\hat{\mathbf{x}}_{I_{s+1}}^{(s)}, \hat{\mathbf{x}}_{I_{s}}^{(s)}\right) {}_{G}^{s}\hat{\mathbf{R}}^{(s)} \mathbf{E}_{\mathbf{q}}^{s} \Delta t+\Phi_{\mathbf{p q}}\left(\hat{\mathbf{x}}_{I_{j+1}}^{(j)}, \hat{\mathbf{x}}_{I_{j}}^{(j)}\right) {}^{j}_{G}\hat{\mathbf{R}}^{(j)} \mathbf{E}_{\mathbf{q}}^{j}\right)\right) ({}^{k}_{G}\hat{\mathbf{R}}^{(k)})^{T}
   \end{aligned}
   $$
   
   其中形如$\mathrm{\hat{X}}_{s}^{(s)}$就是在 s 时刻第 s 个节点的值，可以看到其中有较多的同一节点在不同时刻的值，这些值导致了扰动项的产生，具体形式可以参考参考【5】中的公式。
   
3. $\mathbf{M}$矩阵是一些公共项，这里不多做分析；

&nbsp;

#### 锦囊3. 对同一变量使用第一次预测出的值作为观测方程的线性化点——FEJ

年轻人从推导理想情况的步骤中得到启发，于是他尝试将同一变量使用同一时刻的值进行实际情况的推导，选择怎样的时刻呢？年轻人有三个选择，以 t 时刻产生的节点为例：

- 该变量被 t-1 时刻的更新值预测出来的值——$\hat{\mathrm{x}}_{t}^{(t-1)}$；
- 该变量在 t 时刻更新之后的值——$\hat{\mathrm{x}}_{t}^{(t)}$；
- 该变量在之后时刻被更新之后的值——$\hat{\mathrm{x}}_{t}^{(\alpha_i)}$；

最后年轻人选择了第一个，因为在能观性矩阵中涉及到状态转移矩阵$\Phi(\hat{\mathrm{x}}_{t},\hat{\mathrm{x}}_{t-1})$，此时对于 $t$ 节点，仅有刚刚预测出来的值。

于是年轻人对公式做了如下的改动：

1. 将所有的状态转移矩阵做如下变换：
   $$
   \begin{aligned}
   \hat{\Phi}(t, t-1) &= \begin{bmatrix} 
   {}^{I_{t}}_{G}\hat{R}^{(t-1)}({}^{I_{t-1}}_{G}\hat{R}^{(t-1)})^{T} & \mathbf{0} & \mathbf{0} \\
   -\lfloor{\mathrm{\hat{y}}}_{t}^{(t-1)}\rfloor_{\times}({}^{I_{t-1}}_{G}\hat{R}^{(t-1)})^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
   -\lfloor{\mathrm{\hat{s}}}_{t}^{(t-1)}\rfloor_{\times}({}^{I_{t-1}}_{G}\hat{R}^{(t-1)})^{T} & \mathbf{0} & \mathbf{I}
   \end{bmatrix}  \\
   \Rightarrow
   \hat{\Phi}(t, t-1) &= \begin{bmatrix} 
   {}^{I_{t}}_{G}\hat{R}^{(t-1)}({}^{I_{t-1}}_{G}\hat{R}^{(t-2)})^{T} & \mathbf{0} & \mathbf{0} \\
   -\lfloor{\mathrm{\hat{y}}}_{t}^{(t-1)}\rfloor_{\times}({}^{I_{t-1}}_{G}\hat{R}^{(t-2)})^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
   -\lfloor{\mathrm{\hat{s}}}_{t}^{(t-1)}\rfloor_{\times}({}^{I_{t-1}}_{G}\hat{R}^{(t-2)})^{T} & \mathbf{0} & \mathbf{I}
   \end{bmatrix}
   \end{aligned} \tag{12}
   $$
   里面的 $\hat{\mathrm{y}}$ 和 $\hat{\mathrm{s}}$ 中的相应项也要变为FEJ的形式；

2. 将所有的观测矩阵做如下变换：
   $$
   \begin{aligned}
   \mathbf{H}_{f_j|t}&=J_{(f_j|t)} \quad {}^{C}_{I}R \quad {}_{G}^{I_t}\hat{R}^{(\alpha_i)}
   \begin{bmatrix}\begin{array}{ccc|c}
   \lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_t}^{(\alpha_i)}\rfloor_{\times}({}_{G}^{I_l}\hat{R}^{(\alpha_i)})^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3}
   \end{array}\end{bmatrix} \\
   \Rightarrow
   \mathbf{H}_{f_j|t}&=J_{(f_j|t)} \quad {}^{C}_{I}R \quad {}_{G}^{I_t}\hat{R}^{(t-1)}
   \begin{bmatrix}\begin{array}{ccc|c}
   \lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_t}^{(t-1)}\rfloor_{\times}({}_{G}^{I_l}\hat{R}^{(t-1)})^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3}
   \end{array}\end{bmatrix}
   \end{aligned} \tag{13}
   $$

将上述的改动带入到能观性矩阵中之后，果然消掉了所有的扰动项，得到了形如公式（10）的零空间：
$$
\mathbf{N}_{t}^{(\alpha_i)}=
\begin{bmatrix}
\mathbf{0}_{3} & {}^{I_{t_0}}_{G}\mathbf{\hat{R}}^{(t_0)} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t_0}^{(t_0)}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t_0}^{(t_0)}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{14}
$$
年轻人很开心，他连忙打开老师给予的第三个锦囊，果然老师的提示与自己的想法是一致的。

> 以上的详细推导可以看参考【1】

&nbsp;

---

### Observability-Constraint 方法

时间回到第二个年轻人那里，他下了山只有，并没有直接回家，而是去参加了一个朋友主办的研讨会，会议上他了解到如何求解带约束的最优化问题，这个研讨会在后面帮助了他很多。第二个年轻人也从老师的建议中获取了很多；

#### 锦囊1. 根据IMU运动方程，推导误差状态转移方程的闭式解，并用该理想情况下的转移矩阵传递初始的零空间

根据年轻人之前的研究，根据IMU的运动方程推导得到的误差状态微分方程为：
$$
\dot{\tilde{\mathbf{X}}}_{\mathrm{IMU}}=\mathbf{F} \tilde{\mathbf{X}}_{\mathrm{IMU}}+\mathbf{G} \mathbf{n}_{\mathrm{IMU}}  \tag{15}
$$
其中：
$$
\mathbf{F}=\left[\begin{array}{ccc}
-\lfloor\hat{\boldsymbol{\omega}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}_{G}^{I_l}R)^{T}\lfloor\hat{\mathbf{a_m}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
$$
这里误差驱动矩阵$\mathbf{G}$不对零空间的分析有贡献，所以暂时就不分析了；

那么根据微分方程，可以得到状态转移方程为：
$$
\boldsymbol{\tilde{\mathrm{X}}}\left(t_{l+1}\right)=\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \boldsymbol{\tilde{\mathrm{X}}}\left(t_{l}\right)+\int_{t_{l}}^{t_{l+1}} \boldsymbol{\Phi}\left(t_{l+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{16}
$$
其中：
$$
\begin{cases}
\dot{\boldsymbol{\Phi}}\left(t_{l+1}, t_{l}\right) = \boldsymbol{F}(t)\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \\
\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right)=\exp(\int_{t_{l}}^{t_{l+1}} \boldsymbol{F}(t) \mathrm{d} t)  
\end{cases} \tag{17}
$$
根据公式（17）可以列出如下公式：
$$
\begin{aligned}
\begin{bmatrix}
\dot{\Phi}_{11}(t) & \dot{\Phi}_{12}(t) & \dot{\Phi}_{13}(t) \\
\dot{\Phi}_{21}(t) & \dot{\Phi}_{22}(t) & \dot{\Phi}_{23}(t) \\
\dot{\Phi}_{31}(t) & \dot{\Phi}_{32}(t) & \dot{\Phi}_{33}(t) \\
\end{bmatrix}
&=\left[\begin{array}{ccc}
-\lfloor{\boldsymbol{\omega}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}^{G}_{t}R)^{T}\lfloor{\mathbf{a_m}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
\begin{bmatrix}
{\Phi}_{11}(t) & {\Phi}_{12}(t) & {\Phi}_{13}(t) \\
{\Phi}_{21}(t) & {\Phi}_{22}(t) & {\Phi}_{23}(t) \\
{\Phi}_{31}(t) & {\Phi}_{32}(t) & {\Phi}_{33}(t) \\
\end{bmatrix} \\
\begin{bmatrix}
{\Phi}_{11}(t_0) & {\Phi}_{12}(t_0) & {\Phi}_{13}(t_0) \\
{\Phi}_{21}(t_0) & {\Phi}_{22}(t_0) & {\Phi}_{23}(t_0) \\
{\Phi}_{31}(t_0) & {\Phi}_{32}(t_0) & {\Phi}_{33}(t_0) \\
\end{bmatrix} 
&= 
\begin{bmatrix}
\mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} \\
\end{bmatrix}
\end{aligned} \tag{18}
$$
求解上述的微分方程，易得：
$$
\boldsymbol{\Phi}\left(t, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t}R & 0 & 0 \\
-\lfloor \mathbf{{y}} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_{t_0}^{t} \\
-\lfloor \mathbf{{s}} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{19}
$$
其中：
$$
\begin{cases}
\mathrm{y} = {}^{G}p_{t}-{}^{G}p_{t_0}-{}^{G}v_{t_0}(t-t_0)+\frac{1}{2}{}^{G}\mathbf{g}(t-t_0)^2 \\
\mathrm{s} = {}^{G}v_{t}-{}^{G}v_{t_0}+{}^{G}\mathbf{g}\Delta{t}_{t_0}^t \\
\end{cases} \tag{20}
$$
需要特别注意的是上述的推导是建立在初始时刻为 t0 的前提下，且认为初始时刻的转移矩阵为单位阵。

在初始的 t0 时刻，以特征点 $f_j$ 为例的系统能观性矩阵为：
$$
\mathcal{O}^{(t_0)}=\left[\mathbf{H}^{(t_0)}_{t_0|f_j}\right]
$$
因此得到初始的零空间如下：
$$
\mathbf{N}_{t_0}^{(t_0)}=
\begin{bmatrix}
\mathbf{0}_{3} & {}^{I_{t_0}}_{G}\mathbf{{R}}^{(t_0)} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t_0}^{(t_0)}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t_0}^{(t_0)}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{21}
$$
和公式（14）是一样的，于是按照老师的提示，年轻人使用**理想情况**的状态转移矩阵对初始的零空间进行传递，那么在 t 时刻得到：
$$
\Phi(t, t_0)\mathbf{N_{t_0}}=\begin{bmatrix}
\mathbf{0}_{3} & {}^{{t}}_{G}\mathbf{{R}} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{22}
$$
上述公式中因为假设是在理想情况下，因此去掉了时间的影响。

&nbsp;

#### 锦囊2. 根据状态转移矩阵，求解相邻时刻间零空间的传递情况

根据公式（19），年轻人很快的求得了相邻时刻的状态转矩矩阵为：
$$
\begin{aligned}
\Phi(t_2, t_1)&=\Phi(t_2, t_0) (\Phi(t_1, t_0))^{-1} \\
&=\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}_{t_0}^{t_1}R^{T} & 0 & 0 \\
(\lfloor \mathbf{{y}}^{(t_1)} - \mathbf{{s}}^{(t_1)}\Delta{t}_1 \rfloor_{\times})({}^{t_1}_{G}R)^{T} & \mathbf{I} & -\mathbf{I}\Delta{t}_1 \\
\lfloor \mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)}-\mathbf{{y}}^{(t_1)}+\mathbf{\hat{s}}^{(t_1)}\Delta{t}_1- \mathbf{\hat{s}}^{(t_1)}\Delta{t}_2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_2-\Delta{t}_1） \\
-\lfloor \mathbf{{s}}^{(t_2)}-\mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\end{aligned} \tag{23}
$$
于是根据锦囊1中的分析，在 t1 时刻，理想的零空间为：
$$
\Phi(t_1, t_0)\mathbf{N_{t_0}}=\begin{bmatrix}
\mathbf{0}_{3} & {}^{{t_1}}_{G}\mathbf{{R}} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{t_1}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{t_1}\right]_\times{\mathbf{g}} \\ \hline
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}} \\
\end{bmatrix} \tag{24}
$$
那么 t2 时刻的理想状态下零空间应为：
$$
\begin{aligned}
\mathbf{N_{t_2}}&=\Phi(t_2, t_1)\mathbf{N_{t_1}} \\
&=
\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} 
\begin{bmatrix}
\mathbf{0} & {}^{t_1}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_1} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_1} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \\
&= \begin{bmatrix}
\mathbf{0} & {}^{t_2}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_2} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_2} \rfloor_{\times}\mathbf{g} \\
\end{bmatrix}
\end{aligned}  \tag{25}
$$
可见在理想的情况下，零空间会跟着状态传递一起传递到当前时刻，且零空间的状态量满足一个通用的形式，该形式的物理意义为三个绝对位置和以重力方向为旋转轴的旋转。

那么公式（25）是否依旧是能观性矩阵的零空间呢？此时还需要用观测矩阵进行验证，引入公式（5）所表示的理想观测矩阵可以发现：
$$
\begin{aligned}
\mathbf{H}_{f_j|t_2}\mathbf{N_{t_2}}=\begin{bmatrix}\begin{array}{ccc|c}
\lfloor{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\rfloor_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} &  \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3}
\end{array}\end{bmatrix}\begin{bmatrix}
\mathbf{0} & {}^{t_2}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_2} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_2} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}}
\end{bmatrix} = \mathbf{0}
\end{aligned} \tag{26}
$$
年轻人也很快就悟得了老师所想要表达的意思——在理想情况下，零空间会随着状态转移一起向前传递，且保持一个通用的形式。

&nbsp;

#### 锦囊3. 修改状态传递矩阵和观测矩阵，使之满足理想情况的约束

从上面的分析可以知道理想情况下，整个系统应满足如下性质：

1. 性质一：系统的零空间可以通过状态转移矩阵传播到当前时刻 t，且形式与初始零空间相似，由 t 时刻状态的预测值有关；
2. 性质二：相邻时刻的零空间可以通过状态转移矩阵进行传播，且零空间的形式也满足通用形式；
3. 性质三：通过状态传递矩阵传播到当前时刻 t 的零空间与观测矩阵正交，亦即优化问题的优化方向与零空间正交，因此优化量不会破坏系统的零空间；

但是实际情况下，状态传递矩阵和观测矩阵并不能很好的满足上面的三个性质，但是根据老师的锦囊，年轻人很快便有了破解的方法：

1. 对于 t 时刻，假设之前的状态都已经是理想状态了，那么该时刻所需要维护的零空间为：
   $$
   \mathbf{\hat{N}}_{t-1|t-1}=
   \begin{bmatrix}
   \mathbf{0} & {}^{t-1}_{G}R^{(t-2)}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{0} & -\lfloor {}^{G}v_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\ \hline
   \mathbf{I} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}}
   \end{bmatrix} \tag{27}
   $$
   其中各个变量为 t-1 时刻的预测值，之所以为预测值，有两个原因：

   - 由公式（22）可以看到理想情况下的初始零空间由纯状态传递过程传递过来，所以在 t-1 时刻的零空间由 t-1 时刻的预测值组成；
   - 由性质三可知，理想情况下，观测矩阵与零空间正交，所以经过更新阶段不应该影响 t-1 预测阶段传递过来的零空间；

2. 经过 t 时刻的预测阶段，理想的零空间应该变为：
   $$
   \mathbf{\hat{N}}_{t|t-1}=
   \begin{bmatrix}
   \mathbf{0} & {}^{t}_{G}R^{(t-1)}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{0} & -\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\ \hline
   \mathbf{I} & -\left[^{G} \mathbf{p}_{f_j}\right]_\times{\mathbf{g}}
   \end{bmatrix}=\check{\Phi}(t,t-1) \mathbf{\hat{N}}_{t-1|t-1}  \tag{28}
   $$
   其中$\check{\Phi}(t,t-1)$为算法期望的状态转移矩阵；

3. 在更新阶段，理想的观测矩阵应该与预测阶段之后的零空间正交：
   $$
   \mathbf{\check{H}}_{f_j|t}\mathbf{\hat{N}}_{t|t-1}=\mathbf{0} \tag{29}
   $$
   其中$\mathbf{\check{H}}_{f_j|t}$为算法期望的观测矩阵；

对于上面公式（28）（29），年轻人并没有直接对方程进行求解，而是结合当前系统的状态转移矩阵$\mathbf{\hat{\Phi}}(t,t-1)$和观测矩阵$\mathbf{H}_{f_j|t}$共同求解一个最优化问题，问题形式如下：
$$
\begin{aligned}
\mathop{min}_{A} &\quad \| A-\hat{A} \|_{\mathcal{F}}^{2} \\
s.t. &\quad A\mathbf{u}=\mathbf{w} 
\end{aligned} \tag{30}
$$
对优化问题（30）运用拉格朗日乘子法以及KKT条件求解对偶问题可以得到最优解为：
$$
A=\hat{A}-(\hat{A}\mathrm{u}-\mathrm{w})(\mathrm{u}^{T}\mathrm{u})^{-1}\mathrm{u}^{T} \tag{31}
$$

> 该部分详细推导见参考【2】

&nbsp;

### 小结

可以看到，FEJ和OC-KF的方法都可以理论上较好的维护零空间，相对而言，FEJ有如下优缺点：

1. 状态传递矩阵和观测矩阵数值上直接维护了零空间，并没有像OC-KF一样额外构建最优化问题并修改系统方程的一些东西；
2. 自然而然的扩展到了Graph-base的方法上；
3. 但是比较依赖初始的线性化点，如果线性化点不好，那么后续的系统可能向不好的方向发展；

而OC-KF方法的优缺点如下：

1. 对于线性化点的依赖性不是很强，后续的优化方向虽然和FEJ方法的零空间一样正交，但是结合了当前的状态值；

2. 缺点笔者个人觉得就是比较理想，维护的都是假设理想情况的零空间；

&nbsp;

----

## DSO对于零空间的维护

如果各个SLAM或者VO的开源系统也有相应的星座的话，那笔者认为DSO当属其中的处女座了。

整个DSO中对于理论应用用料相当的足，同时各种细节也处理的相当到位，其中就不乏本文的主题——零空间。

DSO对于零空间的维护主要用了三个技巧：FEJ、增量方程的正交化、增量值的矫正，诚意算是相当足了。FEJ就不多介绍了，和上面的EKF-base中相同，这里的FEJ也是第一次得到状态变量时，把当时的值作为后续的线性化点；下面的重点放在对后面两个部分的介绍上。

### DSO为什么使用了FEJ还是用别的技术维护零空间？

从上面的分析不难知道，FEJ技术是在EKF-base的系统中推导出来的，那么对于Graph-base的系统，其实这个方法稍微显得“水土不服”，原因很简单：Graph-base的方法中并没有预测部分！如果不涉及到滑动窗口方法的话，其实整个Graph-base系统本质上是一个对数最大似然问题，即：
$$
\mathbf{E}=\mathop{log}\mathrm{p}(\mathrm{z}|\mathrm{x}) = \| \mathrm{z}- \mathrm{h}(\mathrm{x}) \|^{2}_{\omega} \tag{32}
$$
硬说的话笔者认为这里只有EKF-Base中的update部分，而预测部分由一些PnP等操作代替了，因而无法使用像EKF那样的分析方法来分析能观性与零空间的变化。

然而笔者认为FEJ的引入还是有作用的，至少整个优化问题的H矩阵被固定住了，同时