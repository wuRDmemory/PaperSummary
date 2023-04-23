# FEJ2

---

## 写在前面

最近看到黄国权老师组又出了FEJ最新的讨论，赶紧拿来看了一下，整体而言是对于线性化点如何影响系统更加深入的讨论，在一些数据集的测试中也取得了比FEJ或者OC-VINS更好的效果。

&nbsp;

---

## 符号表示

首先还是把文章的符号在前面表示清楚：

- 状态变量，这里依旧是JPL表示法： $\mathbf{x=\begin{bmatrix}x_I^T & x_f^T\end{bmatrix}^{T}=\begin{bmatrix}{}^{I}_{G}\overline{q} & {}^{G}p_{I}^{T} & {}^{G}v_{I}^{T} & b_{g}^{T} & b_{a}^{T} |& {}^{G}p_{f_{1}}^{T} & \dots & {}^{G}p_{f_{M}}^{T} \end{bmatrix}^{T}}$
- 状态转移矩阵：$\mathbf{x_{I_{k+1}}=f(x_{I_k},a_{m_k},w_{m_k})}$，得到的状态转移矩阵为：$\Phi(k+1,k)$；
- 误差状态的协方差矩阵：后验协方差表示为$\mathbf{P_k}$，似然协方差表示为$\mathbf{P_{k+1|k}}$；
- 观测方程：$\mathbf{z_{k}=h(x_{k})+n_{k} \simeq h(\hat x)+\hat H({x}\boxminus \hat x)+n_{k}}$；

特别的，对于上述的变量，$\mathbf{\hat{x}}$ 表示当前的估计变量，x表示理想的状态变量， $\mathbf{\tilde{x}=x\boxminus \hat x}$ 表示误差变量；

&nbsp;

-----

## 方法

### 老方法的总结

目前来看，真正在理论上能保证能观性矩阵保持一致的方法有两种

- OC-VINS： observability-based consistency维护方法，这种方法主要的步骤如下：

  - 作者发现真值所构成的状态转移矩阵是可以将 0 时刻的零空间传递到 k 时刻的，且仅与 k 时刻的真值有关；

  - 于是作者就通过 **修改** 状态转移矩阵 $\Phi(k+1,k) \rightarrow \hat{\Phi}(k+1, k)$ 来使得状态转移矩阵不破坏 k 时刻的零空间，亦即与之前的零空间正交；

  - 当然除了修改状态转移矩阵，还要对观测矩阵 $H$ 进行修改，使得观测矩阵也与零空间正交，保证能观性矩阵的零空间一直保持一致；

  - 上述过程中其实作者构建了一个最小化问题，使用拉格朗日乘子法进行了求解；

  - 上述步骤简单表示为：
    $$
    \begin{cases}
    \mathrm{N_{k+1}=\Phi'(k+1,k)N_{k}} \\
    \mathrm{H'_{k}N_k=0}
    \end{cases}  \tag{1}
    $$

- FEJ-VINS：FEJ的核心思想是固定住后续过程中的线性化点，这样在状态的propagation中，可以保证所有变量不引入更新过程中所带来的扰动，从而保证零空间的一致性，主要的方法步骤如下：

  - 作者也是从初始状态出发，按照能观性矩阵的公式推导 k 时刻的能观性矩阵；
  - 作者发现如果在推理 k 时刻的能观性矩阵时，k 时刻之前的状态转移矩阵使用更新后的状态作为线性化点，那么整个能观性矩阵将会引入很多扰动项，导致零空间的变化；
  - 但是如果将线性化点都使用propagation之后的状态变量，即 $\mathbf{X_{k}^{k+1}}$ 的话，就不会引入这些扰动项，且零空间得以保持；

对于两者而言：

- OC-VINS会比较依赖初始的零空间 $\mathbf{N_0}$，也就是比较依赖最开始时刻的propagation值；
- FEJ-VINS因为固定了线性化点，如果线性化点比较差，那么后续的效果也会比较差；

&nbsp;

### FEJ2方法

FEJ2 在FEJ的基础上，把当前时刻的线性化点考虑进来，并将该线性化点的影响加入进来，从而提升非线性优化问题的性能：
$$
\begin{aligned}
\mathbf{z_{k+1}} &\simeq \mathrm{h(\hat{x}_{k+1})+\hat{H}_{k+1}\tilde{x}_{k+1}+n_{k+1} } \\
&=\mathrm{h(\hat{x}_{k+1})+(\overline{H}_{k+1}+\hat{H}_{k+1}-\overline{H}_{k+1})\tilde{x}_{k+1}+n_{k+1}} 
\end{aligned} \tag{2}
$$
对上面进行变换得到：
$$
\begin{aligned}
\mathrm{\hat{r}_{k+1}}&=\mathrm{z_{k+1}-h(\hat{x}_{k+1})} \\ 
&\simeq \mathrm{\overline{H}_{k+1}\tilde{x}_{k+1}+\Delta{H}_{k+1}\tilde{x}_{k+1}+n_{k+1}}
\end{aligned} \tag{3}
$$
于是看到，在 FEJ 的基础上，FEJ2 把当前线性化点与固定线性化点的差考虑了进来，消除该误差矩阵 $\Delta{H}$ 的办法使用QR分解：
$$
\mathrm{\Delta H_{k+1}}=\begin{bmatrix} \Delta{Q}_{k+1} & \Delta{U}_{k+1} \end{bmatrix}\begin{bmatrix}\Delta{T}_{k+1} \\ 0\end{bmatrix} \tag{4}
$$
于是在公式（3）的左右均使用 $\Delta{U}_{k+1}$ 映射到 $\Delta{H}_{k+1}$ 的零空间中，则有：
$$
\begin{aligned}
\mathrm{\Delta{U}_{k+1}^{T}\hat{r}_{k+1}}&=\mathrm{\Delta{U}_{k+1}^{T}\overline{H}_{k+1}\tilde{x}_{k+1}+\Delta{U}^{T}n_{k+1}} \\
\mathrm{r^{*}_{k+1}} &= \mathrm{H^{*}_{k+1}\tilde{x}_{k+1}+n^{*}_{k+1}}
\end{aligned} \tag{5}
$$
噪声服从新的高斯分布 $\mathrm{n^{*}_{k+1} \sim N(0, \Delta{U}_{k+1}^{T}R\Delta{U}_{k+1})}$； 

&nbsp;

### FEJ2的一些性质

#### 结论一：FEJ2能保持系统的能观性

这部分比较好理解，不过多赘述；

&nbsp;

#### 结论二：FEJ2 能更好的估计系统的信息矩阵

由公式（5）可知，系统的信息矩阵如下：
$$
\begin{aligned}
\Sigma^{*}&=\mathrm{{H^{*}}^{T}H^{*}=(\Delta{U}^{T}\overline{H})^{T}(\Delta{U}^{T}\overline{H})} \\
 &= \mathrm{ \overline{H}^{T}\overline{H} - \overline{H}^{T} \Delta{Q} \Delta{Q}^{T} \overline{H} } \\ 
 &= \overline{\Sigma}-\Delta{\Sigma}
\end{aligned} \tag{6}
$$
所以看到：

- 在更新阶段，因为有 $\Delta{H}$ 的存在，导致系统的信息矩阵变小，协方差矩阵变大（也就是说当前线性化点 $\mathrm{\hat{x}}$ 与FEJ线性化点 $\mathrm{\overline{x}}$ 偏离的越多，观测所携带的信息就越小，表示越不可靠）；
- 相比于FEJ，上述的变化显然是比较合理的，因为在FEJ中，当前线性化点产生的 error 的信息矩阵一致保持了原先的信息矩阵，即 $\overline{H}^{T}\overline{H}$；

&nbsp;

### 实现中的FEJ2

作者给出了 FEJ2 在 EKF 和 MSCKF 中的实现细节，EKF不是本文所讨论的重点，所以这里不做过多的叙述，重点关注一下MSCKF的实现细节；

#### MSCKF with FEJ2

MSCKF 在窗口中会保存一些相机的位姿以及特征点，在观测更新阶段，主要也是求解这些相机位姿和特征点的观测矩阵，如下：
$$
\begin{aligned}
\mathrm{H}_{\theta}&=\lfloor{}^{I}_{G}R({}^{G}p_{f_{i}-{}^{G}p_{I}})\rfloor \\
\mathrm{H}_{p}&=-\mathrm{H}_{f}=-{}^{I}_{G}R \\
\mathrm{H}_{cam}&=\begin{bmatrix}\mathrm{H}_{\theta} & \mathrm{H}_{p} & | & \mathrm{H}_{f} \end{bmatrix}
\end{aligned} \tag{7}
$$
那么对于 $k$ 时刻相机观测特征点 $j$ 而言，其 FEJ2 中的 $\Delta{H}$ 如下：
$$
\mathrm{\Delta{H}_{cam}^{kj}} {}_{3 \times 9} =\begin{bmatrix} \Delta{H}_{\theta}^{k} {}_{3\times3} & \Delta{H}_{p}^{k} {}_{3\times3} & | & \Delta{H}_{f_j}^{k} {}_{3\times3} \end{bmatrix} \tag{8}
$$
假设观测到 m 个特征点，则整体的 $\Delta{H}$ 的维度为 $\left[ 3m \times （6+3m） \right]$，于是我们看到，整个矩阵随着m的增多，不可能成为一个列满秩矩阵，因而就不能进行QR分解，作者说道，即使是使用双目，矩阵的维度为$\left[4m \times (6+3m) \right]$ ，此时一定存在一定数量的观测，可以使得列满秩，但是由于 $\Delta H_{f}$ 的数值（旋转矩阵的差）比较小，因而在QR分解中会出现数值的不稳定情况，所以在实际的过程中，作者仅仅取了位姿部分来进行QR分解。

&nbsp;

----

## 总结

如上所述，FEJ2 对线性化点进行了更加深入的探索，随着当前状态的更新，FEJ2 考虑了当前的线性化点会使得系统：

1. 继续保持了系统的能观性；
2. 得到了更加准确的系统信息矩阵；

同时，作者也讨论了在EKF以及MSCKF下的实现细节：为了QR分解的数值稳定，**注意** 在分解的时候仅考虑位姿部分的 $\Delta H$ 就可以了。