# DSO——初始化



## 写在前面

DSO的初始化着实是十分的难看懂，个人总结为以下三个原因：

- 没有采用常规的（很大程度上是因为笔者目光比较短浅）本质矩阵或者单应性矩阵求解位姿、三角化初始的landmark确定尺度，而是直接作为一个优化问题进行优化（恩，又知道了一种初始化方式）；
- 本身的光度模型就比较麻烦，而且有些公式也没有采用常规的方法（又是目光短浅），所以初看起来会比较“反常识”；
- 作者写代码确实很厉害，基本上优化过程都是自己写的，而且schur补的变量都是边求解Jacobian边构建的，最后直接一个简单的运算；

这篇文章主要是总结一下自己在看初始化代码的过程，希望能帮助更多的小伙伴。



## 优化的模型

整个优化的能量函数（也就是我们常说的误差函数）分为两个部分：一部分是光度误差；另一部分是为了帮助收敛而添加的正则项（虽然我不明白为啥添加一个能帮助收敛）；下面分两部分来说这两个部分：

### 光度误差

光度误差模型如下：
$$
E_{\mathbf{p} j}:=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\mathcal{Y}}
$$
再次说明一下Notation：

- $i, j$为参考帧和当前帧；
- $t_i, t_j$为参考帧和当前帧的曝光时间，不知道写作1；$a_i, b_i$为待求解的光度响应参数；
- $p\prime$为$p$点在$j$帧的投影点；

下面结合公式来求解Jacobian，首先把整个误差分为两个部分：几何部分和光度参数部分：
$$
\begin{aligned}
E_{\mathbf{p} j}:&=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\mathcal{Y}} \\ 
&=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}I_{i}[\mathbf{p}]\right)-\left(b_{j}-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_{i}\right)\right\|_{\mathcal{Y}}
\end{aligned}
$$
然后将光度误差$e$拿出来进行分析：
$$
e(T_{ij}, \rho_p, a_i, b_i, a_j, b_j)=\left(I_{j}\left[\mathbf{p}^{\prime}\right]-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}I_{i}[\mathbf{p}]\right)-\left(b_{j}-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_{i}\right)
$$
对于初始化过程来说，$a_i, b_i$都是0，因此可以只计算$j$帧的参数就可以了，在求解Jacobian之前，一个需要注意的细节就是在DSO中，作者使用的transform公式与常规的不太一样，对于一个3D点$dP_c$（其中$d$为该点的实际深度，$Pc$为该点在主导帧归一化平面上的坐标），通常使用如下的公式进行坐标系之间的变换，如果使用的是逆深度，通常也是把逆深度作为分母：
$$
P_j = T_{i}^{j}\begin{bmatrix}dPi \\ 1\end{bmatrix} = dR_{j}^{i}P_i+t_{i}^{j}
$$
但是DSO中对于这块的处理个人感觉还是比较好的，作者把3D点的齐次坐标改写为$[P_i, \rho]^T$的形式，即添加的齐次项并不为1，但是本身齐次坐标就没有尺度的概念，因此是完全正确的，所以使用这种形式的话，整个transform过程变作：
$$
\begin{aligned}
P_j = T_{i}^{j}\begin{bmatrix}Pi \\ \rho\end{bmatrix} = R_{j}^{i}P_i+\rho t_{i}^{j}
\end{aligned}
$$
恩，这时候对逆深度求导就舒服多了。下面分两个小部分对几何部分求导：

**首先是对相机位姿求导**：
$$
\begin{aligned}
J_{geo}&=\frac{\partial{e}}{\partial T_{ji}}=\frac{\partial{e}}{\partial I_j}\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial T_{ji}}  \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial T_{ji}} \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\begin{bmatrix} \rho \mathbf{I}_{3×3} & -[RP_{i}+\rho t]_{\times} \end{bmatrix}
\end{aligned}
$$

> 这里对$P_j$对位姿的偏导进行额外说明：
> $$
> \begin{aligned}
> \frac{\partial{P_{j}}}{\partial T_{ji}} &= \frac{Exp(\delta{\zeta})\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}-\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\delta{\zeta}} \\
> &=\frac{[\delta{\zeta}]_{\times} \begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}-\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\delta{\zeta}} \\
> &=\frac{\begin{bmatrix}[\delta{\theta}]_{\times} & \delta{\epsilon} \\ \mathbf{0} & 0\end{bmatrix}\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} \\
> &=\frac{\begin{bmatrix}[\delta{\theta}]_{\times} & \delta{\epsilon} \\ \mathbf{0} & 0\end{bmatrix}
> \begin{bmatrix}RP_{i}+\rho t\\ \rho \end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} = \frac{\begin{bmatrix}[\delta{\theta}]_{\times}(RP_{i}+\rho t)+\rho \delta{\epsilon} \\ 0\end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} \\
> &=\begin{bmatrix} \rho \mathbf{I}_{3×3} & -[RP_{i}+\rho t]_{\times} \end{bmatrix}
> \end{aligned}
> $$

**接下来是对逆深度进行求导**：
$$
\begin{aligned}
J_{geo}&=\frac{\partial{e}}{\partial \rho}=\frac{\partial{e}}{\partial I_j}\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial \rho}  \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial \rho} \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\mathbf{t_i^j}
\end{aligned}
$$
