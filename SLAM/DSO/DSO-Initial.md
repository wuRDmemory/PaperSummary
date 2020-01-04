# DSO——初始化



## 写在前面

DSO的初始化着实是十分的难看懂，个人总结为以下三个原因：

- 没有采用常规的（很大程度上是因为笔者目光比较短浅）本质矩阵或者单应性矩阵求解位姿、三角化初始的landmark确定尺度，而是直接作为一个优化问题进行优化（恩，又知道了一种初始化方式）；
- 本身的光度模型就比较麻烦，而且有些公式也没有采用常规的方法（又是目光短浅），所以初看起来会比较“反常识”；
- 作者写代码确实很厉害，基本上优化过程都是自己写的，而且schur补的变量都是边求解Jacobian边构建的，最后直接一个简单的运算；

这篇文章主要是总结一下自己在看初始化代码的过程，希望能帮助更多的小伙伴。



## 优化的模型

整个优化的能量函数（也就是我们常说的误差函数）分为两个部分：一部分是光度误差；另一部分是添加的正则项，帮助收敛；下面分两部分来说这两个部分：

### 光度误差

光度误差模型如下：
$$
E_{\mathbf{p} j}:=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\mathcal{Y}}
$$
再次说明一下Notation：

- $i, j$为参考帧和当前帧；
- $t_i, t_j$为参考帧和当前帧的曝光时间，不知道写作1；$a_i, b_i$为待求解的光度响应参数；
- $p\prime$为$p$点的投影点；

下面结合公式来求解Jacobian：
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
对于初始化过程来说，$a_i, b_i$都是0，因此可以只计算$j$帧的参数就可以了。

