# 2D射影变换的估计

## 写在前面

说实话这一章节在看的时候真的比较容易“陷”进去，所以在开始的时候，不妨将整个章节的内容抽简出来——如何从点集与点集的对应关系中，得到一个比较鲁邦的射影变换（单应性矩阵）。

针对上面这个简单的目的，本章节主要探讨了如下的东西：

1. 求解代数解的方法——DLT，超定方程；
2. 迭代式的求解方法——误差函数（进而讨论了一些误差函数的定义方法）；
3. 更加鲁邦的方法；

本文也按照上面的流程来一步步的总结；

---

## 求解代数解的方法

### 测量数

测量数是指如果要计算单应性矩阵，需要多少对应的匹配点才可以。因为射影变换矩阵H的变量总数为9，但是由于是齐次矩阵，所以自由度为8。又因为一对匹配点能提供两个约束，因此4对点就可以提供8个约束，求解该问题，该解称为最小配置解。但是通常情况下，对应点中充满了噪声，所以一般情况下需要给出多于4组的匹配点数用于更加鲁邦的估计——超定方程或者RANSAC求解。

&nbsp;

### DLT——直接线性求解

直接线性求解在书中其实有两种方案：

#### 方案1——求解形如Ax=0的解

首先给出射影变换的公式$\mathbf{x'=Hx}$，可以看到一对真正的匹配之间必然满足：
$$
\mathbf{x'\times Hx}=\begin{bmatrix}\mathbf{y'h_{3}^{T}x-w'h_{2}^{T}x} \\ \mathbf{w'h_{1}^{T}x-x'h_{3}^{T}x} \\ \mathbf{x'h_{2}^{T}x-y'h_{1}^{T}x} \end{bmatrix}=
\underbrace{\begin{bmatrix}0 & \mathbf{-w'x^{T}} & \mathbf{y'x^{T}} \\ \mathbf{w'x^{T}} & 0 & \mathbf{-x'x^{T}} \\ \mathbf{-y'x^{T}} & \mathbf{x'x^{T}} & 0\end{bmatrix}}_{A} \begin{bmatrix}\mathbf{h_1} \\\mathbf{h_2} \\ \mathbf{h_3} \end{bmatrix} =0 \tag{1}
$$
其中矩阵A的秩为2，所以其实保留其中两个等式就可以了，一般选择前两个等式。对于这样的一种问题来说，如果我们有4对匹配点，那么前面的A矩阵的维度就是8x9，如果我们把求解的矩阵当做齐次向量来解，那么其实求得的解是差一个绝对的尺度的，所以我们可以添加一个约束条件为$\left|\mathbf{x}\right|=1$的约束条件，其实添加了约束条件之后，就等于可以直接使用SVD得到最优解，同时该最优解一定满足该约束条件；

#### 方案2——求解形如Ax=b的解

同样的，如果仅仅使用$\mathbf{x'=Hx}$的公式进行构建方程的话，则对于每一个点对，有：
$$
\left[\begin{array}{ccccccc}
0 & 0 & 0 & -x_{i} w_{i}^{\prime} & -y_{i} w_{i}^{\prime} & -w_{i} w_{i}^{\prime} & x_{i} y_{i}^{\prime} & y_{i} y_{i}^{\prime} \\
x_{i} w_{i}^{\prime} & y_{i} w_{i}^{\prime} & w_{i} w_{i}^{\prime} & 0 & 0 & 0 & -x_{i} x_{i}^{\prime} & -y_{i} x_{i}^{\prime}
\end{array}\right] \overline{\mathbf{h}}=\left(\begin{array}{l}
-w_{i} y_{i}^{\prime} \\
-w_{i} x_{i}^{\prime}
\end{array}\right) \tag{2}
$$
所以同样的，4对匹配点能构建一个8x9的求解问题，在h中，仅仅需要把其中的某一个变量设置为1，例如$h_j=1$，则整个问题都可以进一步求解，但是如此最大的问题就是，如果真正的$h_j=0$，那么整个问题的求解将会变得不稳定。

&nbsp;

### 超定方程

上面的方法都是基于有4个匹配点对，当匹配点对多于4个的时候，不管是哪个方程的A矩阵都是Nx9的形式，其中N>9，因此此时可以求解一个超定方程，这样的解会稍微稳定一些。

&nbsp;

-----

## 迭代式的求解方法

这部分是笔者觉得整个书中编排问题（或者说是大佬们没有特别照顾笔者这种菜鸡）比较大的一点，整个章节读下来相当的突兀，不知道为什么就出了一个小节讲这个，一会儿又蹦出来一个小节讲那个。所以笔者在这部分打算先总观的进行俯瞰干什么，之后再看达到这样的目的需要什么，这样会比较清晰一些。

优化迭代的求解方法其实无非就是找一个代价函数，之后就是使用各种迭代的方法求解到最优解，整个章节的大部分也是围绕着这样的目的展开：

1. 代价函数——对应书中的第二节，介绍了代数距离，几何距离以及Sampson距离；
2. 建模部分——如何使用单个代价函数构建最大似然估计；
3. 数据处理——如何处理数据能进行更好的问题估计，这里面也研究了一些不变性的东西；
4. 优化方法；

疏通了上面的顺序之后，其实再会过头去看整个章节的内容就会比较清晰了。



### 不同的代价函数

#### 代数距离

代数距离是一种数学意义的距离，通常称$\xi=\mathrm{A}\mathbf{h}$为残差矢量，在我们的目的中，我们可以把该矢量的范数作为代数距离进而进行优化：
$$
d_{n k}\left(\mathbf{x}_{i}^{\prime}, H \mathbf{x}_{i}\right)^{2}=\left\|\boldsymbol{\varepsilon}_{i}\right\|=\left\|\left[\begin{array}{ccc}
\boldsymbol{0}^{\boldsymbol{T}} & -w_{i}^{\prime} \mathbf{x}_{i}^{\top} & y_{i}^{\prime} \mathbf{x}_{i}^{\top} \\
w^{\prime} \mathbf{x}_{i}^{\top} & \boldsymbol{0}^{\top} & -x_{i} \mathbf{x}_{i}^{\top}
\end{array}\right] \mathbf{h}\right\|^{2} \tag{3}
$$
对于任意两个矢量$\mathbf{x_1}$和$\mathbf{x_2}$，我们可以将如下公式作为代数距离：
$$
d_{alg}(\mathrm{x_1}, \mathrm{x_2})^{2}=a_1^{2}+a_2^{2} \quad \mathbf{a}=(a_1, a_2, a_3)^{T}=\mathrm{x_1}\times\mathrm{x_2} \tag{4}
$$
根据公式（3）的定义，所以一对匹配点$\mathrm{x'}=(x_1', x_2', x_3')^{T}$ 和 $\mathrm{\hat{x}}'=H\mathrm{x}=(\mathrm{\hat{x}_1}', \mathrm{\hat{x}_2}', \mathrm{\hat{x}_3}')$ 的代数距离为：
$$
\mathrm{A}\mathbf{h}=\varepsilon=\begin{bmatrix}\mathrm{y}'\mathrm{\hat{w}}'-\mathrm{w}\mathrm{\hat{y}}' \\ \mathrm{w}'\mathrm{\hat{x}}'-\mathrm{x}\mathrm{\hat{w}}' \end{bmatrix} \tag{5}
$$
所以取其范数为：
$$
d_{alg}(\mathrm{x}',\mathrm{\hat{x}}')^{2}=(\mathrm{y}'\mathrm{\hat{w}}'-\mathrm{w}\mathrm{\hat{y}}')^{2}+(\mathrm{w}'\mathrm{\hat{x}}'-\mathrm{x}\mathrm{\hat{w}}')^{2}
$$

#### 几何距离

几何距离比较好理解，但是书中对于几何距离又进行了一系列的划分：单图像误差，对称转移误差，以及重投影误差；其中单图像误差和对称转移误差都比较好理解，这里不再赘述，重投影误差是一个值得去讲一下的东西，这里就讲一下笔者自己的理解；

##### 重投影误差

书中这里介绍的重投影误差其实（至少和笔者理解的）和之前见到的重投影误差不同，但是仔细想想又很相同！在笔者看来，重投影误差是给了一个空间3D点，之后通过位姿关系和相机投影矩阵投影到当前的图像坐标系中。

对于仅有两个图像坐标观测的例子中，我们是无法找到上述这样的投影关系的，但是我们可以想象一下有一个“隐式”的3D点，经过了两次投影之后，分别投影到了两幅图的图像上，他们之间由一个单应矩阵联系起来！于是整个重投影误差看起来就是这样子的：

<img src="pictures/1.png">

其中：

1. $\mathrm{x, x'}$ 是图像的测量值，而$\mathrm{\hat{x}, \hat{x}'}$ 则是由“隐式"的3D点投影下来的图像坐标；

2. $\mathrm{\hat{x}, \hat{x}'}$ 之间由一个单应性矩阵直接连接（映射）；

3. 所以重投影误差的公式就是：
   $$
   \sum_{i} d\left(\mathbf{x}_{i}, \hat{\mathbf{x}}_{i}\right)^{2}+d\left(\mathbf{x}_{i}^{\prime}, \hat{\mathbf{x}}_{i}^{\prime}\right)^{2} \quad \text { s.t } \quad \hat{\mathbf{x}}_{i}^{\prime}=\hat{H} \hat{\mathbf{x}}_{i} \quad \forall i \tag{6}
   $$


#### Sampson误差

Sampson误差笔者认为还是从二次曲线的拟合开始比较好，毕竟本身也是这个方法的来源。

考虑一个场景：我们有一些数据之后，需要拟合一个二次曲线，形如 $\mathrm{x^{T}Cx}=0$ ，那么一个比较大的问题就是如何度量某个数据点与二次曲线的**几何误差（代数误差就是代入公式之后的误差了）**，Sampson给出了如下的方法求解：

1. 设函数$\mathcal{V}(\mathrm{x})=\mathrm{x^{T}Cx}$，显然，真正在曲线上的点$\mathrm{\overline{x}}$ 一定可以满足$\mathcal{V}(\mathrm{\overline{x}})=0$；

2. 假设测量 $\mathrm{x}$ 与曲线上的最近点 $\mathrm{\overline{x}}$ 之间的距离为 $\delta{\mathrm{x}}$；

3. 将 2 的假设代入 1 中，就可以得到：
   $$
   \begin{aligned}
   \mathcal{V}(\mathrm{\overline{x}})&=\mathcal{V}(\mathrm{x+\delta{x}}) \\ 
   &=\mathcal{V}(\mathrm{x})+\mathbf{J}\delta{\mathrm{x}}=0 
   \end{aligned}
   \tag{7}
   $$

4. 在 2 中，其实我们做了一个最优化的假设，就是 $\delta{\mathrm{x}}$ 是测量与曲线点最近的点，所以其实我们需要求解一个最优化的问题：
   $$
   \begin{aligned}
   \mathrm{min}_{\delta x} \quad &\lVert\delta{\mathrm{x}}\rVert^{2} \\ 
   s.t. \quad &\mathcal{V}(\mathrm{x})+\mathbf{J}\delta{\mathrm{x}}=0 
   \end{aligned} \tag{8}
   $$
   直接上拉格朗日乘子法，得到（这里和书中的公式稍微有些不同的是把中间的减号换成了加号，因为整个问题在求最小值，但是如果用减法，可以约定$\lambda$是小于等于0的）：
   $$
   \mathcal{L}(\delta{\mathrm{x}})=\delta{\mathrm{x}}^{T}\delta{\mathrm{x}}+2\lambda^{T}(\mathcal{V}(\mathrm{x})+\mathbf{J}\delta{\mathrm{x}}) \tag{9}
   $$
   于是按照拉格朗日乘子法的求解可以得到：
   $$
   \begin{aligned}
   \frac{\partial \mathcal{L}(\delta{\mathrm{x}})}{\partial \delta{\mathrm{x}}}=0 &\Rightarrow \delta{\mathrm{x}}^{T}+\lambda \mathbf{J}=0 \\
   \frac{\partial \mathcal{L}(\delta{\mathrm{x}})}{\partial \lambda}=0 &\Rightarrow \mathcal{V}(\mathrm{x})+\mathbf{J}\delta{\mathrm{x}}=0  
   \end{aligned} \tag{10}
   $$
   公式（10）联立就可以得到$\lambda=(\mathbf{J}\mathbf{J}^{T})^{-1}\mathcal{V}(\mathrm{x})$，再次代入公式（10）的第一个约束中，得到：
   $$
   \delta{\mathrm{x}}=-\mathbf{J}^{T}(\mathbf{J}\mathbf{J}^{T})^{-1}\mathcal{V}(\mathrm{x}) \tag{11}
   $$
   于是距离值，也就是
   $$
   \lVert\delta{\mathrm{x}}\rVert^{2}=\delta{\mathrm{x}}^{T}\delta{\mathrm{x}}=\mathcal{V}(\mathrm{x})^{T}(\mathbf{J}\mathbf{J}^{T})^{-1}\mathcal{V}(\mathrm{x}) \tag{12}
   $$


如果我们将$\mathbf{J}\mathbf{J}^{T}$当做是$\mathcal{V}(\mathrm{x})$的协方差矩阵，那么我们推了半天就推出来了马氏距离。其实我们仅仅是用公式（12）的结论，也就是我们如果有一个曲线函数$\mathcal{V}(\mathrm{x})=0$，那么我们就可以用公式（12）定义测量的Sampson误差。

回到这节的主旨上来：为了迭代优化求解找合适的代价函数，而Sampson误差给与了我们一种方法，只要我们有理想点的曲线方程，那么就可以很直接的得到测量到该曲线的误差。

书中给出了一个例子：使用代数误差来作为上述的曲面函数$\mathcal{V}(\mathrm{x})=\mathrm{A}(\mathrm{x})\mathbf{h}$，其中 $\mathrm{x}$ 是由两个 2D 测量堆叠起来，形如 $\mathrm{x=[x, y, x',y']}$。所以我们可以用公式（12）得到优化步骤中的误差部分，同时因为公式中的 $\mathcal{V}(\mathrm{x})$ 和 $\mathbf{J}$ 都是关于 $\mathbf{H}$ 的函数，所以整个误差可以对射影变换 $\mathbf{H}$ 进行求导（目之所及就觉得贼麻烦的形式），也就提供了后面对 $\mathbf{H}$ 的优化方向 。



