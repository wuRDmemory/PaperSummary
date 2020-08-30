# Fisher Information matrix, Hessian matrix and Covariance matrix的关系



# Reference

1. State Estimation for Robotics.
2. [https://wiseodd.github.io/techblog/2018/03/11/fisher-information/](https://wiseodd.github.io/techblog/2018/03/11/fisher-information/) 关于Fisher Information很好的讲解



---

## 三者分别是什么

先分别介绍一下三者：

### Fisher Information matrix（下文中简单称之为information matrix）

首先给出定义，引入[wiki](https://en.wikipedia.org/wiki/Fisher_information)的定义：Fisher information matrix表示变量$x$携带了建模变量$\theta$多少的信息量（the **Fisher information** (sometimes simply called **information**) is a way of measuring the amount of information that an observable random variable *X* carries about an unknown parameter *θ* of a distribution that models *X*）。

上述的定义可能比较难理解一些，参考【2】中对公式进行了详细的推导，感兴趣的可以看一下，这里不做赘述（实际上是没有人家讲得好）。

值得注意的一点是说：Information matrix主要针对的是条件概率（似然分布）。



### Hessian matrix

Hessian矩阵平时接触的可能不多，但是Hessian矩阵的近似矩阵H矩阵就比较多了，因为总是在求解优化问题，必不可少的就会接触到优化问题的H矩阵，通常我们见到的都是最小二乘问题中的H矩阵，如下有：
$$
\begin{aligned}
E&=||z-f(x)||^2_w=||z-f(x)+J\delta{x}||^2_w \\
&=(e+J\delta{x})^TW(e+J\delta{x}) \\
&=e^Twe+\delta{x}^TJ^TWe+e^TWJ\delta{x}+\delta{x}^TJ^TWJ\delta{x} \\
\end{aligned} \tag{1}
$$
其中$J^TWJ$就称为H矩阵。

Hessian矩阵其实说白了就是$E$对于状态变量$x$的二阶偏导数。



### Covariance matrix

这个就是熟知的协方差矩阵了，这里不做多介绍，仅仅给出定义就好了：
$$
p(\mathbf{x} | \boldsymbol{\mu}, \mathbf{\Sigma})=\frac{1}{\sqrt{(2 \pi)^{N} \operatorname{det} \mathbf{\Sigma}}} \exp \left(-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^{T} \mathbf{\Sigma}^{-1}(\mathbf{x}-\boldsymbol{\mu})\right) \tag{2}
$$
注意其中$\Sigma$是协方差矩阵，它的逆不叫作协方差矩阵。



---

## 三者的关系

先给出结论，简单来说：

1. **对于似然分布$p(y|x)$而言**，Information矩阵就是负对数似然问题的Hessian矩阵的期望；
2. **对于分布$p(x)$而言**，负对数似然问题的协方差矩阵的逆就是Hessian矩阵；

第二个关系其实可以应用到似然问题中，毕竟似然概率分布也是一种概率分布。证明如下：

### Information矩阵和Hessian矩阵

假定条件概率分布为$p(x|\theta)$，那么它的Information矩阵为：
$$
\mathrm{F}=\underset{p(x | \theta)}{\mathbb{E}}\left[\nabla \log p(x | \theta) \nabla \log p(x | \theta)^{\mathrm{T}}\right] \tag{3}
$$
现取条件概率分布的对数形式有，注意这里还没有取负对数：
$$
E=\log{p(x|\theta)} \tag{4}
$$
因为Hessian矩阵就是对公式（4）中的E进行二次求导：
$$
\begin{aligned}
\mathrm{H}_{\log p(x | \theta)} &=\mathrm{J}\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right) \\
&=\frac{\mathrm{H}_{p(x | \theta)} p(x | \theta)-\nabla p(x | \theta) \nabla p(x | \theta)^{\mathrm{T}}}{p(x | \theta) p(x | \theta)} \\
&=\frac{\mathrm{H}_{p(x | \theta)} p(x | \theta)}{p(x | \theta) p(x | \theta)}-\frac{\nabla p(x | \theta) \nabla p(x | \theta)^{\mathrm{T}}}{p(x | \theta) p(x | \theta)} \\
&=\frac{\mathrm{H}_{p(x | \theta)}}{p(x | \theta)}-\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right)\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right)^{\mathrm{T}}
\end{aligned}  \tag{5}
$$
其中：

- $\nabla$表示$\frac{\part}{\part{\theta}}$操作，公式（3）中的符号也是这个意思；
- $J(.)$表示对括号内的式子求关于$\theta$的偏导数；
- $H_{p(x|\theta)}$表示$p(x|\theta)$对于$\theta$的二阶导数；

于是看到，似然问题的对数形式包含了两个部分：其中第二个部分其实就已经跟上面的information矩阵只差一个期望了，于是对于公式（5）取关于$p(x|\theta)$的期望，有：
$$
\begin{aligned}
\underset{p(x | \theta)}{\mathbb{E}}\left[\mathrm{H}_{\log p(x | \theta)}\right] &=\underset{p(x | \theta)}{\mathbb{E}}\left[\frac{\mathrm{H}_{p(x | \theta)}}{p(x | \theta)}-\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right)\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right)^{\mathrm{T}}\right] \\
&\left.\left.=\mathbb{E}_{p(x | \theta)}\left[\frac{\left.\mathrm{H}_{p(x | \theta)}\right]}{p(x | \theta)}\right]-\underset{p(x | \theta)}{\left[\frac{\mathrm{E}}{p(x | \theta)}\right.}\right)\left(\frac{\nabla p(x | \theta)}{p(x | \theta)}\right)^{\mathrm{T}}\right] \\
&=\int \frac{\mathrm{H}_{p(x | \theta)}}{p(x | \theta)} p(x | \theta) \mathrm{d} x-\underset{p(x | \theta)}{\mathbb{E}}\left[\nabla \log p(x | \theta) \nabla \log p(x | \theta)^{\mathrm{T}}\right] \\
&=\mathrm{H}_{\int p(x | \theta) \mathrm{d} x}-\mathrm{F} \\
&=\mathrm{H}_{1}-\mathrm{F} \\
&=-\mathrm{F}
\end{aligned} \tag{6}
$$
化简中使用了一个性质，因为$H_{p(x|\theta)}$是对$\theta$的二阶导，与$x$没有关系（可以想象为$p(x|\theta)$是一个观测函数$x=f(\theta)$，因此求导的步骤与x无关），因此这个二阶导的符号可以直接提出去，导致了对常数1进行了二阶导，因此前一部分为0。

在实际的应用中，通常都用Hessian矩阵表示Information矩阵，原因是因为（笔者个人认为）对于某个时刻而言，其似然概率密度函数是一个单位脉冲函数，整个自变量取值范围内仅仅只有一个地方的概率为1，其余为0，所以Hessian矩阵的期望就退化为Hessian矩阵，也就是为什么在g2o等优化库中直接认为协方差矩阵的逆是信息矩阵。

&nbsp;

### Hessian matrix与Covariance matrix

这个部分就更简单了，证明如下：
$$
p(\boldsymbol{\theta})=(2 \pi)^{-\frac{N_{\theta}}{2}}\left|\boldsymbol{\Sigma}_{\theta}\right|^{-\frac{1}{2}} \exp \left[-\frac{1}{2}\left(\boldsymbol{\theta}-\boldsymbol{\theta}^{\star}\right)^{T} \boldsymbol{\Sigma}_{\theta}^{-1}\left(\boldsymbol{\theta}-\boldsymbol{\theta}^{\star}\right)\right] \tag{7}
$$
取负对数形式：
$$
J(\boldsymbol{\theta}) \equiv-\ln p(\boldsymbol{\theta})=\frac{N_{\boldsymbol{\theta}}}{2} \ln 2 \pi+\frac{1}{2} \ln \left|\boldsymbol{\Sigma}_{\boldsymbol{\theta}}\right|+\frac{1}{2}\left(\boldsymbol{\theta}-\boldsymbol{\theta}^{\star}\right)^{T} \boldsymbol{\Sigma}_{\boldsymbol{\theta}}^{-1}\left(\boldsymbol{\theta}-\boldsymbol{\theta}^{\star}\right) \tag{8}
$$
最后对参数$\theta$进行二次求导，显然得到：
$$
H(\theta)=\Sigma_{\theta}^{-1} \tag{9}
$$


