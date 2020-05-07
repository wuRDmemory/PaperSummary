# 基础概率知识

## Reference

1. [https://zhuanlan.zhihu.com/p/103157485](https://zhuanlan.zhihu.com/p/103157485)  关于最基础的三种离散部分的讲解
2. [https://zhuanlan.zhihu.com/p/37976562](https://zhuanlan.zhihu.com/p/37976562)  gamma分布和beta分布的讲解，感觉还是很不错的
3. [https://cosx.org/2013/01/lda-math-gamma-function/](https://cosx.org/2013/01/lda-math-gamma-function/)  关于gamma函数的硬科普
4. [https://towardsdatascience.com/beta-distribution-intuition-examples-and-derivation-cf00f4db57af?gi=32fa0291b303](https://towardsdatascience.com/beta-distribution-intuition-examples-and-derivation-cf00f4db57af?gi=32fa0291b303)  较好的关于beta分布的讲解

---
## 基本知识

### 基本表示
1. 事件: 使用$X$表示
2. 变量: 使用$x$表示

### 概率分类
1. 离散概率：使用$P(X=x)$表示，表示事件取$x$的概率；
2. 连续概率：更加准确的说法应该是连续概率密度，使用$p(x)$表示，注意连续概率中不能表示单点概率，只能表示区间概率，公式如下：
$$
P(a<x<b)=\int_a^bp(x)dx
$$
​		所以当表示单点概率的时候，发生的概率是无穷小的

---
## 几个基础的概率分布

### 伯努利分布（离散分布）
1. 形象记为掷一次硬币, $X=1/0$发生的概率；
2. 做一次实验，事件发生的概率为$P(X=1)=p$，不发生的概率为$P(X=0)=(1-p)$，所以结合起来就是：
    $$
    P(X)=p^{x}(1-p)^{1-x}
    $$



### 二项分布（离散分布）

1. 形象记为抛多次硬币，$X=1/0$发生多少次的概率；
2. 公式直接写作：
    $$
    P(X=k)=C_{n}^{k}p^{k}(1-p)^{n-k}
    $$
3. 期望为：$E(X)=np$



### 多项分布（离散分布）

1. 形象记为掷股子，该事件有多种不同的情况：

2. 公式写作（以掷股子为例）：
   $$
   P\left(x_{1}, x 2, \cdots, x_{6}\right)=\frac{n !}{x_{1} ! x_{2} ! \cdots x_{6} !} p_{1}^{x_{1}} p_{2}^{x_{2}} \cdots p_{6}^{x_{6}}
   $$



### 高斯分布（连续分布）

这个就不细说了，遇到情况最多的一种连续概率分布
$$
f(x)=\frac{1}{\sqrt{2 \pi} \sigma} \exp \left(-\frac{(x-\mu)^{2}}{2 \sigma^{2}}\right)
$$


### Beta分布（连续分布）

1. 举个形象的例子：当你购物时候，知道好评有K，差评有(N-K)个时，估计这家店铺好评率最可能是多少，注意不要简单的说是$K/N$哦

2. 祭出分布的公式：
   $$
   f(x)=\frac{\Gamma(\alpha+\beta)}{\Gamma(\alpha) \Gamma(\beta)} x^{\alpha-1}(1-x)^{\beta-1}
   $$

3. 这个公式和前面的二项分布公式很相似，但是实际上，二项分布讨论的是成功的次数，而Beta分布则讨论的是在成功这么多次的条件下，成功的概率是多少；