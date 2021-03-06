# CNN

1. yolo2的边界框聚类是如何做的？

	yolo2对voc和coco的边界框进行了聚类，主要的做法是：
    - a. 先保存所有的ground truth位置，[xi, yi, wi, hi];
    - b. 随机初始化k个聚类中心[wj, hj];
    - c. 在每个中心[xi, yi]处，用真实值[wi, hi]与聚类中心[wj, hj]求解IOU，用$d=1-IOU$当做度量值，挑选最近接的聚类中心[wl, hl]，并将该距离值累计；
    - d. 所有中心遍历完成之后，得到累计值sum，同时对于每一个聚类中心[wk, hk]，其有一堆真实的值[wi, hi]，使用他们的平均值作为新的聚类中心，直到聚类中心变化不大为止；

	refer: [https://blog.csdn.net/shiheyingzhe/article/details/83995213](https://blog.csdn.net/shiheyingzhe/article/details/83995213)

2. label-smoothing是什么？怎么被提出的？

	label-smoothing在InceptionV3中被提出，主要的思想如下：
    交叉熵的公式如下：
    $$\ell=-\sum_{k=1}^{K} \log (p(k)) q(k)$$
	其中$p(k)=\frac{exp(z_k)}{\sum{exp(z_k)}}$, 而$q(k)$则是第k个样本的概率，取值为[0, 1]，由此引入一个思考: 当网络的预测$z_k >> z_j \quad where \quad j!=k$时，诚然，此时的loss是比较小的，但是反过来，网络在前期的时候，输出的$z_k$并不满足上述的条件，此时loss依旧只去调整$z_k$而没有很好的利用其他的信息(比如loss此时应该意识到要把输出的z中最大的给降下去，但是此时信息是丢失的)；作者讲到，这样的问题有可能产生两个结果：
    - a. overfitting
    - b. too confident

 基于上述的事实，作者提出了label-smoothing的方法，该方法的公式如下：
    $$q^{\prime}(k | x)=(1-\epsilon) \delta_{k, y}+\epsilon u(k)$$
 也就是说smoothing之后的label在是原先的label与一个概率分布$u(k)$的加权平均，一般情况下$u(k)=\frac{1}{K}$，经过这样的调整之后，CE的公式变为如下情况：
    $$H\left(q^{\prime}, p\right)=-\sum_{k=1}^{K} \log p(k) q^{\prime}(k)=(1-\epsilon) H(q, p)+\epsilon H(u, p)$$
 可以看到，在训练的时候，即使$q(j)=0$, loss中也不会把这一项忽略掉，而是计算该预测值与$u(k)$的交叉熵并进行调节，而对于真正的标签$q(k)=1$，整体的loss值也没有拉低很多；按照作者的测试，加入该技术之后网络的进行均提升了0.2%(top-1和top-5)。
    
3. 样本不均衡的问题？如何进行解决？

 样本不均衡的问题主要由于anchor过多造成的，当anchor过多的时候，表征背景的anchor就会很多，此时造成两个问题：
    - a. 过多的简单的负样本对网络的学习没有积极的作用；
    - b. 过多的简单负样本会主导网络的学习方向；

 解决方法有：1）使用hard negative mining; 2) 使用bootstrap(这个不太懂，大致就是有放回的抽取)；3）使用Focal loss
 
4. Dropout的作用？原理？训练时候和测试时候的区别？
 - Dropout层是为了防止过拟合的一种方式，本质上是期望训练多个网络之后进行加权的方式获得最终的结果；
 - 原理：Dropout的方式是以一定概率把网络中的某些单元在该次训练中的作用丢弃掉，所以每次训练阶段，使用的网络都是不一样的！！！所以最终的结果就很像说使用了很多个不同的网络对数据进行了训练；
 - 训练和测试的不同：网络在训练的时候，对于没有丢弃的单元，输出要乘以$1/(1-p)$，以此来抵消被丢弃的单元的作用，同时，在对该单元进行参数反向调节的时候，也有$1/(1-p)$这样一个比例因子在，**所以，网络最终的参数可以说基本上都是较大的，大约为标准训练的$1/(1-p)$倍**， 所以在**测试阶段，网络对任何单元都不进行丢弃(可以看做集所有专家之力)，但是单元的输出要乘以$p$来作为最终的输出**。
 - Dropout层和BN层不要一起使用，效果会变差