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