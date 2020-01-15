# DSO-tracking



## 写在前面

上一篇记录了DSO的初始化阶段，本文主要记录下对于新帧的追踪，即tracking，整个部分其实不是很难，主要的思路也简单，就是由粗到细的进行光度误差的最小化，以此来计算位姿。



## 特点

虽然都是最小化光度误差，这里先列出DSO做的很不同的地方：

1. 放弃使用了像素块的方法，这个个人深有感触，一般而言虽然像素块的大小不大，但是耐不住点多啊，多数情况下如果用像素块的话这个地方就变成了十分花费时间；
2. 当某一层的初始位姿不好的时候，DSO并不放弃"治疗"，反而会给足了机会去优化，但是如果优化出来的结果与阈值差距太大，那么就直接放弃了；
3. 作者假设了5种运动模型：匀速、倍速、半速、零速以及没有运动；
4. 与此同时，作者假定了有26×3种旋转情况（四元数的26种旋转情况，3个比较小的角度），作者认为是在丢失的情况下，这样的方法会十分有用，如果没有丢失应该前五种假设就够了；



## 深入探讨

整个跟踪函数仅有一个，即代码中的trackNewCoarse函数，该函数主要做了两件事：

1. 准备相应的运动初值，详细来说就是五种运动假设和N种旋转假设；
2. 对每一个初值进行L-M迭代，求得最佳的位姿；

下面分两部分来说明：



### 准备运动初值

这个部分用文字说明比较苍白，这里配上一张图会比较清晰：

![](/home/ubuntu/Projects/PaperSummary/SLAM/DSO/pictures/DSO4.png)

如图所示，图中的变量使用的都是程序中的变量名称，最终需要的运动初值为红线所示的LastF_2_fh，然后作者使用的参考运动值为sprelast_2_slast，假设为slast_2_fh，因此五种模型如下：

1. 匀速模型：$T_{lastF}^{fh} = T_{slast}^{fh}*T_{lastF}^{slast}$；
2. 倍速模型：$T_{lastF}^{fh} = T_{slast}^{fh}*T_{lastF}^{fh}$，相当于说匀速的从slast帧运动到当前帧，之后以当前帧为起点再匀速运动一次；
3. 半速模型：$T_{lastF}^{fh} = 0.5×T_{slast}^{fh}*T_{lastF}^{slast}$；
4. 零速模型：$T_{lastF}^{fh} = T_{lastF}^{slast}$；
5. 不动模型：$T_{lastF}^{fh} = T_I$；

随后就是26×3种旋转模型，这里不在赘述，代码如下：

```c++
shared_ptr<FrameHessian> lastF = coarseTracker->lastRef; // last key frame
shared_ptr<Frame> slast = allFrameHistory[allFrameHistory.size() - 2];
shared_ptr<Frame> sprelast = allFrameHistory[allFrameHistory.size() - 3];

SE3 slast_2_sprelast;
SE3 lastF_2_slast;
{    // lock on global pose consistency!
    unique_lock<mutex> crlock(shellPoseMutex);
    slast_2_sprelast = sprelast->getPose() * slast->getPose().inverse();
    lastF_2_slast = slast->getPose() * lastF->frame->getPose().inverse();
    aff_last_2_l = slast->aff_g2l;
}
SE3 fh_2_slast = slast_2_sprelast;// assumed to be the same as fh_2_slast.

// get last delta-movement.
lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast);    // assume constant motion.
lastF_2_fh_tries.push_back(fh_2_slast.inverse() * fh_2_slast.inverse() * lastF_2_slast);    // assume double motion (frame skipped)
lastF_2_fh_tries.push_back(SE3::exp(fh_2_slast.log() * 0.5).inverse() * lastF_2_slast); // assume half motion.
lastF_2_fh_tries.push_back(lastF_2_slast); // assume zero motion.
lastF_2_fh_tries.push_back(SE3()); // assume zero motion FROM KF.


// just try a TON of different initializations (all rotations). In the end,
// if they don't work they will only be tried on the coarsest level, which is super fast anyway.
// also, if tracking rails here we loose, so we really, really want to avoid that.
for (float rotDelta = 0.02;rotDelta < 0.05; rotDelta += 0.01) {    // TODO changed this into +=0.01 where DSO writes ++
    lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast *
                               SE3(Sophus::Quaterniond(1, rotDelta, 0, 0),
                                   Vec3(0, 0, 0)));            // assume constant motion.
    ...
    lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast *
                                   SE3(Sophus::Quaterniond(1, rotDelta, rotDelta, rotDelta),
                                       Vec3(0, 0, 0)));    // assume constant motion.
}
```

这里说明一点的是，虽然注释上写说旋转模型仅仅在金字塔最顶层进行优化，但是在代码上个人并没有发现。



### 位姿优化

位姿优化从金字塔的最顶层由粗到细的进行优化，大致步骤如下：

1. 从上到下遍历每一层；

2. 对于每一层使用L-M优化的方式，不过对于初始化的残差和增量方程，作者采取了及其容忍的态度，**这大概率是因为在计算光度误差的时候，作者仅仅使用一个点而不是像素块的方式**，具体方法是每次都会判断超出阈值的点所占的比例，如果比例过大（60%），则增加阈值（2倍）并重新来过，直到满足条件或者阈值增大到一定程度，代码如下：

   ```c++
   Vec6 resOld = calcRes(lvl, refToNew_current, aff_g2l_current, setting_coarseCutoffTH * levelCutoffRepeat);
   // 如果误差大的点的比例大于60%， 那么增大阈值再计算N次
   while (resOld[5] > 0.6 && levelCutoffRepeat < 50) {
       // more than 60% is over than threshold, then increate the cut off threshold
       levelCutoffRepeat *= 2;
       resOld = calcRes(lvl, refToNew_current, aff_g2l_current, setting_coarseCutoffTH * levelCutoffRepeat);
   }
   
   // Compute H and b
   calcGSSSE(lvl, H, b, refToNew_current, aff_g2l_current);
   ```

3. 进行L-M算法，