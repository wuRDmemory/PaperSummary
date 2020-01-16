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
   // 内部也是用SSE实现的，公式可以参考上一篇文章
   calcGSSSE(lvl, H, b, refToNew_current, aff_g2l_current);
   ```

3. 进行L-M算法，这里都比较正常，代码如下：代码中不太明白的是作者在求解出增量了之后，为什么又与权重做了积？

   ```C++
   for (int iteration = 0; iteration < maxIterations[lvl]; iteration++) {
       Mat88 Hl = H;
       for (int i = 0; i < 8; i++) Hl(i, i) *= (1 + lambda);
       Vec8 inc = Hl.ldlt().solve(-b);
   
       // depends on the mode, if a,b is fixed, don't estimate them
       if (setting_affineOptModeA < 0 && setting_affineOptModeB < 0)    // fix a, b
       {
           inc.head<6>() = Hl.topLeftCorner<6, 6>().ldlt().solve(-b.head<6>());
           inc.tail<2>().setZero();
       }
       if (!(setting_affineOptModeA < 0) && setting_affineOptModeB < 0)    // fix b
       {
           inc.head<7>() = Hl.topLeftCorner<7, 7>().ldlt().solve(-b.head<7>());
           inc.tail<1>().setZero();
       }
       if (setting_affineOptModeA < 0 && !(setting_affineOptModeB < 0))    // fix a
       {
           Mat88 HlStitch = Hl;
           Vec8 bStitch = b;
           HlStitch.col(6) = HlStitch.col(7);
           HlStitch.row(6) = HlStitch.row(7);
           bStitch[6] = bStitch[7];
           Vec7 incStitch = HlStitch.topLeftCorner<7, 7>().ldlt().solve(-bStitch.head<7>());
           inc.setZero();
           inc.head<6>() = incStitch.head<6>();
           inc[6] = 0;
           inc[7] = incStitch[6];
       }
   
       float extrapFac = 1;
       if (lambda < lambdaExtrapolationLimit)
           extrapFac = sqrtf(sqrt(lambdaExtrapolationLimit / lambda));
       inc *= extrapFac;
   
       // 这里为什么要再乘一个scale
       Vec8 incScaled = inc;
       incScaled.segment<3>(0) *= SCALE_XI_ROT;
       incScaled.segment<3>(3) *= SCALE_XI_TRANS;
       incScaled.segment<1>(6) *= SCALE_A;
       incScaled.segment<1>(7) *= SCALE_B;
   
       if (!std::isfinite(incScaled.sum())) incScaled.setZero();
   
       // left multiply the pose and add to a,b
       SE3 refToNew_new = SE3::exp((Vec6) (incScaled.head<6>())) * refToNew_current;
       AffLight aff_g2l_new = aff_g2l_current;
       aff_g2l_new.a += incScaled[6];
       aff_g2l_new.b += incScaled[7];
   
       // calculate new residual after this update step
       Vec6 resNew = calcRes(lvl, refToNew_new, aff_g2l_new, setting_coarseCutoffTH * levelCutoffRepeat);
   
       // decide whether to accept this step
       // res[0]/res[1] is the average energy
       bool accept = (resNew[0] / resNew[1]) < (resOld[0] / resOld[1]);
   
       if (accept) {
   
           // decrease lambda
           calcGSSSE(lvl, H, b, refToNew_new, aff_g2l_new);
           resOld = resNew;
           aff_g2l_current = aff_g2l_new;
           refToNew_current = refToNew_new;
           lambda *= 0.5;
       } else {
           // increase lambda in LM
           lambda *= 4;
           if (lambda < lambdaExtrapolationLimit) lambda = lambdaExtrapolationLimit;
       }
   
       // terminate if increment is small
       if (!(inc.norm() > 1e-3)) {
           break;
       }
   } // end of L-M iteration
   ```

4. 看一下该层最终的标准差，如果标准差大于阈值的1.5倍时，认为该次优化失败了，直接退出，这里阈值是动态调节的，作者认为，如果上一层的误差为x，那么该层的误差决不能超过上层的一个倍数（作者使用1.5倍），毕竟粗优化提供了初值，细优化应该能达到更好的状态才对；