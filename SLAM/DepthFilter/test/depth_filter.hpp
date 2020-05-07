/*
 * @Author: your name
 * @Date: 2020-04-13 09:15:11
 * @LastEditTime: 2020-05-02 16:20:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /DepthFilter/test/depth_filter.hpp
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sophus/se3.h>
#include <utils.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Sophus;

#define DEPTH_MIN         0.5
#define DEPTH_MEAN        1.0
#define DEPTH_PARAM_DEPTH 2
#define MIN_COV           0.10
#define MAX_COV           10.0

class Seed {
public:
    double mu;
    double sigma2;
    double a;
    double b;

    double zRange;
};


/**
 * @description: depth filter updating
 * @param {type} 
 * @return: true: success; false: failed
 */
bool depthFilterUpdate(const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, Camera& camera, Mat& depth, Mat& depth_cov);


bool depthFilterUpdateNG(const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, Camera& camera, Mat& depth, Mat& depth_cov, Mat& depth_a, Mat& depth_b);
