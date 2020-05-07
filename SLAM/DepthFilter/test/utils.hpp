
#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Sophus;

class Config {
public:
    float depth_min_;
    float depth_mean_;

    float depth_cov_min_;
    float depth_cov_max_;

public:
    Config(float depth_min, float depth_mean, float depth_cov_min, float depth_cov_max) {
        this->depth_cov_min_ = depth_cov_min;
        this->depth_cov_max_ = depth_cov_max;
        this->depth_min_     = depth_min;
        this->depth_mean_    = depth_mean;
    }
};

/**
 * @description: camera parameters
 */
class Camera {
public:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    int   width_;
    int   height_;
    int   board_;

public:
    Camera(float fx, float fy, float cx, float cy, int width, int height, int board):
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), 
        width_(width), height_(height), board_(board)
    {
        ;
    }

    /**
     * @description: camera coordinate to image coordinate
     * @param {type} 
     * @return: uv in image coordinate
     */ 
    Vector2d cam2im(Vector3d Pc) {
        double u = fx_*Pc.x()/Pc.z() + cx_;
        double v = fy_*Pc.y()/Pc.z() + cy_;
        return Vector2d(u, v);
    }

    /**
     * @description: image coordinate to camera coordinate
     * @param {type} 
     * @return: Pc in camera normal plane
     */
    Vector3d im2cam(Vector2d uv) {
        double x = (uv.x() - cx_)/fx_;
        double y = (uv.y() - cy_)/fy_;
        return Vector3d(x, y, 1);
    }

    /**
     * @description: world coordinate to camera coordinate
     * @param {type} 
     * @return: Pc in camera normal plane
     */
    Vector3d world2cam(Vector3d Pw) {
        double x = Pw.x()/Pw.z();
        double y = Pw.y()/Pw.z();
        return Vector3d(x, y, 1);
    }

    /**
     * @description: world coordinate to image coordinate directly
     * @param {type} 
     * @return: uv in image coordinate
     */
    Vector2d world2im(Vector3d Pw) {
        Vector3d Pc = world2cam(Pw);
        return cam2im(Pc);
    }

    /**
     * @description: check the px in the image 
     * @param {type} 
     * @return: true is in, false is out 
     */
    bool isVisual(Vector2d px) {
        bool c1 = board_ < px.x() && px.x() < width_  - board_;
        bool c2 = board_ < px.y() && px.y() < height_ - board_;

        return c1 && c2;
    }
};

/**
 * @description: get interpolated pixel value
 * @param {type} 
 * @return: pixel value
 */
float getPixelInterpolate(const Mat &image, Point2f px);


/**
 * @description: triangle to get landmark's depth in host frame
 * @param {type} 
 * @return: depth
 */
double getDepthNew(const SE3 &T_cur_ref, Vector3d f_ref, Vector3d f_cur);
double getDepth(const SE3 &T_cur_ref, Vector3d f_ref, Vector3d f_cur);

/**
 * @description: calculate NCC score within a image patch
 * @param {type} 
 * @return: 
 */
double getNCCScore(const Mat& ref_image, const Mat& cur_image, Vector2d ref_px, Vector2d cur_px, const int board);

/**
 * @description: search match in epipolar line 
 * @param {type} 
 * @return: true is found, false is not found
 */
bool epipolarSearch(const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, const Vector2d& pc_ref, 
    Camera& camera, const double d_min, const double d_max, const double d_mean, Vector2d& pc_cur);

/**
 * @description: 
 * @param {type} 
 * @return: 
 */
double computeTau(const Vector3d& f_ref, const Vector3d& t, const double z, const double noise_angle);


Mat convertInvDepthMap(Mat& inv_depth);


bool readDatasetFiles(const string& path, vector< string >& color_image_files, std::vector<SE3>& poses);
