/*
 * @Author: your name
 * @Date: 2020-04-12 18:13:03
 * @LastEditTime: 2020-05-02 23:55:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /DepthFilter/test/main.cc
 */
#include <utils.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl-1.10/pcl/visualization/pcl_visualizer.h"

#include <utils.hpp>
#include <depth_filter.hpp>

using namespace std;
using namespace cv;
using namespace Sophus;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZRGB         PointT;
typedef pcl::PointCloud<PointT>  PointCloudT;

bool drawPointCloud(const Mat& image, const Mat& depth, const Mat& depth_cov, Camera& camera, double threshold, bool inverse=false) {
    int rows = image.rows;
    int cols = image.cols;

    PointCloudT::Ptr cloud( new PointCloudT );

    for ( int i = 0; i < rows; i++ )
    for ( int j = 0; j < cols; j++ ) {
        double cov = depth_cov.at<double>( i, j );
        if (cov > threshold) {
            continue;
        }

        Vec3b rgb = image.at<Vec3b>( i, j );
        double d  = depth.at<double>( i, j );

        Vector3d f_ref = camera.im2cam(Vector2d( i, j ));
        f_ref.normalize();

        Vector3d Pw = f_ref*d;

        PointT point;
        point.r = rgb[0];
        point.g = rgb[1];
        point.b = rgb[2];

        point.x = Pw.x();
        point.y = Pw.y();
        point.z = Pw.z();

        cloud->points.push_back(point);
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
    cloud->points.clear();

    return true;
}

/**
 * @description: main function
 * @param {type} 
 * @return: 1: success; 0: failed
 */
int main(int argc, char** argv) {
    if ( argc != 2 ) {
        cout<<"Usage: depth_filter path_to_test_dataset"<<endl;
        return -1;
    }
    
    Camera camera(481.2f, -480.0f, 319.5f, 239.5f, 640, 480, 20);

    // 从数据集读取数据
    vector<string> color_image_files; 
    vector<SE3> poses_TWC;
    bool ret = readDatasetFiles( argv[1], color_image_files, poses_TWC );
    if ( ret==false ) {
        cout<<"Reading image files failed!"<<endl;
        return -1; 
    }
    cout<<"read total "<<color_image_files.size()<<" files."<<endl;
    
    // 第一张图
    Mat ref_color = imread( color_image_files[0], -1 );
    Mat ref;

    cvtColor(ref_color, ref, cv::COLOR_BGR2GRAY);
    SE3 pose_ref_TWC = poses_TWC[0];

    #define METHOD 1
    #if (METHOD == 0)
        
        double init_depth   = 3.0;    // 深度初始值
        double init_cov2    = 3.0;    // 方差初始值 

        Mat depth(     480, 640, CV_64F, init_depth);             // 深度图    
        Mat depth_cov( 480, 640, CV_64F, init_cov2);
    
    #elif (METHOD == 1)

        double range        = 1/DEPTH_MIN;
        double init_depth   = 1/DEPTH_MEAN;    // 深度初始值
        double init_cov2    = range*range/36;    // 方差初始值 
        
        Mat depth(     480, 640, CV_64F, init_depth);             // 深度图    
        Mat depth_cov( 480, 640, CV_64F, init_cov2);   
        Mat depth_a(   480, 640, CV_64F, 10);
        Mat depth_b(   480, 640, CV_64F, 10);

    #endif
    
    for ( int index=1; index<color_image_files.size(); index++ )
    {
        cout<<"*** loop "<<index<<" ***"<<endl;

        Mat curr = imread( color_image_files[index], 0 );       
        
        if (curr.data == nullptr) 
            continue;
       
        SE3 pose_curr_TWC = poses_TWC[index];
        SE3 pose_T_C_R = pose_curr_TWC.inverse() * pose_ref_TWC;
   
        #if (METHOD == 0)

            depthFilterUpdate( ref, curr, pose_T_C_R, camera, depth, depth_cov);
            imshow("depth", 0.4*depth);

        #elif (METHOD == 1)

            depthFilterUpdateNG( ref, curr, pose_T_C_R, camera, depth, depth_cov, depth_a, depth_b);
            imshow("depth", depth);

        #endif

        imshow("image", curr);
        waitKey(1);
    }

    cout<<"estimation returns, saving depth map ..."<<endl;

    drawPointCloud(ref_color, depth, depth_cov, camera, (1/DEPTH_MIN)/200, true);
    
    
    return 0; 
}
