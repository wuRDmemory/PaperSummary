/*
 * @Author: your name
 * @Date: 2020-04-13 09:21:34
 * @LastEditTime: 2020-05-02 16:22:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /DepthFilter/test/depth_filter.cc
 */
#include <depth_filter.hpp>
#include <utils.hpp>
#include "boost/math/distributions.hpp"

bool guassianUpdate( const Vector3d& f_ref, const Vector3d& f_cur, const SE3& T_cur_ref, double& mu, double& sigma2, double z, double fx) {    

    SE3 T_R_C = T_cur_ref.inverse();
    Vector3d t = T_R_C.translation();

    
    // 计算不确定性（以一个像素为误差）
    double d_cov  = computeTau(f_ref, t, z, atan(1/fx)); 
    double d_cov2 = d_cov*d_cov;
    
    // 高斯融合
    float new_mu     = (d_cov2*mu + sigma2*z) / ( sigma2 + d_cov2);
    float new_sigma2 = ( sigma2*d_cov2 ) / ( sigma2 + d_cov2 );
    
    mu     = new_mu;
    sigma2 = new_sigma2;

    return true;
}

bool depthFilterUpdate( const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, Camera& camera, Mat& depth, Mat& depth_cov) {
    
    const int board  = camera.board_;
    const int width  = camera.width_;
    const int height = camera.height_;

#pragma omp parallel for
    for ( int x = board; x < width-board;  x++ )
#pragma omp parallel for
    for ( int y = board; y < height-board; y++ )
    {
        double mu     = depth.at<double>(y, x);
        double sigma2 = depth_cov.at<double>(y, x);
        // 遍历每个像素
        if (   sigma2 < MIN_COV 
            || sigma2 > MAX_COV ) // 深度已收敛或发散
            continue;

        // 在极线上搜索 (x,y) 的匹配 
        Vector2d pc_cur;

        double d_min = max(0.1, mu - 3*sqrt(sigma2));
        double d_max = mu + 3*sqrt(sigma2);
        bool ret = epipolarSearch (
            ref_image, cur_image, T_cur_ref, 
            Vector2d(x, y), camera, d_min, d_max, mu, pc_cur);
        
        if ( ret == false ) // 匹配失败
            continue; 

        // calculate point in normal plane
        Vector3d f_ref = camera.im2cam(Vector2d(x, y)).normalized();
        Vector3d f_cur = camera.im2cam(pc_cur).normalized();
        double       z = getDepth(T_cur_ref, f_ref, f_cur);

        if ( z < 0 ) continue;
        
        // 匹配成功，更新深度图 
        guassianUpdate( f_ref, f_cur, T_cur_ref, mu, sigma2, z, camera.fx_);

        depth.at<double>(y, x)     = mu;
        depth_cov.at<double>(y, x) = sigma2;
    }
    return true;
}

bool updateSeed(const double x, const double tau2, Seed* seed) {
    double norm_scale = sqrt(seed->sigma2 + tau2);
    if(std::isnan(norm_scale))
        return false;
    boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
    float s2 = 1./(1./seed->sigma2 + 1./tau2);
    float m = s2*(seed->mu/seed->sigma2 + x/tau2);
    float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
    float C2 = seed->b/(seed->a+seed->b) * 1./seed->zRange;
    float normalization_constant = C1 + C2;
    C1 /= normalization_constant;
    C2 /= normalization_constant;
    float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
    float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
            + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

    // update parameters
    float mu_new = C1*m+C2*seed->mu;
    seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
    seed->mu = mu_new;
    seed->a = (e-f)/(f-e/f);
    seed->b = seed->a*(1.0f-f)/f; 

    return true;
}

bool guassianUpdateNG( const Vector3d& Pc_ref, const Vector3d& Pc_cur, const SE3& T_cur_ref, double& mu, double& sigma2, double& a, double& b, double z, double fx) {    

    SE3 T_R_C = T_cur_ref.inverse();
    Vector3d t = T_R_C.translation();

    
    // 计算不确定性（以一个像素为误差）
    double noise_angle = atan(1/fx);
    double tau         = computeTau(Pc_ref, t, z, noise_angle);
    double tauInv      = 0.5*(1/max(z-tau, 0.0001) - 1/(z+tau));
    
    Seed seed;
    seed.a      = a;
    seed.b      = b;
    seed.mu     = mu;
    seed.sigma2 = sigma2;
    seed.zRange = 1/DEPTH_MIN;

    // 高斯融合
    updateSeed(1/z, tauInv*tauInv, &seed);

    mu      = seed.mu;
    sigma2  = seed.sigma2;
    a       = seed.a;
    b       = seed.b;

    return true;
}


bool depthFilterUpdateNG(const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, Camera& camera, Mat& depth, Mat& depth_cov, Mat& depth_a, Mat& depth_b) {
    
    const int board  = camera.board_;
    const int width  = camera.width_;
    const int height = camera.height_;

    int unvisual = 0;
    int converge = 1;

    // Mat show_cvg = ref_image.clone();
    // cvtColor(show_cvg, show_cvg, COLOR_GRAY2BGR);

#pragma omp parallel for
    for ( int x = board; x < width-board;  x++ )
#pragma omp parallel for
    for ( int y = board; y < height-board; y++ )
    {
        double mu     = depth.at<double>(y, x);
        double sigma2 = depth_cov.at<double>(y, x);
        double a      = depth_a.at<double>(y, x);
        double b      = depth_b.at<double>(y, x);

        double range  = 1/DEPTH_MIN;

        // 遍历每个像素
        if ( sqrt(sigma2) < range/200 ) { // 深度已收敛
            // cv::circle(show_cvg, Point(x, y), 1, cv::Scalar(0, 255, 0), 1);
            converge++;
            continue;
        }

        // 在极线上搜索 (x,y) 的匹配
        double d_min = 1/(mu + sqrt(sigma2));
        double d_max = 1/max(mu - sqrt(sigma2), 0.01);        
        Vector2d pc_cur;
        
        bool ret = epipolarSearch (
            ref_image, cur_image, T_cur_ref, 
            Vector2d(x, y), camera, d_min, d_max, 1/mu, pc_cur);
        
        if ( ret == false ) { // 匹配失败
            unvisual ++;
            continue; 
        }

        // calculate point in normal plane
        Vector3d f_ref = camera.im2cam(Vector2d(x, y)).normalized();
        Vector3d f_cur = camera.im2cam(pc_cur).normalized();
        double z       = getDepth(T_cur_ref, f_ref, f_cur);

        if ( z < 0 ) continue;
        
        // 匹配成功，更新深度图 
        guassianUpdateNG( f_ref, f_cur, T_cur_ref, mu, sigma2, a, b, z, camera.fx_);

        depth.at<double>    (y, x) = mu;
        depth_cov.at<double>(y, x) = sigma2;
        depth_a.at<double>  (y, x) = a;
        depth_b.at<double>  (y, x) = b;
        
    }

    float all_cnt = width * height;
    char log[200];
    sprintf(log, "== converge: %d/%d=%.2f  unvisual: %d/%d=%.2f", 
        converge, int(all_cnt), converge/all_cnt, 
        unvisual, int(all_cnt), unvisual/all_cnt);
    cout << log << endl;

    // cv::imshow("converge", show_cvg);
    return true;
}

