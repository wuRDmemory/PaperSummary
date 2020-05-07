/*
 * @Author: your name
 * @Date: 2020-04-12 17:39:39
 * @LastEditTime: 2020-05-02 22:34:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /DepthFilter/test/utils.cc
 */
#include "utils.hpp"


float getPixelInterpolate(const Mat &image, Vector2d px) {
    int   stride = image.cols;
    int   i_u    = int( floorf(px.x()) );
    int   i_v    = int( floorf(px.y()) );
    float d_u    = px.x() - i_u;
    float d_v    = px.y() - i_v;

    float w_tl = (1 - d_u)*(1 - d_v);
    float w_tr =      d_u *(1 - d_v);
    float w_bl = (1 - d_u)*     d_v ;
    float w_br =      d_u *     d_v ;

    uchar* ptr = (uchar*)image.data + i_v*stride + i_u; 
    return w_tl*ptr[0] + w_tr*ptr[1] + w_bl*ptr[stride] + w_br*ptr[stride+1];
}

double getDepthNew(const SE3 &T_C_R, Vector3d f_ref, Vector3d f_cur) {
    SE3 T_R_C  = T_C_R.inverse();
    
    Vector3d t  = T_R_C.translation();
    Vector3d f2 = T_R_C.rotation_matrix() * f_cur;
    Vector2d b  = Vector2d ( t.dot ( f_ref ), t.dot ( f2 ) );
    double A[4];
    A[0] = f_ref.dot ( f_ref );
    A[2] = f_ref.dot ( f2 );
    A[1] = -A[2];
    A[3] = - f2.dot ( f2 );
    double d = A[0]*A[3]-A[1]*A[2];
    Vector2d lambdavec =
        Vector2d (  A[3] * b ( 0,0 ) - A[1] * b ( 1,0 ),
                -A[2] * b ( 0,0 ) + A[0] * b ( 1,0 )) /d;
    Vector3d xm = lambdavec ( 0,0 ) * f_ref;
    Vector3d xn = t + lambdavec ( 1,0 ) * f2;
    Vector3d d_esti = ( xm+xn ) / 2.0;  // 三角化算得的深度向量
    return d_esti.norm();   // 深度值
}

double getDepth(const SE3 &T_cur_ref, Vector3d f_ref, Vector3d f_cur) {
    Matrix<double, 3, 2> A;
    A.col(0) =  T_cur_ref.rotation_matrix()*f_ref;
    A.col(1) = -f_cur;

    Matrix<double, 3, 1> b = -T_cur_ref.translation();

    Matrix<double, 2, 2> ATA = A.transpose()*A;
    Matrix<double, 2, 1> ATb = A.transpose()*b;

    // ensure we can get inverse
    if (ATA.determinant() < 1e-6) {
        return -1;
    }

    Matrix<double, 2, 1> x = ATA.inverse()*ATb;
    if (isnan(x(0)) || isinf(x(0))) {
        return -1;
    }
    
    Vector3d Pw_ref = f_ref*x(0);
    Vector3d Pw_cur = f_cur*x(1);
    Vector3d Pp_ref = T_cur_ref.inverse() * Pw_cur;
    
    Vector3d Pw = (Pw_ref + Pp_ref)/2;
    return Pw.norm();
    // return x(0);
}


double getNCCScore(const Mat& ref_image, const Mat& cur_image, Vector2d ref_px, 
                   Vector2d cur_px, const int board) {
    double mean_ref = 0, mean_cur = 0;
    vector<double> values_ref, values_cur;
    for ( int x = -board; x <= board; x++ )
    for ( int y = -board; y <= board; y++ )
    {
        // double value_ref = getPixelInterpolate( ref_image, ref_px+Vector2d(x, y) )/255.0;
        double value_ref = ref_image.at<uchar>(int(ref_px(1)+y), int(ref_px(0)+x))/255.0;
        mean_ref += value_ref;
        
        double value_cur = getPixelInterpolate( cur_image, cur_px+Vector2d(x, y) )/255.0;
        mean_cur += value_cur;
        
        values_ref.push_back(value_ref);
        values_cur.push_back(value_cur);
    }
    
    float ncc_area = (board*2 + 1)*(board*2 + 1);
    mean_ref /= ncc_area;
    mean_cur /= ncc_area;
    
	// 计算 Zero mean NCC
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for ( int i=0; i<values_ref.size(); i++ )
    {
        double n = (values_ref[i]-mean_ref) * (values_cur[i]-mean_cur);
        numerator += n;
        demoniator1 += (values_ref[i]-mean_ref)*(values_ref[i]-mean_ref);
        demoniator2 += (values_cur[i]-mean_cur)*(values_cur[i]-mean_cur);
    }
    return numerator / sqrt( demoniator1*demoniator2+1e-10 );   // 防止分母出现零
}


bool  epipolarSearch(const Mat& ref_image, const Mat& cur_image, const SE3& T_cur_ref, const Vector2d& pc_ref, Camera& camera,
    const double d_min, const double d_max, const double d_mean, Vector2d& pc_cur) 
{
    assert( d_min < d_max );
    Vector3d f_ref = camera.im2cam( pc_ref );
    f_ref.normalize();
    
    Vector2d px_mean_curr = camera.cam2im( T_cur_ref*(f_ref*d_mean) );
    Vector2d px_min_curr  = camera.cam2im( T_cur_ref*(f_ref*d_min ) );	// 按最小深度投影的像素
    Vector2d px_max_curr  = camera.cam2im( T_cur_ref*(f_ref*d_max ) );	// 按最大深度投影的像素
    
    Vector2d epipolar_line = px_max_curr - px_min_curr;	            // 极线（线段形式）
    Vector2d epipolar_direction = epipolar_line;		// 极线方向 
    epipolar_direction.normalize();

    double step   = sqrt(2)/2;
    double length = 0.5*epipolar_line.norm();	// 极线线段的半长度
    int    nstep  = ceil(length/step);

    if ( nstep>100 ) nstep = 100;   // 我们不希望搜索太多东西 
    
    // 在极线上搜索，以深度均值点为中心，左右各取半长度
    double best_ncc = -1.0;
    Vector2d px_cur(-1, -1);
    Vector2d best_px_cur(-1, -1); 
    Vector2i last_px(-1, -1);
    
    for ( int l = -nstep; l <= nstep; l+=1  )  // l+=sqrt(2) 
    {
        px_cur = px_mean_curr + l*step*epipolar_direction;  // 待匹配点
        
        if ( !camera.isVisual(px_cur) )
            continue; 
        
        int pt_x = (int)floor(px_cur.x());
        int pt_y = (int)floor(px_cur.y());
        if ((pt_x == last_px.x()) && (pt_y == last_px.y())) {
            continue;
        }

        last_px.x() = pt_x;
        last_px.y() = pt_y;
        
        // 计算待匹配点与参考帧的 NCC
        double ncc = getNCCScore(ref_image, cur_image, pc_ref, px_cur, 2);
        if ( ncc>best_ncc ) {
            best_ncc = ncc; 
            best_px_cur = px_cur;
        }
    }
    
    if ( best_ncc < 0.85f )      // 只相信 NCC 很高的匹配
        return false; 
    
    pc_cur = best_px_cur;
    return true;
}



double computeTau(const Vector3d& f_ref, const Vector3d& t, const double z, const double noise_angle) {
    Vector3d p = f_ref*z;
    Vector3d a = p - t; 
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos( f_ref.dot(t)/t_norm );
    double beta = acos( -a.dot(t)/(a_norm*t_norm));
    double beta_prime = beta + noise_angle;
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    
    return p_prime - z;
}

Mat convertInvDepthMap(Mat& inv_depth) {
    Mat depth = Mat::zeros(inv_depth.rows, inv_depth.cols, inv_depth.type());

    for (int row = 0; row < depth.rows; row++)
    for (int col = 0; col < depth.cols; col++) {
        double inv_d = inv_depth.at<double>(row, col);
        if (inv_d < 0.1f || inv_d > 100) {
            depth.at<double>(row, col) = 10;
        } else {
            depth.at<double>(row, col) = 1/inv_d;
        }
    }

    return depth;
}

bool readDatasetFiles(const string& path, vector< string >& color_image_files, std::vector<SE3>& poses) {
    ifstream fin( path+"/first_200_frames_traj_over_table_input_sequence.txt");
    if ( !fin ) return false;
    
    while ( !fin.eof() ) {
		// 数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ，注意是 TWC 而非 TCW
        string image; 
        fin >> image; 
        double data[7];
        for ( double& d:data ) fin>>d;
        
        color_image_files.push_back( path+string("/images/")+image );
        poses.push_back(
            SE3( Quaterniond(data[6], data[3], data[4], data[5]), 
                 Vector3d(data[0], data[1], data[2]))
        );
        if ( !fin.good() ) break;
    }
    return true;    
}
