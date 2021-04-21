#include "iostream"
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;

struct LINE{
  cv::Point end1;
  cv::Point end2;
};


int          point_idx = 0;
cv::Mat      image;
string       image_file = "/home/local/EUROPRO/chenhao.wu/Project/PaperSummary/SLAM/MVG/chapter2/test/2_calibr.png";
vector<cv::Point> line_points;

void metricRecovery(const vector<cv::Point>& orthogonal_line_points) {
  assert(orthogonal_line_points.size() == 8);

  vector<pair<Eigen::Vector3d, Eigen::Vector3d>> orthogonal_lines;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector3d p0(orthogonal_line_points[4*i+0].x, orthogonal_line_points[4*i+0].y, 1.);
    Eigen::Vector3d p1(orthogonal_line_points[4*i+1].x, orthogonal_line_points[4*i+1].y, 1.);
    Eigen::Vector3d p2(orthogonal_line_points[4*i+2].x, orthogonal_line_points[4*i+2].y, 1.);
    Eigen::Vector3d p3(orthogonal_line_points[4*i+3].x, orthogonal_line_points[4*i+3].y, 1.);

    Eigen::Vector3d line1_dir = (p0.cross(p1)).normalized();
    Eigen::Vector3d line2_dir = (p2.cross(p3)).normalized();

    orthogonal_lines.emplace_back(line1_dir, line2_dir);
  }

  assert(orthogonal_lines.size() == 2);
  
  Eigen::Matrix<double, 2, 3> KKT;
  for (size_t i = 0; i < orthogonal_lines.size(); ++i) {
    const auto&  lm = orthogonal_lines[i];
    const double l1 = lm.first.x(), l2 = lm.first.y();
    const double m1 = lm.second.x(), m2 = lm.second.y();
    KKT.row(i) = Eigen::Vector3d(l1*m1, (l1*m2+l2*m1), l2*m2);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 2, 3>> svd(KKT, Eigen::ComputeFullV);
  Eigen::Vector3d X = svd.matrixV().col(2);
  Eigen::Matrix2d H_A;
  H_A << X(0), X(1), X(1), X(2);

  //! chelosky decomposition and s.t. det(K)=1
  Eigen::Matrix2d K = H_A.llt().matrixL();
  K /= sqrt(K.determinant());

  cv::Mat H = (cv::Mat_<double>(3, 3) << K(0,0), K(1,0), 0, K(0,1), K(1,1), 0, 0, 0, 1);
  cv::Mat trans_image;

  //! this H maps original image to image, and we have image.
  //! so we need use inverse of it.
  cv::warpPerspective(image, trans_image, H.inv(), image.size());

  cv::imshow("[recovery]", trans_image);
  cv::waitKey();
}

void affineCalibr(const vector<cv::Point>& horizon_line_points) {
  assert(horizon_line_points.size() == 8);

  vector<Eigen::Vector3d> inf_point_in_inf_line;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector3d p0(horizon_line_points[4*i+0].x, horizon_line_points[4*i+0].y, 1.);
    Eigen::Vector3d p1(horizon_line_points[4*i+1].x, horizon_line_points[4*i+1].y, 1.);
    Eigen::Vector3d p2(horizon_line_points[4*i+2].x, horizon_line_points[4*i+2].y, 1.);
    Eigen::Vector3d p3(horizon_line_points[4*i+3].x, horizon_line_points[4*i+3].y, 1.);

    Eigen::Vector3d line1_dir = (p0.cross(p1)).normalized();
    Eigen::Vector3d line2_dir = (p2.cross(p3)).normalized();

    inf_point_in_inf_line.push_back(line1_dir.cross(line2_dir));
  }

  assert(inf_point_in_inf_line.size() == 2);

  Eigen::Vector3d inf_line_dir = (inf_point_in_inf_line[0].cross(inf_point_in_inf_line[1])).normalized();
  //! must convert l3's sign is positive
  //! or the orientation of your transform is wrong.
  if (inf_line_dir.z() < 0) {
    inf_line_dir *= -1;
  }
  cv::Mat H = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, inf_line_dir[0], inf_line_dir[1], inf_line_dir[2]);
  cout << H << endl;

  cv::Mat trans_image;
  cv::warpPerspective(image, trans_image, H, image.size());

  cv::imshow("[calibr]", trans_image);
  cv::waitKey();
}

void On_mouse(int event, int x, int y, int flags, void* user_data) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		// cv::circle(image_copy, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), 2);
    line_points.push_back(cv::Point(x, y));
    ++point_idx;
	}
	else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
    cv::Mat image_copy = image.clone();
		// cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), 2);
    cv::line(image_copy, line_points.back(), cv::Point(x, y), cv::Scalar(0, 0, 255), 1);
    cv::imshow("[show line]", image_copy);
    // cv::waitKey(1);
	}
  else if (event == cv::EVENT_LBUTTONUP) {
		// cv::circle(image_copy, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), 2);
    cv::destroyWindow("[show line]");
    line_points.push_back(cv::Point(x, y));
    ++point_idx;
    if (point_idx >= 8) {
      cout << "finished" << endl;
      point_idx = 0;
      // affineCalibr(line_points);
      metricRecovery(line_points);
    }

    // cout << "Mouse up : " << x << " " << y << endl;
  }

}

int main(int argc, char** argv) {
  image = cv::imread(image_file.c_str(), 1);
  cv::resize(image, image, cv::Size(640, 480));

  cv::namedWindow("[origin]");
  cv::setMouseCallback("[origin]", On_mouse);

  if (0) {
    cv::Mat H = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, -6.341233382043989e-05, 0.002319544500210153, 0.9999973078424699);
    cv::Mat dst_image;
    
    cv::warpPerspective(image, dst_image, H, cv::Size(640, 480));
    cv::imshow("[test]", dst_image);
    cv::waitKey();

    cv::imwrite("./2_calibr.png", dst_image);
  }

  if (1) {
    cv::Mat H = (cv::Mat_<double>(3, 3) << 1.608139993217691, -0.2165462875459639, 0, 0, 0.62183640990056, 0, 0, 0, 1);
    cv::Mat dst_image;
    
    cv::warpPerspective(image, dst_image, H.inv(), cv::Size(640, 480));
    cv::imshow("[test]", dst_image);
    cv::waitKey();

    cv::imwrite("./2_recovery.png", dst_image);
  }

  while (true) {
    cv::Mat image_show = image.clone();

    for (size_t i = 0; i < line_points.size()/2; ++i) {
      cv::circle(image_show, line_points[2*i+0], 2, cv::Scalar(0, 255, 0), 2);
      cv::circle(image_show, line_points[2*i+1], 2, cv::Scalar(0, 255, 0), 2);
      cv::line(image_show, line_points[2*i+0], line_points[2*i+1], cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("[origin]", image_show);
    cv::waitKey(30);
  }

  return 1;
}
