#include "pangolin_eigen_cv_head.h"

void load_images(string root, cv::Mat *left_image, cv::Mat *right_image) {
  string left_image_path = absl::StrFormat("%s/%s", root, "left.png");
  string right_image_path = absl::StrFormat("%s/%s", root, "right.png");
  cv::imread(left_image_path, cv::IMREAD_GRAYSCALE).copyTo(*left_image);
  cv::imread(right_image_path, cv::IMREAD_GRAYSCALE).copyTo(*right_image);
  if (left_image->empty() || right_image->empty()) {
    LOG(ERROR) << "failed to load image!";
    return;
  }
  LOG(INFO) << __FUNCTION__ << "success loaded images!"
            << "left image size: "
            << absl::StrFormat("%d, %d, %d", left_image->cols, left_image->rows,
                               left_image->channels());
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
        LOG(ERROR) << "Usage: " << argv[0] << " <image_root_directory>";
        return -1;
  }
  cv::Mat left, right, combine_view;
  load_images(argv[1], &left, &right);

  // // visualize
  // cv::vconcat(left, right, combine_view);
  // cv::resize(combine_view, combine_view, cv::Point(640, 194 * 2));
  // cv::imshow("img", combine_view);
  // cv::waitKey(0);

  // 内参
  double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  // 基线
  double b = 0.573;

  cv::Ptr<cv::StereoSGBM> sgbm =
      cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100,
                             32); // 神奇的参数

  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
  vector<Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;

  LOG(INFO) << __FUNCTION__
             << " ,start caculate pointcloud.";

  for (int v = 0; v < left.rows; v++) {
    for (int u = 0; u < left.cols; u++) {
      if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
        continue;
      // point[3] is gray value of image
      Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); 

      // 计算 point 的位置
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      double depth = fx * b / (disparity.at<float>(v, u));

      point[0] = x * depth;
      point[1] = y * depth;
      point[2] = depth;
      LOG_EVERY_N(INFO, 100) << "point: " << absl::StrFormat("(%f, %f, %f, %f)", point[0], point[1], point[2], point[3]);

      pointcloud.push_back(point);
    }
  }
  // cv::imshow("disparity", disparity / 96.0);
  // cv::waitKey(0);
  showPointCloud(pointcloud);
  return 0;
}
