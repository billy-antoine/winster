
#include "point_cloud_viewer.h"


void matToPointXYZ(cv::Mat &color, cv::Mat &depth,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   const double u0, const double v0, const double f ) {

  const double fx = f;
  const double fy = f;
  int rows = color.rows;
  int cols = color.cols;
  cloud->height = (uint32_t) rows;
  cloud->width = (uint32_t) cols;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  for (unsigned int u = 0; u < rows; ++u) {
    for (unsigned int v = 0; v < cols; ++v) {
      float Xw = 0, Yw = 0, Zw = 0;

      Zw = depth.at<ushort>(u, v);
      Xw = (float) ((v - v0) * Zw / fx);
      Yw = (float) ((u - u0) * Zw / fy);

      cloud->at(v, u).b = color.at<cv::Vec3b>(u, v)[0];
      cloud->at(v, u).g = color.at<cv::Vec3b>(u, v)[1];
      cloud->at(v, u).r = color.at<cv::Vec3b>(u, v)[2];
      cloud->at(v, u).x = Xw;
      cloud->at(v, u).y = Yw;
      cloud->at(v, u).z = Zw;
    }
  }
}
