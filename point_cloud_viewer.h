#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H


#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

void matToPointXYZ(cv::Mat &color, cv::Mat &depth,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   const double u0, const double v0, const double f );


#endif // POINT_CLOUD_VIEWER_H

