/*
 * DepthToPly.cpp
 *
 *  Created on: 24 août 2017
 *      Author: abilly
 */


#include "DepthToPly.h"

using namespace cv;
using namespace std;


void savePLY(String fileName, Mat &points, Mat &intensities, int MaxValue)
{
    // Save in .ply with projected intensity

    // Estimate number of points
    int number_of_points = 0;
    for (int i = 0; i < points.rows; i++) {
        for (int j = 0; j < points.cols; j++) {
            if (abs(points.at<Vec3f>(i, j)[2]) < MaxValue) {
                number_of_points++;
            }
        }
    }

    ofstream point_cloud_file;
    point_cloud_file.open(fileName.c_str());
    point_cloud_file << "ply" << endl;
    point_cloud_file << "format ascii 1.0" << endl;
    point_cloud_file << "element vertex " << number_of_points << endl;
    point_cloud_file << "property float x" << endl;
    point_cloud_file << "property float y" << endl;
    point_cloud_file << "property float z" << endl;
    //file.Write("property double confidence\n");
    point_cloud_file << "property uchar red" << endl;
    point_cloud_file << "property uchar green" << endl;
    point_cloud_file << "property uchar blue" << endl;
    point_cloud_file << "end_header" << endl;


    for (int i = 0; i < points.rows; i++) {
        for (int j = 0; j < points.cols; j++) {
            if (abs(points.at<Vec3f>(i, j)[2]) < MaxValue) {
                point_cloud_file << comma_sep << (float)points.at<Vec3f>(i, j)[0]   << " " << comma_sep << (float)points.at<Vec3f>(i, j)[1] << " " << comma_sep << -1.0*(float)points.at<Vec3f>(i, j)[2];
                point_cloud_file << " " << (float)intensities.at<Vec3b>(i, j)[2] << " " << (float)intensities.at<Vec3b>(i, j)[1] << " " << (float)intensities.at<Vec3b>(i, j)[0] << endl;
            }
        }
    }
    point_cloud_file.close();
}

void saveXYZ(const char* filename, const Mat& mat)
{
    // quick writting in .xyz format
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for (int y = 0; y < mat.rows; y++){
        for (int x = 0; x < mat.cols; x++)   {
            Vec3f point = mat.at<Vec3f>(y, x);
                if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
                    fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}
