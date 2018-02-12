#include <iostream>
#include <time.h>
#include "struct_from_motion.h"
#include "stereo_calib.h"
#include "disparity_map.h"
#include "DepthToPly.h"
#include "opencv2/viz.hpp"
#include "parameters.h"
#include "opencv2/surface_matching.hpp"
#include "opencv2/imgproc.hpp"
#include "point_cloud_viewer.h"
#include <fstream>

#include <pcl/visualization/cloud_viewer.h>


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace cv;

int main(){

    // Input images
    const string num_image = "000000";
    const string jpg_filenameL  = inputDir + "left/"+ num_image +"_10.png";
    const string jpg_filenameL2 = inputDir + "left/"+ num_image +"_11.png";
    const string jpg_filenameR  = inputDir + "right/"+ num_image +"_10.png";
    const string jpg_filenameR2 = inputDir + "right/"+ num_image +"_11.png";
    cv::Mat left01  = imread(jpg_filenameL);
    cv::Mat left02  = imread(jpg_filenameL2);
    cv::Mat right01 = imread(jpg_filenameR);
    cv::Mat right02 = imread(jpg_filenameR2);

    // Calibration


    if(calibration){
        std::vector<std::string> imagelist;
        getdir(inputDir + "calib_old/", imagelist);

        StereoCalib(imagelist, inputDir+"matrices/",boardSize, squareSize, displayCorner, useCalibrated, showRectified);
    }
    cv::Mat M1, D1, M2, D2, R1, R2, P1, P2, R, T, Q;

    // Load Calibration files
    /*loadIntrinsics(intrinsicsPath, M1, M2, D1, D2);
    loadExtrinsics(extrinsicsPath, R1, R2, P1, P2, R, T, Q);


    // Prepare matrices for Remapping
    cv::Mat rmap[2][2], rleft, rright;
    initUndistortRectifyMap(M1, D1, R1, P1, left01.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, left01.size(), CV_16SC2, rmap[1][0], rmap[1][1]);

    // Remapping
    remap(left01, rleft, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    remap(right01, rright, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);*/

    cv::Mat dispMap01 = compute_disparity_map(left01, right01);
    cv::Mat dispMap02 = compute_disparity_map(left02, right02);

    cv::Mat dispN, dispN2;

    cv::normalize(dispMap01, dispN,  0, 255, CV_MINMAX, CV_8U);
    cv::normalize(dispMap02, dispN2, 0, 255, CV_MINMAX, CV_8U);

    cv::applyColorMap( dispN, dispN, COLORMAP_PARULA );
    cv::applyColorMap( dispN2, dispN2, COLORMAP_PARULA );

    cv::Mat canvas, canvas2;
    cv::vconcat(left01, dispN, canvas);
    cv::vconcat(left02, dispN2, canvas2);

    cv::imshow("DispMap", canvas);
    cv::imshow("DispMap2", canvas2);
    cv::waitKey(50);

    std::cout << "reprojecting point cloud ..." << std::endl;

    double cx=6.071928e+02;
    double cy=1.852157e+02;
    double focal=7.188560e+02;
    double baseline_length=540.0;

    Q = (Mat_<double>(4,4) <<   1., 0., 0., -cx,
                                0., 1., 0., -cy,
                                0., 0., 0., focal,
                                0., 0., -1., 0);

    cv::Mat points01, points02;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    matToPointXYZ( left01, dispMap01, cloud, cx, cy, focal);

    pcl::visualization::CloudViewer viewer;
    viewer.showCloud(cloud);

    //cv::reprojectImageTo3D(dispMap01, points01, Q, true, -1);
    //cv::reprojectImageTo3D(dispMap02, points02, Q, true, -1);


    /*// VIZ 3D
    points01 *= 100;
    cv::FileStorage file(inputDir+"lolilol.txt", cv::FileStorage::WRITE);
    file << "mat1" << dispMap01;

    viz::Viz3d myWindow("Viz Demo");

    cv::viz::WCloud cloud_widget = cv::viz::WCloud(points01);
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 20 );

     myWindow.setBackgroundMeshLab();
     myWindow.showWidget( "coosys", viz::WCoordinateSystem() );
     myWindow.showWidget( "text2d", viz::WText( "Stereo projection", Point(20, 20), 20, viz::Color::green() ) );
     myWindow.showWidget("CloudWidget1", cloud_widget);

     myWindow.spin();*/





    std::cout << "saving to ply format " << std::endl;

    savePLY(pointCloudPath+"01.ply", points01, left01);
    savePLY(pointCloudPath+"02.ply", points02, left02);


    std::cout << "ply saved" << std::endl;
    std::cout << "SfM computation ..." << std::endl;


    openMVG::image::Image<unsigned char> imageL, imageR;

    ReadImage(jpg_filenameL.c_str(), &imageL);
    ReadImage(jpg_filenameR.c_str(), &imageR);

    int start_s = clock();
    std::vector<Vec3> vec_camPos;
    vec_camPos = reconstruct_scene(imageL, imageR, true, inputDir);
    int stop_s = clock();
    cout << "time: " << double(stop_s-start_s) / CLOCKS_PER_SEC << endl;

    for (auto i: vec_camPos)
        cout << i << ' ';



    return 0;
}

