#include <iostream>
#include <time.h>
#include "struct_from_motion.h"
#include "stereo_calib.h"
#include "disparity_map.h"
#include "DepthToPly.h"
#include "parameters.h"
#include "opencv2/surface_matching.hpp"
#include "opencv2/imgproc.hpp"
#include "point_cloud_viewer.h"
#include "tool_box.h"

#include <fstream>
#include <iostream>
#include <dirent.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using namespace cv;

int multimaps(){

    std::string path = "/home/doctorant/Images/Stereo/stereopoupsoftheflyingpoupiesanymore/left/";
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("/home/doctorant/Images/Stereo/stereopoupsoftheflyingpoupiesanymore/left/")) != NULL) {
      while ((ent = readdir (dir)) != NULL) {
        printf ("%s\n", ent->d_name);

        const string jpg_filenameL  = path + ent->d_name;
        const string jpg_filenameR  = path + "../right/"+ ent->d_name;

        cv::Mat left  = imread(jpg_filenameL);
        cv::Mat right = imread(jpg_filenameR);

        if(! left.data)
            continue;

        cv::Mat dispMap = compute_disparity_map(left, right);

        dispMap.convertTo(dispMap, CV_32FC1);

        cv::String filename = path+"../maps/" + ent->d_name + ".txt";
        writeMatToFile(dispMap,filename);

      }
      closedir (dir);
    }

    return 0;
}

int main(){

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
    cv::Mat M1, D1, M2, D2, R1, R2, P1, P2, R, T;

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

    cv::imwrite(outputDir+"dmap_01.png", dispMap01);
    cv::imwrite(outputDir+"dmap_02.png", dispMap02);

    if(writeImage){
        //dispMap01.convertTo(dispMap01, CV_32FC1);
        //dispMap02.convertTo(dispMap02, CV_32FC1);
        //cv::normalize(dispMap01, dispMap01,  0, 255, CV_MINMAX, CV_8U);
        cv::String filename = outputDir+"mat01.txt";
        writeMatToFile(dispMap01,filename);
    }

    cv::Mat dispN, dispN2;

    cv::normalize(dispMap01, dispN,  0, 255, CV_MINMAX, CV_8U);
    cv::normalize(dispMap02, dispN2, 0, 255, CV_MINMAX, CV_8U);

    cv::applyColorMap( dispN, dispN, COLORMAP_PARULA );
    cv::applyColorMap( dispN2, dispN2, COLORMAP_PARULA );

    if(writeImage){
        cv::imwrite(outputDir+"dmap_01_visu.png", dispN);
        cv::imwrite(outputDir+"dmap_02_visu.png", dispN2);
    }

    cv::Mat canvas, canvas2;
    cv::vconcat(left01, dispN, canvas);
    cv::vconcat(left02, dispN2, canvas2);

    cv::imshow("DispMap", canvas);
    //cv::imshow("DispMap2", canvas2);
    cv::waitKey(50);

    std::cout << "reprojecting point cloud ..." << std::endl;

    cv::Mat points01, points02;

    cv::reprojectImageTo3D(dispMap01, points01, Q, true, -1);
    cv::reprojectImageTo3D(dispMap02, points02, Q, true, -1);


    // Canny filter


    cv::Mat cannied01, cannied02, points01_cannied, points02_cannied;
    getCannyMask(left01, dispMap01, cannied01);
    getCannyMask(left02, dispMap02, cannied02);

    // Reproject cannied images
    cv::reprojectImageTo3D(cannied01, points01_cannied, Q, true, -1);
    cv::reprojectImageTo3D(cannied02, points02_cannied, Q, true, -1);    

    //Q.convertTo(Q, CV_32F);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = MatToPoinXYZ(points01, left01);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = MatToPoinXYZ(points02, left02);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sparse01 = MatToPoinXYZ(points01_cannied, left01);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sparse02 = MatToPoinXYZ(points02_cannied, left02);


    if(writeImage){
        pcl::io::savePLYFileBinary (outputDir+"cloud1.ply", *cloud1);
        pcl::io::savePLYFileBinary (outputDir+"cloud2.ply", *cloud2);

        pcl::io::savePLYFileBinary (outputDir+"cloud_sparse01.ply", *cloud_sparse01);
        pcl::io::savePLYFileBinary (outputDir+"cloud_sparse02.ply", *cloud_sparse02);
    }


    if(performSFM){   
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


        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << vec_camPos[1][0], vec_camPos[1][1], vec_camPos[1][2];
    }


    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();


    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_sparse02);
    icp.setInputTarget(cloud_sparse01);

    icp.setTransformationEpsilon (1e-8);
    //icp.setMaxCorrespondenceDistance (0.05);
    //icp.setEuclideanFitnessEpsilon (1);
    icp.setMaximumIterations (150);

    pcl::PointCloud<pcl::PointXYZRGB> Final_sparse;
    pcl::PointCloud<pcl::PointXYZRGB> Final_dense;
    icp.align(*cloud_sparse02);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    
    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation ();
    std::cout<<"trans : \n"<<transformationMatrix<<std::endl;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sparse01_trans (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dense02_trans  (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //pcl::transformPointCloud( *cloud_sparse01, *cloud_sparse01_trans, transformationMatrix);
    pcl::transformPointCloud( *cloud2        , *cloud_dense02_trans , transformationMatrix);

    Final_sparse  = *cloud_sparse01;
    Final_sparse += *cloud_sparse02;

    Final_dense  = *cloud1;
    Final_dense += *cloud_dense02_trans;

    pcl::io::savePLYFileBinary (outputDir+"IcpResult_sparse.ply", Final_sparse);
    pcl::io::savePLYFileBinary (outputDir+"IcpResult_dense.ply" , Final_dense);

    // VIEWER

   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer->setBackgroundColor(0,0,0);

   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1 (cloud_sparse01,  20, 100, 200);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2 (cloud_sparse02, 200, 50 , 50 );

   viewer->addPointCloud<pcl::PointXYZRGB> (cloud_sparse01, single_color1, "sample_cloud_1");
   viewer->addPointCloud<pcl::PointXYZRGB> (cloud_sparse02, single_color2, "sample_cloud_2");

    viewer->addCoordinateSystem(0.1);
      while(!viewer->wasStopped())
      {
          viewer->spinOnce();
          boost::this_thread::sleep (boost::posix_time::microseconds(100000));
      }




    return 0;
}

