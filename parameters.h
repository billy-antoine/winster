#ifndef PARAMETERS
#define PARAMETERS

#include <opencv2/highgui.hpp>

bool multi_threading = true;
int ROI_size = 80;

// Calibration parameters
bool showRectified = true;
bool displayCorner = true;
bool useCalibrated = true;
bool calibration = false;

cv::Size boardSize(9,6);
float squareSize = 3.0f;
//const char* imagelistfn = "Data/Calibration_images/test3/";
const string inputDir = "/home/doctorant/Vidéos/Stereo/Winster/kitti/";

const string imagelistfn = inputDir+"calib";

// SGBM parameters
int s_SADWindowSize = 3;
int s_numberOfDisparities = 5;
int s_preFilterCap = 48;
int s_minDisparity = 64;
int s_uniquenessRatio =2;
int s_speckleWindowSize = 150;

int s_speckleRange = 32;
int s_disp12MaxDiff = 11;

// Post-processing parameters
int s_write3D = 0;
int s_closing = 0;
int s_inpainting = 0;
int s_scale = 40; // In percentage
int s_mblur = 3;
int s_color = 1;

// GUI
const char *splashPath  = "Data/Images/UI/main.png";
const char *windowsName = "Stereo Vision";

// calibration directories
const string intrinsicsPath	= inputDir + "matrices/intrinsics.yml";
const string extrinsicsPath	= inputDir + "matrices/extrinsics.yml";
const string roisPath		= inputDir + "matrices/Rois.yml";

// 3D parameters
int maxDepth = 120;
int ratioCrop3D = 10; // Percent of croped borders for 3D point cloud
const string pointCloudPath = inputDir + "Results/PointClouds/pc";
int cloudNumber = 0;

//Camera resolution
int PROP_FRAME_WIDTH = 1280;
int PROP_FRAME_HEIGHT = 960;

#endif
