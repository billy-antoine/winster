#ifndef STRUCT_FROM_MOTION
#define STRUCT_FROM_MOTION

#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/matching/indMatchDecoratorXY.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"


#include <iostream>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::image;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace std;

/// Read intrinsic K matrix from a file (ASCII)
/// F 0 ppx
/// 0 F ppy
/// 0 0 1
bool readIntrinsic(const std::string & fileName, Mat3 & K);

bool exportToPly(const std::vector<Vec3> & vec_points,
  const std::vector<Vec3> & vec_camPos,
  const std::string & sFileName);

std::vector<Vec3> reconstruct_scene(Image<unsigned char> &imageL, Image<unsigned char> &imageR, bool drawImages, const std::string inputDir);

#endif // STRUCT_FROM_MOTION

