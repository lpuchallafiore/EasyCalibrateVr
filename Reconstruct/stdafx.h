#pragma once

// FIXME: replace with includes of VRWindowLib
// C/C++
#include <vector>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <algorithm>

// G3D
#include <G3D/G3D.h>
#include <GLG3D/GLG3D.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

// link to the correct version of opencv libraries
#define OPENCV_VERSION "249"
#ifdef _DEBUG
    #define OPENCV_LIB_EXT "d.lib"
#else
    #define OPENCV_LIB_EXT ".lib"
#endif

#pragma comment(lib, "opencv_core" OPENCV_VERSION OPENCV_LIB_EXT)
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION OPENCV_LIB_EXT)
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION OPENCV_LIB_EXT)
#pragma comment(lib, "opencv_calib3d" OPENCV_VERSION OPENCV_LIB_EXT)
#pragma comment(lib, "opencv_features2d" OPENCV_VERSION OPENCV_LIB_EXT)
#pragma comment(lib, "opencv_contrib" OPENCV_VERSION OPENCV_LIB_EXT)
