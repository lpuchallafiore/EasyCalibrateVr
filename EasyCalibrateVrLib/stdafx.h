// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

// C/C++
#include <vector>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <algorithm>
#include <unordered_map>
#include <iomanip>

// G3D
#include <G3D/G3D.h>
#include <GLG3D/GLG3D.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/contrib.hpp>

#ifdef WIN32
  #include <conio.h>   // For _kbhit()
  #include <windows.h> // For Sleep()
#endif // WIN32