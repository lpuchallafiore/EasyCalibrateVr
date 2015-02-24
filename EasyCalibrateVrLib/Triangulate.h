#pragma once

#include "CalibData.h"

// helper class
class Triangulate {
protected:
	// computeRay()
	static const int npoints = 4; // needs to be > 1 otherwise cv::undistortPoints crashes
	cv::Mat srcPts, dstPts;

	// camToRoomRay()
	cv::Mat srcMat, dstMat;

	// computePoint()
	cv::Mat A, B;

public:
	Triangulate();
	virtual ~Triangulate();

	// computes a room coordinate ray from a CalibMeasure
	cv::Point3f computeRay(CalibMeasure& m, cv::Mat& intrinsic, cv::Mat& distortion);

	// transforms a ray from camera space to room space
	cv::Point3f camToRoomRay(cv::Point3f& ray, cv::Mat& rotation);

	// compute room space intersection of multiple room space rays
	CalibResult computePoint(std::vector<CalibMeasure>& m);

	// computes intermediary results for computePoint()
	cv::Mat computeA(std::vector<CalibMeasure>& m);

	// computes intermediary results for computePoint()
	cv::Mat computeb(std::vector<CalibMeasure>& m);
};