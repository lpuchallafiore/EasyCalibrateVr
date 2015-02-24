#include "stdafx.h"

#include "GrayCodeComputeState.h"
#include "catch.hpp"

using namespace std;
using namespace cv;

TEST_CASE("PointCloudOps/centroid-zero-pc", "[PointCloudOps]") {
	vector<CloudPoint> pc;

	REQUIRE(PointCloudOps::findCentroid(pc) == Point3d(0.f, 0.f, 0.f));
}

TEST_CASE("PointCloudOps/centroid-pc", "[PointCloudOps]") {
	vector<CloudPoint> pc;

	for (int x = -15; x <= -5; ++x) {
		for (int y = -15; y <= -5; ++y) {
			for (int z = -15; z <= -5; z++) {
				CloudPoint cp;
				cp.wpt = Point3d(x, y, z);
				pc.push_back(cp);
			}
		}
	}

	REQUIRE(PointCloudOps::findCentroid(pc) == Point3d(-10.f, -10.f, -10.f));
}

TEST_CASE("Affine3DAlign/translation-only", "[PointCloudAlignAlgo]") {
	vector<CloudPoint> pcA;
	vector<CloudPoint> pcB;

	Point3d trans(5, 10, 15);

	CloudPoint a1; a1.wpt = Point3d(-1, 1, -1); pcA.push_back(a1);
	CloudPoint b1; b1.wpt = a1.wpt + trans; pcB.push_back(b1);
	
	CloudPoint a2; a2.wpt = Point3d(1, 1, -1); pcA.push_back(a2);
	CloudPoint b2; b2.wpt = a2.wpt + trans; pcB.push_back(b2);

	CloudPoint a3; a3.wpt = Point3d(1, -1, -1); pcA.push_back(a3);
	CloudPoint b3; b3.wpt = a3.wpt + trans; pcB.push_back(b3);

	CloudPoint a4; a4.wpt = Point3d(-1, -1, -1); pcA.push_back(a4);
	CloudPoint b4; b4.wpt = a4.wpt + trans; pcB.push_back(b4);

	CloudPoint a5; a5.wpt = Point3d(-1, 1, 1); pcA.push_back(a5);
	CloudPoint b5; b5.wpt = a5.wpt + trans; pcB.push_back(b5);

	CloudPoint a6; a6.wpt = Point3d(1, 1, 1); pcA.push_back(a6);
	CloudPoint b6; b6.wpt = a6.wpt + trans; pcB.push_back(b6);

	CloudPoint a7; a7.wpt = Point3d(1, -1, 1); pcA.push_back(a7);
	CloudPoint b7; b7.wpt = a7.wpt + trans; pcB.push_back(b7);

	CloudPoint a8; a8.wpt = Point3d(-1, -1, -1); pcA.push_back(a8);
	CloudPoint b8; b8.wpt = a8.wpt + trans; pcB.push_back(b8);

	vector<std::pair<size_t, size_t>> matches;
	for (size_t i = 0; i < 5; i++) {
		matches.push_back(make_pair(i, i));
	}

	Affine3DAlign algo;
	vector<CloudPoint> pcC = algo.alignToRoom(pcA, pcB, matches);

	REQUIRE(pcC.size() == pcA.size());
	for (size_t j = 0; j < pcC.size(); j++) {
		CHECK(pcB[j].wpt.x == Approx(pcC[j].wpt.x));
		CHECK(pcB[j].wpt.y == Approx(pcC[j].wpt.y));
		CHECK(pcB[j].wpt.z == Approx(pcC[j].wpt.z));
	}
}

TEST_CASE("AbsOrHorn/translation-only", "[PointCloudAlignAlgo]") {
	vector<CloudPoint> pcA;
	vector<CloudPoint> pcB;

	Point3d trans(5, 10, 15);

	CloudPoint a1; a1.wpt = Point3d(-1, 1, -1); pcA.push_back(a1);
	CloudPoint b1; b1.wpt = a1.wpt + trans; pcB.push_back(b1);
	
	CloudPoint a2; a2.wpt = Point3d(1, 1, -1); pcA.push_back(a2);
	CloudPoint b2; b2.wpt = a2.wpt + trans; pcB.push_back(b2);

	CloudPoint a3; a3.wpt = Point3d(1, -1, -1); pcA.push_back(a3);
	CloudPoint b3; b3.wpt = a3.wpt + trans; pcB.push_back(b3);

	CloudPoint a4; a4.wpt = Point3d(-1, -1, -1); pcA.push_back(a4);
	CloudPoint b4; b4.wpt = a4.wpt + trans; pcB.push_back(b4);

	CloudPoint a5; a5.wpt = Point3d(-1, 1, 1); pcA.push_back(a5);
	CloudPoint b5; b5.wpt = a5.wpt + trans; pcB.push_back(b5);

	CloudPoint a6; a6.wpt = Point3d(1, 1, 1); pcA.push_back(a6);
	CloudPoint b6; b6.wpt = a6.wpt + trans; pcB.push_back(b6);

	CloudPoint a7; a7.wpt = Point3d(1, -1, 1); pcA.push_back(a7);
	CloudPoint b7; b7.wpt = a7.wpt + trans; pcB.push_back(b7);

	CloudPoint a8; a8.wpt = Point3d(-1, -1, -1); pcA.push_back(a8);
	CloudPoint b8; b8.wpt = a8.wpt + trans; pcB.push_back(b8);

	vector<std::pair<size_t, size_t>> matches;
	for (size_t i = 0; i < 3; i++) {
		matches.push_back(make_pair(i, i));
	}

	AbsOrHorn algo;
	vector<CloudPoint> pcC = algo.alignToRoom(pcA, pcB, matches);

	REQUIRE(pcC.size() == pcA.size());
	for (size_t j = 0; j < pcC.size(); j++) {
		CHECK(pcB[j].wpt.x == Approx(pcC[j].wpt.x));
		CHECK(pcB[j].wpt.y == Approx(pcC[j].wpt.y));
		CHECK(pcB[j].wpt.z == Approx(pcC[j].wpt.z));
	}
}


TEST_CASE("PointCloudAlignAlgo/rotation-only", "[PointCloudAlignAlgo]") {
	// TODO
}

TEST_CASE("PointCloudAlignAlgo/scale-only", "[PointCloudAlignAlgo]") {
	// TODO
}

TEST_CASE("PointCloudAlignAlgo/translation-scale-rotation", "[PointCloudAlignAlgo]") {
	// TODO
}

cv::Point3d xform(cv::Point3d pt, cv::Mat R, cv::Mat t, double s) {
	Mat mpt = (Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
	Mat npt = s*R*mpt + t;
	return Point3d(npt.at<double>(0), 
				   npt.at<double>(1),
				   npt.at<double>(2));
}

TEST_CASE("AbsOrHorn/all", "[PointCloudAlignAlgo]") {
	vector<CloudPoint> pcA;
	vector<CloudPoint> pcB;

	Mat t = (Mat_<double>(3, 1) << 5, -10, 15);
	double a = 45.0 / 180.0 * 3.14159;
	Mat R = (Mat_<double>(3, 3) << cos(a), 0, sin(a), 0, 1, 0, -sin(a), 0, cos(a));
	double s = 1.3;

	CloudPoint a1; a1.wpt = Point3d(-1, 1, -1); pcA.push_back(a1);
	CloudPoint b1; b1.wpt = xform(a1.wpt, R, t, s); pcB.push_back(b1);
	
	CloudPoint a2; a2.wpt = Point3d(1, 1, -1); pcA.push_back(a2);
	CloudPoint b2; b2.wpt = xform(a2.wpt, R, t, s); pcB.push_back(b2);

	CloudPoint a3; a3.wpt = Point3d(1, -1, -1); pcA.push_back(a3);
	CloudPoint b3; b3.wpt = xform(a3.wpt, R, t, s); pcB.push_back(b3);

	CloudPoint a4; a4.wpt = Point3d(-1, -1, -1); pcA.push_back(a4);
	CloudPoint b4; b4.wpt = xform(a4.wpt, R, t, s); pcB.push_back(b4);
	
	CloudPoint a5; a5.wpt = Point3d(-1, 1, 1); pcA.push_back(a5);
	CloudPoint b5; b5.wpt = xform(a5.wpt, R, t, s); pcB.push_back(b5);

	CloudPoint a6; a6.wpt = Point3d(1, 1, 1); pcA.push_back(a6);
	CloudPoint b6; b6.wpt = xform(a6.wpt, R, t, s); pcB.push_back(b6);

	CloudPoint a7; a7.wpt = Point3d(1, -1, 1); pcA.push_back(a7);
	CloudPoint b7; b7.wpt = xform(a7.wpt, R, t, s); pcB.push_back(b7);

	CloudPoint a8; a8.wpt = Point3d(-1, -1, -1); pcA.push_back(a8);
	CloudPoint b8; b8.wpt = xform(a8.wpt, R, t, s); pcB.push_back(b8);

	vector<std::pair<size_t, size_t>> matches;
	for (size_t i = 0; i < 4; i++) {
		matches.push_back(make_pair(i, i));
	}

	AbsOrHorn algo;
	vector<CloudPoint> pcC = algo.alignToRoom(pcA, pcB, matches);

	REQUIRE(pcC.size() == pcA.size());
	for (size_t j = 0; j < pcC.size(); j++) {
		CHECK(pcB[j].wpt.x == Approx(pcC[j].wpt.x));
		CHECK(pcB[j].wpt.y == Approx(pcC[j].wpt.y));
		CHECK(pcB[j].wpt.z == Approx(pcC[j].wpt.z));
	}
}