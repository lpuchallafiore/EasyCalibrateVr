#include "stdafx.h"

#include "CalibData.h"
#include "catch.hpp"

using namespace cv;

SCENARIO("MatEqual", "") {
	GIVEN("two empty matrices") {
		cv::Mat a, b;
		WHEN("compared") {
			bool equal = MatEqual(a, b);
			THEN("should equal") {
				REQUIRE(equal == true);
			}
		}
	}

	GIVEN("one empty and one non-empty matrix") {
		cv::Mat a, b = Mat::eye(3, 3, CV_32FC1);
		WHEN("compared") {
			bool equal = MatEqual(a, b);
			THEN("should not equal") {
				REQUIRE(equal == false);
			}
		}
	}

	GIVEN("two equal non-empty matrices") {
		cv::Mat a = Mat::eye(3, 3, CV_32FC1);
		cv::Mat b = Mat::eye(3, 3, CV_32FC1);
		WHEN("compared") {
			bool equal = MatEqual(a, b);
			THEN("should equal") {
				REQUIRE(equal == true);
			}
		}
	}

	GIVEN("two non-equal different sized non-empty matrices") {
		cv::Mat a = Mat::eye(3, 3, CV_32FC1);
		cv::Mat b = Mat::eye(5, 5, CV_32FC1);
		WHEN("compared") {
			bool equal = MatEqual(a, b);
			THEN("should not equal") {
				REQUIRE(equal == false);
			}
		}
	}

	GIVEN("two non-equal same sized non-empty matrices") {
		cv::Mat a = Mat::eye(3, 3, CV_32FC1);
		cv::Mat b = Mat::eye(3, 3, CV_32FC1) * 30;
		WHEN("compared") {
			bool equal = MatEqual(a, b);
			THEN("should not equal") {
				REQUIRE(equal == false);
			}
		}
	}
}

TEST_CASE("RawDataPiece XML IO", "[xml]") {
	RawDataPiece r, s;
	r.imagePath = "image";
	r.chessRot = Mat::eye(2, 2, CV_32FC1);
	r.chessTrans.x = 100;
	r.camRot = Mat::eye(3, 3, CV_32FC1);
	r.camTrans.y = 1000;
	r.chessRootRot = Mat::eye(4, 4, CV_32FC1);
	r.chessRootTrans.z = 10;
	r.projRegionWidth = 20;
	r.projRegionHeight = 15;
	for (int i = 0; i < 100; i++) {
		r.projRegionGrid.push_back(Point2i(i, i/2));
	}
	for (int i = 0; i < 100; i++) {
		r.projRegionGrid.push_back(Point2f(i, 2*i));
	}

	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}

TEST_CASE("RawProjData XML IO", "[xml]") {
	RawProjData r, s;

	r.left = 120;
	r.top = 324;
	r.width = 43;
	r.height = 321;

	r.gridBounds = Rect_<float>(4, 3, 2, 1);
	r.numPointsX = 100;
	r.numPointsY = 10;
	for (int i = 0; i < 10; i++) {
		r.measurements.push_back(RawDataPiece());
	}

	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}

TEST_CASE("CalibResult XML IO", "[xml]") {
	CalibResult r(Point2i(50, 50), 
	              Point2f(100.5, 25.5),
				  Point3f(10, 20, 18.8)), s;
	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}

TEST_CASE("CalibMeasure XML IO", "[xml]") {
	CalibMeasure r(Point2i(50, 50),
		           Point2f(10, 20),
				   Point2f(20, 20),
				   Mat(3, 3, CV_32FC1),
				   Point3f(100, 200, 300), -1), s;
	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}

TEST_CASE("ProjectorData XML IO", "[xml]") {
	ProjectorData r, s;
	r.left = 100;
	r.top = 100;
	r.width = 20;
	r.height = 1024;
	r.gridBounds = Rect_<float>(4, 3, 2, 1);
	r.numPointsX = 100;
	r.numPointsY = 10;
	for (int i = 0; i < 10; i++) {
		r.measurements.push_back(CalibMeasure(Point2i(10, 20),
			Point2f(10, 20), Point2f(10, i), Mat(3, 3, CV_32FC1),
			Point3f(10, i, i), -1));
	}
	for (int i = 0; i < 5; i++) {
		r.mesh.push_back(CalibResult(Point2i(i, i), 
			Point2f(2*i, i), Point3f(2*i, 3*i, 4*i)));
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}

TEST_CASE("ScreenData XML IO", "[xml]") {
	ScreenData r, s;
	r.screenHeight = 1280;
	r.screenWidth = 1024;
	Mat m(3, 3, CV_32FC1);
	m.at<float>(0, 0) = 100;
	r.compIntrinsic = m;
	r.compDistortion = m;
	for (int j = 0; j < 4; j++) {
		ProjectorData w;
		w.left = 100 * j;
		w.top = 100 + j;
		w.width = 20;
		w.height = 1024;
		w.gridBounds = Rect_<float>(4, 3, 2, 1);
		w.numPointsX = 100;
		w.numPointsY = 10;
		for (int i = 0; i < 10; i++) {
			w.measurements.push_back(CalibMeasure(Point2i(10, 20),
				Point2f(10, 20), Point2f(10, i), Mat(3, 3, CV_32FC1),
				Point3f(10, i, i), -1));
		}
		for (int i = 0; i < 5; i++) {
			w.mesh.push_back(CalibResult(Point2i(i, i), 
				Point2f(2*i, i), Point3f(2*i, 3*i, 4*i)));
		}
		r.compProj.push_back(w);
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::WRITE);
		fs << "temp" << r;
	}
	{
		FileStorage fs("temp.xml.gz", FileStorage::READ);
		fs["temp"] >> s;
	}

	REQUIRE(r == s);
}