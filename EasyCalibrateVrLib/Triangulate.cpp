#include "stdafx.h"
#include "Triangulate.h"

using namespace cv;

// TODO - move all calculations from float to double

Triangulate::Triangulate() {
	// preallocate variables for speed

	// vars for computeRay()
	srcPts = Mat(1, npoints, CV_32FC2);
	dstPts = Mat(1, npoints, CV_32FC2); // not sure if this actually is used

	// vars for camToRoomRay()
	srcMat = Mat(3, 1, CV_32FC1);
	dstMat = Mat(3, 1, CV_32FC1);

	// vars for computePoint()
	B = Mat(3, 1, CV_64FC1);
	A = Mat(3, 3, CV_64FC1);
}

Triangulate::~Triangulate() {
}

cv::Point3f Triangulate::computeRay(CalibMeasure& m, cv::Mat& intrinsic, cv::Mat& distortion) {
	float u = m.pixelCamera.x;
	float v = m.pixelCamera.y;

	// undistort the point and normalize
	/*
	Vec2f in, out;
	in[0] = u;
	in[1] = v;
	undistortPoints(in, out, m_SD->m_CamIntrinsic, m_SD->m_CamDistortion);
	float un = out[0];
	float vn = out[1];
	*/
	float un, vn;
	{
		// fill src matrix
		float * src_ptr = (float *)srcPts.data;
		for (int pi = 0; pi < npoints; ++pi) {
			*(src_ptr + pi * 2 + 0) = u;
			*(src_ptr + pi * 2 + 1) = v;
		}
		undistortPoints(srcPts, dstPts, intrinsic, distortion);
		float * dst_ptr = (float *)dstPts.data;
		un = dst_ptr[0];
		vn = dst_ptr[1];
	}

	// compute ray in camera space
	// from: http://stackoverflow.com/questions/3107771/pixel-coordinates-to-3d-line-opencv
	Point3f ray(un, // this is done by undistort points: u / fx - cx / fx,
				vn, // v / fy - cy / fy,
				1.0);

	// convert to room space
	return camToRoomRay(ray, m.rotation);
}

cv::Point3f Triangulate::camToRoomRay(cv::Point3f& ray, cv::Mat& rotation) {
	srcMat.at<float>(0, 0) = ray.x;
	srcMat.at<float>(1, 0) = ray.y;
	srcMat.at<float>(2, 0) = ray.z;
	dstMat = rotation * srcMat;

	//std::stringstream ss;
	//ss << "src is\n" << srcMat << "\n";
	//ss << "rot is\n" << rotation << "\n";
	//ss << "dst is\n" << dstMat << "\n";
	//OutputDebugString(ss.str().c_str());
	
	// convert back to point
	Point3f rayrs = Point3f(dstMat.at<float>(0, 0),
	                        dstMat.at<float>(1, 0),
	                        dstMat.at<float>(2, 0));

	// normalize
	double len = sqrt(pow(rayrs.x, 2) + pow(rayrs.y, 2) + pow(rayrs.z, 2));
	rayrs.x /= len;
	rayrs.y /= len;
	rayrs.z /= len;

	return rayrs;
}

// implementation of: https://docs.google.com/viewer?a=v&q=cache:8ZY9VFuCShAJ:citeseerx.ist.psu.edu/viewdoc/download?doi%3D10.1.1.15.6117%26rep%3Drep1%26type%3Dpdf+&hl=en&gl=us&pid=bl&srcid=ADGEESi-Awh3bIW5wTXlQkWVWRpFBSGYXWBhlNNq77DW0omsOcPoWzq2VtRYSgcwZhqm3X9Ad0F_kjp3Tu5Re8SnFcP0QjIHlT54ZQuvlrsZoCwJzUB0roV1ERLOg3kOgG60c6fSbmAi&sig=AHIEtbSFp0qWDzZ94w-VS8xG3XsWbgP-0w&pli=1
CalibResult Triangulate::computePoint(std::vector<CalibMeasure>& m) {
	using namespace cv;
	using namespace std;

	Mat& A = computeA(m);
	Mat& b = computeb(m);

	// solve
	Mat x;// = A.inv() * b;
	solve(A, b, x, DECOMP_EIG); // A is symmetric so eigenvalue decomp can be used

	//stringstream ss;
	//ss << "A = " << endl << A << endl << "b = " << b << endl << "x = " << x << endl;
	//OutputDebugString(ss.str().c_str());

	// build result
	Point3f wp(x.at<double>(0, 0),
			   x.at<double>(1, 0),
			   x.at<double>(2, 0));
	return CalibResult(m[0].gridPoint, m[0].pixelScreen, wp);
}

cv::Mat Triangulate::computeA(std::vector<CalibMeasure>& m) {
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			A.at<double>(r, c) = 0;
		}
	}
	
	for (auto it = m.begin(); it != m.end(); ++it) {
		const Point3f& p = it->translation;
		const Point3f& d = it->ray;

		// notation from the paper
		double a = d.x;
		double b = d.y;
		double c = d.z;
		double x = p.x;
		double y = p.y;
		double z = p.z;

		A.at<double>(0, 0) += 1 - a*a; A.at<double>(0, 1) += -a*b;    A.at<double>(0, 2) += -a*c;
		A.at<double>(1, 0) += -a*b;    A.at<double>(1, 1) += 1 - b*b; A.at<double>(1, 2) += -b*c;
		A.at<double>(2, 0) += -a*c;    A.at<double>(2, 1) += -b*c;    A.at<double>(2, 2) += 1 - c*c;
	}

	return A;
}

cv::Mat Triangulate::computeb(std::vector<CalibMeasure>& m) {
	for (int r = 0; r < 3; r++) {
		B.at<double>(r, 0) = 0;
	}
	
	for (auto it = m.begin(); it != m.end(); ++it) {
		const Point3f& p = it->translation;
		const Point3f& d = it->ray;

		// notation from the paper
		double a = d.x;
		double b = d.y;
		double c = d.z;
		double x = p.x;
		double y = p.y;
		double z = p.z;

		B.at<double>(0, 0) += (1 - a*a)*x - a*b*y - a*c*z;
		B.at<double>(1, 0) += -a*b*x + (1 - b*b)*y - b*c*z;
		B.at<double>(2, 0) += -a*c*x - b*c*y + (1 - c*c)*z;
	}

	return B;
}

// Tests -----------------------------------------------------------------------
// -----------------------------------------------------------------------------
//#include "catch.hpp"

/*
// test entire to mesh subsystem using simulated data
// data simulator class is tested elsewhere
TEST_CASE("Triangulate Simulated", "simulated data test 1") {
	Mat rot(3, 3, CV_64FC1);
	Mat trans(3, 1, CV_64FC1);
	Mat rvec;
	std::vector<Point2d> opts;
	Triangulate tri;

	// create simple intrinsic matrix
	Mat in = Mat::zeros(3, 3, CV_64FC1);
	in.at<double>(0, 0) = 10; // fx
	in.at<double>(1, 1) = 10; // fy
	in.at<double>(0, 2) = 100; // cx
	in.at<double>(1, 2) = 100; // cy
	in.at<double>(2, 2) = 1.0; // 1.0
	INFO("cam intrinsics are\n" << in);

	// create zero distortion vector
	Mat distortion = Mat::zeros(4, 1, CV_64FC1);
	INFO("cam distortion is\n" << distortion);

	// create some test points
	std::vector<Point3d> pts;
	pts.push_back(Point3d(0, 0, 2));
	//pts.push_back(Point3d(0, 0, 1));
	for (size_t i = 0; i < pts.size(); i++) {
		INFO("pt " << i << " is " << pts[i]);
	}

	// create a few camera views
	std::vector<CalibMeasure> m;

	// view 1 from x=2,y=0,z=1 facing along -x
	//trans.at<double>(0,0) = 2;
	//trans.at<double>(1,0) = 0;
	//trans.at<double>(2,0) = 1;
	//rot.at<double>(0,0) = 0; rot.at<double>(0,1) = 0; rot.at<double>(0,2) = -1;
	//rot.at<double>(1,0) = 1; rot.at<double>(1,1) = 0; rot.at<double>(1,2) = 0;
	//rot.at<double>(2,0) = 0; rot.at<double>(2,1) = -1; rot.at<double>(2,2) = 0;

	trans = (Mat_<double>(3,1) << -2, -0, -1);
	rot = (Mat_<double>(3,3) << 
		 0,  1,  0,
		 0,  0, -1,
		-1,  0,  0);
	Rodrigues(rot, rvec);
	projectPoints(pts, rvec, trans, in, distortion, opts);
	
	Mat ptMat = (Mat_<double>(4, 1) << 0, 0, 2, 1);
	Mat tt = (Mat_<double>(3, 4) << 
		1, 0, 0, -2,
		0, 1, 0,  0,
		0, 0, 1, -1);
	Mat rtt = rot * tt;
	Mat projMat = in * rtt;
	Mat pelMat = projMat * ptMat;
	std::stringstream ss;
	ss << "in is \n" << in << "\n";
	ss << "ptMat is\n" << ptMat << "\n";
	ss << "tt is\n" << tt << "\n";
	ss << "rtt is\n" << rtt << "\n";
	ss << "projMat is\n" << projMat << "\n";
	ss << "pelMat is\n" << pelMat << "\n";
	OutputDebugString(ss.str().c_str());
	for (size_t i = 0; i < opts.size(); i++) {
		INFO("opts view 1 " << i << " is " << opts[i]);
	}
	for (size_t i = 0; i < opts.size(); i++) {
		CalibMeasure mm;
		mm.m_GridPoint = Point2i(i, 0);
		mm.m_PixelCamera = opts[i];
		mm.m_Rotation = rot.clone();
		mm.m_Translation.x = trans.at<double>(0, 0);
		mm.m_Translation.y = trans.at<double>(1, 0);
		mm.m_Translation.z = trans.at<double>(2, 0);
		m.push_back(mm);
	}

	// view 2 from x=0,y=2,z=1 facing along -y
	trans.at<double>(0,0) = 0;
	trans.at<double>(1,0) = 2;
	trans.at<double>(2,0) = 1;
	rot.at<double>(0,0) = -1; rot.at<double>(0,1) = 0; rot.at<double>(0,2) = 0;
	rot.at<double>(1,0) = 0; rot.at<double>(1,1) = 0; rot.at<double>(1,2) = -1;
	rot.at<double>(2,0) = 0; rot.at<double>(2,1) = -1; rot.at<double>(2,2) = 0;
	Rodrigues(rot, rvec);
	projectPoints(pts, rvec, trans, in, distortion, opts);
	for (size_t i = 0; i < opts.size(); i++) {
		INFO("opts view 2 " << i << " is " << opts[i]);
	}
	for (size_t i = 0; i < opts.size(); i++) {
		CalibMeasure mm;
		mm.m_GridPoint = Point2i(i, 0);
		mm.m_PixelCamera = opts[i];
		mm.m_Rotation = rot.clone();
		mm.m_Translation.x = trans.at<double>(0, 0);
		mm.m_Translation.y = trans.at<double>(1, 0);
		mm.m_Translation.z = trans.at<double>(2, 0);
		m.push_back(mm);
	}

	// compute rays
	for (auto it = m.begin(); it != m.end(); it++) {
		it->m_Ray = tri.computeRay(*it, in, distortion);
	}

	// compute world points
	std::vector<CalibMeasure> pp0, pp1;
	for (size_t j = 0; j < m.size(); j++) {
		if (m[j].m_GridPoint.x == 0) {
			pp0.push_back(m[j]);
		} else if (m[j].m_GridPoint.x == 1) {
			pp1.push_back(m[j]);
		}
	}
	CalibResult wp0 = tri.computePoint(pp0);
	//CalibResult wp1 = tri.computePoint(pp1);

	REQUIRE(wp0.m_WorldPoint == Point3f(0, 0, 1));
	//REQUIRE(wp1.m_WorldPoint == Point3f(0, 0, 0.5));
}
*/

/*
// compute ray tests
TEST_CASE("Triangulate::camToRoomRay", "Ray rotation") {
	Triangulate tri;
	Point3f p;
	cv::Mat rot(3, 3, CV_32FC1);

	p.x = 0.5;
	p.y = 0.5;
	p.z = 1.0;

	// rotate 90deg about y
	rot.at<float>(0, 0) = 0; rot.at<float>(0, 1) = 0; rot.at<float>(0, 2) = 1;
	rot.at<float>(1, 0) = 0; rot.at<float>(1, 1) = 1; rot.at<float>(1, 2) = 0;
	rot.at<float>(2, 0) = -1; rot.at<float>(2, 1) = 0; rot.at<float>(2, 2) = 0;
	
	Point3f r = tri.camToRoomRay(p, rot);
	INFO("r is " << r);
	REQUIRE(r.x == 1.0);
	REQUIRE(r.y == 0.5);
	REQUIRE(r.z == -0.5);
}

// ray intersection tests
TEST_CASE("Triangulate::computeA", "Paper example compute A") {
	Triangulate tri;
	std::vector<CalibMeasure> v;
	CalibMeasure a;

	// Camera 1
	a.m_Translation = Point3f(0, 0, 0);
	a.m_Ray = Point3f(1, 0, 0);
	v.push_back(a);

	// Camera 2
	a.m_Translation = Point3f(3, 2, 5);
	a.m_Ray = Point3f(0, 0, -1);
	v.push_back(a);

	Mat A = tri.computeA(v);
	INFO("A is\n" << A);
	REQUIRE(A.at<double>(0, 0) == 1);
	REQUIRE(A.at<double>(1, 1) == 2);
	REQUIRE(A.at<double>(2, 2) == 1);
	// zeros elsewhere
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			if (r != c) REQUIRE(A.at<double>(r, c) == 0);
		}
	}
}

TEST_CASE("Triangulate::computeB", "Paper example compute b") {
	Triangulate tri;
	std::vector<CalibMeasure> v;
	CalibMeasure a;

	// Camera 1
	a.m_Translation = Point3f(0, 0, 0);
	a.m_Ray = Point3f(1, 0, 0);
	v.push_back(a);

	// Camera 2
	a.m_Translation = Point3f(3, 2, 5);
	a.m_Ray = Point3f(0, 0, -1);
	v.push_back(a);

	Mat b = tri.computeb(v);
	INFO("b is " << b);
	REQUIRE(b.at<double>(0, 0) == 3);
	REQUIRE(b.at<double>(1, 0) == 2);
	REQUIRE(b.at<double>(2, 0) == 0);
}

TEST_CASE("Triangulate::computePoint", "Paper example compute world point") {
	Triangulate tri;
	std::vector<CalibMeasure> v;
	CalibMeasure a;

	// Camera 1
	a.m_Translation = Point3f(0, 0, 0);
	a.m_Ray = Point3f(1, 0, 0);
	v.push_back(a);

	// Camera 2
	a.m_Translation = Point3f(3, 2, 5);
	a.m_Ray = Point3f(0, 0, -1);
	v.push_back(a);

	CalibResult r = tri.computePoint(v);
	Point3f ans(3, 1, 0);
	float d = sqrt(pow(r.m_WorldPoint.x - ans.x, 2) +
		pow(r.m_WorldPoint.y - ans.y, 2) +
		pow(r.m_WorldPoint.z - ans.z, 2));
	INFO("world point is " << r.m_WorldPoint);
	REQUIRE(d < 0.001f);
}
*/