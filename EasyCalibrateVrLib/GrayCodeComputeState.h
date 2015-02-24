#pragma once

// SfM algorithm is based on SfM-Toy by Roy Shilkrot and associated chapter
// in the book "Mastering OpenCV with Practical Computer Vision Projects"

// TODO: extract SfM into stand-alone classes so can do SfM on arbitrary
// images using ORB features or optical flow ... for fun mainly
// but might also be useful as a second gen kinect-less reconstruction like
// Peng is working on. Make sure support auto-calibration, since that would
// make it much easier.

#include "AppState.h"
#include "Calibration.h"
#include "ScreenMesh.h"

#define V3DLIB_ENABLE_SUITESPARSE

#include <Math/v3d_linear.h>
#include <Base/v3d_vrmlio.h>
#include <Geometry/v3d_metricbundle.h>

#pragma comment(lib, "V3D.lib")
#pragma comment(lib, "COLAMD.lib")

struct Feature { // TODO: switch from "Feature" to KeyPoint+Descriptor pattern so can use ORB/SURF/etc.
	cv::Point2f pt; ///< location in camera image
	cv::Point3i id; ///< location in projector image (gray code), z=projector#
	cv::Vec3b color; ///< color for point
};

struct CloudPoint {
	cv::Point3d wpt;
	cv::Vec3b color;
	std::vector<int> imgpt_for_img;
	double reprojection_error;
	CloudPoint() { }
	CloudPoint(const cv::Point3d& w, const cv::Vec3b clr) : wpt(w), color(clr) { }
	bool operator==(const CloudPoint& other) {
		return wpt == other.wpt 
			&& color == other.color
			&& imgpt_for_img == other.imgpt_for_img
			&& reprojection_error == other.reprojection_error;
	}
};

class IBundleAdjuster {
public:
	virtual ~IBundleAdjuster() { }
	virtual void adjustBundle(std::vector<CloudPoint>& pointCloud,
							  cv::Mat& cam_matrix,
							  cv::Mat& dist_coeffs,
							  const std::vector<std::vector<Feature> >& img_pts,
							  std::map<int, cv::Matx34d>& P_mats) = 0;
};

// TODO: make OpenCV and CVSBA adjuster classes and use everywhere BA is used
class SSBABundleAdjuster : public IBundleAdjuster {
public:
	SSBABundleAdjuster();
	~SSBABundleAdjuster();

	void adjustBundle(std::vector<CloudPoint>& pointCloud,
					  cv::Mat& cam_matrix,
					  cv::Mat& dist_coeffs,
					  const std::vector<std::vector<Feature> >& img_pts,
					  std::map<int, cv::Matx34d>& P_mats) override;

private:
	int count2DMeasurements(const std::vector<CloudPoint>& pointCloud);
};

class cmpPoint2i {
public:
	bool operator()(const cv::Point2i& a, const cv::Point2i& b) {
		int y_diff = a.y - b.y;
		int x_diff = a.x - b.x;
		if (y_diff != 0) {
			return y_diff < 0;
		} else {
			return x_diff < 0;
		}
	}
};

class cmpPoint3i {
public:
	bool operator()(const cv::Point3i& a, const cv::Point3i& b) {
		int y_diff = a.y - b.y;
		int x_diff = a.x - b.x;
		int z_diff = a.z - b.z;
		if (z_diff != 0) {
			return z_diff < 0;
		} else if (y_diff != 0) {
			return y_diff < 0;
		} else {
			return x_diff < 0;
		}
	}
};

class GrayCodeComputeState : public IAppState {
protected:
	// progress bar names
	struct ProgressBars {
		const std::string CALIBRATE,
						  DECODE_GC,
						  DECODE_ARUCO,
						  FIND_MATCHES,
						  CAMERA_RECOVERY,
						  COMPUTE_OVERLAP;

		ProgressBars() : CALIBRATE("0. Calibrate"),
						 COMPUTE_OVERLAP("1. Overlap"),
						 DECODE_GC("2. Decode GC"),
						 DECODE_ARUCO("3. Decode arUco"),
						 FIND_MATCHES("4. Find Matches"),
						 CAMERA_RECOVERY("5. Camera Recovery") { }
	} _PB;

	// window
	float _left, _top, _width, _height;

	// progress bar
	CRITICAL_SECTION _csProgress;
	struct SProgressBar {
		float percent;
		G3D::Color3 color;
	};
	std::map<std::string, SProgressBar> _progressMap;

	// working variables
	cv::Mat _K, _K_inv, _dist_coeffs;
	std::vector<std::vector<Feature> > _img_pts;
	std::vector<std::vector<Feature> > _img_pts_good;
	std::map<std::pair<int, int>, std::vector<cv::DMatch> > _matches_matrix;
	int _first_view, _second_view;
	std::map<int, cv::Matx34d> _P_mats;
	std::vector<CloudPoint> _pcloud;
	std::vector<Feature> _correspImg1Pt, _correspImg2Pt;
	std::set<int> _done_views, _good_views;

	ScreenMesh _mesh;

	// binary images of projector overlap
	// 255 - overlap, 0 - no overlap
	// (i, j) -> image representing overlap of projector i onto projector j
	std::map<std::pair<int, int>, cv::Mat> _overlapImages;

	// image display
	CRITICAL_SECTION _csImage;
	cv::Mat _image;

	// parameters
	struct params {
		int				intrinsic_start;
		std::string		intrinsic_prefix;	///< prefix of all intrinsic calibrate image filenames
		int				intrinsic_end;		///< number of intrinsic calibration images
		int				intrinsic_step;

		std::string		graycode_folder;	///< folder name of graycode images
		int				num_image_sets;		///< number of sets of graycode images
	} _p;

public:
	GrayCodeComputeState(float left, float top, float width, float height,
		const std::shared_ptr<ProcessData>& pd);
	virtual ~GrayCodeComputeState() { }

	Ptr onGraphics2D(G3D::RenderDevice * rd,
					 G3D::Array<G3D::Surface2D::Ref>& posed2D) override;

protected:
	Ptr nextState();

	static DWORD WINAPI t_WorkerStart(LPVOID lpParam);
	DWORD t_Worker();

	// progress api
	void createProgressBar(std::string name, G3D::Color3 color);
	void updateProgressBar(std::string name, float percent);
	float readProgressBar(std::string name);

	// graphics api
	void drawBar(G3D::RenderDevice* rd, std::string name, float percent,
		float x, float y, float w, float h, G3D::Color3 foreColor, 
		G3D::Color3 backColor = G3D::Color3::gray());

	// image api
	void setImage(cv::Mat& image);

	// file I/O helper functions
	std::string getIntrinsicImageFilename(int i) const;
	std::string getGrayCodeImageFilename(int proj, int set, int code) const;
	bool savePointCloud(const std::string& filename);
	bool saveMeshAsPLY(const std::string& filename, bool saveAR = false) const;

	// SfM helper functions
	// FIXME: break apart into general purpose classes
	//        so can support more features in the future
	//        like ORB based dense SfM, autocalibration, and multi-ray triangulation
	//        Qt GUI, image-debugging, "online" reconstruction
	void decodeGrayCode(int proj, int set, cv::Mat& rows, cv::Mat& cols);
	std::vector<Feature> grayCodeToFeatures(const cv::Mat& rows, const cv::Mat& cols, int proj) const;
	void filterMatchesUsingFundamental();
	void computeBaselineTriangulation();
	void adjustCurrentBundle();
	int findHomographyInliers2Views(int vi, int vj);
	cv::Mat getFundamentalMat(const std::vector<Feature>& imgpts1,
							  const std::vector<Feature>& imgpts2,
							  std::vector<Feature>& imgpts1_good,
							  std::vector<Feature>& imgpts2_good,
							  std::vector<cv::DMatch>& matches);
	bool findCameraMatrices(const cv::Mat& K,
							const cv::Mat& Kinv,
							const cv::Mat& distCoeffs,
							const std::vector<Feature>& imgpts1,
							const std::vector<Feature>& imgpts2,
							std::vector<Feature>& imgpts1_good,
							std::vector<Feature>& imgpts2_good,
							cv::Matx34d& P,
							cv::Matx34d& P1,
							std::vector<cv::DMatch>& matches,
							std::vector<CloudPoint>& outCloud);
	bool triangulatePointsBetween2Views(int working_view,
										int older_view,
										std::vector<CloudPoint>& new_triangulated,
										std::vector<int>& add_to_cloud);
	double myTriangulatePoints(const std::vector<Feature>& pt_set1,
							   const std::vector<Feature>& pt_set2,
							   const cv::Mat& K,
							   const cv::Mat& Kinv,
							   const cv::Mat& distcoeff,
							   const cv::Matx34d& P,
							   const cv::Matx34d& P1,
							   std::vector<CloudPoint>& pointcloud,
							   std::vector<Feature>& correspImg1Pt);
	bool decomposeEtoRandT(cv::Mat_<double>& E,
						   cv::Mat_<double>& R1,
						   cv::Mat_<double>& R2,
						   cv::Mat_<double>& t1,
						   cv::Mat_<double>& t2);
	bool testTriangulation(const std::vector<CloudPoint>& pcloud, 
						   const cv::Matx34d& P, 
						   std::vector<uchar>& status);
	bool checkCoherentRotation(cv::Mat_<double>& R);
	void find2D3DCorrespondences(int working_view, 
								 std::vector<cv::Point3f>& ppcloud, 
								 std::vector<cv::Point2f>& imgPoints);
	bool findPoseEstimation(int working_view, cv::Mat_<double>& rvec,
							cv::Mat_<double>& t, cv::Mat_<double>& R,
							std::vector<cv::Point3f> ppcloud,
							std::vector<cv::Point2f> imgPoints);
	std::vector<Feature> getArucoFeatures(int image_set);

	// worker thread functions
	void wt_doIntrinsicCalibrate();
	void wt_doGrayCodeDecode();
	void wt_doArucoDecode();
	void wt_doMatchFeatures();
	void wt_doFirstTriangulation();
	void wt_doCameraRecovery();
	void wt_doColorizeCloud();
	void wt_doAlignPointCloud();
	void wt_doPointCloudToMesh(int size);
	void wt_doComputeOverlap();

	std::map<cv::Point3i, CloudPoint*, cmpPoint3i> GrayCodeComputeState::genIdToPCMap();
};

class IPointCloudAlignAlgo {
public:
	virtual std::vector<CloudPoint> alignToRoom(const std::vector<CloudPoint>& pc,
												const std::vector<CloudPoint>& roompc,
												const std::vector<std::pair<size_t, size_t>>& matches) = 0;
};

class Affine3DAlign : public IPointCloudAlignAlgo {
public:
	virtual std::vector<CloudPoint> alignToRoom(const std::vector<CloudPoint>& pc,
												const std::vector<CloudPoint>& roompc,
												const std::vector<std::pair<size_t, size_t>>& matches) override;
};

// implements Absolute Orientation algorithm, Horn's method.
// based on: http://www.mathworks.com/matlabcentral/fileexchange/26186-absolute-orientation-horn-s-method
class AbsOrHorn : public IPointCloudAlignAlgo {
public:
	virtual std::vector<CloudPoint> alignToRoom(const std::vector<CloudPoint>& pc,
												const std::vector<CloudPoint>& roompc,
												const std::vector<std::pair<size_t, size_t>>& matches) override;

private:
	cv::Mat doHornMethod(const std::vector<cv::Point3d>& src,
						 const std::vector<cv::Point3d>& dst);
	cv::Point3d findCentroid(const std::vector<cv::Point3d>& pts);
};

class PointCloudOps {
public:
	static cv::Point3d findCentroid(const std::vector<CloudPoint>& pc);
};

// http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}