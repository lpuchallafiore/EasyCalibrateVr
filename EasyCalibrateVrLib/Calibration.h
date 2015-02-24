/* Classes for stereo and monocular camera calibration (intrinsic & extrinsic)
 * Requires OpenCV 2.x (tested with 2.3.1)
 * author: Loren Puchalla Fiore <loren.fiore@gmail.com>
 * version: June 2012
 */

/* TODO:
 * mono & stereo - openGL projection & modelview from calibration (from ArUco)
 * mono & stereo - calibrate live/video/images w/o highgui - use callback
 * document every function in header and code so easy to use API
 * mono & stereo - OpenCV's new asymetric circle grid pattern
 * mono & stereo - integrage ArUco? calibration via ArUco board? AR? -> AugmentReality.cpp/.h
 * mono & stereo - verbose mode :) so I can tell what the hell is going on
 * stereo - re-enable Hartley rectification
 * stereo - total average error
 * stereo - image correspondence & depth calculation
 * mono and stereo - to-world rewrite and test routines
 * mono and stereo - non-square grids ?
 * mono and stereo - 4/5/8 distortion coeffiecients
 * mono and stereo - set undistort interpolation mode
 * mono and stereo - calibration flags set/unset
 * mono and stereo - const ints -> user config parameters
 * stereo - option to calibrate each camera separately or together
 * stereo - rect method user selectable
 * stereo - OpenCV GPU depth calculation
 */

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//----- Monocular Camera -------------------------------------------------------
class MonoCalibration {
public:
	// argW, argH - number of interior corners in the chessboard pattern
	MonoCalibration(int argW = 5, int argH = 8, float argSS = 0.0254);
	~MonoCalibration();

	bool load(const std::string& filename = "monocalib.yml");
	bool save(const std::string& filename = "monocalib.yml");

	double calibrateFromImages(std::vector<std::string> images);
	double calibrateFromImages(std::vector<cv::Mat> images);

	// FIXME: should have a better way to do this
	// and have image callbacks, so can render using OpenGL instead
	// of highgui
	double calibrateFromCapture(cv::VideoCapture& cap);

	double calibrateFromVideo(std::string filename);
	double calibrateFromCam(int camNum = 0);

	void undistortImage(cv::Mat image);

	cv::Mat getIntrinsic() { return m_Intrinsic; }
	cv::Mat getDistortion() { return m_Distortion; }
	std::vector<cv::Mat> getRotationVectors() { return m_RotVecs; }
	std::vector<cv::Mat> getTranslationVectors() { return m_TransVecs; }

	void setName(std::string name) { m_Name = name; }
	std::string getName() { return m_Name; }
	void setDescription(std::string description) { 
		m_Description = description; 
	}
	std::string getDescription() { return m_Description; }

	double getAvgError() { return m_AvgError; }

	void getGLProjectionMatrix(cv::Mat &projection,
							   cv::Size imgSize,
							   cv::Size renderSize,
							   double gNear, double gFar,
							   bool invert);

	// stored board from calibration
	void getGLModelViewMatrix(cv::Mat &modelView, int boardId);
	// new board image
	bool getGLModelViewMatrix(cv::Mat &modelView, cv::Mat &img, bool real = true);

// FIXME: should be protected, public now to make OpenGL hacks work
public:
	void startCalibration();
	bool processCalibration(cv::Mat& image);
	double endCalibration();
	void precomputeValues();
	bool imageLoopCommon(cv::Mat image, int i, bool video = false);
	double computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs,
		const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		std::vector<double>& perViewErrors);

public:
	// input parameters
	int     m_BoardW;
	int     m_BoardH;
	float   m_SquareSize;
	int     m_BoardN;

	// calibration variables
	int                                     m_ImageCount;
	cv::Size                                m_ImageSize;
	std::vector<std::vector<cv::Point3f> >  m_ObjPoints;
	std::vector<std::vector<cv::Point2f> >  m_ImagePoints;
	cv::Mat                                 m_PointCounts;
	int                                     m_Successes;
	cv::Mat                                 m_SmallImage;

	// results of the calibration
	cv::Mat                 m_Intrinsic;
	cv::Mat                 m_Distortion;
	std::vector<cv::Mat>    m_RotVecs;
	std::vector<cv::Mat>    m_TransVecs;
	double                  m_AvgError;

	// precomputed variables for image undistortion
	cv::Mat     m_UMapX;
	cv::Mat     m_UMapY;

	// extra data
	std::string m_Name;
	std::string m_Description;

	bool m_ShowImage;
};

//----- Stereo Cameras ---------------------------------------------------------
// uses the terms "left" and "right" but doesn't really matter,
// cameras can also be up/down vertical alligned pair and it will still work.
enum StereoCameraID {
	LEFT_CAMERA,
	RIGHT_CAMERA
};

class StereoCalibration {
public:
	// argW, argH - number of interior corners in the chessboard pattern
	// argSS - size of a side of the square in the pattern, in meters
	StereoCalibration(int argW = 5, int argH = 8, double argSS = 0.0254);
	~StereoCalibration();

	bool load(std::string filename = "stereocalib.yml");
	bool save(std::string filename = "stereocalib.yml", bool saveR = false);

	double calibrateFromImages(std::vector<std::string> leftImages,
							   std::vector<std::string> rightImages);
	double calibrateFromImages(std::vector<cv::Mat> leftImages,
							   std::vector<cv::Mat> rightImages);
	
	double calibrateFromVideo(std::string leftVideoFile,
							  std::string rightVideoFile);
	double calibrateFromCam(int leftCamNum = 1,
							int rightCamNum = 2);

	void undistortImage(cv::Mat& image, StereoCameraID camId);
	void rectifyAndUndistortImages(cv::Mat& leftImage, cv::Mat& rightImage);

	cv::Mat getIntrinsic(StereoCameraID camId) { return m_Intrinsic[camId]; }
	cv::Mat getDistortion(StereoCameraID camId) { return m_Distortion[camId]; }
	std::vector<cv::Mat> getRotation() { return m_Rotation; }
	std::vector<cv::Mat> getTranslation() { return m_Translation; }
	cv::Mat getEssential() { return m_Essential; }
	cv::Mat getFundamental() { return m_Fundamental; }
	cv::Mat getPrect(StereoCameraID camId) { return m_Prect[camId]; }
	// for depth via cvReprojectImageTo3D
	cv::Mat getQ() { return m_Q; }

	//// 3xN -> 3xN (CV_64FC1)
	//void world2camera(cv::Mat& world, cv::Mat& cam,
	//                  StereoCameraID camId = LEFT_CAMERA,
	//                  bool rectified = true);
	//// 3xN -> 3xN (CV_64FC1)
	//void camera2world(cv::Mat& cam, cv::Mat& world,
	//                  StereoCameraID camId = LEFT_CAMERA,
	//                  bool rectified = true);
	//// 3xN -> 2xN (CV_64FC1)
	//void world2image(cv::Mat& world, cv::Mat& image,
	//                 StereoCameraID camId = LEFT_CAMERA,
	//                 bool rectified = true);
	//// 3xN -> 2xN (CV_64FC1)
	//void camera2image(cv::Mat& cam, cv::Mat& image,
	//                  StereoCameraID camId = LEFT_CAMERA,
	//                  bool rectified = true);

	void setName(std::string name) { m_Name = name; }
	std::string getName() { return m_Name; }
	void setDescription(std::string description) { 
		m_Description = description; 
	}
	std::string getDescription() { return m_Description; }

	double getAvgError() { return m_AvgError; }
	bool isVerticalStereo() { return m_IsVerticalStereo; }

protected:
	void startCalibration();
	bool processCalibration(cv::Mat images[2]);
	double endCalibration();
	void precomputeValues();
	double calibrateFromCapture(cv::VideoCapture& capLeft, 
								cv::VideoCapture& capRight);
	bool imageLoopCommon(cv::Mat images[2], int i, bool video = false);

protected:
	int         m_BoardW;
	int         m_BoardH;
	int         m_BoardN;
	double      m_SquareSize;
	std::string m_Name;
	std::string m_Description;

	int                                     m_ImageCount;
	cv::Size                                m_ImageSize;
	std::vector<std::vector<cv::Point3f> >  m_ObjPoints;
	std::vector<std::vector<cv::Point2f> >  m_ImagePoints[2];
	int                                     m_Successes;
	cv::Mat                                 m_SmallImage;

	// results of the calibration
	cv::Mat m_Intrinsic[2];
	cv::Mat m_Distortion[2];
	cv::Mat m_Rotation, m_Translation;
	cv::Mat m_Essential, m_Fundamental;
	double  m_AvgError;
	bool    m_IsVerticalStereo;

	// precomputed matrices for image undistortion and rectification
	cv::Mat m_UMapX[2], m_UMapY[2];
	cv::Mat m_RMapX[2], m_RMapY[2];
	cv::Mat m_Rrect[2], m_Prect[2], m_Q; // FIXME: better names

	// transformations from world to camera (rectified, and unrectified)
	cv::Mat m_RW[2], m_TW[2]; // R and T from world to unrectified
	cv::Mat m_RRW[2], m_TRW[2]; // R and T from world to rectified
};

#endif