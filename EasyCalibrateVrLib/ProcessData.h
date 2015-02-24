#pragma once

#include "Vicon.h"
#include "ViconManipulator.h"
#include "CalibData.h"

/**
 * data and classes shared between states but not saved to disk
 * vicon objects, G3D fonts, temporary coordinates frames, etc.
 **/
class ProcessData {
public:
	// frames, measured with a tapemeasure
	G3D::CoordinateFrame roomToVicon;
	G3D::CoordinateFrame viconToRoom;

	// frames, measured by the vicon
	G3D::CoordinateFrame camRootToVicon;
	G3D::CoordinateFrame chessboardRootToVicon;

	// frames, calibrated by placing at origin and pressing button
	G3D::CoordinateFrame chessboardToChessboardRoot;

	// frames, measured via opencv and cam intrinsics
	G3D::CoordinateFrame chessboardToCam;

	// frames, computed from the vicon and opencv frames
	G3D::CoordinateFrame camToCamRoot;

	// derived transformations
	G3D::CoordinateFrame roomToCamRoot;
	G3D::CoordinateFrame camToRoom;
	G3D::CoordinateFrame chessboardToRoom;

	Vicon *vicon;
	ViconManipulator *viconCamManipulator;
	ViconManipulator *viconChessboardManipulator;

	// graphical stuff
	G3D::GFontRef df; // TODO create a new high-res font instead of using debugFont

	cv::VideoCapture cam;

	struct {
		int gridWidth;
		int gridHeight;
		float circleRadius;
		int minViews;
		float waitTime;
		int regionsX;
		int regionsY;
		int groupSize;
	} props;

	int currProj;

	ScreenData data; // this is saved to disk :)

public:
	ProcessData(const G3D::GFontRef& df);

	void initCam();
	// TODO FIXME: getFrame function which reconnects if camera disconnected
	void initVicon();
	void onNetwork();

	cv::Mat Matrix3ToMat(const G3D::Matrix3& m) const;
	cv::Point3f Vector3ToPoint3f(const G3D::Vector3& v) const;
	RawDataPiece buildRawDataPiece(const std::string& prefix, int num, 
		const cv::Mat& img, const std::vector<cv::Point2i>& grid, 
		const std::vector<cv::Point2f>& screen, int groupid) const;
};