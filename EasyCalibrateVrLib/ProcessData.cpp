#include "stdafx.h"
#include "ProcessData.h"

void ProcessData::initVicon() {
	using namespace G3D;
	if (vicon) return;

	vicon = new Vicon("160.94.77.67"); //"vicon");
	viconCamManipulator = new ViconManipulator(vicon, "newCam", "p_rootJnt");
	viconChessboardManipulator = new ViconManipulator(vicon, "newCheckerboard", "endBone");

	// measured with tapemeasure
	viconToRoom = CoordinateFrame(
		Matrix3::identity(),
		Vector3(3.048, 4.9784, 0.0)
	);
	roomToVicon = viconToRoom.inverse();

	// measured by placing on floor
	chessboardToChessboardRoot = CoordinateFrame(Matrix3::identity(), Vector3(0, 0, 0));

	// from extrinsic calibration
	camToCamRoot = CoordinateFrame(Matrix3::identity(), Vector3(0, 0, 0));
}

void ProcessData::onNetwork() {
	if (vicon) vicon->update();

	if (viconCamManipulator) {
		viconCamManipulator->onNetwork();
		viconCamManipulator->getFrame(camRootToVicon);
		// update frames
		camToRoom = viconToRoom * camRootToVicon * camToCamRoot;
	}

	if (viconChessboardManipulator) {
		viconChessboardManipulator->onNetwork();
		viconChessboardManipulator->getFrame(chessboardRootToVicon);
		chessboardToRoom = viconToRoom * chessboardRootToVicon * chessboardToChessboardRoot;
	}

	/*
	wchar_t buf[1024];
	swprintf(buf, L"camToRoom x: %f y: %f z:%f\n", 
		camToRoom.translation.x, 
		camToRoom.translation.y,
		camToRoom.translation.z);
	OutputDebugString(buf);
	*/
}

cv::Mat ProcessData::Matrix3ToMat(const G3D::Matrix3& m) const {
	return cv::Mat_<float>(3, 3) <<
		m[0][0], m[0][1], m[0][2],
		m[1][0], m[1][1], m[1][2],
		m[2][0], m[2][1], m[2][2];
}

cv::Point3f ProcessData::Vector3ToPoint3f(const G3D::Vector3& v) const {
	return cv::Point3f(v.x, v.y, v.z);
}

RawDataPiece ProcessData::buildRawDataPiece(const std::string& prefix, int num, 
	const cv::Mat& img, const std::vector<cv::Point2i>& grid, 
	const std::vector<cv::Point2f>& screen, int groupid) const {

	// save to raw data
	RawDataPiece p;
	std::stringstream ss;
	ss << prefix << "_" << currProj << "_" << num << ".jpg";
	cv::imwrite(ss.str(), img);
	p.imagePath = ss.str();
	p.groupId = groupid;

	p.chessRootRot = Matrix3ToMat(chessboardToChessboardRoot.rotation);
	p.chessRootTrans = Vector3ToPoint3f(chessboardToChessboardRoot.translation);
		
	p.camRot = Matrix3ToMat(this->camRootToVicon.rotation);
	p.camTrans = Vector3ToPoint3f(this->camRootToVicon.translation);

	p.chessRot = Matrix3ToMat(this->chessboardRootToVicon.rotation);
	p.chessTrans = Vector3ToPoint3f(this->chessboardRootToVicon.translation);

	p.projRegionHeight = this->props.gridHeight / this->props.regionsY;
	p.projRegionWidth = this->props.gridWidth / this->props.regionsX;

	p.projRegionGrid = grid;
	p.projRegionScreen = screen;

	return p;
}

ProcessData::ProcessData(const G3D::GFontRef& font) : vicon(NULL), 
	viconCamManipulator(NULL), viconChessboardManipulator(NULL), df(font), 
	currProj(0) { 
}

void ProcessData::initCam() { 
	if (cam.isOpened()) return;
	cam = cv::VideoCapture(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
}