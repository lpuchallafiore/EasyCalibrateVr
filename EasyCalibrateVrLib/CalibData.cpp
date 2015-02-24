#include "stdafx.h"
#include "CalibData.h"

using namespace cv;

#define READ_WRITE_IMPL(type) \
	void write(cv::FileStorage& fs, const std::string&, const type & x) { \
		x.write(fs); \
	} \
	void read(const cv::FileNode& node, type& x, const type& default_value) { \
		if (node.empty()) { \
			x = default_value; \
		} else { \
			x.read(node); \
		} \
	}

// fix for missing functionality in opencv
void readPoint2i(const cv::FileNode& node, cv::Point2i& value, const cv::Point2i& default_value = cv::Point2i()) {
	std::vector<int> vals;
	node >> vals;
	if (vals.size() == 2) {
		value = cv::Point2i(vals[0], vals[1]);
	} else {
		value = default_value;
	}
}
void readPoint2f(const cv::FileNode& node, cv::Point2f& value, const cv::Point2f& default_value = cv::Point2f()) {
	std::vector<float> vals;
	node >> vals;
	if (vals.size() == 2) {
		value = cv::Point2f(vals[0], vals[1]);
	} else {
		value = default_value;
	}
}
void readPoint3f(const cv::FileNode& node, cv::Point3f& value, const cv::Point3f& default_value = cv::Point3f()) {
	std::vector<float> vals;
	node >> vals;
	if (vals.size() == 3) {
		value = cv::Point3f(vals[0], vals[1], vals[2]);
	} else {
		value = default_value;
	}
}
void readRect(const cv::FileNode& node, cv::Rect_<float>& value, const cv::Rect_<float>& default_value = cv::Rect_<float>()) {
	std::vector<float> vals;
	node >> vals;
	if (vals.size() == 4) {
		value = cv::Rect_<float>(vals[0], vals[1], vals[2], vals[3]);
	} else {
		value = default_value;
	}
}

// ScanConfig File I/O ---------------------------------------------------------
// -----------------------------------------------------------------------------
void ScanConfig::write(cv::FileStorage& fs) const {
}

void ScanConfig::read(const cv::FileNode& node) {
}

READ_WRITE_IMPL(ScanConfig);

// RawDataPiece File I/O -------------------------------------------------------
// -----------------------------------------------------------------------------
void RawDataPiece::write(cv::FileStorage& fs) const {
	fs << "{"; // begin map
	fs << "imagePath" << imagePath;
	fs << "groupId" << groupId;
	fs << "chessRot" << chessRot;
	fs << "chessTrans" << chessTrans;
	fs << "camRot" << camRot;
	fs << "camTrans" << camTrans;
	fs << "chessRootRot" << chessRootRot;
	fs << "chessRootTrans" << chessRootTrans;
	fs << "projRegionWidth" << projRegionWidth;
	fs << "projRegionHeight" << projRegionHeight;

	fs << "projRegionGrid" << "["; // begin list
	for (auto it = projRegionGrid.begin(); it != projRegionGrid.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	fs << "projRegionScreen" << "["; // begin list
	for (auto it = projRegionScreen.begin(); it != projRegionScreen.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	fs << "}"; // end map
}

void RawDataPiece::read(const cv::FileNode& node) {
	node["imagePath"] >> imagePath;
	node["groupId"] >> groupId;
	node["chessRot"] >> chessRot;
	readPoint3f(node["chessTrans"], chessTrans);
	node["camRot"] >> camRot;
	readPoint3f(node["camTrans"], camTrans);
	node["chessRootRot"] >> chessRootRot;
	readPoint3f(node["chessRootTrans"], chessRootTrans);
	node["projRegionWidth"] >> projRegionWidth;
	node["projRegionHeight"] >> projRegionHeight;

	FileNode n = node["projRegionGrid"];
	projRegionGrid.clear();
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end(); it++) {
			Point2i p;
			readPoint2i(*it, p);
			projRegionGrid.push_back(p);
		}
	} else {
		// error TODO
	}

	n = node["projRegionScreen"];
	projRegionScreen.clear();
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end(); it++) {
			Point2f p;
			readPoint2f(*it, p);
			projRegionScreen.push_back(p);
		}
	} else {
		// error TODO
	}
}

READ_WRITE_IMPL(RawDataPiece);

// RawProjData File I/O --------------------------------------------------------
// -----------------------------------------------------------------------------
void RawProjData::write(cv::FileStorage& fs) const {
	// begin map
	fs << "{";

	// projector setup
	fs << "left" << left;
	fs << "top" << top;
	fs << "width" << width;
	fs << "height" << height;

	// bounds
	fs << "gridBounds" << gridBounds;
	fs << "numPointsX" << numPointsX;
	fs << "numPointsY" << numPointsY;

	// measurements
	fs << "measurements" << "["; // begin list
	for (auto it = measurements.begin(); it != measurements.end(); it++) {
		fs << *it;
	}
	fs << "]"; // end list

	// end map
	fs << "}";
}

void RawProjData::read(const cv::FileNode& node) {
	// projector setup
	node["left"] >> left;
	node["top"] >> top;
	node["width"] >> width;
	node["height"] >> height;

	// bounds
	readRect(node["gridBounds"], gridBounds);
	node["numPointsX"] >> numPointsX;
	node["numPointsY"] >> numPointsY;

	// measurements
	FileNode n = node["measurements"];
	measurements.clear();
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			RawDataPiece p;
			it >> p;
			measurements.push_back(p);
		}
	} else {
		// error TODO
	}
}

READ_WRITE_IMPL(RawProjData);

// CalibMeasure File I/O -------------------------------------------------------
// -----------------------------------------------------------------------------
void CalibMeasure::write(cv::FileStorage& fs) const {
	fs << "{"; // begin map
	fs << "groupId" << groupId;
	fs << "gridPoint" << gridPoint;
	fs << "pixelScreen" << pixelScreen;
	fs << "pixelCamera" << pixelCamera;
	fs << "translation" << translation;
	fs << "rotation" << rotation;
	fs << "ray" << ray;
	fs << "}"; // end map
}

void CalibMeasure::read(const cv::FileNode& node) {
	node["groupId"] >> groupId;
	readPoint2i(node["gridPoint"], gridPoint);
	readPoint2f(node["pixelScreen"], pixelScreen);
	readPoint2f(node["pixelCamera"], pixelCamera);
	readPoint3f(node["translation"], translation);
	node["rotation"] >> rotation;
	readPoint3f(node["ray"], ray);
}

READ_WRITE_IMPL(CalibMeasure);

// CalibResult File I/O --------------------------------------------------------
// -----------------------------------------------------------------------------
void CalibResult::write(cv::FileStorage& fs) const {
	fs << "{"; // begin map
	fs << "gridPoint" << gridPoint;
	fs << "pixelScreen" << pixelScreen;
	fs << "worldPoint" << worldPoint;
	fs << "}"; // end map
}

void CalibResult::read(const cv::FileNode& node) {
	readPoint2i(node["gridPoint"], gridPoint);
	readPoint2f(node["pixelScreen"], pixelScreen);
	readPoint3f(node["worldPoint"], worldPoint);
}

READ_WRITE_IMPL(CalibResult);

// ProjectorData File I/O ------------------------------------------------------
// -----------------------------------------------------------------------------
void ProjectorData::write(cv::FileStorage& fs) const {
	// begin map
	fs << "{";

	// projector setup
	fs << "left" << left;
	fs << "top" << top;
	fs << "width" << width;
	fs << "height" << height;

	// calibration data
	fs << "gridBounds" << gridBounds;
	fs << "numPointsX" << numPointsX;
	fs << "numPointsY" << numPointsY;
	fs << "measurements" << "["; // begin list
	for (auto it = measurements.begin(); it != measurements.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list */

	// result data
	fs << "mesh" << "["; // begin list
	for (auto it = mesh.begin(); it != mesh.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list */

	// end map
	fs << "}";
}

void ProjectorData::read(const cv::FileNode& node) {
	// projector setup
	node["left"] >> left;
	node["top"] >> top;
	node["width"] >> width;
	node["height"] >> height;

	// calibration data
	readRect(node["gridBounds"],  gridBounds);
	node["numPointsX"] >> numPointsX;
	node["numPointsY"] >> numPointsY;
	FileNode n = node["measurements"];
	measurements.clear();
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			CalibMeasure m;
			it >> m;
			measurements.push_back(m);
		}
	} else {
		// error TODO
	}

	// result data
	n = node["mesh"];
	mesh.clear();
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			CalibResult r;
			it >> r;
			mesh.push_back(r);
		}
	} else {
		// error TODO
	}
}

READ_WRITE_IMPL(ProjectorData);

bool ProjectorData::writeXML(const std::string& filename) const {
	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened()) {
		return false;
	}

	fs << "ProjData" << *this;

	return true;
}

bool ProjectorData::readXML(const std::string& filename) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		return false;
	}

	fs["ProjData"] >> *this;

	return true;
}

bool ProjectorData::writeMesh(const std::string& filename) const {
	return writeMesh(filename, 0, 0, numPointsX, numPointsY);
}

bool ProjectorData::writeMesh(const std::string& filename, 
	int left, int top, int width, int height) const {

	FILE *fp = fopen(filename.c_str(), "w");
	if (fp == NULL) return false;

	// write vertices
	for (auto it = mesh.begin(); it != mesh.end(); ++it) {
		fprintf(fp, "v %f %f %f\n", 
			it->worldPoint.x,
			it->worldPoint.y,
			it->worldPoint.z);
	}

	// *---*
	// |   |
	// *---*
	int n = numPointsX;
	int m = numPointsY;
	// write quad faces
	for (int y = top; y < top + height - 1; y++) {
		for (int x = left; x < left + width - 1; x++) {
			int i = y * n + x;
			i = i + 1; // obj files index starts at 1
			fprintf(fp, "f %d %d %d %d\n",
				i + 1, i, i + n, i + n + 1);
		}
	}

	fclose(fp);

	return true;
}

// ScreenData File I/O ---------------------------------------------------------
// -----------------------------------------------------------------------------
void ScreenData::write(cv::FileStorage& fs) const {
	fs << "{"; // begin map
	// RawData
	fs << "screenWidth" << screenWidth;
	fs << "screenHeight" << screenHeight;

	fs << "rawProj" << "["; // begin list
	for (auto it = rawProj.begin(); it != rawProj.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	fs << "rawIntrinsic" << "["; // begin list
	for (auto it = rawIntrinsic.begin(); it != rawIntrinsic.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	fs << "rawExtrinsic" << "["; // begin list
	for (auto it = rawExtrinsic.begin(); it != rawExtrinsic.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	// ComputedData
	fs << "compIntrinsic" << compIntrinsic;
	fs << "compDistortion" << compDistortion;

	fs << "compProj" << "["; // begin list
	for (auto it = compProj.begin(); it != compProj.end(); ++it) {
		fs << *it;
	}
	fs << "]"; // end list

	// end map
	fs << "}";
}

void ScreenData::read(const cv::FileNode& node) {
	// RawData
	node["screenWidth"] >> screenWidth;
	node["screenHeight"] >> screenHeight;

	rawProj.clear();
	FileNode n = node["rawProj"];
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			RawProjData rpd;
			it >> rpd;
			rawProj.push_back(rpd);
		}
	} else {
		// error TODO
	}

	rawIntrinsic.clear();
	n = node["rawIntrinsic"];
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			RawDataPiece rdp;
			it >> rdp;
			rawIntrinsic.push_back(rdp);
		}
	} else {
		// error TODO
	}

	rawExtrinsic.clear();
	n = node["rawExtrinsic"];
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			RawDataPiece rdp;
			it >> rdp;
			rawExtrinsic.push_back(rdp);
		}
	} else {
		// error TODO
	}


	// ComputedData
	node["compIntrinsic"] >> compIntrinsic;
	node["compDistortion"] >> compDistortion;

	compProj.clear();
	n = node["compProj"];
	if (n.type() == FileNode::SEQ) {
		for (auto it = n.begin(); it != n.end();) {
			ProjectorData pd;
			it >> pd;
			compProj.push_back(pd);
		}
	} else {
		// error TODO
	}
}

READ_WRITE_IMPL(ScreenData);

bool ScreenData::writeXML(const std::string& filename) const {
	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened()) {
		return false;
	}

	fs << "ScreenData" << *this;

	return true;
}

bool ScreenData::readXML(const std::string& filename) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		return false;
	}

	fs["ScreenData"] >> *this;

	return true;
}

// Equality operators ----------------------------------------------------------
// -----------------------------------------------------------------------------
bool MatEqual(const cv::Mat& a, const cv::Mat& b) {
	return (a.empty() && b.empty())
		|| (a.rows == b.rows &&
			a.cols == b.cols &&
			a.type() == b.type() &&
			countNonZero(a != b) == 0);
}

bool RawDataPiece::operator==(const RawDataPiece& other) const {
	return (imagePath == other.imagePath)
		&& (groupId == other.groupId)
		&& (MatEqual(chessRot, other.chessRot))
		&& (chessTrans == other.chessTrans)
		&& (MatEqual(chessRootRot, other.chessRootRot))
		&& (camTrans == other.camTrans)
		&& (MatEqual(chessRootRot, other.chessRootRot))
		&& (chessRootTrans == other.chessRootTrans)
		&& (projRegionWidth == other.projRegionWidth)
		&& (projRegionGrid == other.projRegionGrid)
		&& (projRegionScreen == other.projRegionScreen);
}

bool RawProjData::operator==(const RawProjData& other) const {
	return (left == other.left)
		&& (top == other.top)
		&& (width == other.width)
		&& (height == other.height)
		&& (gridBounds == other.gridBounds)
		&& (numPointsX == other.numPointsX)
		&& (numPointsY == other.numPointsY)
		&& (measurements == other.measurements);
}

bool CalibMeasure::operator==(const CalibMeasure& other) const {
	return (groupId == other.groupId)
		&& (gridPoint == other.gridPoint)
		&& (pixelScreen == other.pixelScreen)
		&& (pixelCamera == other.pixelCamera)
		&& (translation == other.translation)
		&& (MatEqual(rotation, other.rotation))
		&& (ray == other.ray);
}

bool CalibResult::operator==(const CalibResult& other) const {
	return (gridPoint == other.gridPoint)
		&& (pixelScreen == other.pixelScreen)
		&& (worldPoint == other.worldPoint);
}

bool ProjectorData::operator==(const ProjectorData& other) const {
	return (left == other.left)
		&& (top == other.top)
		&& (width == other.width)
		&& (height == other.height)
		&& (gridBounds == other.gridBounds)
		&& (numPointsX == other.numPointsX)
		&& (numPointsY == other.numPointsY)
		&& (measurements == other.measurements)
		&& (mesh == other.mesh);
}

bool ScreenData::operator==(const ScreenData& other) const {
	return (screenWidth == other.screenWidth)
		&& (screenHeight == other.screenHeight)
		&& (rawProj == other.rawProj)
		&& (rawIntrinsic == other.rawIntrinsic)
		&& (rawExtrinsic == other.rawExtrinsic)
		&& (MatEqual(compIntrinsic, other.compIntrinsic))
		&& (MatEqual(compDistortion, other.compDistortion))
		&& (compProj == other.compProj);
}
