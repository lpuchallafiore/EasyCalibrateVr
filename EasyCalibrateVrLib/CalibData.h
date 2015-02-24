#pragma once

#include <string>
#include <opencv2/core/core.hpp>

#define READ_WRITE_DECL(type) \
	public: \
	friend void write(cv::FileStorage& fs, const std::string&, const type& x); \
	friend void read(const cv::FileNode& node, type& x, const type& default_value = type()); \
	private: \
	void write(cv::FileStorage& fs) const; \
	void read(const cv::FileNode& node)

class ScanConfig {
public:
	enum PatternType_t {
		CIRCLE_GRID,
		GRAY_CODE_1x1,
		GRAY_CODE_2x2,
		GRAY_CODE_4x4
	} patternType;
	bool useVicon;
	bool useAruco;

	enum AlgorithmType_t {
		TRIANGULATION, // Vicon-only
		TRIANGULATION_WITH_MOTION, // Vicon-only, but uses Fundamental Matrix estimation to fine tune Vicon results
		STRUCTURE_FROM_MOTION // no Vicon
	} algorithmType;
	enum BundleAdjustType_t {
		NONE,
		OpenCV,
		CVSBA,
		SSBA
	} bundleType;

public:
	ScanConfig() {
		// default
		patternType = GRAY_CODE_2x2;
		useVicon = false;
		useAruco = true;
		algorithmType = STRUCTURE_FROM_MOTION;
		bundleType = SSBA;
	}
	
	bool isValid() {
		// need to have some method of localization
		bool hasLocalization = (useVicon || useAruco);
		// cannot do triangulation w/o vicon
		bool algoCorrect = !(algorithmType == TRIANGULATION && !useVicon); 
		return hasLocalization && algoCorrect;
	}

	bool operator==(const ScanConfig& other) const;

	READ_WRITE_DECL(ScanConfig);
};

bool MatEqual(const cv::Mat& a, const cv::Mat& b);

class RawDataPiece {
public:
	std::string imagePath;
	int groupId;

	// chess to vicon
	cv::Mat chessRot;
	cv::Point3f chessTrans;

	// cam to vicon
	cv::Mat camRot;
	cv::Point3f camTrans;
	 
	// chess to chess root
	cv::Mat chessRootRot;
	cv::Point3f chessRootTrans;

	// projector info
	int projRegionWidth;
	int projRegionHeight;
	std::vector<cv::Point2i> projRegionGrid;
	std::vector<cv::Point2f> projRegionScreen;

public:
	RawDataPiece() { }

	bool operator==(const RawDataPiece& other) const;

	READ_WRITE_DECL(RawDataPiece);
};

class RawProjData {
public:
	int left;
	int top;
	int width;
	int height;

	cv::Rect_<float> gridBounds;
	int numPointsX; // number of grid points width-wise
	int numPointsY; // number of grid points height-wise

	std::vector<RawDataPiece> measurements;

public:
	RawProjData() { }

	bool operator==(const RawProjData& other) const;

	READ_WRITE_DECL(RawProjData);
};

// a measurement from the camera and vicon, 
// along with computed intermediate results
class CalibMeasure {
public:
	// for supporting multiple measurements from the same place
	// used to average out any noise in the measurements
	int groupId; // id of -1 is no group

	// known
	cv::Point2i gridPoint; // index of grid point
	cv::Point2f pixelScreen; // pixel coordinates drawn to projector (relative to projector ROI)

	// measured via camera
	cv::Point2f pixelCamera; // pixel center in camera image

	// measured via vicon
	cv::Point3f translation; // camera location when image acquired
	cv::Mat rotation; // camera rotation when image acquired (3x3)

	// computed from the above and camera intrinsics
	cv::Point3f ray; // ray from camera origin through pixel

public:
	CalibMeasure() { groupId = -1; }
	CalibMeasure(cv::Point2i gp, cv::Point2f sp, cv::Point2f cp, 
		cv::Mat r, cv::Point3f t, int gid) 
		: gridPoint(gp), pixelScreen(sp), pixelCamera(cp), 
		rotation(r), translation(t), ray(0, 0, 0), groupId(gid) {
	}

	bool operator==(const CalibMeasure& other) const;

	READ_WRITE_DECL(CalibMeasure);
};

// a single point in the resulting mesh
class CalibResult {
public:
	cv::Point2i gridPoint;		// index of grid point
	cv::Point2f pixelScreen;	// pixel location drawn to on the projector (relative to projector ROI)
	cv::Point3f worldPoint;		// world point

public:
	CalibResult() { }
	CalibResult(cv::Point2i gp, cv::Point2f sp, cv::Point3f wp) 
		: gridPoint(gp), pixelScreen(sp), worldPoint(wp) { }

	bool operator==(const CalibResult& other) const;

	READ_WRITE_DECL(CalibResult);
};

// measurements and results of the calibration
// each projector data is written to disk individually and combined into
// a single ScreenData XML file at the end
// obj meshes are created for each projector, and one at the end
class ProjectorData {
public:
	// projector setup
	int left;		// this projector ROI left
	int top;		// this projector ROI top
	int width;		// this projector ROI width
	int height;		// this projector ROI height
	
	// calibration data 
	cv::Rect_<float> gridBounds; // bounds on the calibration pattern
	int numPointsX; // number of grid points width-wise
	int numPointsY; // number of grid points height-wise
	std::vector<CalibMeasure> measurements; // the measurements of the grid from the camera

	// the final result
	std::vector<CalibResult> mesh; // world point mesh, sorted in row-major grid order

public:
	ProjectorData() { }

	bool operator==(const ProjectorData& other) const;

	// supports yaml, yaml.gz, xml, xml.gz
	bool writeXML(const std::string& filename) const;
	bool readXML(const std::string& filename);
	// obj file writing
	bool writeMesh(const std::string& filename) const;
	bool writeMesh(const std::string& filename, 
		int left, int top, int width, int height) const;

	READ_WRITE_DECL(ProjectorData);
};

class ScreenData {
// Configuration
public:
	ScanConfig scanCfg;

// RawData
public:
	int screenWidth; // pixels
	int screenHeight; // pixels
	std::vector<RawProjData> rawProj;
	std::vector<RawDataPiece> rawIntrinsic;
	std::vector<RawDataPiece> rawExtrinsic;

// ComputedData
public:
	cv::Mat compIntrinsic;
	cv::Mat compDistortion;
	std::vector<ProjectorData> compProj;

public:
	ScreenData() { }

	bool operator==(const ScreenData& other) const;

	bool writeXML(const std::string& filename) const;
	bool readXML(const std::string& filename);

	// obj file writing
	// bool writeMesh(std::string filename) const; // TODO

	READ_WRITE_DECL(ScreenData);
};

// fix for stupid OpenCV missing functions
// TODO: next opencv version (> 2.4.3) will fix this I think, and support reading and
// writing vectors which will make the CalibData code even cleaner
// if it doesn't, submit this as a patch (if I can get it to work)
//template<typename _Tp> inline void read(const cv::FileNode& node, cv::Point_<_Tp>& value, const cv::Point_<_Tp>& default_value = cv::Point_<_Tp>()) {
//	if (node.empty() || !node.isSeq()) {
//		value = default_value;
//	} else {
//		auto it = node.begin();
//		value.x = (_Tp)*it++;
//		value.y = (_Tp)*it;
//	}
//}
//template<typename _Tp> inline void read(const cv::FileNode& node, cv::Point3_<_Tp>& value, const cv::Point3_<_Tp>& default_value = cv::Point3_<_Tp>()) {
//	if (node.empty() || !node.isSeq()) {
//		value = default_value;
//	} else {
//		auto it = node.begin();
//		value.x = (_Tp)*it++;
//		value.y = (_Tp)*it++;
//		value.z = (_Tp)*it;
//	}
//}
//// explicit instantiation
//template void read<float>(const cv::FileNode& node, cv::Point3f& value, const cv::Point3f& default_value);