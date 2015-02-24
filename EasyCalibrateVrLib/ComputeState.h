#pragma once

#include "AppState.h"
#include "Triangulate.h"
#include "Calibration.h"
#include "MeasureState.h"

// bundle adjust libraries
// TODO / FIXME: use cmake to manage dependencies?
//#define HAS_CVSBA
#ifdef HAS_CVSBA
	#include <cvsba.h>

	#pragma comment(lib, "blas.lib")
	#pragma comment(lib, "lapack.lib")
	#pragma comment(lib, "libf2c.lib")
	#pragma comment(lib, "cvsba.lib")
#endif

//#define HAS_SSBA
#ifdef HAS_SSBA
	#define V3DLIB_ENABLE_SUITESPARSE

	#include <Math/v3d_linear.h>
	#include <Base/v3d_vrmlio.h>
	#include <Geometry/v3d_metricbundle.h>

	#pragma comment(lib, "V3D.lib")
	#pragma comment(lib, "COLAMD.lib")
#endif

class ComputeState : public IAppState {
protected:
	// window
	float left, top, width, height;

	cv::Mat image;
	bool showImage;

	float intrinsicPercent;
	int intrinsicCounter;
	MonoCalibration calibrator;

	float extrinsicPercent;
	int extrinsicCounter;
	std::vector<G3D::CoordinateFrame> extrinsicRT;

	float measurePercent;
	int measureCounterProj;
	int measureCounter;
	std::vector<std::vector<CalibMeasure>> tempMeasures;
	float measureAvgPercent;
	int measureAvgCounterProj;
	int measureAvgCounterX;
	int measureAvgCounterY;

	float rayPercent;
	int rayCounterProj;
	int rayCounter;
	Triangulate tri;

	float pointPercent;
	int pointCounterProj, pointCounterX, pointCounterY;

	float bundlePercent;

	bool dataSaved;

	enum {
		INTRINSIC,
		EXTRINSIC,
		EXTRINSIC_AVG, // TODO average all extrinsic estimates into single estimate
		MEASUREMENT, // get measurements from image
		MEASUREMENT_AVG, // average grouped measurements into single measures
		TORAYS,
		TOPOINTS,
		BUNDLEADJUST, // refine points (and cam locations) using bundle adjustment
		DONE,
		REALLY_DONE // bad naming, this exits the worker thread
	} state;

public:
	ComputeState(float left_, float top_, float width_, float height_, 
		const std::shared_ptr<ProcessData>& pd);
	virtual ~ComputeState() { }

	// IAppState
	virtual Ptr onGraphics2D(G3D::RenderDevice *rd,
		                     G3D::Array<G3D::Surface2D::Ref>& posed2D) override;

	// returns valid group ids for a grid point
	static std::vector<int> findGroups(const std::vector<CalibMeasure>& m, int x, int y);
	// computes average measurement for a grid point and group id
	static CalibMeasure computeAvgMeasure(const std::vector<CalibMeasure>& m, int x, int y, int id);

	static std::vector<CalibMeasure> gatherMeasurements(int gx, int gy, int gid, 
		const std::vector<CalibMeasure>& m);
	static std::vector<CalibMeasure> gatherMeasurements(int gx, int gy, 
		const std::vector<CalibMeasure>& m);

	static std::vector<int> getGroups(const std::vector<CalibMeasure>& m);

	// for sorting the mesh (row major, left to right top to bottom)
	static bool cmpMesh(CalibResult& a, CalibResult& b);

protected:
	Ptr nextState();

	void intrinsicController();
	void extrinsicController();
	void measureController();
	void measureAvgController();
	void rayController();
	void pointController();
	void bundleController();
	void doneController();
	void SSBABundle();

	G3D::CoordinateFrame ComputeState::averageCF(const std::vector<G3D::CoordinateFrame>& exs);

	void drawBar(G3D::RenderDevice* rd, std::string name, float percent,
		float x, float y, float w, float h, G3D::Color3 foreColor, 
		G3D::Color3 backColor = G3D::Color3::gray());

	G3D::CoordinateFrame toCF(const cv::Mat& rot, const cv::Point3f& t) const;

protected:
	static DWORD WINAPI t_Worker(LPVOID lpParam);
	CRITICAL_SECTION csImage;
};