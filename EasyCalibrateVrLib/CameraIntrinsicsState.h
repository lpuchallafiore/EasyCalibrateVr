#pragma once

#include "AppState.h"
#include "Calibration.h"

class CameraIntrinsicsState : public IAppState {
protected:
	MonoCalibration calibrator;
	G3D::RealTime dt;
	G3D::RealTime tt;
	cv::Mat image;
	// just a queue; using deque because it supports iterators
	std::deque<std::string> txtQ;
	int done;
	int frameNum;

	enum {
		CALIB_LOOP = 0,
		DONE
	} state;

public:
	CameraIntrinsicsState(const std::shared_ptr<ProcessData>& pd);
	virtual ~CameraIntrinsicsState() { }

	virtual Ptr onNetwork();
	virtual Ptr onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) { 
		dt = rdt; tt += dt; return stayInState(); 
	}
	virtual Ptr onGraphics2D(G3D::RenderDevice* rd,
		                     G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D);

	void print(std::string str, bool k = false) {
		if (k) txtQ.pop_back();
		txtQ.push_back(str);
		if (txtQ.size() > 10) txtQ.pop_front();
	}

protected:
	Ptr nextState();
};