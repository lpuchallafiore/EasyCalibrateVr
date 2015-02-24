#pragma once

#include "AppState.h"

class MeasureState : public IAppState {
protected:
	RawProjData * p;
	int X, Y;
	G3D::RealTime dt;
	G3D::RealTime tt;
	enum {
		MOVING_CAMERA,
		TAKE_MEASURE,
		DONE
	} state;

	cv::Mat image;
	cv::Mat imageRaw;

	int currGroup;
	int groupIndex;

public:
	static std::vector<cv::Point2f> findGrid(cv::Mat& img, int gridW, 
											 int gridH, bool& found);

public:
	MeasureState(const std::shared_ptr<ProcessData>& pd);
	virtual ~MeasureState() { }

	virtual Ptr onNetwork();
	virtual Ptr onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) { 
		dt = rdt; 
		tt += dt; 
		return stayInState(); 
	}
	virtual Ptr onGraphics2D(G3D::RenderDevice* rd,
		                     G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D);
	virtual Ptr onUserInput(G3D::UserInput* ui);

protected:
	Ptr nextState();
	int getViewCount(int x, int y) const;
};