#pragma once

#include "AppState.h"

class CameraExtrinsicsState : public IAppState {
protected:
	G3D::RealTime dt;
	G3D::RealTime tt;
	cv::Mat image, drawImage;

public:
	CameraExtrinsicsState(const std::shared_ptr<ProcessData>& pd);
	virtual ~CameraExtrinsicsState() { }

	virtual Ptr onNetwork();
	virtual Ptr onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) {
		dt = rdt; 
		tt += dt; 
		return stayInState(); 
	}
	virtual Ptr onGraphics3D(G3D::RenderDevice* rd,
		                     G3D::Array<std::shared_ptr<G3D::Surface> >& posed3D);
	virtual Ptr onUserInput(G3D::UserInput* ui);

protected:
	Ptr nextState();
};