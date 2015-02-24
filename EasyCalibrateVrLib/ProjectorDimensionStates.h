#pragma once

#include "AppState.h"

class GridState : public IAppState {
protected:
	G3D::RealTime dt;
	RawProjData *p;

public:
	GridState(const std::shared_ptr<ProcessData>& pd);
	virtual ~GridState() { }

	virtual Ptr onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) { 
		dt = rdt; 
		return stayInState(); 
	}
	// FIXME: do this all in the onPose instead of the onGraphics?
	virtual Ptr onGraphics2D(G3D::RenderDevice* rd,
		                     G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D);

	virtual Ptr onUserInput(G3D::UserInput* ui);

protected:
	Ptr nextState();
};