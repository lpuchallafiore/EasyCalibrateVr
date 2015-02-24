#pragma once

// this is the core of the calibration algorithm
// following steps in order to calibrate screen

// 1 - calibrate camera intrinsics (use chessboard)
// 2 - calibrate camera extrinsics (use vicon tracked chessboard)
// for each projector (1 at a time):
//    3 - calibrate projector areas and calibration grid
//    4 - project calibration grid and get camera images
//        process each image as it comes in to get lists of CalibMeasures
//        save measurements to file (in case of crash)
//    5 - compute 3D model and show progress bar / std error / etc
//        save results and mesh to output files
//        if last projector, save entire screen data to file and mesh

#include <G3D/G3D.h>
#include <GLG3D/GLG3D.h>

#include "CalibData.h"
//#include "ProcessData.h"
class ProcessData;

// FIXME: not really an interface since it implements stuff
class IAppState {
protected:
	std::shared_ptr<ProcessData> PD;

public:
	typedef std::shared_ptr<IAppState> Ptr;
	IAppState(const std::shared_ptr<ProcessData>& pd) { PD = pd; }
	virtual ~IAppState() { }

	virtual Ptr onAI() { return stayInState(); }
	virtual Ptr onNetwork() { return stayInState(); }
	virtual Ptr onSimulation(G3D::RealTime rdt, 
		                     G3D::SimTime sdt,
		                     G3D::SimTime idt) { return stayInState(); }
	virtual Ptr onPose(G3D::Array<G3D::Surface::Ref>& posed3D,
		               G3D::Array<G3D::Surface2D::Ref>& posed2D) { return stayInState(); }

	virtual Ptr onGraphics3D(G3D::RenderDevice* rd,
		                     G3D::Array<G3D::Surface::Ref>& posed3D) { return stayInState(); }
	virtual Ptr onGraphics2D(G3D::RenderDevice* rd,
		                     G3D::Array<G3D::Surface2D::Ref>& posed2D) { return stayInState(); }

	virtual Ptr onUserInput(G3D::UserInput* ui) { return stayInState(); }

protected:
	Ptr stayInState() { return Ptr(nullptr); }
};