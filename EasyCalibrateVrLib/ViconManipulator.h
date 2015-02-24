#pragma once

#include <G3D/G3D.h>
#include <GLG3D/GLG3D.h>
#include "ViconDataStreamSDK.h"
#include "Vicon.h"

class ViconManipulator :
	public G3D::Manipulator
{
public:
	ViconManipulator(Vicon* vicon, const std::string& subject, const std::string& segment);
	~ViconManipulator(void);

	G3D::CoordinateFrame frame() const;
	void getFrame(G3D::CoordinateFrame& c) const;

	// this is our signal to poll the network
	virtual void onNetwork();

	const Vicon* getVicon() const { return vicon; }
	const std::string& getSubject() const { return subject; }
	const std::string& getSegment() const { return segment; }

protected:
	Vicon* vicon;
	std::string subject;
	std::string segment;

	G3D::CoordinateFrame mframe;
};
