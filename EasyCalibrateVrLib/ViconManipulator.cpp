#include "stdafx.h"
#include "ViconManipulator.h"

ViconManipulator::ViconManipulator(Vicon* vicon, const std::string& subject,
								   const std::string& segment) :
vicon(vicon),
subject(subject),
segment(segment)
{
}

ViconManipulator::~ViconManipulator(void)
{
}

G3D::CoordinateFrame ViconManipulator::frame() const
{
	return mframe;
}

void ViconManipulator::getFrame(G3D::CoordinateFrame& c) const
{
	c = mframe;
}

void ViconManipulator::onNetwork()
{
	// assume hiball has been updated
	// get coordinate frame of the sensor
	vicon->getPosition(subject, segment, mframe.translation);
	G3D::Quat q;
	vicon->getRotation(subject, segment, q);
	mframe.rotation = G3D::Matrix3(q);
}
