#pragma once

#include <G3D/G3D.h>

#include "ViconDataStreamSDK.h"
using namespace ViconDataStreamSDK::CPP;

class Vicon
{
public:
	Vicon(const std::string& server);
	~Vicon(void);

	bool isValid() const { return valid; }
	void update();

	void getPosition(const std::string& subject, const std::string& segment, G3D::Vector3& v);
	void getRotation(const std::string& subject, const std::string& segment, G3D::Quat& q);

private:
	bool valid;
	Client tracker;
};
