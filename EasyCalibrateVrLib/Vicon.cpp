#include "stdafx.h"
#include "Vicon.h"

namespace
{
  std::string Adapt( const bool i_Value )
  {
	return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
	switch( i_Direction )
	{
	  case Direction::Forward:
		return "Forward";
	  case Direction::Backward:
		return "Backward";
	  case Direction::Left:
		return "Left";
	  case Direction::Right:
		return "Right";
	  case Direction::Up:
		return "Up";
	  case Direction::Down:
		return "Down";
	  default:
		return "Unknown";
	}
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
	switch( i_DeviceType )
	{
	  case DeviceType::ForcePlate:
		return "ForcePlate";
	  case DeviceType::Unknown:
	  default:
		return "Unknown";
	}
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
	switch( i_Unit )
	{
	  case Unit::Meter:
		return "Meter";
	  case Unit::Volt:
		return "Volt";
	  case Unit::NewtonMeter:
		return "NewtonMeter";
	  case Unit::Newton:
		return "Newton";
	  case Unit::Unknown:
	  default:
		return "Unknown";
	}
  }

  std::string Adapt( const Result::Enum res )
  {
	switch( res )
	{
	default:
	case Result::Unknown:
		return "Unknown";
	case Result::NotImplemented:
		return "NotImplemented";
	case Result::Success:
		return "Success";
	case Result::InvalidHostName:
		return "InvalidHostName";
	case Result::InvalidMulticastIP:
		return "InvalidMulticastIP";
	case Result::ClientAlreadyConnected:
		return "ClientAlreadyConnected";
	case Result::ClientConnectionFailed:
		return "ClientConnectionFailed";
	case Result::ServerAlreadyTransmittingMulticast:
		return "ServerAlreadyTransmittingMulticast";
	case Result::ServerNotTransmittingMulticast:
		return "ServerNotTransmittingMulticast";
	case Result::NotConnected:
		return "NotConnected";
	case Result::NoFrame:
		return "NoFrame";
	case Result::InvalidIndex:
		return "InvalidIndex";
	case Result::InvalidSubjectName:
		return "InvalidSubjectName";
	case Result::InvalidSegmentName:
		return "InvalidSegmentName";
	case Result::InvalidMarkerName:
		return "InvalidMarkerName";
	case Result::InvalidDeviceName:
		return "InvalidDeviceName";
	case Result::InvalidDeviceOutputName:
		return "InvalidDeviceOutputName";
	case Result::InvalidLatencySampleName:
		return "InvalidLatencySampleName";
	case Result::CoLinearAxes:
		return "CoLinearAxes";
	case Result::LeftHandedAxes:
		return "LeftHandedAxes";
	}
  }
}

Vicon::Vicon(const std::string& HostName)
{
	valid = false;

  // Connect to a server
  //std::cout << "Connecting to " << HostName << " ..." << std::flush;
  G3D::logPrintf("Connecting to %s ...\n", HostName.c_str());
  while( !tracker.IsConnected().Connected )
  {
	// Direct connection
	tracker.Connect( HostName );

	// Multicast connection
	// tracker.ConnectToMulticast( HostName, "224.0.0.0" );

	//std::cout << ".";
#ifdef WIN32
	Sleep( 200 );
#else
	sleep(1);
#endif
  }
  //std::cout << std::endl;

  // Enable some different data types
  tracker.EnableSegmentData();
  tracker.EnableMarkerData();
  tracker.EnableUnlabeledMarkerData();
  tracker.EnableDeviceData();

  //std::cout << "Segment Data Enabled: "          << Adapt( tracker.IsSegmentDataEnabled().Enabled )         << std::endl;
  //std::cout << "Marker Data Enabled: "           << Adapt( tracker.IsMarkerDataEnabled().Enabled )          << std::endl;
  //std::cout << "Unlabeled Marker Data Enabled: " << Adapt( tracker.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
  //std::cout << "Device Data Enabled: "           << Adapt( tracker.IsDeviceDataEnabled().Enabled )          << std::endl;
  G3D::logPrintf("Segment Data Enabled: %s\n", Adapt( tracker.IsSegmentDataEnabled().Enabled ).c_str());
  G3D::logPrintf("Marker Data Enabled: %s\n", Adapt( tracker.IsMarkerDataEnabled().Enabled ).c_str());
  G3D::logPrintf("Unlabeled Marker Data Enabled: %s\n", Adapt( tracker.IsUnlabeledMarkerDataEnabled().Enabled ).c_str());
  G3D::logPrintf("Device Data Enabled: %s\n", Adapt( tracker.IsDeviceDataEnabled().Enabled ).c_str());

  // Set the streaming mode
  tracker.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
  // tracker.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
  // tracker.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

  // Set the global up axis
  tracker.SetAxisMapping( Direction::Forward, 
						   Direction::Left, 
						   Direction::Up ); // Z-up
  // tracker.SetGlobalUpAxis( Direction::Forward, 
  //                           Direction::Up, 
  //                           Direction::Right ); // Y-up

  Output_GetAxisMapping _Output_GetAxisMapping = tracker.GetAxisMapping();
  //std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
  //                       << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
  //                       << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;
  G3D::logPrintf("Axis Mapping: X-%s Y-%s  Z-%s\n", Adapt( _Output_GetAxisMapping.XAxis ).c_str(), 
						 Adapt( _Output_GetAxisMapping.YAxis ).c_str(),
						 Adapt( _Output_GetAxisMapping.ZAxis ).c_str());

  // Discover the version number
  Output_GetVersion _Output_GetVersion = tracker.GetVersion();
  //std::cout << "Version: " << _Output_GetVersion.Major << "." 
  //                         << _Output_GetVersion.Minor << "." 
  //                         << _Output_GetVersion.Point << std::endl;
  G3D::logPrintf("Version: %d.%d.%d\n", _Output_GetVersion.Major,
	  _Output_GetVersion.Minor, _Output_GetVersion.Point);

  valid = true;
}

Vicon::~Vicon(void)
{
	tracker.Disconnect();
}

void Vicon::update()
{
	using namespace G3D;
	// TODO we should put this in a separate thread
	Result::Enum res = tracker.GetFrame().Result;
	while(res != Result::Success )
	{
		G3D::logPrintf("GetFrame(): %s\n", Adapt(res).c_str());

	  // Sleep a little so that we don't lumber the CPU with a busy poll
	  #ifdef WIN32
		Sleep( 200 );
	  #else
		sleep(1);
	  #endif

		res = tracker.GetFrame().Result;
	}
}

void Vicon::getPosition(const std::string& subject, const std::string& segment, G3D::Vector3& v)
{
	using namespace G3D;
	if (!valid)
		return;

	Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
	  tracker.GetSegmentGlobalTranslation( subject, segment );
	if (_Output_GetSegmentGlobalTranslation.Result==Result::Success) {
		v[0] = _Output_GetSegmentGlobalTranslation.Translation[0]/1000.0;
		v[1] = _Output_GetSegmentGlobalTranslation.Translation[1]/1000.0;
		v[2] = _Output_GetSegmentGlobalTranslation.Translation[2]/1000.0;
		//G3D::logPrintf("%f, %f, %f\n", v[0], v[1], v[2]);
	} else {
		G3D::logPrintf("Failure in getPosition(%s, %s): %s\n",
			subject.c_str(), segment.c_str(),
			Adapt(_Output_GetSegmentGlobalTranslation.Result).c_str());
		valid = false;
	}
}

void Vicon::getRotation(const std::string& subject, const std::string& segment, G3D::Quat& q)
{
	using namespace G3D;
	if (!valid)
		return;

	// Get the global segment rotation in quaternion co-ordinates
	Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
	  tracker.GetSegmentGlobalRotationQuaternion( subject, segment );
	if (_Output_GetSegmentGlobalRotationQuaternion.Result==Result::Success) {
		q[0] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
		q[1] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
		q[2] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
		q[3] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
		//G3D::logPrintf("%f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
	} else {
		G3D::logPrintf("Failure in getRotation(%s, %s): %s\n",
			subject.c_str(), segment.c_str(),
			Adapt(_Output_GetSegmentGlobalRotationQuaternion.Result).c_str());
		valid = false;
	}
}
