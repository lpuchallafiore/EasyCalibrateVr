#pragma once

#define EXCLUDE_HIBALL 1

#include "ScreenMesh.h"

class Hiball;
class HiballManipulator;

class Vicon;
class ViconManipulator;

class RenderApp : public G3D::GApp {
public:
	enum TrackingType_t {
		MANUAL_TRACKING,
		HIBALL_TRACKING,
		VICON_TRACKING
	};

	RenderApp(const GApp::Settings& settings = GApp::Settings());

	virtual void onInit() override;
	
	virtual void onGraphics(G3D::RenderDevice* rd, 
		G3D::Array<G3D::Surface::Ref>& surface, 
		G3D::Array<G3D::Surface2D::Ref>& surface2D) override;

	virtual void onUserInput(G3D::UserInput* ui) override;
	virtual void onNetwork() override;

protected:
	G3D::ShadowMap::Ref      m_shadowMap;
	G3D::Lighting::Ref		m_lighting;

	ScreenMesh mesh_;

	G3D::ArticulatedModel::Ref walter107;
	G3D::ArticulatedModel::Ref walter107Mirrored;

	G3D::Texture::Ref tex_;
	G3D::Texture::Ref depthTex_;
	G3D::Framebuffer::Ref fb_;

	// VRG3D cameras
	std::vector<VRG3D::ProjectionVRCamera::Ref> cameras_;

	G3D::CoordinateFrame headFrame_;

	// CFs for Hiball
#if EXCLUDE_HIBALL == 0
	G3D::CoordinateFrame midpointToHiballHead;
	G3D::CoordinateFrame headToHiball;
	G3D::CoordinateFrame roomToHiball;
	G3D::CoordinateFrame hiballToRoom;
	Hiball * hiball;
	HiballManipulator * hiballHeadManipulator;
#endif

	// CFs for Vicon
	// frames, measured with a tapemeasure
	G3D::CoordinateFrame roomToVicon;
	G3D::CoordinateFrame viconToRoom;
	// frames, measured by the vicon
	G3D::CoordinateFrame glassesRootToVicon;
	// frames, calibrated by placing at origin and pressing button
	G3D::CoordinateFrame glassesToGlassesRoot;
	Vicon *vicon;
	ViconManipulator *viconGlassesManipulator;

	enum DrawMode_t {
		SPHERES = 0,
		REAL_MESH,
		STRAIGHT_MESH,
		BOX_ROOM,
		MIRROR_ROOM,
		NUM_DRAW_MODES
	} drawMode_;
	int draw2DMesh;

	static const int texWidth_ = 1280;
	static const int texHeight_ = 1024;

	double lastTime;
	double dt;

	std::vector<cv::Mat> grayCodes;
	std::vector<G3D::Texture::Ref> grayCodeTextures;
	int currentCode;

	bool showMapGUI;

	int debugGridVert;

protected:
	void loadScreenMesh();
	void loadScene();
	void setupGUI();
	void preprocessMesh();
	void makeTextures();
	void makeCameras();
	void drawMapLine(G3D::RenderDevice * rd, G3D::Rect2D map, G3D::Vector3 p1, 
		G3D::Vector3 p2, G3D::Color3 color = G3D::Color3::yellow());
	void drawMapPoint(G3D::RenderDevice * rd, G3D::Rect2D map, 
		G3D::Vector3 pt, G3D::Color3 color);
	void drawPoly2D(G3D::Array<G3D::Vector2> screen, G3D::RenderDevice * rd, 
		G3D::Array<G3D::Vector2> tex);
	G3D::Vector2 projPoint(VRG3D::ProjectionVRCamera::Ref& cam, 
		cv::Point3d pt, int eye);
	G3D::Vector2 world2map(G3D::Rect2D map, G3D::Vector3 w);
};

#define ROOM_MAX_X 10
#define ROOM_MAX_Y 10
#define WALK_SPEED 1

#define DBOUT( s ) {                        \
   std::ostringstream os_;                  \
   os_ << s;                                \
   OutputDebugString( os_.str().c_str() );  \
}