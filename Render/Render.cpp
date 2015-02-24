#include "stdafx.h"
#include "Render.h"

#if EXCLUDE_HIBALL == 0
#include "Hiball.h"
#include "HiballManipulator.h"
#endif

#include "Vicon.h"
#include "ViconManipulator.h"

#include "GrayCodeState.h"

/** TODO
 * - STEREO
 * - BLENDING BETWEEN PROJECTORS
 **/

using namespace VRG3D;
using namespace G3D;
using namespace std;

// online
#define FULLSCREEN true
#define ELEM_WIDTH 1280
#define ELEM_HEIGHT 1024
//#define TRACKING_TYPE RenderApp::VICON_TRACKING
//#define STEREO true

// offline
//#define FULLSCREEN false
//#define ELEM_WIDTH 800
//#define ELEM_HEIGHT 600
#define TRACKING_TYPE RenderApp::MANUAL_TRACKING
#define STEREO false

#define MESH_FILE "screen-mesh-size-4-20140508.ply"

double HACK = 0.0; //0.006;
double HACK2 = 0.0; //-0.037;
// two-d hack / three-d hack 
double TDHX[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //0.28, 0.00, 0.00};
double TDHY[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //0.14, 0.00, 0.00};
double TDHZ[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //0.00, 0.17, 0.04};
int activeTDH = 3;

cv::Point3d getTDHOff(int pid) {
	return cv::Point3d(TDHX[pid+3], TDHY[pid+3], TDHZ[pid+3]);
}

// Tells C++ to invoke command-line main() function even on OS X and Win32
G3D_START_AT_MAIN();

int main(int argc, const char* argv[]) {
	if (FileSystem::exists("..\\data-files", false)) {
		chdir("..\\data-files");
	}

	GApp::Settings settings;

	settings.window.width = 3 * ELEM_WIDTH;
	settings.window.height = ELEM_HEIGHT;
	settings.window.fullScreen = FULLSCREEN;
	settings.window.framed = !FULLSCREEN;
	settings.window.stereo = STEREO;
	settings.window.caption = "VR Window Render";
	settings.film.enabled = false;
	settings.window.asynchronous = false;
	settings.debugFontName = "font/console.fnt"; // load a less crappy font

	return RenderApp(settings).run();
}

RenderApp::RenderApp(const GApp::Settings& settings) : GApp(settings) {
	draw2DMesh = 0;
	drawMode_ = MIRROR_ROOM;
	currentCode = 0;

	// create gray codes
	int n_rows, n_cols, col_shift, row_shift;
	GrayCodeState::generateGrayCodes(1280 / 8, 
		1024 / 8, 
		grayCodes, n_cols, n_rows, col_shift, row_shift, true, true);

	// convert gray codes to OpenGL textures
	for (int i = 0; i < grayCodes.size(); ++i) {
		if (i != 2*n_cols - 1 && i != grayCodes.size() - 1) continue;
		std::stringstream ss;
		ss << "grayCode" << i;
		Texture::Ref tex = Texture::fromMemory(ss.str(),
			(const void *)grayCodes[i].data,
			ImageFormat::L8(),
			grayCodes[i].size().width,
			grayCodes[i].size().height, 1);
		grayCodeTextures.push_back(tex);
		cout << "added gray code " << ss.str() << endl;

		ss << ".png";
		cv::imwrite(ss.str(), grayCodes[i]);
	}

	debugGridVert = 0;
}

// for loading models stored with y pointing up\
// make z point and keep things right-handed
const CoordinateFrame yzSwap(
		Matrix3(
			-1, 0, 0,
			 0, 0, 1,
			 0, 1, 0)
		);
void RenderApp::loadScene() {
	m_shadowMap = ShadowMap::create();
	m_lighting = Lighting::create();
	m_lighting->ambientBottom = m_lighting->ambientTop = Color3::white();

	walter107Mirrored = ArticulatedModel::fromFile("Walter107_Mirrored/Walter107_xyz_mirrored.3ds", yzSwap);
	walter107 = ArticulatedModel::fromFile("Walter107/Walter107_xyz.3ds", yzSwap);
}

void RenderApp::setupGUI() {
	debugWindow->setVisible(false);
	developerWindow->cameraControlWindow->setVisible(false);
	developerWindow->setVisible(false);
	showRenderingStats = false;

	// checkbox to enable / disable warping
}

void RenderApp::onInit() {
	showMapGUI = false;
	mesh_.readPLY(MESH_FILE);

	// load walter107 mirrored mesh
	loadScene();

	// turn off 2d mesh
	draw2DMesh = 3;

	// setup debug & diagnostic GUI
	setupGUI();

	headFrame_ = CoordinateFrame(Matrix3::identity(),
		Vector3(3.5, 4.5, 1.75));

	// make offscreen textures
	makeTextures();

	// make cameras
	makeCameras();

	lastTime = realTime();

	// setup tracking
	if (TRACKING_TYPE == MANUAL_TRACKING) {
	} else if (TRACKING_TYPE == HIBALL_TRACKING) {
#if EXCLUDE_HIBALL == 0
		hiball = new Hiball();
		if (hiball->valid) {
			hiballHeadManipulator = new HiballManipulator(hiball, 1);
		} else {
			delete hiball;
			hiball = NULL;
		}

		midpointToHiballHead = CoordinateFrame(
			Matrix3(0.0, 0.0, 1.0,
					1.0, 0.0, 0.0,
					0.0, 1.0, 0.0),
			Vector3(-0.2921f, 0.0f, -0.0435f)
			);
		roomToHiball = CoordinateFrame(
			Matrix3(-1.0, 0.0, 0.0,
					0.0, -1.0, 0.0,
					0.0, 0.0, 1.0),
			Vector3(7.0040, 9.1250, 0.0)
			);
		hiballToRoom = roomToHiball.inverse();

		// initialize camera position
		headToHiball = CoordinateFrame(Matrix3::identity(), Vector3(3.0, 4.0, 1.5));
		headFrame_ = hiballToRoom * headToHiball * midpointToHiballHead;
#endif
	} else if (TRACKING_TYPE == VICON_TRACKING) {
		vicon = new Vicon("160.94.77.67"); //"vicon");
		viconGlassesManipulator = new ViconManipulator(vicon, "Glasses", "Glasses");

		// measured with tapemeasure
		viconToRoom = CoordinateFrame(
			Matrix3::identity(),
			Vector3(3.048, 4.9784, 0.0)
		);
		roomToVicon = viconToRoom.inverse();

		// measured by placing on floor
		glassesToGlassesRoot = CoordinateFrame(Matrix3::identity(), Vector3(0, 0, 0));
	}
}

void RenderApp::onNetwork() {
	if (TRACKING_TYPE == HIBALL_TRACKING) {
#if EXCLUDE_HIBALL == 0
		if (hiball) hiball->update();

		if (hiballHeadManipulator) {
			hiballHeadManipulator->onNetwork();
			hiballHeadManipulator->getFrame(headToHiball);
			headFrame_ = hiballToRoom * headToHiball * midpointToHiballHead;
		}
#endif
	} else if (TRACKING_TYPE == VICON_TRACKING) {
			if (vicon) vicon->update();

		if (viconGlassesManipulator) {
			viconGlassesManipulator->onNetwork();
			viconGlassesManipulator->getFrame(glassesRootToVicon);
			headFrame_ = viconToRoom * glassesRootToVicon * glassesToGlassesRoot;
		}
	}
}

void RenderApp::makeTextures() {
	Texture::Settings s = Texture::Settings::defaults();
	s.wrapMode = WrapMode::ZERO;

	tex_ = Texture::createEmpty("buffer", texWidth_, texHeight_, ImageFormat::RGB8(), Texture::defaultDimension(), s);
	depthTex_ = Texture::createEmpty("depthtex", texWidth_, texHeight_, ImageFormat::DEPTH32F(), Texture::defaultDimension(), s);
	fb_ = Framebuffer::create("target");

	// attach textures
	fb_->set(Framebuffer::COLOR0, tex_);
	fb_->set(Framebuffer::DEPTH, depthTex_);
}

// FIXME: this is very rudimentary and ad-hoc
// and very very specific to our screen setup
void RenderApp::makeCameras() {
	float delta = 0.50f;
	float delta2 = 1.0f;
	DisplayTile::TileRenderType renderType = DisplayTile::TILE_MONO;

	for (int i = 0; i < 3; i++) {
		ScreenMesh::Projector& p = mesh_.projectors[i];

		// init
		cv::Point3d tL = p.vertices[0].worldPt;
		cv::Point3d tR = p.vertices[0].worldPt;
		cv::Point3d bL = p.vertices[0].worldPt;
		cv::Point3d bR = p.vertices[0].worldPt;

		for (auto it = p.vertices.begin(); it != p.vertices.end(); ++it) {
			// check for top left
			if (it->worldPt.y > tL.y) {
				if (i == 1) tL.x = it->worldPt.x;
				tL.y = it->worldPt.y;
			}
			if (it->worldPt.z > tL.z) {
				if (i == 1) tL.x = it->worldPt.x;
				tL.z = it->worldPt.z;
			}
			if (i == 0) {
				if (it->worldPt.x < tL.x) {
					tL.x = it->worldPt.x;
				}
			} else if (i == 2) {
				if (it->worldPt.x > tL.x) {
					tL.x = it->worldPt.x;
				}
			}

			// check for top right
			if (it->worldPt.y < tR.y) {
				if (i == 1) tR.x = it->worldPt.x;
				tR.y = it->worldPt.y;
			}
			if (it->worldPt.z > tR.z) {
				if (i == 1) tR.x = it->worldPt.x;
				tR.z = it->worldPt.z;
			}
			if (i == 0) {
				if (it->worldPt.x > tR.x) {
					tR.x = it->worldPt.x;
				}
			} else if (i == 2) {
				if (it->worldPt.x < tR.x) {
					tR.x = it->worldPt.x;
				}
			}

			// check for bot left
			if (it->worldPt.y > bL.y) {
				if (i == 1) bL.x = it->worldPt.x;
				bL.y = it->worldPt.y;
			}
			if (it->worldPt.z < bL.z) {
				if (i == 1) bL.x = it->worldPt.x;
				bL.z = it->worldPt.z;
			}
			if (i == 0) {
				if (it->worldPt.x < bL.x) {
					bL.x = it->worldPt.x;
				}
			} else if (i == 2) {
				if (it->worldPt.x > bL.x) {
					bL.x = it->worldPt.x;
				}
			}

			// check for bot right
			if (it->worldPt.y < bR.y) {
				if (i == 1) bR.x = it->worldPt.x;
				bR.y = it->worldPt.y;
			}
			if (it->worldPt.z < bR.z) {
				if (i == 1) bR.x = it->worldPt.x;
				bR.z = it->worldPt.z;
			}
			if (i == 0) {
				if (it->worldPt.x > bR.x) {
					bR.x = it->worldPt.x;
				}
			} else if (i == 2) {
				if (it->worldPt.x < bR.x) {
					bR.x = it->worldPt.x;
				}
			}
		}

		cout << "projector " << i << endl;
		cout << "  top left = " << tL << endl;
		cout << "  top right = " << tR << endl;
		cout << "  bot left = " << bL << endl;
		cout << "  bot right = " << bR << endl;

		Vector3 topLeft = Vector3(tL.x, tL.y, tL.z);
		Vector3 topRight = Vector3(tR.x, tR.y, tR.z);
		Vector3 botLeft = Vector3(bL.x, bL.y, bL.z);
		Vector3 botRight = Vector3(bR.x, bR.y, bR.z);

		// try and even some stuff up
		// bottom z
		float zz = std::min(botLeft.z, botRight.z) + delta * (botRight - topRight).unit().z;
		botLeft.z = zz; botRight.z = zz;
		// top z
		zz = std::max(topLeft.z, topRight.z) + delta * (topRight - botRight).unit().z;
		topLeft.z = zz; topRight.z = zz;
		// left x
		zz = std::min(topLeft.x, botLeft.x) + delta2 * (topLeft - topRight).unit().x;
		topLeft.x = zz; botLeft.x = zz;
		// left y
		zz = std::max(topLeft.y, botLeft.y) + delta2 * (topLeft - topRight).unit().y;
		topLeft.y = zz; botLeft.y = zz;
		// right x
		zz = std::max(topRight.x, botRight.x) + delta2 * (topRight - topLeft).unit().x;
		topRight.x = zz; botRight.x = zz;
		// right y
		zz = std::min(topRight.y, botRight.y) + delta2 * (topRight - topLeft).unit().y;
		topRight.y = zz; botRight.y = zz;

		double nearClip = 0.01;
		double farClip = 500.0;
		int vpX = 0;
		int vpY = 0;
		int vpW = 0;
		int vpH = 0;

		/*
		if (i == 1) {
			topLeft = Vector3(8.0, 7.0, 3.0);
			topRight = Vector3(8.0, 2.5, 3.0);
			botRight = Vector3(8.0, 2.5, 0.25);
			botLeft = Vector3(8.0, 7.0, 0.25);
		}
		*/

		DisplayTile tile = DisplayTile(topLeft, topRight, botLeft, botRight,
									   renderType, nearClip, farClip,
									   vpX, vpY, vpW, vpH);

		cameras_.push_back(make_shared<ProjectionVRCamera>(tile, headFrame_));
	}
}

void RenderApp::onGraphics(RenderDevice* rd, Array<Surface::Ref>& surface, Array<Surface2D::Ref>& surface2D) {
	const Rect2D& proj1 = Rect2D::xywh(0, 0, ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& proj1b = Rect2D::xywh(0, ELEM_HEIGHT*2, ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& proj2 = Rect2D::xywh(ELEM_WIDTH, 0, 
		ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& proj2b = Rect2D::xywh(ELEM_WIDTH, ELEM_HEIGHT*2, ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& proj3 = Rect2D::xywh(2*ELEM_WIDTH, 0, 
		ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& proj3b = Rect2D::xywh(2*ELEM_WIDTH, ELEM_HEIGHT*2, ELEM_WIDTH, ELEM_HEIGHT);
	const Rect2D& minimapTop = Rect2D::xywh(15, 15, 
		ELEM_WIDTH / 4, ELEM_HEIGHT / 4);
	const Rect2D& minimapTwoThirds = Rect2D::xywh(ELEM_WIDTH + 15, 
		15 + ELEM_HEIGHT, ELEM_WIDTH - 30, ELEM_HEIGHT - 30);

	const Sphere& s1 = Sphere(Vector3(8.1, 1.8, 1.75), 1);
	const Sphere& s2 = Sphere(Vector3(8.1, 4.5, 1.75), 1);
	const Sphere& s3 = Sphere(Vector3(8.1, 7.5, 1.75), 1);

	const auto c1 = G3D::AABox(Vector3(9.0, 5.0, 1.0) - Vector3(1, 2, .2), 
							   Vector3(9.0, 5.0, 1.0) + Vector3(1, 2, .2));

	float sm_x = 8.0;
	float sm_y_max = 8.5;
	float sm_y_min = 1.0;
	float sm_z_max = 2.0;
	float sm_z_min = 0.5;
	float sm_step = 0.5;

	if (STEREO) rd->setDrawBuffer(RenderDevice::DrawBuffer::DRAW_BACK_LEFT);

	for (int eye = 0; eye < (STEREO ? 2 : 1); eye++) {

		// clear the screen
		renderDevice->setColorClearValue(Color3::black());
		rd->clear();
		renderDevice->setColorClearValue(Color3::cyan());
	
		// for each projector
		for (int i = 0; i < 3; i++) {
			int cid = i; // debug variable
			// create camera which is centered at person_ and which has a view
			// frustrum that completely encloses the projector mesh

			// render offscreen texture (3D & 2D HUD)
			rd->pushState(fb_); {
				rd->clear();

				// setup camera
				cameras_[cid]->updateHeadFrame(headFrame_);
				if (!STEREO) {
					cameras_[cid]->applyProjection(rd, ProjectionVRCamera::Cyclops, &defaultCamera);
				} else {
					switch (eye) {
					case 0: // left
						cameras_[cid]->applyProjection(rd, ProjectionVRCamera::LeftEye, &defaultCamera);
						break;
					case 1: // right
						cameras_[cid]->applyProjection(rd, ProjectionVRCamera::RightEye, &defaultCamera);
						break;
					}
				}
				rd->setObjectToWorldMatrix(CoordinateFrame());

				// draw some stuff
				if (drawMode_ == SPHERES) {
					Draw::sphere(s1, rd, Color3::blue());
					Draw::sphere(s2, rd, Color3::green());
					Draw::sphere(s3, rd, Color3::red());
				} else if (drawMode_ == REAL_MESH && i == draw2DMesh) {
					//for (int j = 0; j < 3; ++j) {
					int j = draw2DMesh;
						for (auto tri = mesh_.projectors[j].triangles.begin(); 
							tri != mesh_.projectors[j].triangles.end();
							++tri) 
						{
							const cv::Point3d& a = mesh_.projectors[j].vertices[tri->vertIndices[0]].worldPt + getTDHOff(j);
							const cv::Point3d& b = mesh_.projectors[j].vertices[tri->vertIndices[1]].worldPt + getTDHOff(j);
							const cv::Point3d& c = mesh_.projectors[j].vertices[tri->vertIndices[2]].worldPt + getTDHOff(j);

							Vector3 av = Vector3(a.x, a.y, a.z);
							Vector3 bv = Vector3(b.x, b.y, b.z);
							Vector3 cv = Vector3(c.x, c.y, c.z);

							Color3 clr = Color3::red();
							Draw::lineSegment(LineSegment::fromTwoPoints(av, bv), rd, clr, 0.2);
							Draw::lineSegment(LineSegment::fromTwoPoints(bv, cv), rd, clr, 0.2);
							Draw::lineSegment(LineSegment::fromTwoPoints(cv, av), rd, clr, 0.2);
						}
					//}
				} else if (drawMode_ == STRAIGHT_MESH) {
					Draw::box(c1, rd, Color3::orange());
					/*
					// horizontal lines
					for (float z = sm_z_max; z >= sm_z_min; z-=sm_step) {
						Draw::lineSegment(LineSegment::fromTwoPoints(
							Point3(sm_x, sm_y_max, z), Point3(sm_x, sm_y_min, z)),
							rd, Color3::orange());
					}

					// vertical lines
					for (float y = sm_y_max; y >= sm_y_min; y-=sm_step) {
						Draw::lineSegment(LineSegment::fromTwoPoints(
							Point3(sm_x, y, sm_z_max), Point3(sm_x, y, sm_z_min)),
							rd, Color3::orange());
					}
					*/
				} else if (drawMode_ == BOX_ROOM) {
					// numbers measured with tape measure
					// floor
					Draw::plane(Plane(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(1.0, 1.0, 0.0)), rd, Color3::brown());
					// left wall
					// right wall
					// back wall
					// ceiling
					Draw::plane(Plane(Vector3(1.0, 0.0, 3.0), Vector3(0.0, 1.0, 0.0), Vector3(1.0, 1.0, 3.0)), rd, Color3::white());
				} else if (drawMode_ == MIRROR_ROOM) {
					walter107Mirrored->pose(surface);
					rd->setCullFace(RenderDevice::CULL_NONE);
					//rd->setRenderMode(RenderDevice::RenderMode::RENDER_WIREFRAME);
					Surface::sortAndRender(rd, defaultCamera, surface, m_lighting, m_shadowMap);
				}

				// draw some 2D stuff to sanity check
				/*
				rd->push2D(fb_->rect2DBounds()); {
					Draw::rect2D(Rect2D::xywh(0, 0, 64, 64), rd, Color3::white());
					Draw::rect2D(Rect2D::xywh(16, 16, 32, 32), rd, Color3::red());
				} rd->pop2D();
				*/
			} rd->popState();

			// render as a set of 2D textured polygons
			rd->push2D(); {
				rd->setTexture(0, tex_);
				for (auto tri = mesh_.projectors[i].triangles.begin(); 
					tri != mesh_.projectors[i].triangles.end();
					++tri) 
				{
					const ScreenMesh::ProjVert& pt1 = mesh_.projectors[i].vertices[tri->vertIndices[0]];
					const ScreenMesh::ProjVert& pt2 = mesh_.projectors[i].vertices[tri->vertIndices[1]];
					const ScreenMesh::ProjVert& pt3 = mesh_.projectors[i].vertices[tri->vertIndices[2]];

					Array<Vector2> screenpoly;
					screenpoly.append(Vector2(i * ELEM_WIDTH + (pt1.screenPt.x + TDHX[i]) * ELEM_WIDTH, (pt1.screenPt.y + TDHY[i]) * ELEM_HEIGHT));
					screenpoly.append(Vector2(i * ELEM_WIDTH + (pt2.screenPt.x + TDHX[i]) * ELEM_WIDTH, (pt2.screenPt.y + TDHY[i]) * ELEM_HEIGHT));
					screenpoly.append(Vector2(i * ELEM_WIDTH + (pt3.screenPt.x + TDHX[i]) * ELEM_WIDTH, (pt3.screenPt.y + TDHY[i]) * ELEM_HEIGHT));

					// compute warp
					Array<Vector2> texpoly;
					texpoly.append(projPoint(cameras_[cid], pt1.worldPt + getTDHOff(i), eye));
					texpoly.append(projPoint(cameras_[cid], pt2.worldPt + getTDHOff(i), eye));
					texpoly.append(projPoint(cameras_[cid], pt3.worldPt + getTDHOff(i), eye));

					if (i == 2) {
						for (auto v = texpoly.begin(); v != texpoly.end(); ++v) {
							v->x += HACK;
						}
					} else if (i == 0) {
						for (auto v = texpoly.begin(); v != texpoly.end(); ++v) {
							v->x += HACK2;
						}
					}

					drawPoly2D(screenpoly, rd, texpoly);
				}
				rd->setTexture(0, NULL);

				if (currentCode != 0) {
					rd->setTexture(0, grayCodeTextures[currentCode - 1]);
					Draw::rect2D(Rect2D::xywh(i * ELEM_WIDTH, 
											  0, 
											  ELEM_WIDTH, 
											  ELEM_HEIGHT),
											  rd, Color4(1, 1, 1, 0.5));
					rd->setTexture(0, nullptr);
				}

				if (draw2DMesh != 3) {
					for (auto tri = mesh_.projectors[draw2DMesh].triangles.begin(); 
						tri != mesh_.projectors[draw2DMesh].triangles.end();
						++tri) 
					{
						const ScreenMesh::ProjVert& pt1 = mesh_.projectors[draw2DMesh].vertices[tri->vertIndices[0]];
						const ScreenMesh::ProjVert& pt2 = mesh_.projectors[draw2DMesh].vertices[tri->vertIndices[1]];
						const ScreenMesh::ProjVert& pt3 = mesh_.projectors[draw2DMesh].vertices[tri->vertIndices[2]];

						Array<Vector2> screenpoly;
						screenpoly.append(Vector2(draw2DMesh * ELEM_WIDTH + (pt1.screenPt.x + TDHX[draw2DMesh]) * ELEM_WIDTH, (pt1.screenPt.y + TDHY[draw2DMesh]) * ELEM_HEIGHT));
						screenpoly.append(Vector2(draw2DMesh * ELEM_WIDTH + (pt2.screenPt.x + TDHX[draw2DMesh]) * ELEM_WIDTH, (pt2.screenPt.y + TDHY[draw2DMesh]) * ELEM_HEIGHT));
						screenpoly.append(Vector2(draw2DMesh * ELEM_WIDTH + (pt3.screenPt.x + TDHX[draw2DMesh]) * ELEM_WIDTH, (pt3.screenPt.y + TDHY[draw2DMesh]) * ELEM_HEIGHT));

						rd->setLineWidth(1);
						G3D::Draw::poly2DOutline(screenpoly, rd, G3D::Color3::yellow());
					}

					// draw debug vert
					const auto& pt = mesh_.projectors[draw2DMesh].vertices[debugGridVert];
					float sx = draw2DMesh*1280 + (pt.screenPt.x*1280);
					float sy = pt.screenPt.y*1024;
					Draw::rect2D(Rect2D::xywh(sx-5, sy-5, 10, 10), rd, Color3::white());
					stringstream ss;
					ss << "vert[" << debugGridVert << "] = (" << pt.worldPt.x + getTDHOff(draw2DMesh).x << ", "
						<< pt.worldPt.y + getTDHOff(draw2DMesh).y << ", "
						<< pt.worldPt.z + getTDHOff(draw2DMesh).z << ")";
					debugFont->draw2D(rd, ss.str(), 
						Vector2(((draw2DMesh+1)%3)*1280 + 640, 512), 
						16, Color3::black(), 
						Color3::white(), 
						GFont::XALIGN_CENTER);
				}
			} rd->pop2D();
		}

		// render minimap-Top
		if (showMapGUI) {
			rd->push2D(); {
				G3D::Draw::rect2D(minimapTop, rd, Color3::black());

				// map
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < mesh_.projectors[i].triangles.size()/2; ++j) {
						const cv::Point3d& pt1 = mesh_.projectors[i].vertices[mesh_.projectors[i].triangles[j].vertIndices[0]].worldPt + getTDHOff(i);
						const cv::Point3d& pt2 = mesh_.projectors[i].vertices[mesh_.projectors[i].triangles[j].vertIndices[1]].worldPt + getTDHOff(i);

						// FIXME: really inefficient since it redraws a ton ... but it works
						drawMapLine(rd, minimapTop, 
									Vector3(pt1.x, pt1.y, pt1.z), 
									Vector3(pt2.x, pt2.y, pt2.z));
					}

					// draw displayTile
					drawMapLine(rd, minimapTop,
						cameras_[i]->getTile().topLeft,
						cameras_[i]->getTile().topRight,
						Color3::red());
				}

				// headFrame_
				drawMapPoint(rd, minimapTop, headFrame_.translation, Color3::red());

				// spheres
				if (drawMode_ == SPHERES) {
					drawMapPoint(rd, minimapTop, s1.center, Color3::green());
					drawMapPoint(rd, minimapTop, s2.center, Color3::green());
					drawMapPoint(rd, minimapTop, s3.center, Color3::green());
				}
				// lines
				if (drawMode_ == STRAIGHT_MESH) {
					drawMapPoint(rd, minimapTop, c1.center(), Color3::green());
					//drawMapLine(rd, minimapTop, 
						//Vector3(sm_x, sm_y_max, sm_z_max), 
						//Vector3(sm_x, sm_y_min, sm_z_max), Color3::green());
				}

				// look vectors
				for (int i = 0; i < 3; i++) {
					drawMapLine(rd, minimapTop, headFrame_.translation,
						headFrame_.translation + cameras_[i]->getLookVec(),
						Color3::green());
				}

			} rd->pop2D();
		}
		if (STEREO) rd->setDrawBuffer(RenderDevice::DrawBuffer::DRAW_BACK_RIGHT);
	} // stereo loop
}

G3D::Vector2 RenderApp::projPoint(VRG3D::ProjectionVRCamera::Ref& cam, cv::Point3d pt, int eye) {
	Matrix4 view, proj, viewLeft, projLeft, viewRight, projRight;
	cam->getCurrentMatrices(view, proj, viewLeft, projLeft, viewRight, projRight);
	
	Matrix4 V, P;
	if (STEREO) {
		if (eye == 0) {
			V = viewLeft;
			P = projLeft;
		} else {
			V = viewRight;
			P = projRight;
		}
	} else {
		V = view;
		P = proj;
	}
	Vector4 v = P * V * Vector4(Vector3(pt.x, pt.y, pt.z), 1);
	Vector3 vv = v.xyz() / v.w;
	return Vector2((1 + vv.x) / 2, (1 + vv.y) / 2);
}

void RenderApp::drawPoly2D(G3D::Array<G3D::Vector2> screen, 
	G3D::RenderDevice * rd, G3D::Array<G3D::Vector2> tex) {

	if (screen.length() == 0 || (screen.length() != tex.length())) return;

	rd->beginPrimitive(PrimitiveType::TRIANGLE_FAN); {
		rd->setColor(Color3::white());
		for (int i = 0; i < screen.length(); ++i) {
			rd->setTexCoord(0, tex[i]);
			rd->sendVertex(screen[i]);
		}
	} rd->endPrimitive();
}

void RenderApp::drawMapPoint(G3D::RenderDevice * rd, G3D::Rect2D map,
	G3D::Vector3 pt, G3D::Color3 color) {
	Vector2 p = world2map(map, pt);
	Rect2D r = Rect2D::xywh(p.x - 2.5, p.y - 2.5, 5.0, 5.0);
	Draw::rect2D(r, rd, color);
}

void RenderApp::drawMapLine(G3D::RenderDevice * rd, G3D::Rect2D map, 
								G3D::Vector3 p1, G3D::Vector3 p2, G3D::Color3 color) {
	Array<Vector2> pts;
	pts.append(world2map(map, p1));
	pts.append(world2map(map, p2));
	Draw::poly2DOutline(pts, rd, color);
}

G3D::Vector2 RenderApp::world2map(G3D::Rect2D map, G3D::Vector3 w) {
	return Vector2(map.x1() - (w.y / ROOM_MAX_Y) * map.width(),
				   map.y1() - (w.x / ROOM_MAX_X) * map.height());
}

void RenderApp::onUserInput(G3D::UserInput* ui) {
	GApp::onUserInput(ui);

	// update dt
	double now = realTime();
	dt = now - lastTime;
	lastTime = now;

	// manual tracking
	if (TRACKING_TYPE == MANUAL_TRACKING) {
		Vector3 moveVec = Vector3(0, 0, 0);
		if (ui->keyDown(GKey('w')) || ui->keyDown(GKey::UP)) {
			moveVec.x = 1;
		} else if (ui->keyDown(GKey('s')) || ui->keyDown(GKey::DOWN)) {
			moveVec.x = -1;
		}
		if (ui->keyDown(GKey('a')) || ui->keyDown(GKey::LEFT)) {
			moveVec.y = 1;
		} else if (ui->keyDown(GKey('d')) || ui->keyDown(GKey::RIGHT)) {
			moveVec.y = -1;
		}
		if (ui->keyDown(GKey('q'))) { // up
			moveVec.z = 1;
		} else if (ui->keyDown(GKey('z'))) { // down
			moveVec.z = -1;
		}
		headFrame_.translation += moveVec * WALK_SPEED * dt;
	}

	if (ui->keyPressed(GKey('m'))) {
		drawMode_ = (DrawMode_t)((drawMode_ + 1) % NUM_DRAW_MODES);
	}
	if (ui->keyPressed(GKey('f'))) {
		draw2DMesh = (draw2DMesh + 1) % 4;
		activeTDH = draw2DMesh;
		cout << "activeTDH = " << activeTDH << endl;
		debugGridVert = 0;
	}
	if (ui->keyPressed(GKey('r'))) {
		currentCode = (currentCode + 1) % (grayCodeTextures.size() + 1);
	}
	if (ui->keyPressed(GKey('e'))) {
		showMapGUI = !showMapGUI;
	}

	if (ui->keyPressed(GKey('c'))) {
		// next vert
		if (draw2DMesh != 3) {
			debugGridVert = (debugGridVert+1) % mesh_.projectors[draw2DMesh].vertices.size();
		}
	}
	if (ui->keyPressed(GKey('x'))) {
		// prev vert
		if (draw2DMesh != 3) {
			debugGridVert = debugGridVert - 1;
			if (debugGridVert < 0) debugGridVert = mesh_.projectors[draw2DMesh].vertices.size() - 1;
		}
	}
	
	bool printHack = false;
	/*
	if (ui->keyPressed(GKey('.'))) {
		HACK += 0.001;
		printHack = true;
	} 
	if (ui->keyPressed(GKey(','))) {
		HACK -= 0.001;
		printHack = true;
	}

	if (ui->keyPressed(GKey('l'))) {
		HACK2 += 0.001;
		printHack = true;
	} 
	if (ui->keyPressed(GKey('k'))) {
		HACK2 -= 0.001;
		printHack = true;
	}
	*/

	if (ui->keyPressed(GKey('u')) && activeTDH != 3) { // up
		TDHY[activeTDH] += -0.001;
		printHack = true;
	}
	if (ui->keyPressed(GKey('j')) && activeTDH != 3) { // down
		TDHY[activeTDH] += 0.001;
		printHack = true;
	}
	if (ui->keyPressed(GKey('i')) && activeTDH != 3) { // left
		TDHX[activeTDH] += -0.001;
		printHack = true;
	} 
	if (ui->keyPressed(GKey('o')) && activeTDH != 3) { // right
		TDHX[activeTDH] += 0.001;
		printHack = true;
	}

	if (ui->keyPressed(GKey::KP7) && activeTDH != 3) { // + WX
		TDHX[activeTDH+3] += -0.01;
		printHack = true;
	} 
	if (ui->keyPressed(GKey::KP1) && activeTDH != 3) { // - WX
		TDHX[activeTDH+3] += 0.01;
		printHack = true;
	}
	if (ui->keyPressed(GKey::KP8) && activeTDH != 3) { // + WY
		TDHY[activeTDH+3] += -0.01;
		printHack = true;
	} 
	if (ui->keyPressed(GKey::KP2) && activeTDH != 3) { // - WY
		TDHY[activeTDH+3] += 0.01;
		printHack = true;
	}
	if (ui->keyPressed(GKey::KP9) && activeTDH != 3) { // + WZ
		TDHZ[activeTDH+3] += -0.01;
		printHack = true;
	} 
	if (ui->keyPressed(GKey::KP3) && activeTDH != 3) { // - WZ
		TDHZ[activeTDH+3] += 0.01;
		printHack = true;
	}

	if (printHack) {
		cout << "PRINT HACKZ:" << endl;

		cout << "HACK = " << HACK << ", HACK2 = " << HACK2 << endl;

		cout << "TDHX = {";
		for (int i = 0; i < 6; ++i) {
			cout << TDHX[i] << ", ";
		}
		cout << "};" << endl;

		cout << "TDHY = {";
		for (int i = 0; i < 6; ++i) {
			cout << TDHY[i] << ", ";
		}
		cout << "};" << endl;

		cout << "TDHZ = {";
		for (int i = 0; i < 6; ++i) {
			cout << TDHZ[i] << ", ";
		}
		cout << "};" << endl;
	}

	/*
	if (ui->keyPressed(GKey('.'))) {
		auto dt = cameras_[2]->getTile();
		float amount = 0.1;
		dt.topLeft += amount * (dt.topLeft - headFrame_.translation);
		dt.topRight += amount * (dt.topRight - headFrame_.translation);
		dt.botLeft += amount * (dt.botLeft - headFrame_.translation);
		dt.botRight += amount * (dt.botRight - headFrame_.translation);
		cameras_[2]->setDisplayTile(dt);
	} 
	if (ui->keyPressed(GKey(','))) {
		auto dt = cameras_[2]->getTile();
		float amount = -0.1;
		dt.topLeft += amount * (dt.topLeft - headFrame_.translation);
		dt.topRight += amount * (dt.topRight - headFrame_.translation);
		dt.botLeft += amount * (dt.botLeft - headFrame_.translation);
		dt.botRight += amount * (dt.botRight - headFrame_.translation);
		cameras_[2]->setDisplayTile(dt);
	}
	*/
}