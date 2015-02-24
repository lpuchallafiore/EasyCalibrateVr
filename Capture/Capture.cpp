#include "stdafx.h"
#include "Capture.h"

#include <ProcessData.h>
#include <Calibration.h>
#include <GrayCodeState.h>

using namespace std;

// Controls the operation of the program ---------------------------------------
// Change these to match the projector/screen setup prior to calibration

// controls screen/projector configuration
#define NUM_PROJECTORS 3
#define PROJECTOR_WIDTH 1280
#define PROJECTOR_HEIGHT 1024

// controls the calibration process
// number of measurements to take at a single "spot"
#define GROUP_SIZE 5
#define CALIBRATION_GRID_WIDTH 30
#define CALIBRATION_GRID_HEIGHT 30
#define CALIBRATION_CIRCLE_RADIUS 10
#define CALIBRATION_MIN_VIEWS 8
#define CALIBRATION_WAIT_TIME 3
// must evenly divide grid width and height
#define CALIBRATION_REGIONS_X 2
#define CALIBRATION_REGIONS_Y 2

enum calib_type_t {
	CHESSBOARD = 0,
	CIRCLE_GRID
}; // TODO - add this support to CMonoCalibration <- basically rewrite the entire thing

// rudimentary sub-folder support for scanning
#define SUBFOLDER_NAME "GrayCode-20140508"
#define DO_OPENCV_CALIB false
#define CALIB_PATTERN CHESSBOARD

// screen display
// stereo is not support at this time for calibration (nor does it need be)
const bool IS_STEREO = false;
// -----------------------------------------------------------------------------

// Tells C++ to invoke command-line main() function even on OS X and Win32
G3D_START_AT_MAIN();

int main(int argc, const char* argv[]) {
	if (G3D::FileSystem::exists("..\\data-files\\", false)) {
		// Running on Windows, building inside the starter directory
		chdir("..\\data-files");

		// make subdir
		mkdir(SUBFOLDER_NAME);
		chdir(SUBFOLDER_NAME);
	}


	G3D::GApp::Settings settings;

	if (DO_OPENCV_CALIB) {
		settings.window.width = 640;
		settings.window.height = 480;
		settings.window.fullScreen = false;
		settings.window.framed = true;
	} else {
		settings.window.width = PROJECTOR_WIDTH * NUM_PROJECTORS;
		settings.window.height = PROJECTOR_HEIGHT;
		settings.window.fullScreen = true;
		settings.window.framed = false;
	}
	settings.window.stereo = IS_STEREO;
	settings.window.caption = "VR Window Calibration 2";
	settings.film.enabled = false;
	settings.window.asynchronous = false;
	settings.debugFontName = "font/console.fnt"; // load a less crappy font
	
	return CaptureApp(settings).run();
}

CaptureApp::CaptureApp(const GApp::Settings& settings) : GApp(settings) {
}

void CaptureApp::onInit() {
	GApp::onInit();

	showRenderingStats = false;
	debugWindow->setVisible(false);

	// setup screen configuration and process data
	auto pd = shared_ptr<ProcessData>(new ProcessData(debugFont));
	pd->props.gridWidth = CALIBRATION_GRID_WIDTH;
	pd->props.gridHeight = CALIBRATION_GRID_HEIGHT;
	pd->props.circleRadius = CALIBRATION_CIRCLE_RADIUS;
	pd->props.minViews = CALIBRATION_MIN_VIEWS;
	pd->props.waitTime = CALIBRATION_WAIT_TIME;
	pd->props.regionsX = CALIBRATION_REGIONS_X;
	pd->props.regionsY = CALIBRATION_REGIONS_Y;
	pd->props.groupSize = GROUP_SIZE;

	pd->data.screenWidth = PROJECTOR_WIDTH * NUM_PROJECTORS;
	pd->data.screenHeight = PROJECTOR_HEIGHT;
	for (int i = 0; i < NUM_PROJECTORS; i++) {
		RawProjData rd;
		rd.top = 0;
		rd.left = i * PROJECTOR_WIDTH;
		rd.width = PROJECTOR_WIDTH;
		rd.height = PROJECTOR_HEIGHT;
		rd.gridBounds = cv::Rect_<float>(rd.left + rd.width / 5,
			rd.top + rd.height / 5,
			3 * rd.width / 5,
			3 * rd.height / 5);

		pd->data.rawProj.push_back(rd);
	}

	if (DO_OPENCV_CALIB) {
		// do opencv calibration
		MonoCalibration mc;
		mc.m_ShowImage = true;
		mc.calibrateFromCam(0);
	}

	// start!
	//currState = IAppState::Ptr(new CameraIntrinsicsState(pd));
	currState = IAppState::Ptr(new GrayCodeState(pd));
}

void CaptureApp::onAI() {
	GApp::onAI();
	shared_ptr<IAppState> nptr = currState->onAI();
	if (nptr) { currState = nptr; }
}

void CaptureApp::onNetwork() {
	GApp::onNetwork();
	shared_ptr<IAppState> nptr = currState->onNetwork();
	if (nptr) { currState = nptr; }
}

void CaptureApp::onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) {
	GApp::onSimulation(rdt, sdt, idt);
	shared_ptr<IAppState> nptr = currState->onSimulation(rdt, sdt, idt);
	if (nptr) { currState = nptr; }
}

bool CaptureApp::onEvent(const G3D::GEvent& event) {
	return GApp::onEvent(event);
}

void CaptureApp::onUserInput(G3D::UserInput* ui) {
	GApp::onUserInput(ui);
	shared_ptr<IAppState> nptr = currState->onUserInput(ui);
	if (nptr) { currState = nptr; }
}

void CaptureApp::onPose(G3D::Array<G3D::Surface::Ref>& posed3D,
	             G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	GApp::onPose(posed3D, posed2D);
	shared_ptr<IAppState> nptr = currState->onPose(posed3D, posed2D);
	if (nptr) { currState = nptr; }
}

void CaptureApp::onGraphics(G3D::RenderDevice* rd,
	                 G3D::Array<G3D::Surface::Ref>& posed3D,
					 G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	using namespace G3D;

	//rd->setColorClearValue(Color3(0.1f, 0.5f, 1.0f));
	rd->setColorClearValue(Color3::black());
	// Clear the entire screen (needed even though we'll render over it because
	// AFR uses clear() to detect that the buffer is not re-used.)

	if (IS_STEREO) { rd->setDrawBuffer(G3D::RenderDevice::DrawBuffer::DRAW_BACK_LEFT); }
	for (eye = LEFT_EYE; eye < (IS_STEREO ? NUM_EYES : 1); eye++) {
		rd->clear();

		rd->pushState();
		onGraphics3D(rd, posed3D);
		rd->popState();
		rd->push2D();
		onGraphics2D(rd, posed2D);
		rd->pop2D();

		if (IS_STEREO) { rd->setDrawBuffer(G3D::RenderDevice::DrawBuffer::DRAW_BACK_RIGHT); }
	}
}

void CaptureApp::onGraphics3D(G3D::RenderDevice *rd,
	                   G3D::Array<G3D::Surface::Ref>& posed3D) {
	shared_ptr<IAppState> nptr = currState->onGraphics3D(rd, posed3D);
	if (nptr) { currState = nptr; }
}

void CaptureApp::onGraphics2D(G3D::RenderDevice* rd,
	                   G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	shared_ptr<IAppState> nptr = currState->onGraphics2D(rd, posed2D);
	if (nptr) { currState = nptr; }
}

void CaptureApp::onCleanup() {
}

void CaptureApp::endProgram() {
	m_endProgram = true;
}
