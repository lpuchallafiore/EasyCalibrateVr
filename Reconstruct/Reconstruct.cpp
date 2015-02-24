#include "stdafx.h"
#include "Reconstruct.h"

#include <ComputeState.h>
#include <GrayCodeState.h>
#include <GrayCodeComputeState.h>
#include <ProcessData.h>

using namespace std;

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

// screen display
// stereo is not support at this time for calibration (nor does it need be)
const bool IS_STEREO = false;

// Tells C++ to invoke command-line main() function even on OS X and Win32
G3D_START_AT_MAIN();

int main(int argc, const char* argv[]) {
	if (G3D::FileSystem::exists("..\\data-files", false)) {
		// Running on Windows, building inside the starter directory
		chdir("..\\data-files");

		//chdir("GrayCode-20131227-01\\");
	}

	G3D::GApp::Settings settings;

	settings.window.width = WINDOW_WIDTH;
	settings.window.height = WINDOW_HEIGHT;
	settings.window.fullScreen = false;
	settings.window.framed = true;
	settings.window.stereo = IS_STEREO;
	settings.window.caption = "VR Window Calibration 2 OFFLINE";
	settings.film.enabled = false;
	settings.window.asynchronous = false;
	settings.debugFontName = "font/console.fnt"; // load a less crappy font

	/*
	ScreenData sd;
	RawProjData p;
	p.measurements.push_back(RawDataPiece());
	sd.rawProj.push_back(p);
	sd.rawIntrinsic.push_back(RawDataPiece());
	sd.rawExtrinsic.push_back(RawDataPiece());
	sd.writeXML("temp.xml");
	return 0;
	*/

	if (false) {
		/*
		int code_width = 1280 / 2;
		int code_height = 1024 / 2;
		std::string filenameprefix = "graycode_proj_0_set_0_code_";

		cv::Mat mask = GrayCodeState::computeGreenMask(cv::imread(filenameprefix+"0.jpg"));
		cv::imwrite("mask.jpg", mask);
		//cv::Mat mask(cv::Size(1600, 1200), CV_8UC1);
		//mask.setTo(255);
		std::vector<cv::Mat> gray_code_images;
		for (int i = 1; i <= 38; ++i) {
			char buf[512];
			sprintf(buf, "%s%d.jpg", filenameprefix.c_str(), i);
			gray_code_images.push_back(cv::imread(buf));
		}
		cv::Mat decoded_cols, decoded_rows, decoded_mask;
		GrayCodeState::decodeGrayCodes(code_width, code_height, 
			gray_code_images,
			mask, decoded_cols, decoded_rows, decoded_mask);
		cv::imwrite("decoded_mask.jpg", decoded_mask);
		cv::imwrite("decoded_cols.jpg", GrayCodeState::colorize(decoded_cols, code_width));
		cv::imwrite("decoded_rows.jpg", GrayCodeState::colorize(decoded_rows, code_height));
		return 0;
		*/
	} else {
		return ReconstructApp(settings).run();
	}
}

ReconstructApp::ReconstructApp(const GApp::Settings& settings) : GApp(settings) {
}

void ReconstructApp::onInit() {
	GApp::onInit();

	showRenderingStats = false;
	debugWindow->setVisible(false);

	// create process data and load screen data
	// TODO: error msq if file doesn't exists
	shared_ptr<ProcessData> pd = shared_ptr<ProcessData>(new ProcessData(debugFont));
	//pd->data.readXML("rawData-6-mod.xml");	

	// start!
	//currState = IAppState::Ptr(new ComputeState(
	//	25, 25, WINDOW_WIDTH - 50, WINDOW_HEIGHT - 50, pd));
	currState = IAppState::Ptr(new GrayCodeComputeState(
		25, 25, WINDOW_WIDTH - 50, WINDOW_HEIGHT - 50, pd));
}

void ReconstructApp::onAI() {
	GApp::onAI();
	shared_ptr<IAppState> nptr = currState->onAI();
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onNetwork() {
	GApp::onNetwork();
	shared_ptr<IAppState> nptr = currState->onNetwork();
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) {
	GApp::onSimulation(rdt, sdt, idt);
	shared_ptr<IAppState> nptr = currState->onSimulation(rdt, sdt, idt);
	if (nptr) { currState = nptr; }
}

bool ReconstructApp::onEvent(const G3D::GEvent& event) {
	return GApp::onEvent(event);
}

void ReconstructApp::onUserInput(G3D::UserInput* ui) {
	GApp::onUserInput(ui);
	shared_ptr<IAppState> nptr = currState->onUserInput(ui);
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onPose(G3D::Array<G3D::Surface::Ref>& posed3D,
							  G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	GApp::onPose(posed3D, posed2D);
	shared_ptr<IAppState> nptr = currState->onPose(posed3D, posed2D);
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onGraphics(G3D::RenderDevice* rd,
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

void ReconstructApp::onGraphics3D(G3D::RenderDevice *rd,
	                   G3D::Array<G3D::Surface::Ref>& posed3D) {
	shared_ptr<IAppState> nptr = currState->onGraphics3D(rd, posed3D);
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onGraphics2D(G3D::RenderDevice* rd,
	                   G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	shared_ptr<IAppState> nptr = currState->onGraphics2D(rd, posed2D);
	if (nptr) { currState = nptr; }
}

void ReconstructApp::onCleanup() {
}

void ReconstructApp::endProgram() {
	m_endProgram = true;
}
