#include "stdafx.h"
#include "CameraIntrinsicsState.h"
#include "CameraExtrinsicsState.h"

#include "ProcessData.h"

CameraIntrinsicsState::CameraIntrinsicsState(const std::shared_ptr<ProcessData>& pd) 
	: IAppState(pd) {

	// connect to the Vicon
	PD->initVicon();
	// connect to the Camera
	PD->initCam();

	// set initial variables
	tt = 0;
	frameNum = 0;
	state = CALIB_LOOP;

	// startup calibrator 
	// (just used to confirm that we are getting valid images)
	calibrator.startCalibration();
}

IAppState::Ptr CameraIntrinsicsState::onNetwork() {
	// get new tracker data from Vicon
	PD->onNetwork();

	// get new frame from the camera
	PD->cam >> image;

	return Ptr(NULL);
}

IAppState::Ptr CameraIntrinsicsState::nextState() {
	// save data before state change in case of a crash
	PD->data.writeXML("rawData.xml.gz");

	return IAppState::Ptr(new CameraExtrinsicsState(PD));
}

IAppState::Ptr CameraIntrinsicsState::onGraphics2D(G3D::RenderDevice* rd,
		G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D) {
	using namespace G3D;
	using namespace cv;

	// display text
	for (size_t i = 0; i < txtQ.size(); i++) {
		PD->df->draw2D(rd, txtQ[i], Vector2(128, 128 + i * 64), 
			12, Color3::white());
	}

	if (state == CALIB_LOOP) {
		// collection loop
		if (image.data != NULL) {
			if (tt > 1.0) { // only process an image once a second
				tt = 0;

				frameNum++;
				char buf[1024];
				sprintf(buf, "Processing frame %d ...", frameNum);
				print(buf);
				calibrator.imageLoopCommon(image, 0, true);

				// add to raw data
				RawDataPiece p = PD->buildRawDataPiece("intrinsic", frameNum, 
					image, std::vector<Point2i>(), 
					std::vector<cv::Point2f>(), -1);
				PD->data.rawIntrinsic.push_back(p);
			}
		}

		// display result
		auto tex = Texture::fromMemory("frame", 
			(const void*)calibrator.m_SmallImage.data, 
			ImageFormat::BGR8(), 
			calibrator.m_SmallImage.size().width, 
			calibrator.m_SmallImage.size().height, 
			1);
		// letterbox and draw
		int adj = 1;
		const RawProjData& pj = PD->data.rawProj[adj];
		float w = pj.width;
		float scale = (float)w / (float)image.size().width;
		float nh = image.size().height * scale;
		int top = pj.height / 2 - nh / 2;
		int left = pj.left + (pj.width - w) / 2;
		rd->setTexture(0, tex);
		Draw::rect2D(Rect2D::xywh(left, top, w, nh), rd);
		rd->setTexture(0, NULL);

		if (calibrator.m_Successes >= 75) {
			print("done!");
			state = DONE;
		}

	} else if (state == DONE) {
		if (tt > 2) {
			return nextState();
		}
	}

	return stayInState();
}