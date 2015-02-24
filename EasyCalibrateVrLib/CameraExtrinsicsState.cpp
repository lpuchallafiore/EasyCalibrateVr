#include "stdafx.h"
#include "Calibration.h"
#include "CameraExtrinsicsState.h"
#include "ProjectorDimensionStates.h"

#include "ProcessData.h"

CameraExtrinsicsState::CameraExtrinsicsState(const std::shared_ptr<ProcessData>& pd)
	: IAppState(pd) {

	tt = 0;
}

IAppState::Ptr CameraExtrinsicsState::onNetwork() {
	PD->onNetwork(); // update vicon

	// get image
	PD->cam >> image;
	drawImage = image.clone();

	// get modelview (just to find chessboard corners and make sure it is valid)
	static cv::Mat mv;
	static MonoCalibration mc;
	mc.getGLModelViewMatrix(mv, drawImage, false);

	return stayInState();
}

IAppState::Ptr CameraExtrinsicsState::onGraphics3D(G3D::RenderDevice* rd,
		G3D::Array<std::shared_ptr<G3D::Surface> >& posed3D) {
	using namespace G3D;

	rd->pushState();

	// 2D
	rd->push2D();

	// create tex
	auto tex = Texture::fromMemory("frame", 
		(const void*)drawImage.data, 
		ImageFormat::BGR8(), 
		drawImage.size().width, 
		drawImage.size().height, 
		1);

	// letterbox and draw
	int adj = 1;
	RawProjData& pj = PD->data.rawProj[adj];
	float w = pj.width;
	float scale = (float)w / (float)image.size().width;
	float nh = image.size().height * scale;
	int top = pj.height / 2 - nh / 2;
	int left = pj.left + (pj.width - w) / 2;
	rd->setTexture(0, tex);
	Draw::rect2D(Rect2D::xywh(left, top, w, nh), rd);
	rd->setTexture(0, NULL);

	// print axis on screen
	char buff[256];
	sprintf(buff, "x axis (%.3f, %.3f, %.3f)", 
		PD->chessboardToRoom.rotation[0][0],
		PD->chessboardToRoom.rotation[1][0],
		PD->chessboardToRoom.rotation[2][0]);
	PD->df->draw2D(rd, buff, Vector2(128, 128 + 0 * 64), 16, Color3::white());
	sprintf(buff, "y axis (%.3f, %.3f, %.3f)",
		PD->chessboardToRoom.rotation[0][1],
		PD->chessboardToRoom.rotation[1][1],
		PD->chessboardToRoom.rotation[2][1]);
	PD->df->draw2D(rd, buff, Vector2(128, 128 + 1 * 64), 16, Color3::white());
	sprintf(buff, "z axis (%.3f, %.3f, %.3f)",
		PD->chessboardToRoom.rotation[0][2],
		PD->chessboardToRoom.rotation[1][2],
		PD->chessboardToRoom.rotation[2][2]);
	PD->df->draw2D(rd, buff, Vector2(128, 128 + 2 * 64), 16, Color3::white());

	rd->pop2D();

	return stayInState();
}

IAppState::Ptr CameraExtrinsicsState::onUserInput(G3D::UserInput* ui) {
	using namespace G3D;

	/*
	if (ui->keyReleased(GKey('c'))) {
		// calibrate the chessboard vicon root transformation
		if (m_PD->viconChessboardManipulator) {
			m_PD->chessboardToChessboardRoot = m_PD->chessboardRootToVicon.inverse() * CoordinateFrame(Vector3(0.0f, 0.0f, 0.0f));

			std::string mats = m_PD->chessboardToChessboardRoot.rotation.toString();
			mats[0] = '('; mats[mats.length()-1] = ')'; // change brackets to parenthesis
			size_t off = 0;
			while ((off = mats.find(";", off+1)) != std::string::npos)
				mats[off] = ',';
			std::string vecs = m_PD->chessboardToChessboardRoot.translation.toString();

			// now this prints code we can paste right back into this program
			Log::common()->printf("chessboardToChessboardRoot = CoordinateFrame(Matrix3%s, Vector3%s);\n",
				mats.c_str(), vecs.c_str());
		}
	}
	*/

	if (ui->keyReleased(GKey('x'))) {
		// save to raw data
		RawDataPiece p = PD->buildRawDataPiece("extrinsic", 
			PD->data.rawExtrinsic.size() + 1, image,
			std::vector<cv::Point2i>(), std::vector<cv::Point2f>(), -1);
		PD->data.rawExtrinsic.push_back(p);
	}

	if (ui->keyReleased(GKey(' '))) {
		return nextState();
	}

	return stayInState();
}

IAppState::Ptr CameraExtrinsicsState::nextState() {
	PD->currProj = 0; // start with the first projector

	// save raw data
	PD->data.writeXML("rawData.xml.gz");
	return IAppState::Ptr(new GridState(PD));
}