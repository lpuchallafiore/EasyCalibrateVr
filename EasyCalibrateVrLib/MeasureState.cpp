#include "stdafx.h"
#include "MeasureState.h"
#include "ComputeState.h"
#include "ProjectorDimensionStates.h"

#include "ProcessData.h"

MeasureState::MeasureState(const std::shared_ptr<ProcessData>& pd) : IAppState(pd) {
	p = &PD->data.rawProj[PD->currProj];
	p->numPointsX = PD->props.gridWidth;
	p->numPointsY = PD->props.gridHeight;
	X = 0;
	Y = 0;
	tt = 0;
	state = MOVING_CAMERA;
	currGroup = 0;
	groupIndex = 0;
}

IAppState::Ptr MeasureState::onNetwork() {
	// get a new frame from the camera
	PD->cam >> image;
	imageRaw = image.clone();

	// get Vicon data
	PD->onNetwork();

	return stayInState();
}

IAppState::Ptr MeasureState::onGraphics2D(G3D::RenderDevice* rd,
										  G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D) {
	using namespace G3D;

	// compute regions
	int regionW = p->numPointsX / PD->props.regionsX;
	int regionH = p->numPointsY / PD->props.regionsY;

	// draw background on current projector
	// and camera image on adjacent projector
	Rect2D r = Rect2D::xywh(p->left, p->top, p->width, p->height);
	Draw::rect2D(r, rd, Color3::white());
	int adj = PD->currProj + 1;
	if (adj >= PD->data.rawProj.size()) adj = 1;
	bool found;
	std::vector<cv::Point2f> gridPts = findGrid(image, regionW, regionH, found);
	auto tex = Texture::fromMemory("frame", 
		(const void*)image.data, 
		ImageFormat::BGR8(), 
		image.size().width, 
		image.size().height, 
		1);
	// letterbox and draw
	RawProjData& pj = PD->data.rawProj[adj];
	float w = pj.width / 3;
	float scale = (float)w / (float)image.size().width;
	float nh = image.size().height * scale;
	int top = pj.height / 2 - nh / 2;
	int left = pj.left + pj.width / 2 - w / 2;
	rd->setTexture(0, tex);
	Draw::rect2D(Rect2D::xywh(left, top, w, nh), rd); // FIXME: very blurry, use better interpolation, or None?
	rd->setTexture(0, NULL);

	float x0 = p->gridBounds.x;
	float x1 = p->gridBounds.br().x;
	float y0 = p->gridBounds.y;
	float y1 = p->gridBounds.br().y;
	float stepX = (x1 - x0) / (float)p->numPointsX;
	float stepY = (y1 - y0) / (float)p->numPointsY; 

	if (state == MOVING_CAMERA) {
		// waiting for user to move the camera
		// draw the current grid region
		for (int y = 0; y < regionH; y++) {
			for (int x = 0; x < regionW; x++) {
				float sx = x0 + (X * regionW + x) * stepX;
				float sy = y0 + (Y * regionH + y) * stepY;
				Array<Vector2> circle;
				for (int i = 0; i < 32; i++) {
					Vector2 r(cos(pi()*i/16), sin(pi()*i/16));
					circle.append(r * PD->props.circleRadius + Vector2(sx, sy));
				}
				Draw::poly2D(circle, rd, Color3::black());
			}
		}

		// draw some text explaining what is happening
		PD->df->draw2D(rd, "Please reposition the camera",
			Vector2(pj.left + 256, 128), 14, Color3::white());
		PD->df->draw2D(rd, "Press SPACEBAR when done", 
			Vector2(pj.left + 256, 192), 14, Color3::white());

		char buf[256];
		sprintf(buf, "cam = (%.2f, %.2f, %.2f) cam z-axis = (%.2f, %.2f, %.2f)", 
			PD->camToRoom.translation.x,
			PD->camToRoom.translation.y,
			PD->camToRoom.translation.z,
			PD->camToRoom.rotation[0][2],
			PD->camToRoom.rotation[1][2],
			PD->camToRoom.rotation[2][2]);
		PD->df->draw2D(rd, buf,
			Vector2(pj.left + 256, 256), 14, Color3::white());

	} else if (state == TAKE_MEASURE) {
		// draw the current grid region
		for (int y = 0; y < regionH; y++) {
			for (int x = 0; x < regionW; x++) {
				float sx = x0 + (X * regionW + x) * stepX;
				float sy = y0 + (Y * regionH + y) * stepY;
				Array<Vector2> circle;
				for (int i = 0; i < 32; i++) {
					Vector2 r(cos(pi()*i/16), sin(pi()*i/16));
					circle.append(r * PD->props.circleRadius + Vector2(sx, sy));
				}
				Draw::poly2D(circle, rd, Color3::black());
			}
		}

		// draw the completion percentage
		int jj = getViewCount(X * regionW, Y * regionH);
		if (jj > PD->props.minViews) jj = PD->props.minViews;
		float a = (float)(PD->props.minViews - jj) / (float)PD->props.minViews;
		Color3 c = a * Color3::red() + (1 - a) * Color3::green();
		Draw::rect2D(Rect2D::xywh(left, top / 2, w, nh/2), rd, c);
		Draw::rect2D(Rect2D::xywh(left, top / 2, w * (groupIndex / (float)PD->props.groupSize), nh/4), rd, Color3::cyan());
		char buf[256];
		sprintf(buf, "%d / %d - %d", jj, PD->props.minViews, groupIndex);
		PD->df->draw2D(rd, buf, 
			Vector2(left + w/2, top/2 + nh/w), 46, 
			Color3::white(), Color4::clear(), 
			GFont::XALIGN_CENTER, GFont::YALIGN_CENTER);

		// wait to take an image until the circle has been up for a while
		// to fight the evils of latency, for great justice
		if (tt > PD->props.waitTime) {
			// add it (along with the vicon data) to the list of measures
			if (found) {
				std::vector<cv::Point2i> grid;
				std::vector<cv::Point2f> screen;

				for (int y = 0; y < regionH; y++) {
					for (int x = 0; x < regionW; x++) {
						int gx = X * regionW + x;
						int gy = Y * regionH + y;
						float sx = x0 + gx * stepX;
						float sy = y0 + gy * stepY;

						cv::Point2i gridpt(gx, gy);
						grid.push_back(gridpt);

						cv::Point2f screenpt(sx, sy);
						screen.push_back(screenpt);
					}
				}

				// save to raw data
				RawDataPiece rp = PD->buildRawDataPiece("measure", 
					p->measurements.size() + 1,
					imageRaw, grid, screen, currGroup);
				p->measurements.push_back(rp);

				// advance to the next point
				groupIndex++;
				if (groupIndex == PD->props.groupSize) {
					currGroup++; groupIndex = 0;
					X++;
					state = MOVING_CAMERA;
					if (X >= PD->props.regionsX) {
						X = 0;
						Y++;
						if (Y >= PD->props.regionsY) {
							X = 0;
							Y = 0;

							bool allViewed = true;
							for (int y = 0; y < p->numPointsX; y++) {
								for (int x = 0; x < p->numPointsY; x++) {
									if (getViewCount(x, y) < PD->props.minViews) {
										allViewed = false;
										break;
									}
								}
							}

							if (allViewed) {
								state = DONE;
							} else {
								// start another view of the same projector
							}
						}
					}
				} else {
					tt = 0;
				}
			}
		}
	} else if (state == DONE) {
		return nextState();
	}

	return stayInState();
}

IAppState::Ptr MeasureState::onUserInput(G3D::UserInput* ui) {
	using namespace G3D;

	// camera repositioned
	if (ui->keyPressed(GKey(' '))) {
		state = TAKE_MEASURE;
		tt = 0; // reset wait time
	}

	// just for testing
	if (ui->keyPressed(GKey('a'))) {
		return nextState();
	}

	return stayInState();
}

std::vector<cv::Point2f> MeasureState::findGrid(cv::Mat& img, int gridW, int gridH, bool& found) {
	using namespace cv;
	vector<Point2f> pointBuf;
	Size sz = Size(gridH, gridW);
	found = findCirclesGrid(img, sz, pointBuf);
	drawChessboardCorners(img, sz, Mat(pointBuf), found);

	return pointBuf;
}

IAppState::Ptr MeasureState::nextState() {
	// save data to file (in case of crashes)
	PD->data.writeXML("rawData.xml.gz");
	PD->currProj++;
	if (PD->currProj >= PD->data.rawProj.size()) {
		// done gathering data, compute!
		return Ptr(new ComputeState(PD->data.rawProj[1].left + 50,
									PD->data.rawProj[1].top + 50,
									PD->data.rawProj[1].width - 100,
									PD->data.rawProj[1].height - 100,
									PD));
	} else {
		// proceed to next projector
		return Ptr(new GridState(PD));
	}
}

int MeasureState::getViewCount(int x, int y) const {
	std::vector<int> views;
	for (auto it = p->measurements.begin(); it != p->measurements.end(); ++it) {
		for (auto itt = it->projRegionGrid.begin(); itt != it->projRegionGrid.end(); ++itt) {
			if (itt->x == x && itt->y == y && std::find(views.begin(), views.end(), it->groupId) == views.end()) {
				views.push_back(it->groupId);
			}
		}
	}

	return views.size();
}