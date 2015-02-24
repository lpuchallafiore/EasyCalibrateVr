#include "stdafx.h"
#include "ProjectorDimensionStates.h"
#include "MeasureState.h"

#include "ProcessData.h"

GridState::GridState(const std::shared_ptr<ProcessData>& pd) : IAppState(pd) {
	p = &PD->data.rawProj[PD->currProj];
}

IAppState::Ptr GridState::onGraphics2D(G3D::RenderDevice* rd,
									   G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D) { 
	using namespace G3D;

	// draw visible bounds
	Rect2D r = Rect2D::xywh(p->left, p->top, p->width, p->height);
	Draw::rect2D(r, rd, Color3::red());

	// draw calibration grid
	float x0 = p->gridBounds.x;
	float x1 = p->gridBounds.br().x;
	float y0 = p->gridBounds.y;
	float y1 = p->gridBounds.br().y;
	float stepX = (x1 - x0) / (float)PD->props.gridWidth;
	float stepY = (y1 - y0) / (float)PD->props.gridHeight;
	for (int y = 0; y < PD->props.gridWidth; y++) {
		for (int x = 0; x < PD->props.gridHeight; x++) {
			float sx = x0 + x * stepX;
			float sy = y0 + y * stepY;
			Array<Vector2> circle;
			for (int i = 0; i < 32; i++) {
				Vector2 r(cos(pi()*i/16), sin(pi()*i/16));
				circle.append(r * PD->props.circleRadius + Vector2(sx, sy));
			}
			if (x == 0 || y == 0 || 
				x == PD->props.gridWidth - 1 || 
				y == PD->props.gridHeight - 1) {

				Draw::poly2D(circle, rd, Color3::blue());
			} else {
				Draw::poly2D(circle, rd, Color3::green());
			}
		}
	}

	PD->df->draw2D(rd, 
		"Size grid so all circles are visible (use WASD & IJKL keys)",
		Vector2(p->left + 256, 128), 14, Color3::white());
	PD->df->draw2D(rd, 
		"Press SPACEBAR when done", 
		Vector2(p->left + 256, 192), 14, Color3::white());

	return stayInState();
}

IAppState::Ptr GridState::onUserInput(G3D::UserInput* ui) {
	using namespace G3D;

	cv::Rect_<float> old = p->gridBounds;

	// control top-left
	float speed = 100.0f;
	float dts = dt * speed;
	bool w = ui->keyDown(GKey('w'));
	bool a = ui->keyDown(GKey('a'));
	bool s = ui->keyDown(GKey('s'));
	bool d = ui->keyDown(GKey('d'));
	float deltaT = (s ? dts : 0) - (w ? dts : 0);
	float deltaL = (d ? dts : 0) - (a ? dts : 0);

	// control bottom-right
	bool i = ui->keyDown(GKey('i'));
	bool j = ui->keyDown(GKey('j'));
	bool k = ui->keyDown(GKey('k'));
	bool l = ui->keyDown(GKey('l'));
	float deltaB = (k ? dts : 0) - (i ? dts : 0);
	float deltaR = (l ? dts : 0) - (j ? dts : 0);

	if (w || a || s || d || i || j || k || l) {
		Rect2D nr = Rect2D::xyxy(old.x + deltaL, old.y + deltaT,
								 old.br().x + deltaR, 
								 old.br().y + deltaB);

		// keep the rect confined to a single projector
		Rect2D proj = Rect2D::xywh(p->left, p->top, p->width, p->height);
		nr = nr.intersect(proj);

		cv::Rect_<float> nrr(nr.x0(), nr.y0(), nr.width(), nr.height());
		p->gridBounds = nrr;
	}

	// save and move to next state
	if (ui->keyPressed(GKey(' '))) {
		return nextState();
	}

	return stayInState();
}

IAppState::Ptr GridState::nextState() {
	return Ptr(new MeasureState(PD));
}