#include "stdafx.h"
#include "ComputeState.h"

#include "ProcessData.h"

// info on opencv bundle adjustment
// http://ardrone-ailab-u-tokyo.blogspot.co.nz/2012/11/opencv-levmarqsparse-class-for-sparse_13.html

//using namespace G3D;

#define DO_BUNDLE_ADJUST false
// 1 = OpenCV, 2 = CVSBA, 3 = SSBA
#define BUNDLE_TYPE 1
#define BUNDLE_ITERATIONS 10000
#define BUNDLE_ERR 1e-10

ComputeState::ComputeState(float left_, float top_, float width_, float height_, 
	const std::shared_ptr<ProcessData>& pd) : IAppState(pd), left(left_), top(top_),
	width(width_), height(height_) {

	intrinsicPercent = 0;
	intrinsicCounter = 0;

	extrinsicPercent = 0;
	extrinsicCounter = 0;

	measurePercent = 0;
	measureCounterProj = 0;
	measureCounter = 0;

	measureAvgPercent = 0;
	measureAvgCounterProj = 0;
	measureAvgCounterX = measureAvgCounterY = 0;

	rayPercent = 0;
	rayCounterProj = rayCounter = 0;

	pointPercent = 0;
	pointCounterProj = pointCounterX = pointCounterY = 0;

	bundlePercent = 0;

	showImage = false;
	dataSaved = false;

	state = INTRINSIC;

	// start worker thread
	InitializeCriticalSection(&csImage);
	CreateThread(NULL, 0, t_Worker, this, 0, 0);
}

void ComputeState::drawBar(G3D::RenderDevice* rd, std::string name, float percent,
	float x, float y, float w, float h, G3D::Color3 foreColor, G3D::Color3 backColor) {

	// draw text
	PD->df->draw2D(rd, name, G3D::Vector2(x, y + h/2), 16, G3D::Color3::white(),
		G3D::Color4::clear(), G3D::GFont::XALIGN_LEFT, G3D::GFont::YALIGN_CENTER);

	// compute bar box
	float textWidth = 200;
	float padding = 10;
	float bh = h;
	float bw = w - textWidth - padding;
	float cw = (bw-padding) * percent / 100.0f;
	float bx = x + textWidth + padding;
	float by = y;

	// draw back color
	G3D::Draw::rect2D(G3D::Rect2D::xywh(bx, by, bw, bh), rd, 
		backColor, G3D::Rect2D());

	// draw fore color
	G3D::Draw::rect2D(G3D::Rect2D::xywh(bx+5, by+5, cw, bh-10), rd, 
		foreColor, G3D::Rect2D());
}

IAppState::Ptr ComputeState::onGraphics2D(G3D::RenderDevice * rd,
										  G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	using namespace G3D;
	rd->clear();

	// draw progress bars
	float Y = top;
	float padding = 20;
	const int NUM_BARS = 7;
	float h = (1 * height / 2) / NUM_BARS - padding;

	drawBar(rd, "Intrinsic", intrinsicPercent, left, Y, width, h, G3D::Color3::green());
	Y += h + padding;
	drawBar(rd, "Extrinsic", extrinsicPercent, left, Y, width, h, G3D::Color3::red());
	Y += h + padding;
	drawBar(rd, "Measure", measurePercent, left, Y, width, h, G3D::Color3::blue());
	Y += h + padding;
	drawBar(rd, "Measure Avg", measureAvgPercent, left, Y, width, h, G3D::Color3::cyan());
	Y += h + padding;
	drawBar(rd, "Rays", rayPercent, left, Y, width, h, G3D::Color3::yellow());
	Y += h + padding;
	drawBar(rd, "Points", pointPercent, left, Y, width, h, G3D::Color3::purple());
	Y += h + padding;
	drawBar(rd, "Bundle Adjust", bundlePercent, left, Y, width, h, G3D::Color3::orange());

	// draw image if supplied
	EnterCriticalSection(&csImage);
	if (showImage) {
		// FIXME: this is done quite often in the AppStates, extract and make a shared function
		auto tex = G3D::Texture::fromMemory("frame",
			(const void*)image.data,
			G3D::ImageFormat::BGR8(),
			image.size().width,
			image.size().height, 
			1);
		rd->setTexture(0, tex);
		G3D::Draw::rect2D(G3D::Rect2D::xywh(left, Y+h+padding, 512, 288), rd);
		rd->setTexture(0, NULL);
	}
	LeaveCriticalSection(&csImage); // FIXME: scope class thing ...

	// if DONE draw "Show in 3D" button to render in the room environment
	// TODO
	if (state == DONE || state == REALLY_DONE) {
		PD->df->draw2D(rd, "DONE!", Vector2(left + 400, Y+3*padding), 144, 
					   Color3::white(), Color4::clear(), 
					   GFont::XALIGN_CENTER);
	}

	return stayInState();
}

void ComputeState::intrinsicController() {

	/*
	bool exists = calibrator.load();
	if (exists) {
		// use it!
		intrinsicPercent = 100;
		state = EXTRINSIC;

		PD->data.compIntrinsic = calibrator.m_Intrinsic;
		PD->data.compDistortion = calibrator.m_Distortion;
		return;
	}
	*/

	// start case
	if (intrinsicCounter == 0) {
		calibrator.startCalibration();
	}

	int dataSize = PD->data.rawIntrinsic.size();
	calibrator.m_ImageCount = dataSize;
	if (intrinsicCounter < dataSize) {
		cv::Mat im = cv::imread(PD->data.rawIntrinsic[intrinsicCounter].imagePath);
		calibrator.imageLoopCommon(im, intrinsicCounter, false);
		EnterCriticalSection(&csImage);
		image = calibrator.m_SmallImage.clone();
		showImage = true;
		LeaveCriticalSection(&csImage);
	}

	// end case
	if (!(++intrinsicCounter < dataSize)) {
		double err = calibrator.endCalibration();

		// copy results
		PD->data.compIntrinsic = calibrator.m_Intrinsic;
		PD->data.compDistortion = calibrator.m_Distortion;

		state = EXTRINSIC;
	}

	// percent
	intrinsicPercent = 100 * ((float)intrinsicCounter / (float)dataSize);
	if (intrinsicPercent > 100) intrinsicPercent = 100;
}

G3D::CoordinateFrame ComputeState::toCF(const cv::Mat& rot, const cv::Point3f& t) const {
	G3D::CoordinateFrame f;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			f.rotation[r][c] = rot.at<float>(r, c);
		}
	}
	f.translation.x = t.x;
	f.translation.y = t.y;
	f.translation.z = t.z;

	return f;
}

void ComputeState::extrinsicController() {
	int idx = extrinsicCounter;

	// load data
	cv::Mat myimage = cv::imread(PD->data.rawExtrinsic[idx].imagePath);

	PD->camRootToVicon = toCF(
		PD->data.rawExtrinsic[idx].camRot,
		PD->data.rawExtrinsic[idx].camTrans);
	PD->chessboardRootToVicon = toCF(
		PD->data.rawExtrinsic[idx].chessRot,
		PD->data.rawExtrinsic[idx].chessTrans);
	PD->chessboardToChessboardRoot = toCF(
		PD->data.rawExtrinsic[idx].chessRootRot,
		PD->data.rawExtrinsic[idx].chessRootTrans);

	// compute chess to camera R/T
	cv::Mat mv;
	if (calibrator.getGLModelViewMatrix(mv, myimage)) {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				PD->chessboardToCam.rotation[r][c] = mv.at<double>(r, c);
			}
		}
		PD->chessboardToCam.translation.x = mv.at<double>(0, 3);
		PD->chessboardToCam.translation.y = mv.at<double>(1, 3);
		PD->chessboardToCam.translation.z = mv.at<double>(2, 3);
	} else {
		char buff[1024];
		sprintf(buff, "\n\n\n\n !!!! NO CHESSBOARD IN EXTRINSIC IMAGE %d !!!! \n\n\n\n", idx);
		OutputDebugString(buff);
	}

	// compute cam to cam root R/T
	PD->viconToRoom = G3D::CoordinateFrame(
		G3D::Matrix3::identity(),
		G3D::Vector3(3.048, 4.9784, 0.0)
	);
	PD->chessboardToRoom = PD->viconToRoom * PD->chessboardRootToVicon * PD->chessboardToChessboardRoot;
	PD->roomToCamRoot = (PD->viconToRoom * PD->camRootToVicon).inverse();
	PD->camToCamRoot = PD->roomToCamRoot * (PD->chessboardToRoom * PD->chessboardToCam.inverse());
	PD->camToRoom = PD->viconToRoom * PD->camRootToVicon * PD->camToCamRoot;

	extrinsicRT.push_back(PD->camToCamRoot);
	
	// update percentage
	EnterCriticalSection(&csImage);
	showImage = true;
	image = myimage.clone();
	LeaveCriticalSection(&csImage);
	extrinsicPercent = 100 * (++extrinsicCounter / (float)PD->data.rawExtrinsic.size());
	if (extrinsicPercent >= 100) state = MEASUREMENT;

	PD->camToCamRoot = averageCF(extrinsicRT);
}

G3D::CoordinateFrame ComputeState::averageCF(const std::vector<G3D::CoordinateFrame>& exs) {
	G3D::CoordinateFrame res;

	// translation
	double x = 0, y = 0, z = 0;
	for (auto it = exs.begin(); it != exs.end(); ++it) {
		x += it->translation.x;
		y += it->translation.y;
		z += it->translation.z;
	}
	res.translation.x = x / exs.size();
	res.translation.y = y / exs.size();
	res.translation.z = z / exs.size();

	// quaternion average
	// based on http://www.acsu.buffalo.edu/~johnc/ave_sfm07.pdf
	cv::Mat M = cv::Mat::zeros(4, 4, CV_32FC1);
	for (auto it = exs.begin(); it != exs.end(); it++) {
		// convert to Quat
		G3D::Quat q(it->rotation);

		// convert 4x1 vector
		cv::Mat v(4, 1, CV_32FC1);
		v.at<float>(0, 0) = q.x;
		v.at<float>(1, 0) = q.y;
		v.at<float>(2, 0) = q.z;
		v.at<float>(3, 0) = q.w;

		// add to M
		M += v * v.t();
	}
	// eigen decomposition
	cv::Mat evals, evecs;
	cv::eigen(M, true, evals, evecs);
	// convert to Quat
	G3D::Quat q;
	if (evecs.type() == CV_32FC1) {
		q.x = evecs.at<float>(0, 0);
		q.y = evecs.at<float>(0, 1);
		q.z = evecs.at<float>(0, 2);
		q.w = evecs.at<float>(0, 3);
	} else if (evecs.type() == CV_64FC1) {
		q.x = evecs.at<double>(0, 0);
		q.y = evecs.at<double>(0, 1);
		q.z = evecs.at<double>(0, 2);
		q.w = evecs.at<double>(0, 3);
	}
	// convert to Matrix3
	res.rotation = G3D::Matrix3(q);

	return res;
}

void ComputeState::measureController() {
	RawProjData * p = &PD->data.rawProj[measureCounterProj];

	// load data
	cv::Mat myimage = cv::imread(p->measurements[measureCounter].imagePath);

	PD->camRootToVicon = toCF(
		p->measurements[measureCounter].camRot,
		p->measurements[measureCounter].camTrans);
	PD->chessboardRootToVicon = toCF(
		p->measurements[measureCounter].chessRot,
		p->measurements[measureCounter].chessTrans);
	PD->chessboardToChessboardRoot = toCF(
		p->measurements[measureCounter].chessRootRot,
		p->measurements[measureCounter].chessRootTrans);
	int regionW = p->measurements[measureCounter].projRegionWidth;
	int regionH = p->measurements[measureCounter].projRegionHeight;

	// compute derived frames
	PD->camToRoom = PD->viconToRoom * PD->camRootToVicon * PD->camToCamRoot;
	PD->chessboardToRoom = PD->viconToRoom * PD->chessboardRootToVicon * PD->chessboardToChessboardRoot;

	// find grid
	bool found;
	// FIXME: this function (and others like it) should be in a common class, like AppState or
	// triangulate ...
	std::vector<cv::Point2f> imgPts = MeasureState::findGrid(myimage, regionW, regionH, found);

	// create tempMeasures array if first measure in projector
	if (measureCounter == 0) {
		// create new temp measures array for this projector
		tempMeasures.push_back(std::vector<CalibMeasure>());
	}

	// convert to measurement
	if (found) {
		std::vector<cv::Point2i> grid = p->measurements[measureCounter].projRegionGrid;
		std::vector<cv::Point2f> screen = p->measurements[measureCounter].projRegionScreen;

		for (int y = 0; y < regionH; y++) {
			for (int x = 0; x < regionW; x++) {
				cv::Point2i gridpt = grid[x + y * regionW];
				cv::Point2f screenpt = screen[x + y * regionW];
				cv::Point2f campt = imgPts[x + y * regionW];
				cv::Mat rot(3, 3, CV_32FC1);
				G3D::Matrix3 mr = PD->camToRoom.rotation;
				for (int r = 0; r < 3; r++) {
					for (int c = 0; c < 3; c++) {
						rot.at<float>(r, c) = mr[r][c];
					}
				}
				cv::Point3f trans(PD->camToRoom.translation.x,
								  PD->camToRoom.translation.y,
								  PD->camToRoom.translation.z);

				// add to temp measures array
				tempMeasures[measureCounterProj].push_back(CalibMeasure(gridpt,
					screenpt, campt, rot, trans, 
					p->measurements[measureCounter].groupId));
			}
		}
	}

	EnterCriticalSection(&csImage);
	showImage = true;
	image = myimage.clone();
	LeaveCriticalSection(&csImage);

	// move to next measurement & update percentage
	int dataSize = p->measurements.size();
	float chunkSize = 100.0f / PD->data.rawProj.size();
	float off = measureCounterProj * chunkSize;
	measurePercent = off + chunkSize * (++measureCounter / (float)dataSize);

	if (measureCounter >= dataSize) {
		// move to next projector
		measureCounter = 0;
		measureCounterProj++;
		if (measureCounterProj >= PD->data.rawProj.size()) {
			// done
			state = MEASUREMENT_AVG;
		}
	}
}

void ComputeState::measureAvgController() {
	showImage = false;

	// get vector of temp measures
	const std::vector<CalibMeasure>& tm = tempMeasures[measureAvgCounterProj];

	// get raw projector object and computed projector object
	const RawProjData * rpd = &PD->data.rawProj[measureAvgCounterProj];
	if (measureAvgCounterX == 0 && measureAvgCounterY == 0) {
		// create ProjectorData object
		PD->data.compProj.push_back(ProjectorData());
	}
	ProjectorData * pd = &PD->data.compProj[measureAvgCounterProj];

	// copy projector object data if first time through this projector
	if (measureAvgCounterX == 0 && measureAvgCounterY == 0) {
		pd->left = rpd->left;
		pd->top = rpd->top;
		pd->width = rpd->width;
		pd->height = rpd->height;
		pd->gridBounds = rpd->gridBounds;
		pd->numPointsX = rpd->numPointsX;
		pd->numPointsY = rpd->numPointsY;
	}

	// average grid point groups into single measures
	std::vector<int> groups = findGroups(tm, measureAvgCounterX, measureAvgCounterY);
	for (auto it = groups.begin(); it != groups.end(); ++it) {
		CalibMeasure avg = computeAvgMeasure(tm, measureAvgCounterX, measureAvgCounterY, *it);
		pd->measurements.push_back(avg);
	}


	// move to next grid point
	measureAvgCounterX++;
	if (measureAvgCounterX >= rpd->numPointsX) {
		measureAvgCounterX = 0;
		measureAvgCounterY++;
		if (measureAvgCounterY >= rpd->numPointsY) {
			measureAvgCounterY = 0;
			measureAvgCounterProj++;
			if (measureAvgCounterProj >= tempMeasures.size()) {
				measureAvgPercent = 100;
				state = TORAYS;
			}
		}
	}

	if (state == MEASUREMENT_AVG) {
		int dataSize = rpd->numPointsX * rpd->numPointsY;
		float chunkSize = 100.0f / PD->data.rawProj.size();
		float off = measureAvgCounterProj * chunkSize;
		measureAvgPercent = off + chunkSize * 
			((float)(measureAvgCounterY * rpd->numPointsX + measureAvgCounterX) / (float)dataSize);
	}
}

void ComputeState::rayController() {
	showImage = false;

	// get pointer to projector
	ProjectorData * p = &PD->data.compProj[rayCounterProj];

	// get measure
	// FIXME: this hack shows that compute speed is tied to framerate,
	// use a background thread to avoid this stupid error
	for (int jj = 0; jj < 50 && rayCounterProj < PD->data.compProj.size(); jj++) {
		CalibMeasure& m = p->measurements[rayCounter];

		// compute ray
		m.ray = tri.computeRay(m, PD->data.compIntrinsic, PD->data.compDistortion);
	
		// advance to next measure
		rayCounter++;
		if (rayCounter >= p->measurements.size()) {
			rayCounter = 0;
			rayCounterProj++;

			if (rayCounterProj >= PD->data.compProj.size()) {
				rayPercent = 100;
				state = TOPOINTS;
			}
		}
	}

	if (state == TORAYS) {
		int dataSize = p->measurements.size();
		float chunkSize = 100.0f / PD->data.rawProj.size();
		float off = rayCounterProj * chunkSize;

		rayPercent = off + chunkSize * ((float)rayCounter / (float)dataSize);
	}
}

void ComputeState::pointController() {
	showImage = false;

	// get pointer to projector data
	ProjectorData * p = &PD->data.compProj[pointCounterProj];

	// find measures for point (pointCounterX, pointCounterY)
	std::vector<CalibMeasure> measures = gatherMeasurements(pointCounterX,
		pointCounterY, p->measurements);
	CalibResult pt = tri.computePoint(measures);
	p->mesh.push_back(pt);

	// advance to next point
	pointCounterX++;
	if (pointCounterX >= p->numPointsX) {
		pointCounterX = 0;
		pointCounterY++;
		
		if (pointCounterY >= p->numPointsY) {
			pointCounterY = 0;
			pointCounterProj++;

			if (pointCounterProj >= PD->data.compProj.size()) {
				pointPercent = 100;
				PD->data.writeXML("test.xml.gz"); // for temp debugging
				state = DO_BUNDLE_ADJUST ? BUNDLEADJUST : DONE;
			}
		}
	}

	if (state == TOPOINTS) {
		int dataSize = p->numPointsX * p->numPointsY;
		float chunkSize = 100.0f / PD->data.compProj.size();
		float off = pointCounterProj * chunkSize;
		pointPercent = off + chunkSize *
			((float)(pointCounterY * p->numPointsX + pointCounterX) / (float)dataSize);
	}
}

std::vector<int> ComputeState::getGroups(const std::vector<CalibMeasure>& m) {
	std::vector<int> out;
	for (auto it = m.begin(); it != m.end(); ++it) {
		if (std::find(out.begin(), out.end(), it->groupId) == out.end()) {
			out.push_back(it->groupId);
		}
	}

	return out;
}

cv::Mat Rx(double angle_radians) {
	return cv::Mat_<float>(3, 3) <<
		1, 0,                  0,
		0, cos(angle_radians), -sin(angle_radians),
		0, sin(angle_radians),  cos(angle_radians);
}

cv::Mat Ry(double angle_radians) {
	return cv::Mat_<float>(3, 3) <<
		cos(angle_radians), 0,                   sin(angle_radians),
		0,                  cos(angle_radians), -sin(angle_radians),
		0,                  sin(angle_radians),  cos(angle_radians);
}

cv::Mat Rz(double angle_radians) {
	return cv::Mat_<float>(3, 3) <<
		cos(angle_radians),  0, sin(angle_radians),
		0,                   1, 0,
		-sin(angle_radians), 0, cos(angle_radians);
}

// based off the SfMTopExample at:
// https://github.com/royshil/SfM-Toy-Library
void ComputeState::SSBABundle() {
#ifdef HAS_SSBA
	using namespace V3D;

	for (int i = 0; i < PD->data.compProj.size(); ++i) {
		ProjectorData& p = PD->data.compProj[i];
		vector<int> groups = getGroups(p.measurements);
		int numImages = groups.size();
		int meshW = p.numPointsX;
		int meshH = p.numPointsY;
		int numPoints = p.mesh.size();
		StdDistortionFunction distortion;

		// convert camera intrinsics to SSBA data structures
		Matrix3x3d KMat;
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 3; ++c) {
				KMat[r][c] = PD->data.compIntrinsic.at<double>(r, c);
			}
		}
		
		// normalize intrinsic to have unit focal length
		double const f0 = KMat[0][0];
		cout << "intrinsic before SSBA = "; displayMatrix(KMat);
		Matrix3x3d Knorm = KMat;
		scaleMatrixIP(1.0 / f0, Knorm);
		Knorm[2][2] = 1.0;

		// sort mesh into row-major order (makes it easier to convert)
		sort(p.mesh.begin(), p.mesh.end(), ComputeState::cmpMesh);

		// convert 3D mesh to SSBA data structures
		vector<Vector3d> Xs(numPoints);
		for (int j = 0; j < numPoints; ++j) {
			Xs[j][0] = p.mesh[j].worldPoint.x;
			Xs[j][1] = p.mesh[j].worldPoint.y;
			Xs[j][2] = p.mesh[j].worldPoint.z;
		}

		// convert camera locations to SSBA data structures
		vector<CameraMatrix> cams(numImages);
		for (int j = 0; j < numImages; ++j) {
			Matrix3x3d R;
			Vector3d T;

			for (int k = 0; k < p.measurements.size(); ++k) {
				if (p.measurements[k].groupId == groups[j]) {
					cv::Mat RR = p.measurements[k].rotation;
					cv::Point3f TT = p.measurements[k].translation;

					for (int r = 0; r < 3; ++r)
						for (int c = 0; c < 3; ++c)
							R[r][c] = RR.at<float>(r, c);

					T[0] = TT.x; T[1] = TT.y; T[2] = TT.z;

					cams[j].setIntrinsic(Knorm);
					cams[j].setRotation(R);
					cams[j].setTranslation(T);

					break;
				}
			}
		}

		// convert 2D measurements to SSBA data structures
		// make sure to 1) normalize to unit focal length and 2) undistort points
		int numMeasures = p.measurements.size();
		vector<Vector2d> measures; measures.reserve(numMeasures);
		vector<int> correspondingView; correspondingView.reserve(numMeasures);
		vector<int> correspondingPoint; correspondingPoint.reserve(numMeasures);
		for (int j = 0; j < numMeasures; ++j) {
			int point = p.measurements[j].gridPoint.y * meshW + p.measurements[j].gridPoint.x;
			int view = p.measurements[j].groupId;
			vector<cv::Point2f> pts; pts.push_back(p.measurements[j].pixelCamera);
			vector<cv::Point2f> undist;
			Vector3d pt, npt;

			cv::undistortPoints(pts, undist, PD->data.compIntrinsic, PD->data.compDistortion);

			pt[0] = undist[0].x;
			pt[1] = undist[0].y;
			pt[2] = 1.0;

			scaleVectorIP(1.0 / f0, pt);
			measures.push_back(Vector2d(pt[0], pt[1]));
			correspondingView.push_back(view);
			correspondingPoint.push_back(point);
		}

		// BUNDLE ADJUST !
		bool good_adjustment = true;
		double inlierThreshold = 2.0 / fabs(f0);
		Matrix3x3d K0 = cams[0].getIntrinsic();
		{
			ScopedBundleExtrinsicNormalizer extNorm(cams, Xs);
			ScopedBundleIntrinsicNormalizer intNorm(cams, measures, correspondingView);
			CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_FOCAL_LENGTH_PP,
				inlierThreshold, K0, distortion, cams, Xs, measures,
				correspondingView, correspondingPoint);
			opt.tau = BUNDLE_ERR;
			opt.maxIterations = BUNDLE_ITERATIONS;
			opt.minimize();
			good_adjustment = (opt.status != 2);
		}

		cout << "refined K = "; displayMatrix(K0);

		// get all the results back into our data structures
		for (int i = 0; i < numImages; ++i) cams[i].setIntrinsic(K0);

		Matrix3x3d Knew = K0;
		scaleMatrixIP(f0, Knew);
		Knew[2][2] = 1.0;
		cout << "Knew = "; displayMatrix(Knew);

		if (good_adjustment) {
			// extract 3D points
			for (int j = 0; j < Xs.size(); ++j) {
				p.mesh[j].worldPoint.x = Xs[j][0];
				p.mesh[j].worldPoint.y = Xs[j][1];
				p.mesh[j].worldPoint.z = Xs[j][2];
			}

			// extract adjusted camera locations
			for (int j = 0; j < numImages; ++j) {
				Matrix3x3d R = cams[i].getRotation();
				Vector3d T = cams[i].getTranslation();

				for (int k = 0; k < p.measurements.size(); ++k) {
					CalibMeasure& m = p.measurements[k];
					if (m.groupId == groups[j]) {
						// rotation
						for (int r = 0; r < 3; ++r)
							for (int c = 0; c < 3; ++c)
								m.rotation.at<float>(r, c) = R[r][c];

						// translation
						m.translation.x = T[0];
						m.translation.y = T[1];
						m.translation.z = T[2];
					}
				}
			}

			// extract camera intrinsics
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					PD->data.compIntrinsic.at<double>(r, c) = Knew[r][c];
		}
	}
#endif
}

void ComputeState::bundleController() {
	if (BUNDLE_TYPE == 3) {
		SSBABundle();
		state = DONE;
		return;
	}
	for (int i = 0; i < PD->data.compProj.size(); ++i) {
		// LevMarqSparse takes 8 params
		// 1: Nx3 3D world space object points
		// 2: MxNx2 2D image points of N object points for M cameras
		// 3: MxNx1 visibility matrix, [i][j] = 1 when object point i is visible by camera j.
		// 4: Mx3x3 intrinsic camera matrix for each image
		// 5: Mx3x3 rotation matrix for each image
		// 6: Mx3x1 translation vector for each image
		// 7: Mx(4,5,8)x1 distortion vector for each image
		// 8: termination criteria
		std::vector<cv::Point3d> world_points;
		std::vector<std::vector<cv::Point2d>> image_points;
		std::vector<std::vector<int>> visibility;
		std::vector<cv::Mat> cameraMatrix;
		std::vector<cv::Mat> R;
		std::vector<cv::Mat> T;
		std::vector<cv::Mat> distortion;
		cv::TermCriteria criteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, BUNDLE_ITERATIONS, BUNDLE_ERR);

		// convert from our structures to opencv structures --------------------
		ProjectorData * p = &PD->data.compProj[i];
		std::vector<int> groups = getGroups(p->measurements);
		int numImages = groups.size();
		int meshW = p->numPointsX;
		int meshH = p->numPointsY;
		int numPoints = p->mesh.size();

		// sort mesh into row-major order (makes it easier to convert)
		std::sort(p->mesh.begin(), p->mesh.end(), ComputeState::cmpMesh);

		// world points
		for (int j = 0; j < numPoints; ++j) {
			cv::Point3d pt = p->mesh[j].worldPoint;
			world_points.push_back(pt);
		}

		// image coords
		for (int j = 0; j < numImages; ++j) {
			image_points.push_back(std::vector<cv::Point2d>());
			for (int k = 0; k < numPoints; k++) {
				image_points[j].push_back(cv::Point2d());
			}
		}
		for (int j = 0; j < p->measurements.size(); ++j) {
			CalibMeasure& m = p->measurements[j];
			cv::Point2d pt = m.pixelCamera;
			for (int k = 0; k < numImages; ++k) {
				if (groups[k] == m.groupId) {
					int idx = m.gridPoint.y * meshW + m.gridPoint.x;
					image_points[k][idx] = pt;
					break;
				}
			}
		}

		// visibility
		for (int j = 0; j < numImages; ++j) {
			visibility.push_back(std::vector<int>());
			for (int k = 0; k < numPoints; k++) {
				visibility[j].push_back(0);
			}
			RawDataPiece& rdp = PD->data.rawProj[i].measurements[j];
			for (auto it = rdp.projRegionGrid.begin(); it != rdp.projRegionGrid.end(); ++it) {
				int idx = it->y * meshW + it->x;
				visibility[j][idx] = 1;
			}
		}

		// rotation and translation
		for (int j = 0; j < numImages; ++j) {
			for (int k = 0; k < p->measurements.size(); ++k) {
				if (p->measurements[k].groupId == groups[j]) {
					cv::Mat RR(3, 1, CV_64F);
					cv::Rodrigues(p->measurements[k].rotation, RR);
					R.push_back(RR);

					cv::Mat TT(3, 1, CV_64F);
					TT = (cv::Mat_<double>(3, 1) << 
						p->measurements[k].translation.x,
						p->measurements[k].translation.y,
						p->measurements[k].translation.z);
					T.push_back(TT);
					break;
				}
			}
		}
		
		// camera info
		for (int j = 0; j < numImages; ++j) {
			cameraMatrix.push_back(PD->data.compIntrinsic);
			distortion.push_back(PD->data.compDistortion);
		}

		// run!
		if (BUNDLE_TYPE == 1) {
			cv::LevMarqSparse lms;
			lms.bundleAdjust(world_points, image_points, visibility, 
							 cameraMatrix, R, T, distortion, criteria);
		} else if (BUNDLE_TYPE == 2) {
#ifdef HAS_CVSBA
			cvsba::Sba sba;

			cvsba::Sba::Params params;
			params.fixedDistortion = 5;
			params.fixedIntrinsics = 5;
			params.iterations = BUNDLE_ITERATIONS;
			params.minError = BUNDLE_ERR;
			params.type = cvsba::Sba::MOTIONSTRUCTURE;
			//params.type = cvsba::Sba::STRUCTURE;
			params.verbose = true;
			sba.setParams(params);

			sba.run(world_points, image_points, visibility, cameraMatrix,
					R, T, distortion);
#endif
		}

		// extract 3D point results
		for (int j = 0; j < numPoints; ++j) {
			p->mesh[j].worldPoint = world_points[j];
		}

		// extract camera rotation results
		for (int j = 0; j < numImages; ++j) {
			cv::Mat_<double> RR = R[j];
			for (int k = 0; k < p->measurements.size(); ++k) {
				if (p->measurements[k].groupId == groups[j]) {
					for (int r = 0; r < 3; r++)
						for (int c = 0; c < 3; c++)
							p->measurements[k].rotation.at<float>(r, c) = RR.at<double>(r, c);
				}
			}
		}

		// extract camera translation results
		for (int j = 0; j < numImages; ++j) {
			cv::Mat_<double> TT = T[j];
			for (int k = 0; k < p->measurements.size(); ++k) {
				if (p->measurements[k].groupId == groups[j]) {
					p->measurements[k].translation.x = TT.at<double>(0, 0);
					p->measurements[k].translation.y = TT.at<double>(1, 0);
					p->measurements[k].translation.z = TT.at<double>(2, 0);
				}
			}
		}

		// progress bar
		bundlePercent = (float)(i+1) / (float)(PD->data.compProj.size());
	}

	// next state.
	state = DONE;
}

void ComputeState::doneController() {
	if (!dataSaved) {
		// save ------------------------------------------------------------
		// all data
		PD->data.writeXML("results.xml.gz");

		// projector meshes
		for (int i = 0; i < PD->data.compProj.size(); ++i) {
			ProjectorData * p = &PD->data.compProj[i];
		
			std::sort(p->mesh.begin(), p->mesh.end(), ComputeState::cmpMesh);

			char filename[512];
			sprintf(filename, "proj_%d.obj", i);
			p->writeMesh(filename);
		}
		dataSaved = true;
		state = REALLY_DONE;
	}
}

std::vector<CalibMeasure> ComputeState::gatherMeasurements(int gx, int gy, int gid,
	const std::vector<CalibMeasure>& m) {

	std::vector<CalibMeasure> out;
	for (auto it = m.begin(); it != m.end(); ++it) {
		if (it->gridPoint.x == gx && it->gridPoint.y == gy && it->groupId == gid) {
			out.push_back(*it);
		}
	}

	return out;
}

std::vector<int> ComputeState::findGroups(const std::vector<CalibMeasure>& m, int x, int y) {
	std::vector<int> out;
	for (auto it = m.begin(); it != m.end(); it++) {
		if (it->gridPoint.x == x && it->gridPoint.y == y) {
			if (std::find(out.begin(), out.end(), it->groupId) == out.end()) {
				out.push_back(it->groupId);
			}
		}
	}
	return out;
}


CalibMeasure ComputeState::computeAvgMeasure(const std::vector<CalibMeasure>& mm, int x, int y, int id) {
	CalibMeasure out;
	std::vector<CalibMeasure> m = gatherMeasurements(x, y, id, mm);
	if (m.size() == 0) return out;

	// copy grid point, group id, and pixel screen
	// as these don't change between measurements
	out.gridPoint = cv::Point2i(x, y);
	out.groupId = id;
	out.pixelScreen = m[0].pixelScreen;

	// simple average for translation and pixelcamera
	out.pixelCamera = cv::Point2f(0, 0);
	out.translation = cv::Point3f(0, 0, 0);
	for (auto it = m.begin(); it != m.end(); it++) {
		out.pixelCamera += it->pixelCamera;
		out.translation += it->translation;
	}
	out.pixelCamera *= 1.0f / m.size();
	out.translation *= 1.0f / m.size();

	// quaternion average
	// based on http://www.acsu.buffalo.edu/~johnc/ave_sfm07.pdf
	cv::Mat M = cv::Mat::zeros(4, 4, CV_32FC1);
	for (auto it = m.begin(); it != m.end(); it++) {
		// convert to Matrix3
		cv::Mat cvm = it->rotation;
		G3D::Matrix3 m3;
		for (int r = 0; r < 3; r++) 
			for (int c = 0; c < 3; c++)
				m3[r][c] = cvm.at<float>(r, c);
		// convert to Quat
		G3D::Quat q(m3);

		// convert 4x1 vector
		cv::Mat v(4, 1, CV_32FC1);
		v.at<float>(0, 0) = q.x;
		v.at<float>(1, 0) = q.y;
		v.at<float>(2, 0) = q.z;
		v.at<float>(3, 0) = q.w;

		// add to M
		M += v * v.t();
	}
	// eigen decomposition
	cv::Mat evals, evecs;
	cv::eigen(M, true, evals, evecs);
	// convert to Quat
	G3D::Quat q;
	if (evecs.type() == CV_32FC1) {
		q.x = evecs.at<float>(0, 0);
		q.y = evecs.at<float>(0, 1);
		q.z = evecs.at<float>(0, 2);
		q.w = evecs.at<float>(0, 3);
	} else if (evecs.type() == CV_64FC1) {
		q.x = evecs.at<double>(0, 0);
		q.y = evecs.at<double>(0, 1);
		q.z = evecs.at<double>(0, 2);
		q.w = evecs.at<double>(0, 3);
	}
	// convert to Matrix3
	G3D::Matrix3 m3(q);
	// convert to Mat3
	cv::Mat oM(3,3,CV_32FC1);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) 
			oM.at<float>(r, c) = m3[r][c];
	out.rotation = oM;

	return out;
}

std::vector<CalibMeasure> ComputeState::gatherMeasurements(int gx, int gy,
	const std::vector<CalibMeasure>& m) {

	std::vector<CalibMeasure> out;
	for (auto it = m.begin(); it != m.end(); ++it) {
		if (it->gridPoint.x == gx && it->gridPoint.y == gy) {
			out.push_back(*it);
		}
	}

	return out;
}

// a < b
bool ComputeState::cmpMesh(CalibResult& a, CalibResult& b) {
	int y_diff = a.gridPoint.y - b.gridPoint.y;
	int x_diff = a.gridPoint.x - b.gridPoint.x;
	if (y_diff != 0) {
		return y_diff < 0;
	} else {
		return x_diff < 0;
	}
}

DWORD WINAPI ComputeState::t_Worker(LPVOID lpParam) {
	ComputeState *pC = (ComputeState *)lpParam;

	// FIXME: this can be done a lot nicer
	// this design is remaining from when single-thread
	// this is first step towards fast multi-thread design
	while (pC->state != REALLY_DONE) {
		// run controller for one step
		switch (pC->state) {
			case INTRINSIC:			pC->intrinsicController();	break;
			case EXTRINSIC:			pC->extrinsicController();	break;
			case MEASUREMENT:		pC->measureController();	break;
			case MEASUREMENT_AVG:	pC->measureAvgController(); break;
			case TORAYS:			pC->rayController();		break;
			case TOPOINTS:			pC->pointController();		break;
			case BUNDLEADJUST:		pC->bundleController();		break;
			case DONE:				pC->doneController();		break;
		}
	}

	return 0;
}