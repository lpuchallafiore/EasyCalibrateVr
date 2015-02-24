/* Classes for stereo and monocular camera calibration (intrinsic & extrinsic)
 * Requires OpenCV 2.x
 * author: Loren Puchalla Fiore <loren.fiore@gmail.com>
 * version: April 2013
 */

#include "stdafx.h"
#include "Calibration.h"

const int OPENCV_KEYCODE_ENTER = 10;
const int OPENCV_KEYCODE_ESC = 27;
// used when processing video
const int BOARD_DT = 5;
const int SKIP_DT = 15;
const int N_BOARDS = 75;
// used when processing images
const int DISPLAY_DT = 200;

using namespace std;
using namespace cv;

// ----- Monocular Calibration -------------------------------------------------
// argW, argH - number of interior corners in the chessboard pattern
MonoCalibration::MonoCalibration(int argW /* = 5 */,
								 int argH /* = 8 */,
								 float argSS /* = 0.0254 */) {
	m_BoardW = argW;
	m_BoardH = argH;
	m_SquareSize = argSS;
	m_BoardN = m_BoardW * m_BoardH;
	m_Name = "MonoCalibration";
	m_Description = "";
	m_ShowImage = false;
}

MonoCalibration::~MonoCalibration() {
}

bool MonoCalibration::load(const std::string& filename /* = "monocalib.yml" */) {
	bool ret = false;
	FileStorage fs(filename, FileStorage::READ);

	if (fs.isOpened()) {
		fs["Intrinsic"] >> m_Intrinsic;
		fs["Distortion"] >> m_Distortion;

		fs["RotationVectors"] >> m_RotVecs;
		fs["TranslationVectors"] >> m_TransVecs;

		fs["ImageWidth"] >> m_ImageSize.width;
		fs["ImageHeight"] >> m_ImageSize.height;
		fs["BoardWidth"] >> m_BoardW;
		fs["BoardHeight"] >> m_BoardH;
		fs["SquareSize"] >> m_SquareSize;

		fs["AverageReprojectionError"] >> m_AvgError;

		fs["Name"] >> m_Name;
		fs["Description"] >> m_Description;

		precomputeValues();

		ret = true;
	} else {
		cout << "[Error] Cannot open '" << filename 
			<< "' for reading." << endl;
	}

	return ret;
}

bool MonoCalibration::save(const std::string& filename /* = "monocalib.yml" */) {
	bool ret = false;
	FileStorage fs(filename, FileStorage::WRITE);

	if (fs.isOpened()) {
		fs << "Intrinsic" << m_Intrinsic;
		fs << "Distortion" << m_Distortion;

		fs << "RotationVectors" << "[";
		for (int i = 0; i < (int)m_RotVecs.size(); i++) {
			fs << m_RotVecs[i];
		}
		fs << "]";

		fs << "TranslationVectors" << "[";
		for (int i = 0; i < (int)m_TransVecs.size(); i++) {
			fs << m_TransVecs[i];
		}
		fs << "]";

		fs << "ImageWidth" << m_ImageSize.width;
		fs << "ImageHeight" << m_ImageSize.height;
		fs << "BoardWidth" << m_BoardW;
		fs << "BoardHeight" << m_BoardH;
		fs << "SquareSize" << m_SquareSize;

		fs << "AverageReprojectionError" << m_AvgError;

		fs << "Name" << m_Name;
		fs << "Description" << m_Description;

		ret = true;
	} else {
		cout << "[Error] Cannot open '" << filename 
			<< "' for writing." << endl;
	}

	return ret;
}

double MonoCalibration::calibrateFromImages(std::vector<std::string> images) {
	m_ImageCount = images.size();
	startCalibration();

	namedWindow("Calibration", WINDOW_AUTOSIZE);

	for (int i = 0; i < m_ImageCount; i++) {
		cout << "Processing image " << i + 1 << " ... ";
		Mat image = imread(images[i]);

		if (image.data == NULL) {
			cout << "error: cannot open " << images[i] << endl;
			continue;
		}

		if (!imageLoopCommon(image, i)) {
			return -1;
		}
	} // End collection loop

	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	destroyWindow("Calibration");

	return m_AvgError;
}

double MonoCalibration::calibrateFromImages(std::vector<cv::Mat> images) {
	m_ImageCount = images.size();
	startCalibration();

	namedWindow("Calibration");

	for (int i = 0; i < m_ImageCount; i++) {
		cout << "Processing image " << i + 1 << " ... ";
		if (!imageLoopCommon(images[i], i)) {
			return -1;
		}
	} // End collection loop

	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	destroyWindow("Calibration");

	return m_AvgError;
}

double MonoCalibration::calibrateFromVideo(std::string filename) {
	VideoCapture cap(filename);

	if (cap.isOpened()) {
		return calibrateFromCapture(cap);
	} else {
		cout << "[Error] Cannot open '" << filename << "'" << endl;
		return -1;
	}
}

double MonoCalibration::calibrateFromCam(int camNum /* = 0 */) {
	VideoCapture cap(camNum);

	// hack - FIXME
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

	if (cap.isOpened()) {
		return calibrateFromCapture(cap);
	} else {
		cout << "[Error] Cannot open video device #" << camNum << endl;
		return -1;
	}
}

double MonoCalibration::calibrateFromCapture(cv::VideoCapture& cap) {
	startCalibration();

	namedWindow("Calibration", WINDOW_AUTOSIZE);

	int frame = 0;
	Mat image;
	cap.read(image);
	while (m_Successes < N_BOARDS && image.data != NULL) {
		// hack
		stringstream ss; ss << "intrinsic_" << frame << ".jpg";
		imwrite(ss.str(), image);

		if (frame++ % SKIP_DT != 0) {
			cap >> image;
			continue;
		}
		cout << "Processing image " << frame + 1 << " ... ";

		if (!imageLoopCommon(image, frame, true)) {
			return -1;
		}
		cap >> image;
	} // End collection loop

	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	cout << "Found " << m_Successes << " valid point sets." << endl;
	destroyWindow("Calibration");

	return m_AvgError;
}

bool MonoCalibration::imageLoopCommon(cv::Mat image, 
									  int i, bool video /* = false */) {
	// process image
	Scalar color;
	if (processCalibration(image)) {
		color = CV_RGB(0, 255, 0);
		cout << "pattern found." << endl;
	} else {
		color = CV_RGB(255, 0, 0);
		cout << "pattern not found." << endl;
	}
	// draw text // FIXME: stringstream
	char buff[256] = {0};
	if (video) {
		sprintf_s(buff, 256, "%d/%d", m_Successes, N_BOARDS);
	} else {
		sprintf_s(buff, 256, "%d/%d", i + 1, m_ImageCount);
	}
	putText(m_SmallImage, buff, Point(20, m_SmallImage.size().height - 20),
			FONT_HERSHEY_SIMPLEX, 1, color, 2, CV_AA);

	if (m_ShowImage) {
		imshow("Calibration", m_SmallImage);
		// Handle pause/unpause and ESC
		int keycode = (waitKey(video ? BOARD_DT : DISPLAY_DT) & 255);
		if (keycode == 'p') {
			keycode = 0;
			while(keycode != 'p' && keycode != OPENCV_KEYCODE_ESC) {
				keycode = waitKey(250);
			}
		}
		if (keycode == OPENCV_KEYCODE_ESC) {
			destroyWindow("Calibration");
			return false;
		}
	}
	return true;
}

void MonoCalibration::undistortImage(cv::Mat image) {
	Mat t = image.clone();
	remap(t, image, m_UMapX, m_UMapY, INTER_LANCZOS4);
}

void MonoCalibration::startCalibration() {
	m_Intrinsic = Mat::zeros(3, 3, CV_64FC1);
	m_Distortion = Mat::zeros(8, 1, CV_64FC1);
	m_Successes = 0;
}

bool MonoCalibration::processCalibration(cv::Mat& image) {
	bool ret;
	vector<Point2f> corners;

	// Resize the image for display
	int www = 640; // FIXME: magic number
	float scale = ((float)www / image.size().width);
	resize(image, m_SmallImage,
		   Size(www, (int)(scale * image.size().height)),
		   0, 0, scale < 1.0f ? INTER_AREA : INTER_CUBIC);

	// Find chessboard corners
	bool found = findChessboardCorners(image,
									   Size(m_BoardW, m_BoardH),
									   corners,
									   CALIB_CB_ADAPTIVE_THRESH
									   | CALIB_CB_NORMALIZE_IMAGE
									   | CALIB_CB_FAST_CHECK
									   | CALIB_CB_FILTER_QUADS
									   // FIXME: user options
									   );

	if (!found) {
		return false;
	}

	// Get subpixel accuracy on those corners
	Mat grayImage;
	cvtColor(image, grayImage, CV_BGR2GRAY);
	cornerSubPix(grayImage, corners, Size(11, 11), Size(-1, -1),
				 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	// Draw it
	vector<Point2f> smallCorners;
	for (int i = 0; i < (int)corners.size(); i++) {
		Point2f p;
		p.x = corners[i].x * scale;
		p.y = corners[i].y * scale;
		smallCorners.push_back(p);
	}
	drawChessboardCorners(m_SmallImage, Size(m_BoardW, m_BoardH),
						  Mat(smallCorners), found);

	// If we have a good board, add it to our data
	if (corners.size() == m_BoardN) {
		vector<Point3f> worldPoints;

		for (int i = 0; i < m_BoardN; i++) {
			Point3f p;
			p.x = (float) (i % m_BoardW) * m_SquareSize;
			p.y = (float) (i / m_BoardW) * m_SquareSize;
			p.z = 0.0f;
			worldPoints.push_back(p);
		}

		m_ObjPoints.push_back(worldPoints);
		m_ImagePoints.push_back(corners);
		m_Successes++;
		ret = true;
	} else {
		ret = false;
	}

	m_ImageSize = image.size();
	return ret;
}

void MonoCalibration::precomputeValues() {
	// precompute the undistortion mapping
	initUndistortRectifyMap(m_Intrinsic, m_Distortion, Mat(),
							// FIXME: user option
							//getOptimalNewCameraMatrix(m_Intrinsic,
							//                          m_Distortion,
							//                          m_ImageSize,
							//                          0,
							//                          m_ImageSize),
							m_Intrinsic,
							m_ImageSize, CV_32FC1,
							m_UMapX, m_UMapY);
}

double MonoCalibration::endCalibration() {
	// At this point we have all of the points we need
	// Initialize the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0
	m_Intrinsic.at<double>(0, 0) = 1.0f;// * ((float)m_ImageSize.width / (float)m_ImageSize.height);
	m_Intrinsic.at<double>(1, 1) = 1.0f;
	// and initialize primary points
	m_Intrinsic.at<double>(0, 2) = m_ImageSize.width / 2.0f;
	m_Intrinsic.at<double>(1, 2) = m_ImageSize.height / 2.0f;
	// initialize the 1
	m_Intrinsic.at<double>(2, 2) = 1.0f;

	// Calibrate the camera
	double rErr = calibrateCamera(m_ObjPoints, m_ImagePoints, m_ImageSize,
								  m_Intrinsic, m_Distortion,
								  m_RotVecs, m_TransVecs,
								  // FIXME: user configurable
								  0
								  //CV_CALIB_FIX_ASPECT_RATIO
								  //CV_CALIB_USE_INTRINSIC_GUESS
								  // CV_CALIB_RATIONAL_MODEL
								  //| CV_CALIB_FIX_PRINCIPAL_POINT
								  //| CV_CALIB_ZERO_TANGENT_DIST
								  //| CV_CALIB_FIX_K1
								  //| CV_CALIB_FIX_K2
								  | CV_CALIB_FIX_K3
								  | CV_CALIB_FIX_K4
								  | CV_CALIB_FIX_K5
								  | CV_CALIB_FIX_K6
								  );

	//m_Intrinsic.convertTo(m_Intrinsic, CV_64F); // switch to doubles?
	//m_Distortion.convertTo(m_Distortion, CV_64F);
	//for (int i = 0; i < m_RotVecs.size(); i++) {
	//	m_RotVecs[i].convertTo(m_RotVecs[i], CV_64F);
	//}
	//for (int i = 0; i < m_TransVecs.size(); i++) {
	//	m_TransVecs[i].convertTo(m_TransVecs[i], CV_64F);
	//}

	vector<double> reprojErrs;
	rErr = computeReprojectionErrors(m_ObjPoints, m_ImagePoints, m_RotVecs,
									 m_TransVecs, m_Intrinsic, m_Distortion,
									 reprojErrs);

	precomputeValues();

	return rErr;
}

double MonoCalibration::computeReprojectionErrors(
	const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs,
	const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	std::vector<double>& perViewErrors) {

	vector<Point2f> imagePoints2;
	int totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (int i = 0; i < (int)objectPoints.size(); i++ ) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
					  cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr/totalPoints);
}

// from ArUco -----------------------------------------------------------------
double norm( double a, double b, double c )
{
	return( sqrt( a*a + b*b + c*c ) );
}

double dot( double a1, double a2, double a3,
							  double b1, double b2, double b3 )
{
	return( a1 * b1 + a2 * b2 + a3 * b3 );
}

int arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )throw(cv::Exception)
{
	int       r, c;
	double    Cpara[3][4];
	double    rem1, rem2, rem3;

	if ( source[2][3] >= 0 )
	{
		for ( r = 0; r < 3; r++ )
		{
			for ( c = 0; c < 4; c++ )
			{
				Cpara[r][c] = source[r][c];
			}
		}
	}
	else
	{
		for ( r = 0; r < 3; r++ )
		{
			for ( c = 0; c < 4; c++ )
			{
				Cpara[r][c] = -(source[r][c]);
			}
		}
	}

	for ( r = 0; r < 3; r++ )
	{
		for ( c = 0; c < 4; c++ )
		{
			cpara[r][c] = 0.0;
		}
	}
	cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
	trans[2][0] = Cpara[2][0] / cpara[2][2];
	trans[2][1] = Cpara[2][1] / cpara[2][2];
	trans[2][2] = Cpara[2][2] / cpara[2][2];
	trans[2][3] = Cpara[2][3] / cpara[2][2];

	cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
					   Cpara[1][0], Cpara[1][1], Cpara[1][2] );
	rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
	rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
	rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
	cpara[1][1] = norm( rem1, rem2, rem3 );
	trans[1][0] = rem1 / cpara[1][1];
	trans[1][1] = rem2 / cpara[1][1];
	trans[1][2] = rem3 / cpara[1][1];

	cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
					   Cpara[0][0], Cpara[0][1], Cpara[0][2] );
	cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
					   Cpara[0][0], Cpara[0][1], Cpara[0][2] );
	rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
	rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
	rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
	cpara[0][0] = norm( rem1, rem2, rem3 );
	trans[0][0] = rem1 / cpara[0][0];
	trans[0][1] = rem2 / cpara[0][0];
	trans[0][2] = rem3 / cpara[0][0];

	trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
	trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
				   - cpara[0][2]*trans[2][3]) / cpara[0][0];

	for ( r = 0; r < 3; r++ )
	{
		for ( c = 0; c < 3; c++ )
		{
			cpara[r][c] /= cpara[2][2];
		}
	}

	return 0;
}

void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )throw(cv::Exception)
{

	double   icpara[3][4];
	double   trans[3][4];
	double   p[3][3], q[4][4];
	int      i, j;

	cparam[0][2] *= -1.0;
	cparam[1][2] *= -1.0;
	cparam[2][2] *= -1.0;

	if ( arParamDecompMat(cparam, icpara, trans) < 0 )
		throw cv::Exception(9002,"parameter error","MarkerDetector::argConvGLcpara2",__FILE__,__LINE__);

	for ( i = 0; i < 3; i++ )
	{
		for ( j = 0; j < 3; j++ )
		{
			p[i][j] = icpara[i][j] / icpara[2][2];
		}
	}
	q[0][0] = (2.0 * p[0][0] / width);
	q[0][1] = (2.0 * p[0][1] / width);
	q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
	q[0][3] = 0.0;

	q[1][0] = 0.0;
	q[1][1] = (2.0 * p[1][1] / height);
	q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
	q[1][3] = 0.0;

	q[2][0] = 0.0;
	q[2][1] = 0.0;
	q[2][2] = (gfar + gnear)/(gfar - gnear);
	q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

	q[3][0] = 0.0;
	q[3][1] = 0.0;
	q[3][2] = 1.0;
	q[3][3] = 0.0;

	for ( i = 0; i < 4; i++ )
	{
		for ( j = 0; j < 3; j++ )
		{
			m[i+j*4] = q[i][0] * trans[0][j]
					   + q[i][1] * trans[1][j]
					   + q[i][2] * trans[2][j];
		}
		m[i+3*4] = q[i][0] * trans[0][3]
				   + q[i][1] * trans[1][3]
				   + q[i][2] * trans[2][3]
				   + q[i][3];
	}

	if (!invert)
	{
		m[13]=-m[13] ;
		m[1]=-m[1];
		m[5]=-m[5];
		m[9]=-m[9];
	}

}
// end ArUco ------------------------------------------------------------------

void MonoCalibration::getGLProjectionMatrix(cv::Mat &projection,
											cv::Size imgSize,
											cv::Size renderSize,
											double gNear, double gFar,
											bool invert) {
	projection = Mat::zeros(4, 4, CV_64FC1);

	double Ax = (double)renderSize.width / (double)imgSize.width;
	double Ay = (double)renderSize.height / (double)imgSize.height;
	double fx = m_Intrinsic.at<double>(0, 0) * Ax;
	double cx = m_Intrinsic.at<double>(0, 2) * Ax;
	double fy = m_Intrinsic.at<double>(1, 1) * Ay;
	double cy = m_Intrinsic.at<double>(1, 2) * Ay;
	double cparam[3][4] = 
	{ 
		{fx,  0, cx, 0},
		{ 0, fy, cy, 0},
		{ 0,  0,  1, 0}
	};

	// from ArUco - not 100% sure what it is doing :)
	// I think it is doing this: http://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/
	// but need to triple-check
	double proj_matrix[16];
	argConvGLcpara2(cparam, renderSize.width, renderSize.height, gNear, gFar, 
					proj_matrix, invert);

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			// opengl uses column major ordering
			projection.at<double>(r, c) = proj_matrix[4 * c + r];
		}
	}
}

void getGLMVFromRTVec(cv::Mat &modelView, cv::Mat &Rvec, cv::Mat &Tvec) {
	Mat Rot(3, 3, CV_64FC1), Jacob;
	Rodrigues(Rvec, Rot, Jacob);
	modelView = Mat::zeros(4, 4, CV_64FC1);

	modelView.at<double>(0, 0) = Rot.at<double>(0, 0);
	modelView.at<double>(0, 1) = Rot.at<double>(0, 1);
	modelView.at<double>(0, 2) = Rot.at<double>(0, 2);
	modelView.at<double>(0, 3) = Tvec.at<double>(0, 0);

	modelView.at<double>(1, 0) = Rot.at<double>(1, 0);
	modelView.at<double>(1, 1) = Rot.at<double>(1, 1);
	modelView.at<double>(1, 2) = Rot.at<double>(1, 2);
	modelView.at<double>(1, 3) = Tvec.at<double>(1, 0);

	modelView.at<double>(2, 0) = Rot.at<double>(2, 0);
	modelView.at<double>(2, 1) = Rot.at<double>(2, 1);
	modelView.at<double>(2, 2) = Rot.at<double>(2, 2);
	modelView.at<double>(2, 3) = Tvec.at<double>(2, 0);

	modelView.at<double>(3, 0) = 0;
	modelView.at<double>(3, 1) = 0;
	modelView.at<double>(3, 2) = 0;
	modelView.at<double>(3, 3) = 1;
}

// stored board from calibration
void MonoCalibration::getGLModelViewMatrix(cv::Mat &modelView, int boardId) {
	getGLMVFromRTVec(modelView, m_RotVecs[boardId], m_TransVecs[boardId]);
}

// new board image
bool MonoCalibration::getGLModelViewMatrix(cv::Mat &modelView, cv::Mat &img, bool real) {
	bool sanityCheck = true;
	int boardW = 5;
	int boardH = 8;
	int boardN = boardW * boardH;
	double squareSize = 0.0254; // 1"

	// find chessboard corners
	vector<Point2f> imgPoints;
	bool found = findChessboardCorners(img,
									   Size(boardW, boardH),
									   imgPoints,
									   CALIB_CB_ADAPTIVE_THRESH
									   | CALIB_CB_NORMALIZE_IMAGE
									   | CALIB_CB_FAST_CHECK
									   | CALIB_CB_FILTER_QUADS
									   // FIXME: user options
									   );

	if (!found) {
		return false;
	}

	// Get subpixel accuracy on those corners
	Mat grayImage;
	cvtColor(img, grayImage, CV_BGR2GRAY);
	cornerSubPix(grayImage, imgPoints, Size(11, 11), Size(-1, -1),
				 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	// Draw it
	if (sanityCheck) {
		drawChessboardCorners(img, Size(boardW, boardH),
							  Mat(imgPoints), found);
	}

	if (real) {
		// compute world coords
		vector<Point3f> worldPoints;
		if (imgPoints.size() == boardN) {
			for (int i = 0; i < boardN; i++) {
				Point3f p;
				//p.x = (i % boardW) * squareSize;
				p.x = ((boardW - 1) - (i % boardW)) * squareSize;
				p.y = (i / boardW) * squareSize;
				p.z = 0.0f;
				worldPoints.push_back(p);
			}
		} else {
			return false;
		}

		// solve for extrinsics
		Mat RVec, TVec;
		solvePnP(worldPoints, imgPoints, m_Intrinsic, m_Distortion, RVec, TVec,
				 false, CV_ITERATIVE);

		// find modelview
		getGLMVFromRTVec(modelView, RVec, TVec);

		// draw reprojected 3d points
		if (sanityCheck) {
			vector<Point2f> reImagePoints;
			projectPoints(worldPoints, RVec, TVec, m_Intrinsic, m_Distortion, reImagePoints);
			for (unsigned int i = 0; i < reImagePoints.size(); i++) {
				circle(img, reImagePoints[i], 5, CV_RGB(128, 0, 128), CV_FILLED, CV_AA);
			}
			line(img, reImagePoints[0], reImagePoints[boardW-1], CV_RGB(255, 0, 0), 2);
			line(img, reImagePoints[boardW-1], reImagePoints[boardH*boardW - 1], CV_RGB(0, 255, 0), 2);
		}
	}

	return true;
}

// ----- Stereo Calibration ----------------------------------------------------
enum RECTIFICATION_ALGO {
	BOUGUET = 0,
	HARTLEY
};

const RECTIFICATION_ALGO rectMethod = BOUGUET;

StereoCalibration::StereoCalibration(int argW /* = 5 */, int argH /* = 8 */,
									 double argSS /* = 0.0254 */) {
	m_BoardW = argW;
	m_BoardH = argH;
	m_BoardN = m_BoardW * m_BoardH;
	m_SquareSize = argSS; // meters
	m_Name = "StereoCalibration";
	m_Description = "";
}

StereoCalibration::~StereoCalibration() { }

bool StereoCalibration::load(std::string filename /* = "stereocalib.yml" */) {
	bool ret = false;
	FileStorage fs(filename, FileStorage::READ);
	
	if (fs.isOpened()) {
		fs["IntrinsicLeft"] >> m_Intrinsic[LEFT_CAMERA];
		fs["IntrinsicRight"] >> m_Intrinsic[RIGHT_CAMERA];
		fs["DistortionLeft"] >> m_Distortion[LEFT_CAMERA];
		fs["DistortionRight"] >> m_Distortion[RIGHT_CAMERA];
		
		fs["Rotation"] >> m_Rotation;
		fs["Translation"] >> m_Translation;
		
		fs["Q"] >> m_Q;
		fs["Essential"] >> m_Essential;
		fs["Fundamental"] >> m_Fundamental;
		fs["PrectLeft"] >> m_Prect[LEFT_CAMERA];
		fs["PrectRight"] >> m_Prect[RIGHT_CAMERA];
		
		fs["ImageWidth"] >> m_ImageSize.width;
		fs["ImageHeight"] >> m_ImageSize.height;
		fs["BoardWidth"] >> m_BoardW;
		fs["BoardHeight"] >> m_BoardH;
		fs["SquareSize"] >> m_SquareSize;
		fs["IsVerticalStereo"] >> m_IsVerticalStereo;
		
//        fs["Rrect0"] >> m_Rrect[0];
//        fs["Rrect1"] >> m_Rrect[1];

		fs["RWLeft"] >> m_RW[LEFT_CAMERA];
		fs["RWRight"] >> m_RW[RIGHT_CAMERA];
		fs["TWLeft"] >> m_TW[LEFT_CAMERA];
		fs["TWRight"] >> m_TW[RIGHT_CAMERA];

		fs["RRWLeft"] >> m_RRW[LEFT_CAMERA];
		fs["RRWRight"] >> m_RRW[RIGHT_CAMERA];
		fs["TRWLeft"] >> m_TRW[LEFT_CAMERA];
		fs["TRWRight"] >> m_TRW[RIGHT_CAMERA];

		fs["AverageReprojectionError"] >> m_AvgError;
		
		fs["Name"] >> m_Name;
		fs["Description"] >> m_Description;

		precomputeValues();
		ret = true;
	} else {
		cout << "[Error] Cannot open '" << filename 
			<< "' for reading." << endl;
	}
	
	return ret;
}

bool StereoCalibration::save(std::string filename /* = "stereocalib.yml" */,
							 bool saveR /* = false */) {
	bool ret = false;
	FileStorage fs(filename, FileStorage::WRITE);

	if (fs.isOpened()) {
		fs << "IntrinsicLeft" << m_Intrinsic[LEFT_CAMERA];
		fs << "IntrinsicRight" << m_Intrinsic[RIGHT_CAMERA];
		fs << "DistortionLeft" << m_Distortion[LEFT_CAMERA];
		fs << "DistortionRight" << m_Distortion[RIGHT_CAMERA];

		fs << "Rotation" << m_Rotation;

		fs << "Translation" << m_Translation;
		
		fs << "Q" << m_Q;
		fs << "Essential" << m_Essential;
		fs << "Fundamental" << m_Fundamental;
		fs << "PrectLeft" << m_Prect[LEFT_CAMERA];
		fs << "PrectRight" << m_Prect[RIGHT_CAMERA];

		fs << "ImageWidth" << m_ImageSize.width;
		fs << "ImageHeight" << m_ImageSize.height;
		fs << "BoardWidth" << m_BoardW;
		fs << "BoardHeight" << m_BoardH;
		fs << "SquareSize" << m_SquareSize;
		fs << "IsVerticalStereo" << m_IsVerticalStereo;
		
		fs << "RWLeft" << m_RW[LEFT_CAMERA];
		fs << "RWRight" << m_RW[RIGHT_CAMERA];
		fs << "TWLeft" << m_TW[LEFT_CAMERA];
		fs << "TWRight" << m_TW[RIGHT_CAMERA];
		
		fs << "RRWLeft" << m_RRW[LEFT_CAMERA];
		fs << "RRWRight" << m_RRW[RIGHT_CAMERA];
		fs << "TRWLeft" << m_TRW[LEFT_CAMERA];
		fs << "TRWRight" << m_TRW[RIGHT_CAMERA];

		fs << "AverageReprojectionError" << m_AvgError;

		fs << "Name" << m_Name;
		fs << "Description" << m_Description;

		ret = true;
	} else {
		cout << "[Error] Cannot open '" << filename 
			<< "' for writing." << endl;
	}

	return ret;
//    if (saveR) {
//        cvWrite(fs, "Rrect1", m_Rrect[0]);
//        cvWrite(fs, "Rrect2", m_Rrect[1]);
//    }
}

double StereoCalibration::calibrateFromCapture(cv::VideoCapture& capLeft,
											   cv::VideoCapture& capRight) {
	Mat image[2];
	startCalibration();

	capLeft >> image[0];
	capRight >> image[1];
	
	if (image[0].data == NULL || image[1].data == NULL) {
		cout << "[Error] Invalid image data." << endl;
		return -1;
	}
	
	m_ImageSize = image[0].size();
	if (image[1].size() != m_ImageSize) {
		cout << "[Error] Image sizes must match." << endl;
		return -1;
	}

	cvNamedWindow("Calibration");

	int frame = 0;
	while (m_Successes < N_BOARDS && 
		   image[0].data != NULL && image[1].data != NULL) {
		if (frame++ % SKIP_DT != 0) {
			capLeft >> image[0];
			capRight >> image[1];
			continue;
		}
		cout << "Processing image " << frame + 1 << " ... " << endl;

		if (!imageLoopCommon(image, frame, true)) {
			return -1;
		}

		capLeft >> image[0];
		capRight >> image[1];
	} // End collection loop

	cvDestroyWindow("Calibration");

	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	cout << "Found " << m_ImageCount << " valid point sets." << endl;
	
	return m_AvgError;
}

double StereoCalibration::calibrateFromVideo(std::string leftVideoFile,
											 std::string rightVideoFile) {
	VideoCapture capLeft(leftVideoFile);
	VideoCapture capRight(rightVideoFile);
	
	if (!capLeft.isOpened()) {
		cout << "Cannot open '" << leftVideoFile << "'" << endl;
		return -1;
	}
	
	if (!capRight.isOpened()) {
		cout << "Cannot open '" << rightVideoFile << "'" << endl;
		return -1;
	}
	
	return calibrateFromCapture(capLeft, capRight);
}

double StereoCalibration::calibrateFromCam(int leftCamNum,
										   int rightCamNum) {
	VideoCapture capLeft(leftCamNum);
	VideoCapture capRight(rightCamNum);
	
	if (!capLeft.isOpened()) {
		cout << "Cannot open video device #" << leftCamNum << endl;
		return -1;
	}
	
	if (!capRight.isOpened()) {
		cout << "Cannot open video device #" << rightCamNum << endl;
		return -1;
	}
	
	return calibrateFromCapture(capLeft, capRight);
}

double StereoCalibration::calibrateFromImages(vector<string> leftImages,
											  vector<string> rightImages) {
	int leftCount = leftImages.size();
	int rightCount = rightImages.size();
	
	if (leftCount == 0 || rightCount == 0) {
		cout << "[Error] 0-length images vector." << endl;
		return -1;
	}
	
	m_ImageCount = leftCount > rightCount ? rightCount : leftCount;
	 
	startCalibration();
	
	namedWindow("Calibration");
	
	for (int i = 0; i < m_ImageCount; i++) {
		cout << "Processing image " << i + 1 << " ... ";
		
		Mat images[] = {imread(leftImages[i]), imread(rightImages[i])};
		
		if (images[0].size() != images[0].size()) {
		cout << "[Error] Image sizes must match." << endl;
		return -1;
	}
		if (!imageLoopCommon(images, i)) {
			return -1;
		}
	} // End collection loop
	
	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	destroyWindow("Calibration");
	
	return m_AvgError;
}

double StereoCalibration::calibrateFromImages(vector<Mat> leftImages,
											  vector<Mat> rightImages) {
	int leftCount = leftImages.size();
	int rightCount = rightImages.size();
	
	if (leftCount == 0 || rightCount == 0) {
		cout << "[Error] 0-length images vector." << endl;
		return -1;
	}
	
	m_ImageCount = leftCount > rightCount ? rightCount : leftCount;
	
	if (leftImages[0].size() != rightImages[0].size()) {
		cout << "[Error] Image sizes must match." << endl;
		return -1;
	}
	
	startCalibration();
	
	namedWindow("Calibration");
	
	for (int i = 0; i < m_ImageCount; i++) {
		cout << "Processing image " << i + 1 << " ... ";
		Mat images[] = {leftImages[i], rightImages[i]};
		if (!imageLoopCommon(images, i)) {
			return -1;
		}
	} // End collection loop
	
	m_AvgError = endCalibration();
	cout << "Calibration finished." << endl;
	destroyWindow("Calibration");
	
	return m_AvgError;
}

bool StereoCalibration::imageLoopCommon(cv::Mat images[2], int i, 
										bool video /* = false */) {
	// process image
	Scalar color;
	if (processCalibration(images)) {
		color = CV_RGB(0, 255, 0);
		cout << "pattern found." << endl;
	} else {
		color = CV_RGB(255, 0, 0);
		cout << "pattern not found." << endl;
	}
	
	// draw text
	char buff[256] = {0};
	if (video) {
		sprintf_s(buff, 256, "%d/%d", m_Successes, N_BOARDS);
	} else {
		sprintf_s(buff, 256, "%d/#d", i + 1, m_ImageCount);
	}
	putText(m_SmallImage, buff, Point(200, m_SmallImage.size().height - 20), 
			FONT_HERSHEY_SIMPLEX, 1, color, 2, CV_AA);
	
	imshow("Calibration", m_SmallImage);
	
	// Handle pause/unpause and ESC
	int keycode = (waitKey(video ? BOARD_DT : DISPLAY_DT) & 255);
	if (keycode == 'p') {
		keycode = 0;
		while (keycode != 'p' && keycode != OPENCV_KEYCODE_ESC) {
			keycode = waitKey(250);
		}
	}
	if (keycode == OPENCV_KEYCODE_ESC) {
		destroyWindow("Calibration");
		return false;
	}
	return true;
}

void StereoCalibration::startCalibration() {
	m_Successes = 0;
}

bool StereoCalibration::processCalibration(cv::Mat images[2]) {
	bool ret;
	int good = 0;
	
	vector<Point2f> corners[2];

	for (int i = 0; i < 2; i++) {
		// search for chessboard corners
		bool found = findChessboardCorners(images[i],
										   Size(m_BoardW, m_BoardH),
										   corners[i],
										   CV_CALIB_CB_ADAPTIVE_THRESH +
										   CV_CALIB_CB_NORMALIZE_IMAGE +
										   CV_CALIB_CB_FAST_CHECK);
		if (found) {
			// get subpixel accuracy on those corners
			Mat grayImage;
			cvtColor(images[i], grayImage, CV_BGR2GRAY);
			cornerSubPix(grayImage, corners[i], Size(11, 11), Size(-1, -1),
						 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
									  30, 0.1));

			// draw the corners
			drawChessboardCorners(images[i], Size(m_BoardW, m_BoardH),
								  Mat(corners[i]), found);

			if (corners[i].size() == m_BoardN) {
				good++;
			}
		}
	}
	
	// resize the images for display
	// FIXME: don't use imagegrid, overkill for this
	//ImageGrid ig(1, 2);
	//ig.add(images[LEFT_CAMERA], "Left Camera");
	//ig.add(images[RIGHT_CAMERA], "Right Camera");
	//m_SmallImage = ig.generateGridImage(800); // FIXME: magic number

	if (good == 2) {
		// both cameras saw the chessboard, so we have one new set of points
		// to add to out raw calibration data
		vector<Point3f> worldPoints;
		
		for (int i = 0; i < m_BoardN; i++) {
			Point3f p;
			p.x = (float)( (i % m_BoardW) * m_SquareSize );
			p.y = (float)( (i / m_BoardW) * m_SquareSize );
			p.z = 0.0f;
			worldPoints.push_back(p);
		}
		
		m_ObjPoints.push_back(worldPoints);
		m_ImagePoints[LEFT_CAMERA].push_back(corners[LEFT_CAMERA]);
		m_ImagePoints[RIGHT_CAMERA].push_back(corners[RIGHT_CAMERA]);
		m_Successes++;

		ret = true;
	} else {
		ret = true;
	}
	
	m_ImageSize = images[0].size();
	return ret;
}

double StereoCalibration::endCalibration() {
	// calibrate the stereo pair
	double rErr = stereoCalibrate(m_ObjPoints, 
		m_ImagePoints[LEFT_CAMERA], m_ImagePoints[RIGHT_CAMERA],
		m_Intrinsic[LEFT_CAMERA], m_Distortion[LEFT_CAMERA],
		m_Intrinsic[RIGHT_CAMERA], m_Distortion[RIGHT_CAMERA],
		m_ImageSize, m_Rotation, m_Translation, m_Essential, m_Fundamental,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH + CV_CALIB_RATIONAL_MODEL);

	// std::cout << "Hartley rectification ..." << std::endl;
	// HARTLEY'S RECTIFICATION METHOD - pre-precompute
	// for (int i = 0; i < 2; i++) {
		// if (m_Rrect[i] != NULL) cvReleaseMat(&m_Rrect[i]);
		// m_Rrect[i] = cvCreateMat(3, 3, CV_64FC1);
	// }
   // double H1[3][3], H2[3][3], iM[3][3];
   // CvMat _H1 = cvMat(3, 3, CV_64F, H1);
   // CvMat _H2 = cvMat(3, 3, CV_64F, H2);
   // CvMat _iM = cvMat(3, 3, CV_64F, iM);

   // cvStereoRectifyUncalibrated(
	   // &mat_imgPts1, &mat_imgPts2, m_F,
	   // m_imageSize,
	   // &_H1, &_H2, 3
   // );
   // cvInvert(m_Intrinsic[0], &_iM);
   // cvMatMul(&_H1, m_Intrinsic[0], m_Rrect[0]);
   // cvMatMul(&_iM, m_Rrect[0], m_Rrect[0]);
   // cvInvert(m_Intrinsic[1], &_iM);
   // cvMatMul(&_H2, m_Intrinsic[1], m_Rrect[1]);
   // cvMatMul(&_iM, m_Rrect[1], m_Rrect[1]);

	// rectification warp maps
	precomputeValues();

	//// world to camera transformation (for non-rectified images)
	//CvMat r_objPts = cvMat(1, m_boardN, CV_64FC3, &m_objPoints[0]);
	//CvMat r_imgPts1 = cvMat(1, m_boardN, CV_64FC2, &m_imagePoints[0][0]);
	//CvMat r_imgPts2 = cvMat(1, m_boardN, CV_64FC2, &m_imagePoints[1][0]);
	//cvFindExtrinsicCameraParams2(&r_objPts, &r_imgPts1, m_Intrinsic[0],
	//                             m_Distortion[0], m_RW[0], m_TW[0]);
	//cvFindExtrinsicCameraParams2(&r_objPts, &r_imgPts2, m_Intrinsic[1],
	//                             m_Distortion[1], m_RW[1], m_TW[1]);

	//// world to camera transformation (for rectified images)
	//CvMat *r_rectImgPts = cvCreateMat(1, m_boardN, CV_64FC2);
	//CvMat *r_rectI = cvCreateMat(3, 3, CV_64FC1);
	//CvMat *rv = cvCreateMat(3, 3, CV_64FC1);
	//CvMat *tv = cvCreateMat(4, 1, CV_64FC1);
	//// left camera
	//cvUndistortPoints(&r_imgPts1, r_rectImgPts, m_Intrinsic[0], m_Distortion[0],
	//                  m_Rrect[0], m_Prect[0]);
	//cvDecomposeProjectionMatrix(m_Prect[0], r_rectI, rv, tv);
	//cvFindExtrinsicCameraParams2(&r_objPts, r_rectImgPts, r_rectI,
	//                             m_Distortion[0], m_RRW[0], m_TRW[0]);
	//// right camera
	//cvUndistortPoints(&r_imgPts2, r_rectImgPts, m_Intrinsic[1], m_Distortion[1],
	//                  m_Rrect[1], m_Prect[1]);
	//cvDecomposeProjectionMatrix(m_Prect[1], r_rectI, rv, tv);
	//cvFindExtrinsicCameraParams2(&r_objPts, r_rectImgPts, r_rectI,
	//                             m_Distortion[1], m_RRW[1], m_TRW[1]);

	// test the world to image transformation (and save test to disk)
	// std::cout << "testing world to image" << std::endl;
	// IplImage *out[4];
	// out[0] = cvCloneImage(m_firstImg[0]);
	// out[1] = cvCloneImage(m_firstImg[0]);
	// out[2] = cvCloneImage(m_firstImg[1]);
	// out[3] = cvCloneImage(m_firstImg[1]);
	// CvMat *world = cvCreateMat(3, 4, CV_64FC1);
	// CV_MAT_ELEM(*world, double, 0, 0) = 0;
	// CV_MAT_ELEM(*world, double, 1, 0) = 0;
	// CV_MAT_ELEM(*world, double, 2, 0) = 0;
	// CV_MAT_ELEM(*world, double, 0, 1) = 1;
	// CV_MAT_ELEM(*world, double, 1, 1) = 0;
	// CV_MAT_ELEM(*world, double, 2, 1) = 0;
	// CV_MAT_ELEM(*world, double, 0, 2) = 0;
	// CV_MAT_ELEM(*world, double, 1, 2) = 1;
	// CV_MAT_ELEM(*world, double, 2, 2) = 0;
	// CV_MAT_ELEM(*world, double, 0, 3) = 0;
	// CV_MAT_ELEM(*world, double, 1, 3) = 0;
	// CV_MAT_ELEM(*world, double, 2, 3) = 1;
	// CvMat *image = cvCreateMat(2, 4, CV_64FC1);
	// rectifyAndUndistortImages(out[0], out[2]);
	// world2image(world, image, LEFT_CAMERA, true);
	// cvLine(out[0],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 1), (int)cvmGet(image, 1, 1)),
		   // CV_RGB(255, 0, 0), 2, CV_AA);
	// cvLine(out[0],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 2), (int)cvmGet(image, 1, 2)),
		   // CV_RGB(0, 255, 0), 2, CV_AA);
	// cvLine(out[0],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 3), (int)cvmGet(image, 1, 3)),
		   // CV_RGB(0, 0, 255), 2, CV_AA);
	// world2image(world, image, LEFT_CAMERA, false);
	// cvLine(out[1],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 1), (int)cvmGet(image, 1, 1)),
		   // CV_RGB(255, 0, 0), 2, CV_AA);
	// cvLine(out[1],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 2), (int)cvmGet(image, 1, 2)),
		   // CV_RGB(0, 255, 0), 2, CV_AA);
	// cvLine(out[1],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 3), (int)cvmGet(image, 1, 3)),
		   // CV_RGB(0, 0, 255), 2, CV_AA);
	// world2image(world, image, RIGHT_CAMERA, true);
	// cvLine(out[2],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 1), (int)cvmGet(image, 1, 1)),
		   // CV_RGB(255, 0, 0), 2, CV_AA);
	// cvLine(out[2],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 2), (int)cvmGet(image, 1, 2)),
		   // CV_RGB(0, 255, 0), 2, CV_AA);
	// cvLine(out[2],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 3), (int)cvmGet(image, 1, 3)),
		   // CV_RGB(0, 0, 255), 2, CV_AA);
	// world2image(world, image, RIGHT_CAMERA, false);
	// cvLine(out[3],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 1), (int)cvmGet(image, 1, 1)),
		   // CV_RGB(255, 0, 0), 2, CV_AA);
	// cvLine(out[3],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 2), (int)cvmGet(image, 1, 2)),
		   // CV_RGB(0, 255, 0), 2, CV_AA);
	// cvLine(out[3],
		   // cvPoint((int)cvmGet(image, 0, 0), (int)cvmGet(image, 1, 0)),
		   // cvPoint((int)cvmGet(image, 0, 3), (int)cvmGet(image, 1, 3)),
		   // CV_RGB(0, 0, 255), 2, CV_AA);
	// IplImage *combo = cvCombineMultipleImages(true, 4,
											  // out[1], "left",
											  // out[3], "right",
											  // out[0], "left rectified",
											  // out[2], "right rectified");
	// cvSaveImage("coordTest.jpg", combo);

	return rErr;
}

//// 3xN -> 3xN (CV_64FC1)
//void StereoCalibration::world2camera(CvMat *world, CvMat *cam,
//                                     StereoCameraID camId /* = LEFT_CAMERA */,
//                                     bool rectified /* = true */) {
//    CvMat *RV = rectified ? m_RRW[camId] : m_RW[camId];
//    CvMat *R = cvCreateMat(3, 3, CV_64FC1);
//    CvMat *T = rectified ? m_TRW[camId] : m_TW[camId];
//
//    // convert
//    cvRodrigues2(RV, R);
//    // R * Xw
//    cvMatMulAdd(R, world, NULL, cam);
//    // + T
//    for (int r = 0; r < cam->rows; r++) {
//        for (int c = 0; c < cam->cols; c++) {
//            CV_MAT_ELEM(*cam, double, r, c) += CV_MAT_ELEM(*T, double, r, 0);
//        }
//    }
//
//    cvReleaseMat(&R);
//}
//
//// 3xN -> 3xN (CV_64FC1)
//void StereoCalibration::camera2world(CvMat *cam, CvMat *world,
//                                     StereoCameraID camId /* = LEFT_CAMERA */,
//                                     bool rectified /* = true */) {
//    CvMat *RV = rectified ? m_RRW[camId] : m_RW[camId];
//    CvMat *R = cvCreateMat(3, 3, CV_64FC1);
//    CvMat *T = rectified ? m_TRW[camId] : m_TW[camId];
//    CvMat *RT = cvCreateMat(3, 3, CV_64FC1);
//    CvMat *temp = cvCreateMat(3, 1, CV_64FC1);
//
//    // convert
//    cvRodrigues2(RV, R);
//    // R^T
//    cvTranspose(R, RT);
//    // R^T * T
//    cvMatMulAdd(RT, T, NULL, temp);
//    // R^T * Xc
//    cvMatMulAdd(RT, cam, NULL, world);
//    // - R^T * T
//    for (int r = 0; r < world->rows; r++) {
//        for (int c = 0; c < world->cols; c++) {
//            CV_MAT_ELEM(*world, double, r, c) -=
//                CV_MAT_ELEM(*temp, double, r, 0);
//        }
//    }
//
//    cvReleaseMat(&R);
//    cvReleaseMat(&RT);
//    cvReleaseMat(&temp);
//}
//
//// 3xN -> 2xN (CV_64FC1)
//void StereoCalibration::world2image(CvMat *world, CvMat *image,
//                                    StereoCameraID camId /* = LEFT_CAMERA */,
//                                    bool rectified /* = true */) {
//    CvMat *R = rectified ? m_RRW[camId] : m_RW[camId];
//    CvMat *T = rectified ? m_TRW[camId] : m_TW[camId];
//    CvMat *I;
//    CvMat *D = rectified ? NULL : m_Distortion[camId];
//    if (rectified) {
//        I = cvCreateMat(3, 3, CV_64FC1);
//        CvMat *rv = cvCreateMat(3, 3, CV_64FC1);
//        CvMat *tv = cvCreateMat(4, 1, CV_64FC1);
//        cvDecomposeProjectionMatrix(m_Prect[camId], I, rv, tv);
//        cvReleaseMat(&rv);
//        cvReleaseMat(&tv);
//    } else {
//        I = m_Intrinsic[camId];
//    }
//    cvProjectPoints2(world, R, T, I, D, image);
//    if (rectified) cvReleaseMat(&I);
//}
//
//// 3xN -> 2xN (CV_64FC1)
//void StereoCalibration::camera2image(CvMat *cam, CvMat *image,
//                                     StereoCameraID camId /* = LEFT_CAMERA */,
//                                     bool rectified /* = true */) {
//    CvMat *I;
//    CvMat *D = rectified ? NULL : m_Distortion[camId];
//    CvMat *Z = cvCreateMat(3, 1, CV_64FC1);
//    cvZero(Z);
//    if (rectified) {
//        I = cvCreateMat(3, 3, CV_64FC1);
//        CvMat *rv = cvCreateMat(3, 3, CV_64FC1);
//        CvMat *tv = cvCreateMat(4, 1, CV_64FC1);
//        cvDecomposeProjectionMatrix(m_Prect[camId], I, rv, tv);
//        cvReleaseMat(&rv);
//        cvReleaseMat(&tv);
//    } else {
//        I = m_Intrinsic[camId];
//    }
//
//    cvProjectPoints2(cam, Z, Z, I, D, image);
//    if (rectified) cvReleaseMat(&I);
//    cvReleaseMat(&Z);
//}

void StereoCalibration::undistortImage(cv::Mat& image, StereoCameraID camId) {
	Mat t = image.clone();
	remap(t, image, m_UMapX[camId], m_UMapY[camId], INTER_LANCZOS4);
}

void StereoCalibration::rectifyAndUndistortImages(cv::Mat& leftImage,
												  cv::Mat& rightImage) {
	Mat image[] = {leftImage, rightImage};

	for (int i = 0; i < 2; i++) {
		if (image[i].data == NULL) continue;
		Mat t = image[i].clone();
		remap(t, image[i], m_RMapX[i], m_RMapY[i], INTER_LANCZOS4);
	}
}

void StereoCalibration::precomputeValues() {
	// precompute the undistortion mapping
	for (int i = 0; i < 2; i++) {
		initUndistortRectifyMap(m_Intrinsic[i], m_Distortion[i], Mat(),
								getOptimalNewCameraMatrix(m_Intrinsic[i],
														  m_Distortion[i],
														  m_ImageSize,
														  1,
														  m_ImageSize,
														  0),
								m_ImageSize, CV_64FC1,
								m_UMapX[i], m_UMapY[i]);
	}

	//// precompute the rectification mapping
	//if (m_Q != NULL) cvReleaseMat(&m_Q);
	//m_Q = cvCreateMat(4, 4, CV_64FC1);
	//for (int i = 0; i < 2; i++) {
	//    if (m_rmapx[i] != NULL) cvReleaseImage(&m_rmapx[i]);
	//    m_rmapx[i] = cvCreateImage(m_imageSize, IPL_DEPTH_32F, 1);
	//    if (m_rmapy[i] != NULL) cvReleaseImage(&m_rmapy[i]);
	//    m_rmapy[i] = cvCreateImage(m_imageSize, IPL_DEPTH_32F, 1);
	//}

	//if (rectMethod == HARTLEY) {
	//    for (int i = 0; i < 2; i++) {
	//        cvInitUndistortRectifyMap(m_Intrinsic[i], m_Distortion[i],
	//                                  m_Rrect[i], m_Intrinsic[i],
	//                                  m_rmapx[i], m_rmapy[i]);
	//    }
	//} else if (rectMethod == BOUGUET) {

	// alpha = 0 will shift and zoom images so that only valid pixels
		// are visible
		stereoRectify(m_Intrinsic[LEFT_CAMERA], m_Distortion[LEFT_CAMERA],
					  m_Intrinsic[RIGHT_CAMERA], m_Distortion[RIGHT_CAMERA],
					  m_ImageSize, m_Rotation, m_Translation,
					  m_Rrect[LEFT_CAMERA], m_Rrect[RIGHT_CAMERA],
					  m_Prect[LEFT_CAMERA], m_Prect[RIGHT_CAMERA], m_Q,
					  CALIB_ZERO_DISPARITY, 0);

		m_IsVerticalStereo = fabs(m_Prect[1].at<double>(1, 3)) > 
							 fabs(m_Prect[1].at<double>(0, 3));

		for (int i = 0; i < 2; i++) {
			initUndistortRectifyMap(m_Intrinsic[i], m_Distortion[i],
									m_Rrect[i], m_Prect[i], m_ImageSize,
									CV_64FC1, m_RMapX[i], m_RMapY[i]);
		}
	//}
}