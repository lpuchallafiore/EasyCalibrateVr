#include "stdafx.h"
#include "GrayCodeComputeState.h"
#include "GrayCodeState.h"
#include "aruco.h"

#include "ProcessData.h"

using namespace std;
using namespace cv; // FIXME: put in stdafx.h?

#define DO_BUNDLE_ADJUST false
// 1 = OpenCV, 2 = CVSBA, 3 = SSBA
// FIXME: different types not yet supported - the code needs to be ported form old to new algorithm
#define BUNDLE_TYPE 3
#define BUNDLE_ITERATIONS 10000
#define BUNDLE_ERR 1e-10

// fixme: hardcoded magic numbers, should be in a data file along with the gray
// code images
#define GRAY_CODE_GRID_SPACE 1
#define MESH_GRID_MAX 32
#define GRAY_CODE_WIDTH (1280/8)
#define GRAY_CODE_HEIGHT (1024/8)
#define NUM_CODES 31
#define OUTPUT_POSTFIX "20140816"

GrayCodeComputeState::GrayCodeComputeState(float left, float top, float width,
	float height, const std::shared_ptr<ProcessData>& pd) : IAppState(pd),
	_left(left), _top(top), _width(width), _height(height) {

	_image = cv::Mat::zeros(cv::Size(320, 180), CV_8UC3);

	// parameters

	_p.intrinsic_start		= 1;
	_p.intrinsic_prefix		= "newdata\\intrinsic_0_";
	_p.intrinsic_end		= 103;
	_p.intrinsic_step		= 1;

	/*_p.intrinsic_start		= 0;
	_p.intrinsic_prefix		= "GrayCode-20140424\\intrinsic_";
	_p.intrinsic_end		= 1740;
	_p.intrinsic_step		= 20;*/

	/*_p.intrinsic_start		= 1;
	_p.intrinsic_prefix		= "CameraCalibration05-01-2014\\intrinsic_";
	_p.intrinsic_end		= 1844;
	_p.intrinsic_step		= 10;*/

	/*_p.graycode_folder		= "GrayCode-Merged";
	_p.num_image_sets		= 40;*/

	_p.graycode_folder		= "GrayCode-20140816";
	_p.num_image_sets		= 35;

	// start worker thread
	InitializeCriticalSection(&_csProgress);
	InitializeCriticalSection(&_csImage);
	CreateThread(NULL, 0, t_WorkerStart, this, 0, 0);
}

DWORD WINAPI GrayCodeComputeState::t_WorkerStart(LPVOID lpParam) {
	GrayCodeComputeState * pC = (GrayCodeComputeState *)lpParam;

	return pC->t_Worker();
}

std::string GrayCodeComputeState::getIntrinsicImageFilename(int i) const {
	std::stringstream ss;
	ss << _p.intrinsic_prefix << i << ".jpg";
	return ss.str();
}

std::string GrayCodeComputeState::getGrayCodeImageFilename(int proj, int set, int code) const {
	std::stringstream ss;
	ss << _p.graycode_folder << "\\"
		<< "graycode_proj_" << proj
		<< "_set_" << set
		<< "_code_" << code
		<< ".jpg";
	return ss.str();
}

void GrayCodeComputeState::decodeGrayCode(int proj, int set, cv::Mat& rows, cv::Mat& cols) {
	// compute set mask
	cv::Mat image = cv::imread(getGrayCodeImageFilename(proj, set, 0));
	cv::Mat mask = GrayCodeState::computeGreenMask(image);
	setImage(image);

	// decode set
	std::vector<cv::Mat> gcs;
	for (int i = 1; i < NUM_CODES; ++i) {
		gcs.push_back(cv::imread(getGrayCodeImageFilename(proj, set, i)));
	}
	cv::Mat decoded_mask;
	GrayCodeState::decodeGrayCodes(GRAY_CODE_WIDTH, GRAY_CODE_HEIGHT, gcs, 
		mask, cols, rows, decoded_mask);
}

std::vector<Feature> GrayCodeComputeState::grayCodeToFeatures(const cv::Mat& rows, const cv::Mat& cols, int proj) const {
	// allocate matrices
	cv::Mat row_sum = cv::Mat::zeros(cv::Size(GRAY_CODE_WIDTH, GRAY_CODE_HEIGHT), CV_32FC1);
	cv::Mat col_sum = cv::Mat::zeros(cv::Size(GRAY_CODE_WIDTH, GRAY_CODE_HEIGHT), CV_32FC1);
	cv::Mat pt_count = cv::Mat::zeros(cv::Size(GRAY_CODE_WIDTH, GRAY_CODE_HEIGHT), CV_16UC1);

	// accumulate sums
	for (int y = 0; y < rows.size().height; ++y) {
		const ushort * rows_row = rows.ptr<ushort>(y);
		const ushort * cols_row = cols.ptr<ushort>(y);

		for (int x = 0; x < rows.size().width; ++x) {
			ushort row = rows_row[x] - 1; // because starts at 1
			ushort col = cols_row[x] - 1;

			if (row >= 0 && row < GRAY_CODE_HEIGHT &&
				col >= 0 && col < GRAY_CODE_WIDTH) {

				row_sum.at<float>(row, col) += y;
				col_sum.at<float>(row, col) += x;
				pt_count.at<ushort>(row, col)++;
			}
		}
	}

	// compute averages and convert to vector of features
	std::vector<Feature> out;
	for (int y = 0; y < GRAY_CODE_HEIGHT; ++y) {
		const ushort * count_row = pt_count.ptr<ushort>(y);
		const float * row_row = row_sum.ptr<float>(y);
		const float * col_row = col_sum.ptr<float>(y);

		for (int x = 0; x < GRAY_CODE_WIDTH; ++x) {
			ushort count = count_row[x];

			if (count > 0 &&
				x % GRAY_CODE_GRID_SPACE == 0 &&
				y % GRAY_CODE_GRID_SPACE == 0)
			{
				Feature f;
				f.pt.x = col_row[x] / count;
				f.pt.y = row_row[x] / count;
				f.id.x = x;
				f.id.y = y;
				f.id.z = proj;
				if (proj == 0) {
					f.color = Vec3b(255, 0, 0);
				} else if (proj == 1) {
					f.color = Vec3b(0, 255, 0);
				} else if (proj == 2) {
					f.color = Vec3b(0, 0, 255);
				}
				out.push_back(f);
			}
		}
	}

	return out;
}

std::vector<cv::DMatch> flipMatches(const std::vector<cv::DMatch>& matches) {
	std::vector<cv::DMatch> flip;
	for (size_t i = 0; i < matches.size(); ++i) {
		flip.push_back(matches[i]);
		std::swap(flip.back().queryIdx, flip.back().trainIdx);
	}
	return flip;
}

std::vector<cv::DMatch> matchFeatures(const std::vector<Feature>& queryFeatures,
									  const std::vector<Feature>& trainFeatures) {
	std::vector<cv::DMatch> matches;

	// create map for fast searches in the train vector
	std::map<cv::Point3i, size_t, cmpPoint3i> trainMap;
	for (size_t i = 0; i < trainFeatures.size(); ++i) {
		trainMap[trainFeatures[i].id] = i;
	}

	for (size_t i = 0; i < queryFeatures.size(); ++i) {
		const Feature& q = queryFeatures[i];

		auto j = trainMap.find(q.id);
		if (j != trainMap.end()) {
			matches.push_back(cv::DMatch(i, j->second, 0.f));
			if (q.id == Point3i(8, 116, 0)) cout << "Point [8, 116, 0] matched query " << i << " train " << j->second << endl;
		} else {
			if (q.id == Point3i(8, 116, 0)) cout << "Point [8, 116, 0] not matched, query " << i << endl;
		}
	}

	return matches;
}

void getAlignedPointsFromMatch(const std::vector<Feature>& imgpts1,
							   const std::vector<Feature>& imgpts2,
							   const std::vector<DMatch>& matches,
							   std::vector<Feature>& pt_set1,
							   std::vector<Feature>& pt_set2) {
	pt_set1.clear();
	pt_set2.clear();
	for (auto it = matches.begin(); it != matches.end(); ++it) {
		pt_set1.push_back(imgpts1[it->queryIdx]);
		pt_set2.push_back(imgpts2[it->trainIdx]);
	}
}

void featuresToPoints(const std::vector<Feature>& kps, std::vector<cv::Point2f>& ps) {
	ps.clear();
	for (auto it = kps.begin(); it != kps.end(); ++it) {
		ps.push_back(it->pt);
	}
}

int GrayCodeComputeState::findHomographyInliers2Views(int vi, int vj) {
	vector<Feature> ikpts, jkpts;
	vector<cv::Point2f> ipts, jpts;

	getAlignedPointsFromMatch(_img_pts[vi], _img_pts[vj], 
		_matches_matrix[make_pair(vi, vj)], ikpts, jkpts);
	featuresToPoints(ikpts, ipts);
	featuresToPoints(jkpts, jpts);

	double minVal, maxVal;
	minMaxIdx(ipts, &minVal, &maxVal);

	vector<uchar> status;
	Mat H = findHomography(ipts, jpts, status, CV_RANSAC, 0.004 * maxVal);
	return countNonZero(status);
}

bool sort_by_first(std::pair<int, std::pair<int, int> > a, 
				   std::pair<int, std::pair<int, int> > b) { 
	return a.first < b.first; 
}

bool GrayCodeComputeState::findCameraMatrices(const cv::Mat& K,
											  const cv::Mat& Kinv,
											  const cv::Mat& distCoeffs,
											  const std::vector<Feature>& imgpts1,
											  const std::vector<Feature>& imgpts2,
											  std::vector<Feature>& imgpts1_good,
											  std::vector<Feature>& imgpts2_good,
											  cv::Matx34d& P,
											  cv::Matx34d& P1,
											  std::vector<cv::DMatch>& matches,
											  std::vector<CloudPoint>& outCloud) {
	printf("Find camera matrices ...\n");
	Mat F = getFundamentalMat(imgpts1, imgpts2, imgpts1_good, imgpts2_good, matches);

	if (matches.size() < 100) {
		printf("not enough inliers after F matrix\n");
		return false;
	}
	
	// compute essential matrix
	Mat_<double> E = K.t() * F * K; // HZ (9.12)

	// verify the essential matrix is valid
	if (fabsf(determinant(E)) > 1e-07) {
		printf("det(E) != 0 : %f\n", determinant(E));
		P1 = 0;
		return false;
	}

	Mat_<double> R1(3, 3),
				 R2(3, 3),
				 t1(1, 3),
				 t2(1, 3);

	// decompose E to P', HZ (9.19)
	if (!decomposeEtoRandT(E, R1, R2, t1, t2)) return false;
	if (determinant(R1)+1.0 < 1e-09) {
		printf("det(R) == -1 [%f]: flip E's sign\n", determinant(R1));
		E = -E;
		decomposeEtoRandT(E, R1, R2, t1, t2);
	}
	if (!checkCoherentRotation(R1)) {
		printf("resulting rotation R1 is not coherent\n");
		P1 = 0;
		return false;
	}

	P1 = Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
				 R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
				 R1(2, 0), R1(2, 1), R1(2, 2), t1(2));
	cout << "Testing P1 (first) = " << endl << Mat(P1) << endl;

	vector<CloudPoint> pcloud, pcloud1;
	vector<Feature> corresp;
	double reproj_error1 = myTriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distCoeffs, P, P1, pcloud, corresp);
	double reproj_error2 = myTriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distCoeffs, P1, P, pcloud1, corresp);

	vector<uchar> tmp_status;
	// check if points are triangulated in front of cameras for all 4 possible solutions
	if (!testTriangulation(pcloud, P1, tmp_status) ||
		!testTriangulation(pcloud1, P, tmp_status) ||
		reproj_error1 > 100.0 ||
		reproj_error2 > 100.0) {

		P1 = Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t2(0),
					 R1(1, 0), R1(1, 1), R1(1, 2), t2(1),
					 R1(2, 0), R1(2, 1), R1(2, 2), t2(2));
		cout << "testing P1 (second) = " << endl << Mat(P1) << endl;

		pcloud.clear();
		pcloud1.clear();
		corresp.clear();
		reproj_error1 = myTriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distCoeffs, P, P1, pcloud, corresp);
		reproj_error2 = myTriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distCoeffs, P1, P, pcloud1, corresp);

		if (!testTriangulation(pcloud, P1, tmp_status) ||
			!testTriangulation(pcloud1, P, tmp_status) ||
			reproj_error1 > 100.0 ||
			reproj_error2 > 100.0) {

			if (!checkCoherentRotation(R2)) {
				printf("resulting rotation R2 is not coherent\n");
				P1 = 0;
				return false;
			}

			P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t1(0),
						 R2(1, 0), R2(1, 1), R2(1, 2), t1(1),
						 R2(2, 0), R2(2, 1), R2(2, 2), t1(2));
			cout << "Testing P1 (third) = " << endl << Mat(P1) << endl;

			pcloud.clear();
			pcloud1.clear();
			corresp.clear();
			reproj_error1 = myTriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distCoeffs, P, P1, pcloud, corresp);
			reproj_error2 = myTriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distCoeffs, P1, P, pcloud1, corresp);

			if (!testTriangulation(pcloud, P1, tmp_status) ||
				!testTriangulation(pcloud1, P, tmp_status) ||
				reproj_error1 > 100.0 ||
				reproj_error2 > 100.0) {

				P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0),
							 R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
							 R2(2, 0), R2(2, 1), R2(2, 2), t2(2));
				cout << "Testing P1 (fourth) = " << endl << Mat(P1) << endl;

				pcloud.clear();
				pcloud1.clear();
				corresp.clear();
				reproj_error1 = myTriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distCoeffs, P, P1, pcloud, corresp);
				reproj_error2 = myTriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distCoeffs, P1, P, pcloud1, corresp);

				if (!testTriangulation(pcloud, P1, tmp_status) ||
					!testTriangulation(pcloud, P, tmp_status) ||
					reproj_error1 > 100.0 ||
					reproj_error2 > 100.0) {

					printf("all solutions failed. ...\n");
					return false;
				}
			}
		}
	}
	for (size_t i = 0; i < pcloud.size(); ++i) {
		outCloud.push_back(pcloud[i]);
	}
	return true;
}

void GrayCodeComputeState::computeBaselineTriangulation() {
	printf("--- Baseline Triangulation ---");

	cv::Matx34d P(1, 0, 0, 0,
				  0, 1, 0, 0,
				  0, 0, 1, 0),
				P1(1, 0, 0, 0,
				   0, 1, 0, 0,
				   0, 0, 1, 0);
	std::vector<CloudPoint> tmp_pcloud;

	// sort pairwise matches to find the lowest Homography inliers (Snavely07)
	printf("find highest match ...");
	std::list<std::pair<int, std::pair<int, int>>> matches_sizes;
	for (auto it = _matches_matrix.begin(); it != _matches_matrix.end(); ++it) {
		if (it->second.size() < 100) {
			matches_sizes.push_back(std::make_pair(100, it->first));
		} else {
			int Hinliers = findHomographyInliers2Views(it->first.first, it->first.second);
			int percent = (int)(((double)Hinliers) / ((double)it->second.size()) * 100.0);
			printf("[%d, %d] = %f\n", it->first.first, it->first.second, percent);
			matches_sizes.push_back(std::make_pair((int)percent, it->first));
		}
	}
	matches_sizes.sort(sort_by_first);

	// reconstruct from two views
	bool goodF = false;
	int highest_pair = 0;
	_first_view = _second_view = 0;
	// reverse iterate by number of matches
	for (auto highest_pair = matches_sizes.begin(); 
		 highest_pair != matches_sizes.end() && !goodF; 
		 ++highest_pair) {

		_second_view = highest_pair->second.second;
		_first_view = highest_pair->second.first;

		printf("attempting to reconstruct from view %d and %d\n", _first_view, _second_view);

		// see if the Fundamental Matrix between these two views is good
		// by attempting to extract F and R|t
		goodF = findCameraMatrices(_K, _K_inv, _dist_coeffs,
			_img_pts[_first_view],
			_img_pts[_second_view],
			_img_pts_good[_first_view],
			_img_pts_good[_second_view],
			P,
			P1,
			_matches_matrix[std::make_pair(_first_view, _second_view)],
			tmp_pcloud);

		if (goodF) {
			std::vector<CloudPoint> new_triangulated;
			std::vector<int> add_to_cloud;

			_P_mats[_first_view] = P;
			_P_mats[_second_view] = P1;

			bool good_triangulation = triangulatePointsBetween2Views(_second_view, _first_view, new_triangulated, add_to_cloud);
			if (!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
				printf("triangulation failed\n");
				goodF = false;
				_P_mats[_first_view] = 0;
				_P_mats[_second_view] = 0;
			} else {
				printf("before triangulation: %d points\n", _pcloud.size());
				for (size_t j = 0; j < add_to_cloud.size(); ++j) {
					if (add_to_cloud[j] == 1) {
						_pcloud.push_back(new_triangulated[j]);
					}
				}
				printf("after triangulation: %d points\n", _pcloud.size());
			}
		}
	}

	if (!goodF) {
		printf("Cannot find a good pair of images to obtain a baseline triangulation\n");
		throw 0; // kill the sub-thread FIXME: this is a stupid hack
	}
}

cv::Mat GrayCodeComputeState::getFundamentalMat(const std::vector<Feature>& imgpts1,
												const std::vector<Feature>& imgpts2,
												std::vector<Feature>& imgpts1_good,
												std::vector<Feature>& imgpts2_good,
												std::vector<cv::DMatch>& matches) {
	vector<uchar> status(imgpts1.size());
	imgpts1_good.clear();
	imgpts2_good.clear();

	vector<Feature> imgpts1_tmp;
	vector<Feature> imgpts2_tmp;
	if (matches.size() < 7) {
		printf("warning: not enough matches to compute Fundamental matrix\n");
		return Mat();
	} else {
		getAlignedPointsFromMatch(imgpts1, imgpts2, matches, imgpts1_tmp, imgpts2_tmp);
	}

	Mat F;
	{
		vector<Point2f> pts1, pts2;
		featuresToPoints(imgpts1_tmp, pts1);
		featuresToPoints(imgpts2_tmp, pts2);
		double minVal, maxVal;
		minMaxIdx(pts1, &minVal, &maxVal);
		F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006 * maxVal, 0.99, status);
	}

	vector<DMatch> new_matches;
	printf("F keeping %d / %d\n", countNonZero(status), status.size());
	for (size_t i = 0; i < status.size(); ++i) {
		if (status[i]) {
			imgpts1_good.push_back(imgpts1_tmp[i]);
			imgpts2_good.push_back(imgpts2_tmp[i]);
			new_matches.push_back(matches[i]);
		}
	}

	printf("%d matches befores, %d matches after Fundamental Matrix filtering\n", matches.size(), new_matches.size());
	matches = new_matches;

	return F;
}

void GrayCodeComputeState::filterMatchesUsingFundamental() { 
	for (size_t i = 0; i < _p.num_image_sets - 1; ++i) {
		for (size_t j = i + 1; j < _p.num_image_sets; ++j) {
			int older_view = i, working_view = j;
			printf("\nfiltering mataches between view %d and %d\n", i, j);
			getFundamentalMat(_img_pts[older_view],
							  _img_pts[working_view],
							  _img_pts_good[older_view],
							  _img_pts_good[working_view],
							  _matches_matrix[std::make_pair(older_view, working_view)]);
		}
	}
}

void GrayCodeComputeState::wt_doIntrinsicCalibrate() {
	MonoCalibration calibrator;
	calibrator.startCalibration();
	int sz = (_p.intrinsic_end - _p.intrinsic_start + 1);
	calibrator.m_ImageCount = sz / _p.intrinsic_step;
	for (int i = _p.intrinsic_start; i <= _p.intrinsic_end; i+=_p.intrinsic_step) {
		Mat im = imread(getIntrinsicImageFilename(i));
		calibrator.imageLoopCommon(im, i);
		setImage(calibrator.m_SmallImage);

		float percent = 100.f * (float)(i+1) / (float)sz;
		updateProgressBar(_PB.CALIBRATE, percent);
		printf("camera calibration %d/%d %f%%\n", i, sz, percent);
	}
	double err = calibrator.endCalibration();
	_K = calibrator.m_Intrinsic;
	_K_inv = _K.inv();
	_dist_coeffs = calibrator.m_Distortion;
	updateProgressBar(_PB.CALIBRATE, 100.f);

	cout << "CAMERA CALIBRATION RESULTS: " << endl;
	cout << "error = " << err << endl;
	cout << "K = " << endl << _K << endl;
	cout << "dist_coeffs = " << endl << _dist_coeffs << endl;
}

void GrayCodeComputeState::wt_doComputeOverlap() {
	// intialize overlap images
	_overlapImages.clear();
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			_overlapImages[make_pair(i, j)] = Mat::zeros(GRAY_CODE_HEIGHT, 
														 GRAY_CODE_WIDTH, 
														 CV_8UC1);
		}
	}

	// fill overlap images
	for (int k = 0; k < _p.num_image_sets; ++k) {
		Mat rows[3], cols[3];

		// decode gray codes
		for (int p = 0; p < 3; ++p) {
			decodeGrayCode(p, k, rows[p], cols[p]);
		}

		// compute overlap
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {

				for (int y = 0; y < rows[0].size().height; ++y) {
					for (int x = 0; x < rows[0].size().width; ++x) {
						uchar ri = rows[i].at<uchar>(Point(x, y)),
							  ci = cols[i].at<uchar>(Point(x, y)),
							  rj = rows[j].at<uchar>(Point(x, y)),
							  cj = cols[j].at<uchar>(Point(x, y));

						if (ri != 0 && ci != 0 &&
							rj != 0 && cj != 0) {
							_overlapImages[make_pair(i, j)].at<uchar>(rj-1, cj-1) = 255;
						}
					}
				}

			}
		}

		float percent = 100 * (float)(k+1) / (float)_p.num_image_sets;
		updateProgressBar(_PB.COMPUTE_OVERLAP, percent);
	}

	// write to disk
	for (auto it = _overlapImages.begin(); it != _overlapImages.end(); ++it) {
		stringstream ss;
		ss << "overlap_" << it->first.first << "_onto_" << it->first.second << ".jpg";
		imwrite(ss.str(), it->second);
	}

	updateProgressBar(_PB.COMPUTE_OVERLAP, 100);
}

void GrayCodeComputeState::wt_doGrayCodeDecode() {
	_img_pts.resize(_p.num_image_sets);
	_img_pts_good.resize(_p.num_image_sets);
	for (int set_i = 0; set_i < _p.num_image_sets; ++set_i) {
		_img_pts[set_i].clear();
		for (int pj = 0; pj < 3; pj++) {
			// decode graycode for each projector
			Mat rows, cols;
			decodeGrayCode(pj, set_i, rows, cols);

			// convert graycode images to feature vector
			auto pj_fts = grayCodeToFeatures(rows, cols, pj);
			printf("found %d features from proj %d in image set %d\n", pj_fts.size(), pj, set_i);

			// add to image points vector
			for (auto pt = pj_fts.begin(); pt != pj_fts.end(); ++pt) {
				_img_pts[set_i].push_back(*pt);
			}
		}

		printf("found %d total features in image set %d\n", _img_pts[set_i].size(), set_i);

		// update progress bar
		float percent = 100.f * (float)(set_i+1) / (float)_p.num_image_sets;
		updateProgressBar(_PB.DECODE_GC, percent);
		printf("decode graycode %d/%d %f%%\n", set_i+1, _p.num_image_sets, percent);
	}
	updateProgressBar(_PB.DECODE_GC, 100.f);
}

std::vector<Feature> GrayCodeComputeState::getArucoFeatures(int image_set) {
	// get the first image from first projector
	cv::Mat image = cv::imread(getGrayCodeImageFilename(0, image_set, 0));

	// setup aruco
	aruco::MarkerDetector MDetector;
	vector<aruco::Marker> Markers;
	MDetector.setMarkerWarpSize(280);
	MDetector.setThresholdMethod(aruco::MarkerDetector::ThresholdMethods::ADPT_THRES);
	MDetector.setThresholdParams(21, 7);
	MDetector.setCornerRefinementMethod(aruco::MarkerDetector::CornerRefinementMethod::LINES);
	MDetector.setMinMaxSize(0.01f, 0.5f);

	// detect
	aruco::CameraParameters CamParam;
	MDetector.detect(image, Markers, CamParam);

	// convert
	vector<Feature> fts;
	for (auto it = Markers.begin(); it != Markers.end(); ++it) {
		if (!it->isValid()) continue; // only return fully-found markers

		for (size_t i = 0; i < it->size(); ++i) {
			Feature f;
			f.id = cv::Point3i(it->id, i, -1);
			f.pt = (*it)[i];
			f.color = Vec3b(255, 0, 255);

			fts.push_back(f);
		}
	}

	return fts;
}

void GrayCodeComputeState::wt_doArucoDecode() {
	for (int set_i = 0; set_i < _p.num_image_sets; ++set_i) {
		auto fts = getArucoFeatures(set_i);

		// add to feature points vector
		for (auto pt = fts.begin(); pt != fts.end(); ++pt) {
			_img_pts[set_i].push_back(*pt);
		}

		// update progress bar
		float percent = 100.f * (float)(set_i+1) / (float)_p.num_image_sets;
		updateProgressBar(_PB.DECODE_ARUCO, percent);
		printf("decode aruco %d/%d %f%%\n", set_i+1, _p.num_image_sets, percent);
	}
	updateProgressBar(_PB.DECODE_ARUCO, 100.f);
}

void GrayCodeComputeState::wt_doMatchFeatures() {
	// in opencv DMatch, query is first, train is second
	// matching is very simple for structured light, no complicated descriptors
	// or anything needed
	int total = (_p.num_image_sets-1)*_p.num_image_sets / 2; // sum of 1 to n-1
	int curr = 0;
	for (int set_i = 0; set_i < _p.num_image_sets; ++set_i) {
		for (int set_j = set_i + 1; set_j < _p.num_image_sets; ++set_j) {
			std::cout << "--- Match " 
				<< set_i << " , " 
				<< set_j << std::endl;

			std::vector<cv::DMatch> matches = matchFeatures(_img_pts[set_i], _img_pts[set_j]);
			printf("found %d matches between set %d and set %d\n", matches.size(), set_i, set_j);

			_matches_matrix[std::make_pair(set_i, set_j)] = matches;
			_matches_matrix[std::make_pair(set_j, set_i)] = flipMatches(matches);

			// update progress bar
			float percent = 100.f * (float)(++curr) / (float)total;
			updateProgressBar(_PB.FIND_MATCHES, percent);
			printf("find matches %d/%d %f%%\n", curr, total, percent);
		}
	}
	updateProgressBar(_PB.FIND_MATCHES, 100.f);
}

void GrayCodeComputeState::wt_doFirstTriangulation() {
	// filter matches using fundamental matrix ---------------------------------
	filterMatchesUsingFundamental();

	// get base line triangulation ---------------------------------------------
	// this reconstructs the projector 3D point cloud from the first two views
	computeBaselineTriangulation();

	// adjust current bundle ---------------------------------------------------
	// optimize all of the 3D information computed so far using BA
	adjustCurrentBundle();

	// add baseline images to "done" list
	_done_views.clear(); _good_views.clear();

	_done_views.insert(_first_view);
	_done_views.insert(_second_view);
	_good_views.insert(_first_view);
	_good_views.insert(_second_view);
	updateProgressBar(_PB.CAMERA_RECOVERY, 100 * ((float)_done_views.size() / (float)_p.num_image_sets));
}

void GrayCodeComputeState::wt_doCameraRecovery() {
	cv::Matx34d P1 = _P_mats[_second_view];
	cv::Mat_<double> t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
	cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2), 
												   P1(1,0), P1(1,1), P1(1,2), 
												   P1(2,0), P1(2,1), P1(2,2));
	cv::Mat_<double> rvec(1,3); 
	Rodrigues(R, rvec);

	// adjusting the bundle after each new camera is added
	vector<CloudPoint> new_triangulated;	// put this here so the mem can be reused and speed up computation
	vector<int> add_to_cloud;				// same with this one
	vector<cv::Point3f> tmp3d; vector<cv::Point2f> tmp2d;
	vector<cv::Point3f> max_3d; vector<cv::Point2f> max_2d;
	int loop_id = 0;
	while (_done_views.size() != _p.num_image_sets) {
		//find image with highest 2d-3d correspondance [Snavely07 4.2]
		cout << endl << "searching for next image to process" << endl;
		unsigned int max_2d3d_view = -1, max_2d3d_count = 0;
		max_3d.clear(); max_2d.clear();
		for (unsigned int _i=0; _i < _p.num_image_sets; _i++) {
			if (_done_views.find(_i) != _done_views.end()) {
				cout << "already processed view " << _i << endl;
				continue; //already done with this view
			}
			cout << "testing view " << _i << " ";
			tmp3d.clear(); tmp2d.clear();
			find2D3DCorrespondences(_i, tmp3d, tmp2d);
			if(tmp3d.size() > max_2d3d_count) {
				cout << " --- new max " << endl;
				max_2d3d_count = tmp3d.size();
				max_2d3d_view = _i;
				max_3d = tmp3d; max_2d = tmp2d;
			}
		}
		int i = max_2d3d_view; //highest 2d3d matching view

		cout << "PROCESSING VIEW " << i << endl;
		_done_views.insert(i); // don't repeat it for now

		bool pose_estimated = findPoseEstimation(i, rvec, t, R, max_3d, max_2d);
		if (!pose_estimated)
			continue;

		//store estimated pose	
		_P_mats[i] = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
								 R(1,0),R(1,1),R(1,2),t(1),
								 R(2,0),R(2,1),R(2,2),t(2));
		
		// start triangulating with previous GOOD views
		for (set<int>::iterator done_view = _good_views.begin(); done_view != _good_views.end(); ++done_view) {
			int view = *done_view;
			if( view == i ) continue; //skip current...

			cout << " -> " << view << endl;
			
			add_to_cloud.clear();
			new_triangulated.clear();
			bool good_triangulation = triangulatePointsBetween2Views(i, view, new_triangulated, add_to_cloud);
			if(!good_triangulation) continue;

			std::cout << "before triangulation: " << _pcloud.size();
			for (int j=0; j<add_to_cloud.size(); j++) {
				if(add_to_cloud[j] == 1)
					_pcloud.push_back(new_triangulated[j]);
			}
			std::cout << " after " << _pcloud.size() << std::endl;
			//break;
		}
		_good_views.insert(i);
		
		adjustCurrentBundle();
		updateProgressBar(_PB.CAMERA_RECOVERY, 100 * ((float)_done_views.size() / (float)_p.num_image_sets));
		stringstream sslid;
		sslid << "pc-loop-" << loop_id++ << ".ply";
		savePointCloud(sslid.str());
	}
	cout << "camera recovery complete" << endl;
}

void GrayCodeComputeState::wt_doColorizeCloud() {
	for (auto it = _pcloud.begin(); it != _pcloud.end(); ++it) {
		Vec3f sum = Vec3f(0, 0, 0);
		int count = 0;
		for (int i = 0; i < _p.num_image_sets; ++i) {
			if (it->imgpt_for_img[i] != -1) {
				Vec3b clr = _img_pts[i][it->imgpt_for_img[i]].color;
				sum += clr;
				count++;
			}
		}
		it->color = sum / count;
	}
}

// 0 - tape measure, 1 - hiball, 2 - monitor hiball
#define ARUCO_MODE 1
void GrayCodeComputeState::wt_doAlignPointCloud() {
	// hardcoded lab aruco marker positions
	// marker corners are in this order for aruco markers:
	// 0 - top-left
	// 1 - top-right
	// 2 - bottom-right
	// 3 - bototm-left
	vector<CloudPoint> roompc;

	int marker_num = 1;
	int corner_num = 3;

	// hack - store the marker id and point index in the color (just for these hardcoded ones)
	if (ARUCO_MODE == 0) {
		// tape-measured values

		// marker 1
		roompc.push_back(CloudPoint(Point3d(5.207, 0.8175625, 1.3716), Vec3b(1, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(5.08635, 0.9017, 1.3716), Vec3b(1, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(5.207, 0.8175625, 1.2192), Vec3b(1, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(5.08635, 0.9017, 1.2192), Vec3b(1, 2, 0))); // 2

		// marker 2
		roompc.push_back(CloudPoint(Point3d(5.083175, 8.4328, 1.3716), Vec3b(2, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(5.21335, 8.3566, 1.3716), Vec3b(2, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(5.083175, 8.4328, 1.2192), Vec3b(2, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(5.21335, 8.3566, 1.2192), Vec3b(2, 2, 0))); // 2
	
		// marker 3
		roompc.push_back(CloudPoint(Point3d(5.207, 0.8175625, 2.286), Vec3b(3, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(5.08635, 0.9017, 2.286), Vec3b(3, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(5.207, 0.8175625, 2.1336), Vec3b(3, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(5.08635, 0.9017, 2.1336), Vec3b(3, 2, 0))); // 2
	
		// marker 4
		roompc.push_back(CloudPoint(Point3d(5.083175, 8.4328, 2.286), Vec3b(4, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(5.21335, 8.3566, 2.286), Vec3b(4, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(5.083175, 8.4328, 2.1336), Vec3b(4, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(5.21335, 8.3566, 2.1336), Vec3b(4, 2, 0))); // 2
	
		// marker 5
		roompc.push_back(CloudPoint(Point3d(6.76275, 2.0843875, 0.2286), Vec3b(5, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(6.670675, 1.97485, 0.2286), Vec3b(5, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(6.76275, 2.0843875, 0.0762), Vec3b(5, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(6.670675, 1.97485, 0.0762), Vec3b(5, 2, 0))); // 2
	
		// marker 6
		roompc.push_back(CloudPoint(Point3d(7.3025, 3.02895, 0.2286), Vec3b(6, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(7.235825, 2.88925, 0.2286), Vec3b(6, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(7.3025, 3.02895, 0.0762), Vec3b(6, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(7.235825, 2.88925, 0.0762), Vec3b(6, 2, 0))); // 2
	
		// marker 7
		roompc.push_back(CloudPoint(Point3d(7.572375, 3.8862, 0.2286), Vec3b(7, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(7.53745, 3.80365, 0.2286), Vec3b(7, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(7.572375, 3.8862, 0.0762), Vec3b(7, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(7.53745, 3.80365, 0.0762), Vec3b(7, 2, 0))); // 2
	
		// marker 8
		roompc.push_back(CloudPoint(Point3d(7.52475, 5.50545, 0.2286), Vec3b(8, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(7.5644375, 5.32765, 0.2286), Vec3b(8, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(7.52475, 5.50545, 0.0762), Vec3b(8, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(7.5644375, 5.32765, 0.0762), Vec3b(8, 2, 0))); // 2
	
		// marker 9
		roompc.push_back(CloudPoint(Point3d(7.2151875, 6.4277875, 0.2286), Vec3b(9, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(7.28345, 6.24205, 0.2286), Vec3b(9, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(7.2151875, 6.4277875, 0.0762), Vec3b(9, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(7.28345, 6.24205, 0.0762), Vec3b(9, 2, 0))); // 2
	
		// marker 10
		roompc.push_back(CloudPoint(Point3d(6.632575, 7.2517, 0.2286), Vec3b(10, 0, 0))); // 0
		roompc.push_back(CloudPoint(Point3d(6.7262375, 7.15645, 0.2286), Vec3b(10, 1, 0))); // 1
		roompc.push_back(CloudPoint(Point3d(6.632575, 7.2517, 0.0762), Vec3b(10, 3, 0))); // 3
		roompc.push_back(CloudPoint(Point3d(6.7262375, 7.15645, 0.0762), Vec3b(10, 2, 0))); // 2
	} else if (ARUCO_MODE == 1) {
		// hiball measured values

		// marker 1
		roompc.push_back(CloudPoint(Point3d(5.200502, 0.898111, 1.331400), Vec3b(marker_num, (++corner_num % 4), 0)));
		roompc.push_back(CloudPoint(Point3d(5.074415, 0.809715, 1.338548), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.074386, 0.809371, 1.184879), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.198030, 0.894210, 1.182359), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 2
		roompc.push_back(CloudPoint(Point3d(5.072190, 8.433569, 1.337424), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.205739, 8.358469, 1.339611), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.206564, 8.356256, 1.185102), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.070437, 8.429049, 1.176369), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 3
		roompc.push_back(CloudPoint(Point3d(5.198876, 0.886635, 2.258910), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.071925, 0.806732, 2.263170), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.069024, 0.807023, 2.102783), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.199244, 0.884705, 2.106614), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 4
		roompc.push_back(CloudPoint(Point3d(5.074645, 8.435005, 2.254464), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.202751, 8.355309, 2.253119), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.206100, 8.354907, 2.100035), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(5.076169, 8.434076, 2.098759), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 5
		roompc.push_back(CloudPoint(Point3d(6.745815, 2.086629, 0.209342), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.644570, 1.977239, 0.204513), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.656619, 1.975223, 0.070474), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.741322, 2.091895, 0.051621), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 6
		roompc.push_back(CloudPoint(Point3d(7.282308, 3.023762, 0.187309), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.220057, 2.875736, 0.186136), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.213387, 2.888977, 0.032033), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.287534, 3.030685, 0.045455), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 7
		roompc.push_back(CloudPoint(Point3d(7.539008, 3.934842, 0.187239), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.516388, 3.783290, 0.193787), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.515061, 3.795708, 0.043153), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.539378, 3.944202, 0.033620), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 8
		roompc.push_back(CloudPoint(Point3d(7.512077, 5.492311, 0.194672), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.538995, 5.347717, 0.186882), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.533512, 5.348815, 0.036947), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.500030, 5.497694, 0.032292), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 9
		roompc.push_back(CloudPoint(Point3d(7.185083, 6.394013, 0.184465), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.258401, 6.250519, 0.194844), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.239358, 6.256826, 0.026911), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(7.194701, 6.394162, 0.045385), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 10
		roompc.push_back(CloudPoint(Point3d(6.610437, 7.295867, 0.192215), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.700427, 7.181105, 0.190461), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.679712, 7.179300, 0.027594), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(6.603909, 7.296503, 0.039867), Vec3b(marker_num, (++corner_num % 4), 0))); 
	} else if (ARUCO_MODE == 2) {
		marker_num = 0;
		// marker 0
		roompc.push_back(CloudPoint(Point3d(3.937498, 8.329836, 1.223086), Vec3b(marker_num, (++corner_num % 4), 0)));
		roompc.push_back(CloudPoint(Point3d(3.936714, 8.306175, 1.224258), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.934868, 8.307467, 1.199176), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.935263, 8.332590, 1.198274), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 1
		roompc.push_back(CloudPoint(Point3d(3.938691, 7.858940, 1.219340), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.941580, 7.839541, 1.224298), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.936522, 7.839149, 1.199259), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.938369, 7.864775, 1.200578), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 2
		roompc.push_back(CloudPoint(Point3d(3.913317, 7.857716, 0.859339), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.917582, 7.831952, 0.860038), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.914186, 7.829048, 0.834755), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.905429, 7.857234, 0.834949), Vec3b(marker_num, (++corner_num % 4), 0))); 

		marker_num++; // marker 3
		roompc.push_back(CloudPoint(Point3d(3.905679, 8.323234, 0.871090), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.904624, 8.297482, 0.870919), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.900215, 8.296091, 0.846869), Vec3b(marker_num, (++corner_num % 4), 0))); 
		roompc.push_back(CloudPoint(Point3d(3.905818, 8.323249, 0.848061), Vec3b(marker_num, (++corner_num % 4), 0))); 
	}

	// find matches to point cloud points
	vector<pair<size_t, size_t>> matches;
	for (size_t i = 0; i < roompc.size(); ++i) {
		for (size_t j = 0; j < _pcloud.size(); ++j) {
			const CloudPoint& pt1 = roompc[i];
			const CloudPoint& pt2 = _pcloud[j];

			Feature ft;
			for (int k = 0; k < _p.num_image_sets; k++) {
				if (pt2.imgpt_for_img[k] >= 0) {
					ft = _img_pts[k][pt2.imgpt_for_img[k]];
					break;
				}
			}

			if (pt1.color[0] == ft.id.x && pt1.color[1] == ft.id.y) {
				matches.push_back(make_pair(j, i));
				break;
			}
		}
	}

	// align point cloud
	//std::unique_ptr<IPointCloudAlignAlgo> algo(new Affine3DAlign());
	std::unique_ptr<IPointCloudAlignAlgo> algo(new AbsOrHorn());
	_pcloud = algo->alignToRoom(_pcloud, roompc, matches);

	// debug - print the results
	for (auto it = matches.begin(); it != matches.end(); ++it) {
		CloudPoint& pt1 = _pcloud[it->first];
		CloudPoint& pt2 = roompc[it->second];

		cout << "arUco id = " << (int)pt2.color[0] 
			<< ", c = " << (int)pt2.color[1]
			<< " -> [" 
			<< setprecision(4) << pt1.wpt.x << ", "
			<< setprecision(4) << pt1.wpt.y << ", "
			<< setprecision(4) << pt1.wpt.z << "], in = ["
			<< setprecision(4) << pt2.wpt.x << ", "
			<< setprecision(4) << pt2.wpt.y << ", "
			<< setprecision(4) << pt2.wpt.z << "]" << endl;
	}
}

DWORD GrayCodeComputeState::t_Worker() {
	using namespace cv;

	// create progress bars
	createProgressBar(_PB.CALIBRATE, G3D::Color3::green());
	createProgressBar(_PB.DECODE_GC, G3D::Color3::red());
	createProgressBar(_PB.DECODE_ARUCO, G3D::Color3::cyan());
	createProgressBar(_PB.FIND_MATCHES, G3D::Color3::blue());
	createProgressBar(_PB.CAMERA_RECOVERY, G3D::Color3::purple());
	createProgressBar(_PB.COMPUTE_OVERLAP, G3D::Color3::brown());

	// intrinsic calibration ---------------------------------------------------
	wt_doIntrinsicCalibrate();

	// compute projector overlap -----------------------------------------------
	wt_doComputeOverlap();

	// load gray code images and decode region locations -----------------------
	wt_doGrayCodeDecode();

	// extract aruco features from image sets ----------------------------------
	// TODO: give point id of (-aruco_id, corner (tl=1, tr=2, br=3, bl=4), -1)
	wt_doArucoDecode();

	// match features between images -------------------------------------------
	wt_doMatchFeatures();

	// first triangulation between two "best" image sets -----------------------
	wt_doFirstTriangulation();

	// add more images to current reconstruction -------------------------------
	wt_doCameraRecovery();
	wt_doColorizeCloud(); // extract RGB data from images (or in this case just projector id mapped to colors)

	// save the point cloud as an RGB point cloud which can be view by MeshLab -
	// MeshLab supports OBJ files that have only points in them
	// TODO: RGB
	cout << "saving point cloud to disk ... ";
	savePointCloud("pc-before-alignment.ply");
	cout << "done!" << endl;

	// align the point cloud into the room frame using the known aruco markers--
	wt_doAlignPointCloud();

	// save the point cloud again ----------------------------------------------
	savePointCloud("pc-after-alignment.ply");

	// convert the point cloud into a mesh -------------------------------------
	for (int i = GRAY_CODE_GRID_SPACE; i <= MESH_GRID_MAX; i = i << 1) {
		cout << "creating mesh of size " << i << endl;
		wt_doPointCloudToMesh(i);
		stringstream ss;
		ss << "screen-mesh-size-" << i << "-" OUTPUT_POSTFIX ".ply";
		_mesh.writePLY(ss.str());
	}

	// FIXME: modify the rendering program to work on triangles (from PLY file) instead of quads

	// save all compute values as xml.gz for further inspection -----------------
	// TODO - add additional fields to PLY output to accomodate this extra
	// information, and add ability to read point cloud. Create a PointCloud
	// class similar to how the ScreenMesh class works.

	// TODO: put a DONE image on the GUI

	return 0;
}

// writes the point cloud as an ASCII PLY format file
bool GrayCodeComputeState::savePointCloud(const std::string& filename) {
	FILE * fp = fopen(filename.c_str(), "w"); // TODO: should use C++ fstreams
	if (fp == nullptr) return false;

	// write header
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment author: VRWindow2\n");
	fprintf(fp, "comment object: screen point cloud\n");
	fprintf(fp, "element vertex %d\n", _pcloud.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "end_header\n");

	// write vertices w/ color
	for (auto it = _pcloud.begin(); it != _pcloud.end(); ++it) {
		fprintf(fp, "%f %f %f %d %d %d\n",
				it->wpt.x,
				it->wpt.y,
				it->wpt.z,
				it->color[0],
				it->color[1],
				it->color[2]);
	}

	fclose(fp);

	return true;
}

void GrayCodeComputeState::createProgressBar(std::string name, G3D::Color3 color) {
	EnterCriticalSection(&_csProgress);
	SProgressBar pb;
	pb.percent = 0;
	pb.color = color;
	_progressMap[name] = pb;
	LeaveCriticalSection(&_csProgress); // FIXME: RAII
}

void GrayCodeComputeState::updateProgressBar(std::string name, float percent) {
	EnterCriticalSection(&_csProgress);
	_progressMap[name].percent = percent;
	LeaveCriticalSection(&_csProgress);
}

void GrayCodeComputeState::drawBar(G3D::RenderDevice* rd, std::string name, float percent,
	float x, float y, float w, float h, G3D::Color3 foreColor, G3D::Color3 backColor) {

	// draw text
	PD->df->draw2D(rd, name, G3D::Vector2(x, y + h/2), 16, G3D::Color3::white(),
		G3D::Color4::clear(), G3D::GFont::XALIGN_LEFT, G3D::GFont::YALIGN_CENTER);

	// compute bar box
	float textWidth = 250;
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

IAppState::Ptr GrayCodeComputeState::onGraphics2D(G3D::RenderDevice * rd,
	G3D::Array<G3D::Surface2D::Ref>& posed2D) {
	using namespace G3D;

	rd->clear();

	// draw progress bars
	EnterCriticalSection(&_csProgress);
	float Y = _top;
	float padding = 20;
	int NUM_BARS = _progressMap.size();
	float h = std::min<float>((0.55f * _height) / NUM_BARS - padding, 50);

	for (auto it = _progressMap.begin(); it != _progressMap.end(); ++it) {
		drawBar(rd, it->first, it->second.percent, _left, Y, _width, h, it->second.color);
		Y += h + padding;
	}
	LeaveCriticalSection(&_csProgress);

	// draw image
	EnterCriticalSection(&_csImage);
	auto tex = G3D::Texture::fromMemory("frame",
		(const void *)_image.data,
		G3D::ImageFormat::BGR8(),
		_image.size().width,
		_image.size().height,
		1);
	rd->setTexture(0, tex);
	G3D::Draw::rect2D(G3D::Rect2D::xywh(_left + (_width-480)/2, Y, 480, 270), rd);
	LeaveCriticalSection(&_csImage);

	return stayInState();
}

void GrayCodeComputeState::setImage(cv::Mat& image) {
	EnterCriticalSection(&_csImage);
	_image = image.clone();
	LeaveCriticalSection(&_csImage);
}

std::vector<cv::Point3d> cloudPointsToPoints(const std::vector<CloudPoint> cpts) {
	std::vector<cv::Point3d> out;
	for (size_t i = 0; i < cpts.size(); ++i) {
		out.push_back(cpts[i].wpt);
	}
	return out;
}

bool GrayCodeComputeState::testTriangulation(const std::vector<CloudPoint>& pcloud, 
											 const cv::Matx34d& P, 
											 std::vector<uchar>& status) {
	vector<Point3d> pcloud_pt3d = cloudPointsToPoints(pcloud);
	vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());
	
	Matx44d P4x4 = Matx44d::eye(); 
	for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];
	
	perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);
	
	status.resize(pcloud.size(),0);
	for (int i=0; i<pcloud.size(); i++) {
		status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
	}
	int count = countNonZero(status);

	double percentage = ((double)count / (double)pcloud.size());
	cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << endl;
	if(percentage < 0.75)
		return false; //less than 75% of the points are in front of the camera

	//check for coplanarity of points
	if(false) //not
	{
		cv::Mat_<double> cldm(pcloud.size(),3);
		for(unsigned int i=0;i<pcloud.size();i++) {
			cldm.row(i)(0) = pcloud[i].wpt.x;
			cldm.row(i)(1) = pcloud[i].wpt.y;
			cldm.row(i)(2) = pcloud[i].wpt.z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

		int num_inliers = 0;
		cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
		cv::Vec3d x0 = pca.mean;
		double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

		for (int i=0; i<pcloud.size(); i++) {
			Vec3d w = Vec3d(pcloud[i].wpt) - x0;
			double D = fabs(nrm.dot(w));
			if(D < p_to_plane_thresh) num_inliers++;
		}

		cout << num_inliers << "/" << pcloud.size() << " are coplanar" << endl;
		if((double)num_inliers / (double)(pcloud.size()) > 0.85)
			return false;
	}

	return true;
}

void takeSVDOfE(cv::Mat_<double>& E, cv::Mat& svd_u, cv::Mat& svd_vt, cv::Mat& svd_w) {
#if 1
	//Using OpenCV's SVD
	SVD svd(E,SVD::MODIFY_A);
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;
#else
	//Using Eigen's SVD
	cout << "Eigen3 SVD..\n";
	Eigen::Matrix3f  e = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >((double*)E.data).cast<float>();
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(e, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf Esvd_u = svd.matrixU();
	Eigen::MatrixXf Esvd_v = svd.matrixV();
	svd_u = (Mat_<double>(3,3) << Esvd_u(0,0), Esvd_u(0,1), Esvd_u(0,2),
						  Esvd_u(1,0), Esvd_u(1,1), Esvd_u(1,2), 
						  Esvd_u(2,0), Esvd_u(2,1), Esvd_u(2,2)); 
	Mat_<double> svd_v = (Mat_<double>(3,3) << Esvd_v(0,0), Esvd_v(0,1), Esvd_v(0,2),
						  Esvd_v(1,0), Esvd_v(1,1), Esvd_v(1,2), 
						  Esvd_v(2,0), Esvd_v(2,1), Esvd_v(2,2));
	svd_vt = svd_v.t();
	svd_w = (Mat_<double>(1,3) << svd.singularValues()[0] , svd.singularValues()[1] , svd.singularValues()[2]);
#endif
	
	//cout << "----------------------- SVD ------------------------\n";
	//cout << "U:\n"<<svd_u<<"\nW:\n"<<svd_w<<"\nVt:\n"<<svd_vt<<endl;
	//cout << "----------------------------------------------------\n";
}

bool GrayCodeComputeState::decomposeEtoRandT(Mat_<double>& E,
											 Mat_<double>& R1,
											 Mat_<double>& R2,
											 Mat_<double>& t1,
											 Mat_<double>& t2) {

	//Using HZ E decomposition
	Mat svd_u, svd_vt, svd_w;
	takeSVDOfE(E,svd_u,svd_vt,svd_w);

	//check if first and second singular values are the same (as they should be)
	double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
	if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7) {
		cout << "singular values are too far apart\n";
		return false;
	}

	Matx33d W(0,-1,0,	//HZ 9.13
		1,0,0,
		0,0,1);
	Matx33d Wt(0,1,0,
		-1,0,0,
		0,0,1);
	R1 = svd_u * Mat(W) * svd_vt; //HZ 9.19
	R2 = svd_u * Mat(Wt) * svd_vt; //HZ 9.19
	t1 = svd_u.col(2); //u3
	t2 = -svd_u.col(2); //u3

	return true;
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> LinearLSTriangulation(Point3d u,		//homogenous image point (u,v,1)
								   Matx34d P,		//camera 1 matrix
								   Point3d u1,		//homogenous image point in 2nd camera
								   Matx34d P1		//camera 2 matrix
								   ) 
{
	
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),		
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	
	return X;
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> IterativeLinearLSTriangulation(Point3d u,	//homogenous image point (u,v,1)
											Matx34d P,			//camera 1 matrix
											Point3d u1,			//homogenous image point in 2nd camera
											Matx34d P1			//camera 2 matrix
											) {
	const double EPSILON = 0.0001;
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1); 
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
		
		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
		
		//breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
		
		wi = p2x;
		wi1 = p2x1;
		
		//reweight equations and solve
		Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,		
				  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,		
				  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,	
				  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1
				  );
		Mat_<double> B = (Mat_<double>(4,1) <<	  -(u.x*P(2,3)	-P(0,3))/wi,
												  -(u.y*P(2,3)	-P(1,3))/wi,
												  -(u1.x*P1(2,3)	-P1(0,3))/wi1,
												  -(u1.y*P1(2,3)	-P1(1,3))/wi1
						  );
		
		solve(A,B,X_,DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}
	return X;
}

double GrayCodeComputeState::myTriangulatePoints(const std::vector<Feature>& pt_set1,
												 const std::vector<Feature>& pt_set2,
												 const cv::Mat& K,
												 const cv::Mat& Kinv,
												 const cv::Mat& distcoeff,
												 const cv::Matx34d& P,
												 const cv::Matx34d& P1,
												 std::vector<CloudPoint>& pointcloud,
												 std::vector<Feature>& correspImg1Pt) {

	correspImg1Pt.clear();
	
	Matx44d P1_(P1(0, 0), P1(0, 1), P1(0, 2), P1(0, 3),
				P1(1, 0), P1(1, 1), P1(1, 2), P1(1, 3),
				P1(2, 0), P1(2, 1), P1(2, 2), P1(2, 3),
				0,        0,        0,        1);
	Matx44d P1inv(P1_.inv());
	
	cout << "Triangulating...";
	double t = getTickCount();
	vector<double> reproj_error;
	unsigned int pts_size = pt_set1.size();
	
#if 0
	//Using OpenCV's triangulation
	//convert to Point2f
	vector<Point2f> _pt_set1_pt,_pt_set2_pt;
	KeyPointsToPoints(pt_set1,_pt_set1_pt);
	KeyPointsToPoints(pt_set2,_pt_set2_pt);
	
	//undistort
	Mat pt_set1_pt,pt_set2_pt;
	undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
	undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);
	
	//triangulate
	Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2);
	Mat pt_set2_pt_2r = pt_set2_pt.reshape(1, 2);
	Mat pt_3d_h(1,pts_size,CV_32FC4);
	cv::triangulatePoints(P,P1,pt_set1_pt_2r,pt_set2_pt_2r,pt_3d_h);

	//calculate reprojection
	vector<Point3f> pt_3d;
	convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
	cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P(0,0),P(0,1),P(0,2), P(1,0),P(1,1),P(1,2), P(2,0),P(2,1),P(2,2));
	Vec3d rvec; Rodrigues(R ,rvec);
	Vec3d tvec(P(0,3),P(1,3),P(2,3));
	vector<Point2f> reprojected_pt_set1;
	projectPoints(pt_3d,rvec,tvec,K,distcoeff,reprojected_pt_set1);

	for (unsigned int i=0; i<pts_size; i++) {
		CloudPoint cp; 
		cp.pt = pt_3d[i];
		pointcloud.push_back(cp);
		reproj_error.push_back(norm(_pt_set1_pt[i]-reprojected_pt_set1[i]));
	}
#else
	Mat_<double> KP1 = K * Mat(P1);

	for (int i=0; i<pts_size; i++) {
		Point2f kp = pt_set1[i].pt; 
		Point3d u(kp.x,kp.y,1.0);
		Mat_<double> um = Kinv * Mat_<double>(u); 
		u.x = um(0); u.y = um(1); u.z = um(2);

		Point2f kp1 = pt_set2[i].pt; 
		Point3d u1(kp1.x,kp1.y,1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1); 
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);
		
		Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);
		
//		cout << "3D Point: " << X << endl;
//		Mat_<double> x = Mat(P1) * X;
//		cout <<	"P1 * Point: " << x << endl;
//		Mat_<double> xPt = (Mat_<double>(3,1) << x(0),x(1),x(2));
//		cout <<	"Point: " << xPt << endl;
		Mat_<double> xPt_img = KP1 * X;				//reproject
//		cout <<	"Point * K: " << xPt_img << endl;
		Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
				
		{
			double reprj_err = norm(xPt_img_-kp1);
			reproj_error.push_back(reprj_err);

			CloudPoint cp; 
			cp.wpt = Point3d(X(0),X(1),X(2));
			cp.reprojection_error = reprj_err;
			
			pointcloud.push_back(cp);
			correspImg1Pt.push_back(pt_set1[i]);
		}
	}
#endif
	
	Scalar mse = mean(reproj_error);
	t = ((double)getTickCount() - t)/getTickFrequency();
	cout << "Done. ("<<pointcloud.size()<<"points, " << t <<"s, mean reproj err = " << mse[0] << ")"<< endl;
	
	return mse[0];
}

bool GrayCodeComputeState::checkCoherentRotation(cv::Mat_<double>& R) {
	if(fabsf(determinant(R))-1.0 > 1e-07) {
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}

	return true;
}

void GrayCodeComputeState::adjustCurrentBundle() {
	cout << "=== BUNDLE ADJUSTMENT ===" << endl;

	shared_ptr<IBundleAdjuster> BA = make_shared<SSBABundleAdjuster>(); // FIXME don't throw this away every time!
	Mat cam_matrix = _K.clone();
	Mat dist_coeffs = _dist_coeffs.clone();
	BA->adjustBundle(_pcloud, cam_matrix, _dist_coeffs, _img_pts, _P_mats);

	cout << "old K " << endl << _K << endl;
	cout << "old dist_coeffs " << endl << _dist_coeffs << endl;
	_K = cam_matrix;
	_K_inv = _K.inv();
	_dist_coeffs = dist_coeffs;
	cout << "new K " << endl << _K << endl;
	cout << "new dist_coeffs " << endl << _dist_coeffs << endl;
}

void GrayCodeComputeState::find2D3DCorrespondences(int working_view, 
												   std::vector<cv::Point3f>& ppcloud, 
												   std::vector<cv::Point2f>& imgPoints) 
{
	ppcloud.clear(); imgPoints.clear();

	vector<int> pcloud_status(_pcloud.size(), 0);
	for (set<int>::iterator done_view = _good_views.begin(); done_view != _good_views.end(); ++done_view) {
		int old_view = *done_view;
		//check for matches_from_old_to_working between i'th frame and <old_view>'th frame (and thus the current cloud)
		const std::vector<cv::DMatch>& matches_from_old_to_working = _matches_matrix[std::make_pair(old_view,working_view)];

		for (unsigned int match_from_old_view=0; match_from_old_view < matches_from_old_to_working.size(); match_from_old_view++) {
			// the index of the matching point in <old_view>
			int idx_in_old_view = matches_from_old_to_working[match_from_old_view].queryIdx;

			//scan the existing cloud (pcloud) to see if this point from <old_view> exists
			for (unsigned int pcldp=0; pcldp < _pcloud.size(); pcldp++) {
				// see if corresponding point was found in this point
				if (idx_in_old_view == _pcloud[pcldp].imgpt_for_img[old_view] && pcloud_status[pcldp] == 0) //prevent duplicates 
				{
					//3d point in cloud
					ppcloud.push_back(_pcloud[pcldp].wpt);
					//2d point in image i
					imgPoints.push_back(_img_pts[working_view][matches_from_old_to_working[match_from_old_view].trainIdx].pt);

					pcloud_status[pcldp] = 1;
					break;
				}
			}
		}
	}
	cout << "found " << ppcloud.size() << " 3d-2d point correspondences"<<endl;
}

bool GrayCodeComputeState::findPoseEstimation(int working_view,
											  cv::Mat_<double>& rvec,
											  cv::Mat_<double>& t,
											  cv::Mat_<double>& R,
											  std::vector<cv::Point3f> ppcloud,
											  std::vector<cv::Point2f> imgPoints) 
{
	if(ppcloud.size() <= 7 || imgPoints.size() <= 7 || ppcloud.size() != imgPoints.size()) { 
		//something went wrong aligning 3D to 2D points..
		cerr << "couldn't find [enough] corresponding cloud points... (only " << ppcloud.size() << ")" <<endl;
		return false;
	}

	vector<int> inliers;
	// TODO: re-enable GPU computation
	//if(!use_gpu) {
		//use CPU
		double minVal,maxVal; cv::minMaxIdx(imgPoints,&minVal,&maxVal);
		// TODO: profiling macro based on tictoc for all code
		cv::solvePnPRansac(ppcloud, imgPoints, _K, _dist_coeffs, rvec, t,
			true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()),
			inliers, CV_EPNP);
		//CV_PROFILE("solvePnP",cv::solvePnP(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, CV_EPNP);)
	//} else {
#ifdef HAVE_OPENCV_GPU
		//use GPU ransac
		//make sure datatstructures are cv::gpu compatible
		cv::Mat ppcloud_m(ppcloud); ppcloud_m = ppcloud_m.t();
		cv::Mat imgPoints_m(imgPoints); imgPoints_m = imgPoints_m.t();
		cv::Mat rvec_,t_;

		cv::gpu::solvePnPRansac(ppcloud_m,imgPoints_m,K_32f,distcoeff_32f,rvec_,t_,false);

		rvec_.convertTo(rvec,CV_64FC1);
		t_.convertTo(t,CV_64FC1);
#endif
	//}

	vector<cv::Point2f> projected3D;
	cv::projectPoints(ppcloud, rvec, t, _K, _dist_coeffs, projected3D);

	if(inliers.size()==0) { //get inliers
		for(int i=0;i<projected3D.size();i++) {
			if(norm(projected3D[i]-imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}
	}

#if 0
	//display reprojected points and matches
	cv::Mat reprojected; imgs_orig[working_view].copyTo(reprojected);
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::line(reprojected,imgPoints[ppt],projected3D[ppt],cv::Scalar(0,0,255),1);
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::line(reprojected,imgPoints[inliers[ppt]],projected3D[inliers[ppt]],cv::Scalar(0,0,255),1);
	}
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::circle(reprojected, imgPoints[ppt], 2, cv::Scalar(255,0,0), CV_FILLED);
		cv::circle(reprojected, projected3D[ppt], 2, cv::Scalar(0,255,0), CV_FILLED);			
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::circle(reprojected, imgPoints[inliers[ppt]], 2, cv::Scalar(255,255,0), CV_FILLED);
	}
	stringstream ss; ss << "inliers " << inliers.size() << " / " << projected3D.size();
	putText(reprojected, ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255), 2);

	cv::imshow("__tmp", reprojected);
	cv::waitKey(0);
	cv::destroyWindow("__tmp");
#endif
	//cv::Rodrigues(rvec, R);
	//visualizerShowCamera(R,t,0,255,0,0.1);

	if(inliers.size() < (double)(imgPoints.size())/5.0) {
		cerr << "not enough inliers to consider a good pose (" 
			<< inliers.size() << "/" << imgPoints.size() << ")" << endl;
		return false;
	}

	if(cv::norm(t) > 200.0) {
		// this is bad...
		cerr << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	cv::Rodrigues(rvec, R);
	if(!checkCoherentRotation(R)) {
		cerr << "rotation is incoherent. we should try a different base view..." << endl;
		return false;
	}

	std::cout << "found t = " << t << "\nR = \n"<<R<<std::endl;
	return true;
}

bool GrayCodeComputeState::triangulatePointsBetween2Views(
	int working_view, 
	int older_view,
	vector<struct CloudPoint>& new_triangulated,
	vector<int>& add_to_cloud
	) 
{
	// TODO: re-add image names
	// TODO: open-source this app? start making my portfolio
	//       might have to remove tictoc, or write a tictoc2
	//       would need to have significant changes from the sfmtoyexample
	//       to be worth it
	double t = getTickCount();
	cout << " Triangulate " << working_view << " and " << older_view << endl;
	//get the left camera matrix
	//TODO: potential bug - the P mat for <view> may not exist? or does it...
	cv::Matx34d P = _P_mats[older_view];
	cv::Matx34d P1 = _P_mats[working_view];

	std::vector<Feature> pt_set1,pt_set2;
	std::vector<cv::DMatch> matches = _matches_matrix[std::make_pair(older_view,working_view)];
	if (matches.size() == 0) {
		cout << "no matches";
		return false;
	}
	getAlignedPointsFromMatch(_img_pts[older_view], 
							  _img_pts[working_view],
							  matches, pt_set1, pt_set2);


	//adding more triangulated points to general cloud
	double reproj_error = myTriangulatePoints(pt_set1, pt_set2, _K, _K_inv, 
		_dist_coeffs, P, P1, new_triangulated, _correspImg1Pt);
	std::cout << "triangulation reproj error " << reproj_error << std::endl;

	vector<uchar> trig_status;
	if (!testTriangulation(new_triangulated, P, trig_status) || 
		!testTriangulation(new_triangulated, P1, trig_status)) {
		cerr << "Triangulation did not succeed" << endl;
		return false;
	}

	//filter out outlier points with high reprojection
	vector<double> reprj_errors;
	for (int i = 0; i < new_triangulated.size(); i++) { 
		reprj_errors.push_back(new_triangulated[i].reprojection_error); 
	}
	std::sort(reprj_errors.begin(),reprj_errors.end());
	//get the 80% precentile
	double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2
	
	vector<CloudPoint> new_triangulated_filtered;
	std::vector<cv::DMatch> new_matches;
	for (int i = 0; i < new_triangulated.size(); i++) {
		if (trig_status[i] == 0)
			continue; //point was not in front of camera
		if (new_triangulated[i].reprojection_error > 16.0) {
			continue; //reject point
		} 
		if (new_triangulated[i].reprojection_error < 4.0 ||
			new_triangulated[i].reprojection_error < reprj_err_cutoff) 
		{
			new_triangulated_filtered.push_back(new_triangulated[i]);
			new_matches.push_back(matches[i]);
		} else {
			continue;
		}
	}

	cout << "filtered out " << (new_triangulated.size() - new_triangulated_filtered.size()) << " high-error points" << endl;

	//all points filtered out?
	if(new_triangulated_filtered.size() <= 0) return false;
	
	//use filtered points now
	new_triangulated.clear();
	new_triangulated.insert(new_triangulated.begin(), new_triangulated_filtered.begin(), new_triangulated_filtered.end());	
	//use filtered matches
	matches = new_matches;
	
	//update the matches storage
	_matches_matrix[std::make_pair(older_view,working_view)] = new_matches; //just to make sure, remove if unneccesary
	_matches_matrix[std::make_pair(working_view,older_view)] = flipMatches(new_matches);
	
	//now, determine which points should be added to the cloud
	
	add_to_cloud.clear();
	add_to_cloud.resize(new_triangulated.size(),1);
	int found_other_views_count = 0;
	int num_views = _p.num_image_sets;

	//scan new triangulated points, if they were already triangulated before - strengthen cloud
	//#pragma omp parallel for num_threads(1)
	for (int j = 0; j<new_triangulated.size(); j++) {
		new_triangulated[j].imgpt_for_img.resize(_p.num_image_sets, -1);

		//matches[j] corresponds to new_triangulated[j]
		//matches[j].queryIdx = point in <older_view>
		//matches[j].trainIdx = point in <working_view>
		new_triangulated[j].imgpt_for_img[older_view] = matches[j].queryIdx;	//2D reference to <older_view>
		new_triangulated[j].imgpt_for_img[working_view] = matches[j].trainIdx;		//2D reference to <working_view>
		
		bool found_in_other_view = false;
		for (unsigned int view_ = 0; view_ < num_views; view_++) {
			if(view_ != older_view) {
				//Look for points in <view_> that match to points in <working_view>
				const std::vector<cv::DMatch>& submatches = _matches_matrix[std::make_pair(view_,working_view)];
				for (unsigned int ii = 0; ii < submatches.size(); ii++) {
					if (submatches[ii].trainIdx == matches[j].trainIdx &&
						!found_in_other_view) 
					{
						//Point was already found in <view_> - strengthen it in the known cloud, if it exists there
						
						//cout << "2d pt " << submatches[ii].queryIdx << " in img " << view_ << " matched 2d pt " << submatches[ii].trainIdx << " in img " << working_view << endl;
						//if (_done_views.find(view_) != _done_views.end()) {
							for (unsigned int pt3d=0; pt3d<_pcloud.size(); pt3d++) {
								if (_pcloud[pt3d].imgpt_for_img[view_] == submatches[ii].queryIdx) 
								{
									//pcloud[pt3d] - a point that has 2d reference in <view_>

									//cout << "3d point "<<pt3d<<" in cloud, referenced 2d pt " << submatches[ii].queryIdx << " in view " << view_ << endl;
									{
										//_pcloud[pt3d].imgpt_for_img[working_view] = matches[j].trainIdx;
										_pcloud[pt3d].imgpt_for_img[older_view] = matches[j].queryIdx;
										found_in_other_view = true;
										add_to_cloud[j] = 0;
									}
								}
							}
						//}
					}
				}
			}

			if (view_ != working_view) {
				// Look for points in <view_> that match points in <older_view>
				const auto submatches = _matches_matrix[make_pair(older_view, view_)];
				for (size_t ii = 0; ii < submatches.size(); ++ii) {
					if (submatches[ii].queryIdx == matches[j].queryIdx &&
						!found_in_other_view)
					{
						// Point was already found in <view_> strengthen it in the known cloud, if it exists there
						//if (_done_views.find(view_) != _done_views.end()) {
							for (size_t pt3d = 0; pt3d < _pcloud.size(); ++pt3d) {
								if (_pcloud[pt3d].imgpt_for_img[view_] == submatches[ii].trainIdx) {
									_pcloud[pt3d].imgpt_for_img[working_view] = matches[j].trainIdx;
									//_pcloud[pt3d].imgpt_for_img[older_view] = matches[j].queryIdx;
									found_in_other_view = true;
									add_to_cloud[j] = 0;
								}
							}
						//}
					}
				}
			}
		}
// TODO: Re-add / add new, OpenMP pragmas like this one
// #pragma omp critical
		{
			if (found_in_other_view) {
				found_other_views_count++;
			} else {
				add_to_cloud[j] = 1;
			}
		}
	}
	cout << found_other_views_count << "/" << new_triangulated.size() 
		<< " points were found in other views, adding " << cv::countNonZero(add_to_cloud) << " new\n";
	t = ((double)getTickCount() - t)/getTickFrequency();
	cout << "triangulatePointsBetweenViews() done in " << t <<" seconds " << endl;
	return true;
}

// SSBABundleAdjuster ==========================================================
namespace {
	inline void printErrorStatistics(double const f0,
									 V3D::StdDistortionFunction const& distortion,
									 vector<V3D::CameraMatrix> const& cams,
									 vector<V3D::Vector3d> const& Xs,
									 vector<V3D::Vector2d> const& measurements,
									 vector<int> const& correspondingView,
									 vector<int> const& correspondingPoint) {
		int const K = measurements.size();
		
		double meanReprojectionError = 0.0;
		for (int k = 0; k < K; ++k) {
			int const i = correspondingView[k];
			int const j = correspondingPoint[k];
			V3D::Vector2d p = cams[i].projectPoint(distortion, Xs[j]);
			
			double reprojectionError = V3D::norm_L2(f0 * (p - measurements[k]));
			meanReprojectionError += reprojectionError;
		}
		cout << "mean reprojection error (in pixels): " << meanReprojectionError/K << endl;
	}
} // end unnamed namespace

SSBABundleAdjuster::SSBABundleAdjuster() {
}

SSBABundleAdjuster::~SSBABundleAdjuster() {
}

int SSBABundleAdjuster::count2DMeasurements(const std::vector<CloudPoint>& pointCloud) {
	int K = 0;
	for (size_t i=0; i < pointCloud.size(); ++i) {
		for (size_t ii=0; ii < pointCloud[i].imgpt_for_img.size(); ++ii) {
			if (pointCloud[i].imgpt_for_img[ii] >= 0) {
				K++;
			}
		}
	}
	return K;
}

void SSBABundleAdjuster::adjustBundle(std::vector<CloudPoint>& pointCloud,
									  cv::Mat& cam_matrix,
									  cv::Mat& dist_coeffs,
									  const std::vector<std::vector<Feature>>& img_pts,
									  std::map<int, cv::Matx34d>& P_mats) {
	using namespace V3D;

	int N = P_mats.size(), 
		M = pointCloud.size(), 
		K = count2DMeasurements(pointCloud);
	
	cout << "N (cams) = " << N 
		<< " M (points) = " << M 
		<< " K (measurements) = " << K << endl;
	
	StdDistortionFunction distortion;

	// convert camera distortion to BA datastructs
	distortion.k1 = dist_coeffs.at<double>(0);
	distortion.k2 = dist_coeffs.at<double>(1);
	distortion.p1 = dist_coeffs.at<double>(2);
	distortion.p2 = dist_coeffs.at<double>(3);
	cout << "distortion before bundle = " << dist_coeffs << endl;
	
	// convert camera intrinsics to BA datastructs
	Matrix3x3d KMat;
	makeIdentityMatrix(KMat);
	KMat[0][0] = cam_matrix.at<double>(0,0); //fx
	KMat[1][1] = cam_matrix.at<double>(1,1); //fy
	KMat[0][1] = cam_matrix.at<double>(0,1); //skew
	KMat[0][2] = cam_matrix.at<double>(0,2); //ppx
	KMat[1][2] = cam_matrix.at<double>(1,2); //ppy
	
	double const f0 = KMat[0][0];
	cout << "intrinsic before bundle = "; displayMatrix(KMat);
	Matrix3x3d Knorm = KMat;
	// Normalize the intrinsic to have unit focal length.
	scaleMatrixIP(1.0/f0, Knorm);
	Knorm[2][2] = 1.0;
	
	vector<int> pointIdFwdMap(M);
	map<int, int> pointIdBwdMap;
	
	// convert 3D point cloud to BA datastructs
	vector<Vector3d > Xs(M);
	for (int j = 0; j < M; ++j) {
		int pointId = j;
		Xs[j][0] = pointCloud[j].wpt.x;
		Xs[j][1] = pointCloud[j].wpt.y;
		Xs[j][2] = pointCloud[j].wpt.z;
		pointIdFwdMap[j] = pointId;
		pointIdBwdMap.insert(make_pair(pointId, j));
	}
	cout << "Read the 3D points." << endl;
	
	vector<int> camIdFwdMap(N,-1);
	map<int, int> camIdBwdMap;
	
	// convert cameras to BA datastructs
	vector<CameraMatrix> cams(N);
	for (int i = 0; i < N; ++i)
	{
		int camId = i;
		Matrix3x3d R;
		Vector3d T;
		
		Matx34d& P = P_mats[i];
		
		R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); T[0] = P(0,3);
		R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); T[1] = P(1,3);
		R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); T[2] = P(2,3);
		
		camIdFwdMap[i] = camId;
		camIdBwdMap.insert(make_pair(camId, i));
		
		cams[i].setIntrinsic(Knorm);
		cams[i].setRotation(R);
		cams[i].setTranslation(T);
	}
	cout << "Read the cameras." << endl;
	
	vector<Vector2d > measurements;
	vector<int> correspondingView;
	vector<int> correspondingPoint;
	
	measurements.reserve(K);
	correspondingView.reserve(K);
	correspondingPoint.reserve(K);
	
	// convert 2D measurements to BA datastructs
	for (unsigned int k = 0; k < pointCloud.size(); ++k) {
		for (unsigned int i=0; i<pointCloud[k].imgpt_for_img.size(); i++) {
			if (pointCloud[k].imgpt_for_img[i] >= 0) {
				int view = i, point = k;
				Vector3d p, np;
				
				Point cvp = img_pts[i][pointCloud[k].imgpt_for_img[i]].pt;
				p[0] = cvp.x;
				p[1] = cvp.y;
				p[2] = 1.0;
				
				if (camIdBwdMap.find(view) != camIdBwdMap.end() &&
					pointIdBwdMap.find(point) != pointIdBwdMap.end())
				{
					// Normalize the measurements to match the unit focal length.
					scaleVectorIP(1.0/f0, p);
					measurements.push_back(Vector2d(p[0], p[1]));
					correspondingView.push_back(camIdBwdMap[view]);
					correspondingPoint.push_back(pointIdBwdMap[point]);
				}
			}
		}
	} // end for (k)
	
	K = measurements.size();
	
	cout << "Read " << K << " valid 2D measurements." << endl;
	
	printErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

//	V3D::optimizerVerbosenessLevel = 1;
	double const inlierThreshold = 2.0 / fabs(f0);
	
	Matrix3x3d K0 = cams[0].getIntrinsic();
	cout << "K0 = "; displayMatrix(K0);

	bool good_adjustment = false;
	{
		ScopedBundleExtrinsicNormalizer extNorm(cams, Xs);
		ScopedBundleIntrinsicNormalizer intNorm(cams,measurements,correspondingView);
		CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_RADIAL_TANGENTIAL, //V3D::FULL_BUNDLE_FOCAL_LENGTH_PP,
												 inlierThreshold, 
												 K0, 
												 distortion, 
												 cams, 
												 Xs,
												 measurements, 
												 correspondingView, 
												 correspondingPoint);
//		StdMetricBundleOptimizer opt(inlierThreshold,cams,Xs,measurements,correspondingView,correspondingPoint);
		
		/*
		opt.tau = 1e-3;
		opt.maxIterations = 50;
		*/
		opt.tau = 1e-6;
		opt.maxIterations = 5000;

		opt.minimize();
		
		cout << "optimizer status = " << opt.status << endl;
		
		good_adjustment = (opt.status != 2);

		if (good_adjustment) {
			cout << "GOOD ADJUSTMENT" << endl;
		} else {
			cout << "FAILED ADJUSTMENT !!!!!" << endl;
		}
	}
	
	cout << "refined K = "; displayMatrix(K0);
	
	for (int i = 0; i < N; ++i) cams[i].setIntrinsic(K0);
	
	Matrix3x3d Knew = K0;
	scaleMatrixIP(f0, Knew);
	Knew[2][2] = 1.0;
	cout << "Knew = "; displayMatrix(Knew);
	
	printErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);
	
	if(good_adjustment) { //good adjustment?
		
		//Vector3d mean(0.0, 0.0, 0.0);
		//for (unsigned int j = 0; j < Xs.size(); ++j) addVectorsIP(Xs[j], mean);
		//scaleVectorIP(1.0/Xs.size(), mean);
		//
		//vector<float> norms(Xs.size());
		//for (unsigned int j = 0; j < Xs.size(); ++j)
		//	norms[j] = distance_L2(Xs[j], mean);
		//
		//std::sort(norms.begin(), norms.end());
		//float distThr = norms[int(norms.size() * 0.9f)];
		//cout << "90% quantile distance: " << distThr << endl;
		
		//extract 3D points
		for (unsigned int j = 0; j < Xs.size(); ++j) {
			//if (distance_L2(Xs[j], mean) > 3*distThr) makeZeroVector(Xs[j]);
			
			pointCloud[j].wpt.x = Xs[j][0];
			pointCloud[j].wpt.y = Xs[j][1];
			pointCloud[j].wpt.z = Xs[j][2];
		}
		
		//extract adjusted cameras
		for (int i = 0; i < N; ++i) {
			Matrix3x3d R = cams[i].getRotation();
			Vector3d T = cams[i].getTranslation();
			
			Matx34d P;
			P(0,0) = R[0][0]; P(0,1) = R[0][1]; P(0,2) = R[0][2]; P(0,3) = T[0];
			P(1,0) = R[1][0]; P(1,1) = R[1][1]; P(1,2) = R[1][2]; P(1,3) = T[1];
			P(2,0) = R[2][0]; P(2,1) = R[2][1]; P(2,2) = R[2][2]; P(2,3) = T[2];
			
			P_mats[i] = P;
		}

		// extract camera intrinsics
		cam_matrix.at<double>(0,0) = Knew[0][0];
		cam_matrix.at<double>(0,1) = Knew[0][1];
		cam_matrix.at<double>(0,2) = Knew[0][2];
		cam_matrix.at<double>(1,1) = Knew[1][1];
		cam_matrix.at<double>(1,2) = Knew[1][2];

		// extract camera distortion
		dist_coeffs.at<double>(0) = distortion.k1;
		dist_coeffs.at<double>(1) = distortion.k2;
		dist_coeffs.at<double>(2) = distortion.p1;
		dist_coeffs.at<double>(3) = distortion.p2;
		cout << "distortion after bundle = " << dist_coeffs << endl;
	}
}

std::vector<CloudPoint> Affine3DAlign::alignToRoom(const std::vector<CloudPoint>& pc,
												   const std::vector<CloudPoint>& roompc,
												   const std::vector<std::pair<size_t, size_t>>& matches) 
{
	// create sets of just the matches
	vector<Point3d> src;
	vector<Point3d> dst;
	for (size_t i = 0; i < matches.size(); ++i) {
		src.push_back(pc[matches[i].first].wpt);
		dst.push_back(roompc[matches[i].second].wpt);
	}
	Mat transform, inliers;
	estimateAffine3D(src, dst, transform, inliers, 3.0, 9.9);

	vector<CloudPoint> newPc;

	//cout << "PointCloudAlignAlgo::alignToRoom transform = " << endl
	//	<< transform << endl;

	for (size_t i = 0; i < pc.size(); ++i) {
		Mat v = (Mat_<double>(4, 1) << pc[i].wpt.x, pc[i].wpt.y, pc[i].wpt.z, 1.0);
		Mat p = transform * v;
		Point3d npt(p.at<double>(0, 0),
					p.at<double>(1, 0),
					p.at<double>(2, 0));
		CloudPoint cpt = pc[i];
		cpt.wpt = npt;
		newPc.push_back(cpt);
	}
	
	return newPc;
}

cv::Point3d PointCloudOps::findCentroid(const std::vector<CloudPoint>& pc) {
	if (pc.size() == 0) return Point3d();
	Point3d sum = Point3d(0.f, 0.f, 0.f);
	for (auto it = pc.begin(); it != pc.end(); ++it) {
		sum += it->wpt;
	}
	return sum * (1.0 / pc.size());
}

// mesh stuff --- TODO should be a separate class, and much better code ...
size_t addPt(std::map<cv::Point3i, size_t, cmpPoint3i>& idToIdx, 
	const cv::Point3i& ptId,
	const CloudPoint * pt, 
	std::vector<ScreenMesh::ProjVert>& verts,
	const std::vector<std::vector<Feature> >& img_pts) 
{
	auto match = idToIdx.find(ptId);
	if (match == idToIdx.end()) {
		ScreenMesh::ProjVert v;
		v.color = pt->color;
		v.worldPt = pt->wpt;

		if (ptId.z != -1) {
			// if not an arUco point
			for (size_t i = 0; i < img_pts.size(); ++i) {
				if (pt->imgpt_for_img[i] >= 0) {
					const Feature& ft = img_pts[i][pt->imgpt_for_img[i]];

					v.screenPt.x = (ft.id.x + 0.5f) / (float)GRAY_CODE_WIDTH;
					v.screenPt.y = (ft.id.y + 0.5f) / (float)GRAY_CODE_HEIGHT;
					break;
				}
			}
		}

		verts.push_back(v);
		int idx = verts.size() - 1;
		idToIdx[ptId] = idx;
		return idx;
	} else {
		return match->second; 
	}
}

map<cv::Point3i, CloudPoint*, cmpPoint3i> GrayCodeComputeState::genIdToPCMap() {
	map<cv::Point3i, CloudPoint*, cmpPoint3i> idToPC;

	cout << "intializing search structure for point cloud mesh generation" << endl;
	for (size_t j = 0; j < _pcloud.size(); ++j) {
		Point3i id(-1, -1, -1);
		for (size_t k = 0; k < _p.num_image_sets; ++k) {
			if (_pcloud[j].imgpt_for_img[k] >= 0) {
				if (id == Point3i(-1, -1, -1)) {
					id = _img_pts[k][_pcloud[j].imgpt_for_img[k]].id;
				} else {
					// error checking
					if (id != _img_pts[k][_pcloud[j].imgpt_for_img[k]].id) {
						cout << "Severe error!" << endl;
						cout << "  Point " << j
							<< " has mismatched ids." << endl;
						for (size_t kk = 0; kk < _img_pts[k].size(); ++kk) {
							if (_pcloud[j].imgpt_for_img[kk] >= 0) {
								cout << "  imgpt_for_img[" << kk 
									<< "] = " << _pcloud[j].imgpt_for_img[kk] 
									<< endl;
							}
						}
					}
				}
			}
		}
		if (id == Point3i(-1, -1, -1)) continue;

		auto match = idToPC.find(id);
		if (match != idToPC.end()) {
			size_t k = find(_pcloud.begin(), _pcloud.end(), *match->second) - _pcloud.begin();
			cout << "Severe error!?" << endl;
			cout << "  Point " << j 
				<< ", id = " << id 
				<< " already appears in idToPC map as point "
				<< k
				<< endl;

			// compare points
			cout << "  Point " << j << "\t\t\t\t Point " << k << endl;
			cout << "  color " << _pcloud[j].color << ", " << _pcloud[k].color << endl;
			cout << "  reprojection_error " << _pcloud[j].reprojection_error << ", " << _pcloud[k].reprojection_error << endl;
			cout << "  wpt " << _pcloud[j].wpt << ", " << _pcloud[k].wpt << endl;
			for (int kk = 0; kk < _p.num_image_sets; ++kk) {
				if (_pcloud[j].imgpt_for_img[kk] >= 0 ||
					_pcloud[k].imgpt_for_img[kk] >= 0) {
					cout << "  imgpt_for_img[" << kk << "] " 
						<< _pcloud[j].imgpt_for_img[kk] << ", "
						<< _pcloud[k].imgpt_for_img[kk] << endl;
				}
			}
		}
		idToPC[id] = &_pcloud[j];
	}

	return idToPC;
}

void GrayCodeComputeState::wt_doPointCloudToMesh(int size) {
	// create mesh for each projector
	// for now just try to create every triangle in the grid, if
	// a vertex is missing then don't create the triangle

	_mesh.projectors.clear();
	_mesh.vertices.clear();
	_mesh.triangles.clear();

	int step = size;

	// initialize working index map from _pcloud to proj vertex list
	map<cv::Point3i, size_t, cmpPoint3i> idToIdx;
	auto idToPC = genIdToPCMap();

	// for each projector go through list of potential triangles
	// and see if they are in _pcloud. If they are, add to the mesh.
	for (size_t i = 0; i < 3; ++i) {
		cout << "creating triangles for projector " << i << endl;
		_mesh.projectors.push_back(ScreenMesh::Projector());
		int PROJ_WIDTH = 1280; // TODO FIXME configurable generalized
		int PROJ_HEIGHT = 1024;
		_mesh.projectors[i].left = i * PROJ_WIDTH;
		_mesh.projectors[i].top = 0;
		_mesh.projectors[i].width = PROJ_WIDTH;
		_mesh.projectors[i].height = PROJ_HEIGHT;

		for (int y = 0; y < GRAY_CODE_HEIGHT - size; y += step) {
			for (int x = 0; x < GRAY_CODE_WIDTH - size; x += step) {
				map<cv::Point3i, CloudPoint*, cmpPoint3i>::iterator a, b, c;
				Point3i aId, bId, cId;
				// upper triangle
				// a--b
				// | /
				// c
				aId = Point3i(x, y, i);			a = idToPC.find(aId);
				bId = Point3i(x + step, y, i);	b = idToPC.find(bId);
				cId = Point3i(x, y + step, i);	c = idToPC.find(cId);
				if (a != idToPC.end() && b != idToPC.end() && c != idToPC.end()) {
					// add vertices
					ScreenMesh::Triangle ntri;
					ntri.vertIndices[0] = addPt(idToIdx, aId, a->second, _mesh.projectors[i].vertices, _img_pts);
					ntri.vertIndices[1] = addPt(idToIdx, bId, b->second, _mesh.projectors[i].vertices, _img_pts);
					ntri.vertIndices[2] = addPt(idToIdx, cId, c->second, _mesh.projectors[i].vertices, _img_pts);

					// add triangle
					_mesh.projectors[i].triangles.push_back(ntri);
				}


				// lower triangle
				//    a
				//  / |
				// c--b
				aId = Point3i(x + step, y, i);			a = idToPC.find(aId);
				bId = Point3i(x + step, y + step, i);	b = idToPC.find(bId);
				cId = Point3i(x, y + step, i);			c = idToPC.find(cId);
				if (a != idToPC.end() && b != idToPC.end() && c != idToPC.end()) {
					// add vertices
					ScreenMesh::Triangle ntri;
					ntri.vertIndices[0] = addPt(idToIdx, aId, a->second, _mesh.projectors[i].vertices, _img_pts);
					ntri.vertIndices[1] = addPt(idToIdx, bId, b->second, _mesh.projectors[i].vertices, _img_pts);
					ntri.vertIndices[2] = addPt(idToIdx, cId, c->second, _mesh.projectors[i].vertices, _img_pts);

					// add triangle
					_mesh.projectors[i].triangles.push_back(ntri);
				}
			}
		}

		cout << "projector " << i << ": " 
			<< _mesh.projectors[i].vertices.size() << " vertices, " 
			<< _mesh.projectors[i].triangles.size() << " triangles." << endl;
	}

	// create meshes of arUco markers
	for (int id = 1; id <= 10; id++) {
		cout << "creating triangles for arUco code " << id << endl;
		map<cv::Point3i, CloudPoint*, cmpPoint3i>::iterator a, b, c;
		Point3i aId, bId, cId;
		// upper triangle
		// a--b
		// | /
		// c
		aId = Point3i(id, 0, -1); a = idToPC.find(aId);
		bId = Point3i(id, 1, -1); b = idToPC.find(bId);
		cId = Point3i(id, 3, -1); c = idToPC.find(cId);
		if (a != idToPC.end() && b != idToPC.end() && c != idToPC.end()) {
			// add vertices
			ScreenMesh::Triangle ntri;
			ntri.vertIndices[0] = addPt(idToIdx, aId, a->second, _mesh.vertices, _img_pts);
			ntri.vertIndices[1] = addPt(idToIdx, bId, b->second, _mesh.vertices, _img_pts);
			ntri.vertIndices[2] = addPt(idToIdx, cId, c->second, _mesh.vertices, _img_pts);

			// add triangle
			_mesh.triangles.push_back(ntri);
		}


		// lower triangle
		//    a
		//  / |
		// c--b
		aId = Point3i(id, 1, -1); a = idToPC.find(aId);
		bId = Point3i(id, 2, -1); b = idToPC.find(bId);
		cId = Point3i(id, 3, -1); c = idToPC.find(cId);
		if (a != idToPC.end() && b != idToPC.end() && c != idToPC.end()) {
			// add vertices
			ScreenMesh::Triangle ntri;
			ntri.vertIndices[0] = addPt(idToIdx, aId, a->second, _mesh.vertices, _img_pts);
			ntri.vertIndices[1] = addPt(idToIdx, bId, b->second, _mesh.vertices, _img_pts);
			ntri.vertIndices[2] = addPt(idToIdx, cId, c->second, _mesh.vertices, _img_pts);

			// add triangle
			_mesh.triangles.push_back(ntri);
		}
	}
	cout << "aruco: " << _mesh.vertices.size() << " vertices, "
		<< _mesh.triangles.size() << " triangles." << endl;

	cout << "mesh created, debug data: " << endl;
	cout << "  pc size = " << _pcloud.size() << endl;
	cout << "  idToIdx size = " << idToIdx.size() << endl;
	cout << "  idToPC.size = " << idToPC.size() << endl;
}

std::vector<CloudPoint> AbsOrHorn::alignToRoom(const std::vector<CloudPoint>& pc,
											   const std::vector<CloudPoint>& roompc,
											   const std::vector<std::pair<size_t, size_t>>& matches) {
	// convert matches to src, dst vectors
	vector<Point3d> src, dst;
	for (auto it = matches.begin(); it != matches.end(); ++it) {
		src.push_back(pc[it->first].wpt);
		dst.push_back(roompc[it->second].wpt);
	}

	// find rotation, translation, and scale
	Mat transform = doHornMethod(src, dst);

	// transform point cloud
	vector<CloudPoint> newPc;
	for (auto pt = pc.begin(); pt != pc.end(); ++pt) {
		Mat v = (Mat_<double>(4, 1) << pt->wpt.x, pt->wpt.y, pt->wpt.z, 1.0);
		Mat p = transform * v;
		Point3d npt(p.at<double>(0) / p.at<double>(3),
					p.at<double>(1) / p.at<double>(3),
					p.at<double>(2) / p.at<double>(3));
		CloudPoint cpt = *pt;
		cpt.wpt = npt;
		newPc.push_back(cpt);
	}

	return newPc;
}

// TODO: options for disabling scale and weights
// TODO: 2d algorithm
// TODO: error statistics return in IPointCloudAlignAlgo
cv::Mat AbsOrHorn::doHornMethod(const std::vector<cv::Point3d>& src,
								const std::vector<cv::Point3d>& dst) {
	// centering of input data
	Point3d lc = findCentroid(src), rc = findCentroid(dst);
	vector<Point3d> srcc, dstc;
	for (auto pt = src.begin(); pt != src.end(); ++pt) {
		srcc.push_back(*pt - lc);
	}
	for (auto pt = dst.begin(); pt != dst.end(); ++pt) {
		dstc.push_back(*pt - rc);
	}

	// compute S values
	double Sxx = 0, Syx = 0, Szx = 0, 
		   Sxy = 0, Syy = 0, Szy = 0, 
		   Sxz = 0, Syz = 0, Szz = 0;
	for (size_t i = 0; i < srcc.size(); ++i) {
		Point3d ps = srcc[i];
		Point3d pd = dstc[i];

		Sxx += ps.x * pd.x; Syx += ps.y * pd.x; Szx += ps.z * pd.x;
		Sxy += ps.x * pd.y; Syy += ps.y * pd.y; Szy += ps.z * pd.y;
		Sxz += ps.x * pd.z; Syz += ps.y * pd.z; Szz += ps.z * pd.z;
	}
	cout << "Sxx = " << Sxx << endl;
	cout << "Syx = " << Syx << endl;
	cout << "Szx = " << Szx << endl;
	cout << "Sxy = " << Sxy << endl;
	cout << "Syy = " << Syy << endl;
	cout << "Szy = " << Szy << endl;
	cout << "Sxz = " << Sxz << endl;
	cout << "Syz = " << Syz << endl;
	cout << "Szz = " << Szz << endl;

	// compute N
	Mat N = (Mat_<double>(4, 4) << 
		Sxx+Syy+Szz,	Syz-Szy,		Szx-Sxz,		Sxy-Syx,
		Syz-Szy,		Sxx-Syy-Szz,	Sxy+Syx,		Szx+Sxz,
		Szx-Sxz,		Sxy+Syx,		-Sxx+Syy-Szz,	Syz+Szy,
		Sxy-Syx,		Szx+Sxz,		Syz+Szy,		-Sxx-Syy+Szz);
	cout << "N =" << endl << N << endl;

	// find R (rotation)
	Mat evals, evecs;
	eigen(N, evals, evecs);

	Mat q = (Mat_<double>(4, 1) << 
		evecs.at<double>(0, 0), 
		evecs.at<double>(0, 1),
		evecs.at<double>(0, 2),
		evecs.at<double>(0, 3)); // get max eigenvector

	// fix sign ambiguity
	double m = 0;
	for (int i = 0; i < 4; ++i) {
		double v = q.at<double>(i, 0);
		if (abs(v) > abs(m)) m = v;
	}
	q = q * sgn(m);
	cout << "q = " << endl << q << endl;

	// map to orthogonal matrix
	q = q / norm(q);
	double q0 = q.at<double>(0),
		   qx = q.at<double>(1),
		   qy = q.at<double>(2),
		   qz = q.at<double>(3);
	Mat v = (Mat_<double>(3, 1) <<
		q.at<double>(1), q.at<double>(2), q.at<double>(3));
	Mat Z = (Mat_<double>(3, 3) <<
		q0, -qz, qy,
		qz, q0, -qx,
		-qy, qx, q0);
	cout << "Z =" << endl << Z << endl;

	Mat R = v * v.t() + Z*Z;
	cout << "R =" << endl << R << endl;

	// find s (scale)
	double sssT = 0;
	double sssB = 0;
	for (size_t i = 0; i < dstc.size(); ++i) {
		Mat s = (Mat_<double>(3, 1) << srcc[i].x, srcc[i].y, srcc[i].z);
		Mat rs = R * s;
		sssT += dstc[i].x * rs.at<double>(0) +
				dstc[i].y * rs.at<double>(1) +
				dstc[i].z * rs.at<double>(2);

		sssB += (srcc[i].x*srcc[i].x + srcc[i].y*srcc[i].y + srcc[i].z*srcc[i].z);
	}
	double s = sssT / sssB;
	cout << "sssT = " << sssT << " sssB = " << sssB << endl;
	cout << "s = " << s << endl;

	// find t (translation)
	Mat lcm = (Mat_<double>(3, 1) << lc.x, lc.y, lc.z);
	Mat rcm = (Mat_<double>(3, 1) << rc.x, rc.y, rc.z);
	Mat t = rcm - R * (lcm * s);
	cout << "t = " << t << endl;

	// build transform matrix
	return (Mat_<double>(4, 4) <<
		s*R.at<double>(0, 0), s*R.at<double>(0, 1), s*R.at<double>(0, 2), t.at<double>(0),
		s*R.at<double>(1, 0), s*R.at<double>(1, 1), s*R.at<double>(1, 2), t.at<double>(1),
		s*R.at<double>(2, 0), s*R.at<double>(2, 1), s*R.at<double>(2, 2), t.at<double>(2),
		0, 0, 0, 1);
}

cv::Point3d AbsOrHorn::findCentroid(const std::vector<cv::Point3d>& pts) {
	cv::Point3d sum(0, 0, 0);
	for (auto pt = pts.begin(); pt != pts.end(); ++pt) {
		sum += *pt;
	}
	return sum * (1.0 / pts.size());
}