#include "stdafx.h"
#include "GrayCodeState.h"

#include "ProcessData.h"

// FIXME: assumes all projectors are the same size currently
//        algorithmically no reason to do this, just makes the code easier.

#define GOOD_VIEW_COUNT 5

#define DELTA_TIME 1.5
// must be power of two and match CODE_BLOCK_SIZE in GrayCodeComputeState
#define CODE_BLOCK_SIZE 8

// works well for lab images
#define DEFAULT_DECODE_THRESH 32
#define GREEN_HALF_HUE 64
#define GREEN_DELTA 20

// works well for modified laptop images
//#define GREEN_DELTA 10

using namespace std;
using namespace G3D;
using namespace cv;

GrayCodeState::GrayCodeState(const shared_ptr<ProcessData>& pd) : IAppState(pd) {
	// connect to the camera
	PD->initCam();

	// initialize variables
	currentProjector = 0;
	currentSet = 0;
	currentCode = 0;
	isSaving = false;
	dt = tt = 0;
	m_adj = 1;

	// create gray codes
	generateGrayCodes(PD->data.rawProj[0].width / CODE_BLOCK_SIZE, 
		PD->data.rawProj[0].height / CODE_BLOCK_SIZE, 
		grayCodes, n_cols, n_rows, col_shift, row_shift, true, true);

	// convert gray codes to OpenGL textures
	for (int i = 0; i < grayCodes.size(); ++i) {
		std::stringstream ss;
		ss << "grayCode" << i;
		auto tex = Texture::fromMemory(ss.str(),
			(const void *)grayCodes[i].data,
			ImageFormat::L8(),
			grayCodes[i].size().width,
			grayCodes[i].size().height, 1);
		grayCodeTextures.push_back(tex);
		cout << "added texture " << ss.str() << endl;
	}

	// create block visibility image
	Size sz = Size(PD->data.rawProj[0].width / CODE_BLOCK_SIZE,
		PD->data.rawProj[0].height / CODE_BLOCK_SIZE);
	for (int i = 0; i < PD->data.rawProj.size(); ++i) {
		blockVisCount.push_back(Mat(sz, CV_8UC1));
		blockVisColor.push_back(Mat(sz, CV_8UC3));
		blockVisCount[i].setTo(0);
		buildBlockVisColor(i);
	}
}

IAppState::Ptr GrayCodeState::onNetwork() {
	PD->cam >> camImage;

	return Ptr(NULL);
}

IAppState::Ptr GrayCodeState::onGraphics2D(G3D::RenderDevice *rd,
		G3D::Array<shared_ptr<G3D::Surface2D> >& posed2D) {

	// draw gray code on current projector
	if (isSaving) {
		if (currentCode != 0) {
			rd->setTexture(0, grayCodeTextures[currentCode]);
			Draw::rect2D(Rect2D::xywh(PD->data.rawProj[saveProj].left, 
									  PD->data.rawProj[saveProj].top, 
									  PD->data.rawProj[saveProj].width, 
									  PD->data.rawProj[saveProj].height),
									  rd);
			rd->setTexture(0, nullptr);
		} else {
			Draw::rect2D(Rect2D::xywh(PD->data.rawProj[saveProj].left, 
									  PD->data.rawProj[saveProj].top, 
									  PD->data.rawProj[saveProj].width, 
									  PD->data.rawProj[saveProj].height), 
									  rd,
									  Color3::green());
		}

		// save image if in save mode
		if (tt > ((currentCode == 0 ? 2 : 1) * DELTA_TIME)) {
			saveImage();
			tt = 0;
		}
	} else {
		// draw view percentage plot
		auto tex = Texture::fromMemory("blockVisColor",
			(const void *)blockVisColor[currentProjector].data,
			ImageFormat::BGR8(),
			blockVisColor[currentProjector].size().width,
			blockVisColor[currentProjector].size().height,
			1);
		rd->setTexture(0, tex);
		Draw::rect2D(Rect2D::xywh(PD->data.rawProj[currentProjector].left, 
								  PD->data.rawProj[currentProjector].top, 
								  PD->data.rawProj[currentProjector].width, 
								  PD->data.rawProj[currentProjector].height), 
								  rd);
		rd->setTexture(0, nullptr);

		// letterbox and draw camera image on adjacent projector
		int adj = (currentProjector + m_adj) % 3;
		const RawProjData& pj = PD->data.rawProj[adj];
		float ww = 256 + 192;
		float w = pj.width - ww;
		float scale = w / (float)camImage.size().width;
		float nh = camImage.size().height * scale;
		int top = pj.height / 2 - nh / 2;
		int left = ww/2 + pj.left + (pj.width - ww - w) / 2;
		tex = Texture::fromMemory("camImage",
			(const void *)camImage.data,
			ImageFormat::BGR8(),
			camImage.size().width,
			camImage.size().height,
			1);
		rd->setTexture(0, tex);
		Draw::rect2D(Rect2D::xywh(left, top, w, nh), rd);
		rd->setTexture(0, nullptr);

		// draw current set #, code image #, and projector #
		// above camera image
		stringstream ss;
		ss << "Current Proj: " << currentProjector+1
			<< " Set: " << currentSet+1
			<< " Code: " << currentCode+1 << "/" << grayCodes.size();
		PD->df->draw2D(rd, ss.str(), Vector2(left, top - 64), 16, Color3::white());

		PD->df->draw2D(rd, "z: take images, c: next proj",
			Vector2(left, top + nh + 64), 16, Color3::white());
	}

	return stayInState();
}

std::string makeFilename(int proj, int set, int code) {
	std::stringstream ss;
	ss << "graycode_proj_" << proj
		<< "_set_" << set
		<< "_code_" << code
		<< ".jpg";
	return ss.str();
}

void GrayCodeState::buildBlockVisColor(int proj) {
	Vec3b good(255, 0, 0); // BGR
	Vec3b bad(0, 0, 255);

	for (int r = 0; r < blockVisCount[proj].size().height; ++r) {
		for (int c = 0; c < blockVisCount[proj].size().width; ++c) {
			uchar count = blockVisCount[proj].at<uchar>(r, c);
			float a = (float)count / (float)GOOD_VIEW_COUNT;
			blockVisColor[proj].at<Vec3b>(Point(c, r)) = (1.0-a)*bad + a*good;
		}
	}
}

void GrayCodeState::updateBlockVis() {
	for (int p = 0; p < 3; ++p) {
		// compute mask of last set
		Mat image = imread(makeFilename(p, currentSet-1, 0));
		Mat mask = computeGreenMask(image);
		imwrite("mask.jpg", mask);

		// decode last set
		std::vector<Mat> gcs;
		for (int i = 1; i < grayCodes.size(); ++i) {
			gcs.push_back(imread(makeFilename(p, currentSet-1, i)));
		}
		Mat decoded_cols, decoded_rows, decoded_mask;
		decodeGrayCodes(grayCodes[0].size().width, grayCodes[0].size().height,
			gcs, mask, decoded_cols, decoded_rows, decoded_mask);
		imwrite("decoded_cols.jpg", colorize(decoded_cols, grayCodes[0].size().width));
		imwrite("decoded_rows.jpg", colorize(decoded_rows, grayCodes[0].size().height));
		imwrite("decoded_mask.jpg", decoded_mask);

		// update block-vis
		Mat viewed(blockVisCount[p].size(), CV_8UC1);
		viewed.setTo(0);
		for (int r2 = 0; r2 < decoded_mask.size().height; ++r2) {
			for (int c2 = 0; c2 < decoded_mask.size().width; ++c2) {
				ushort row = decoded_rows.at<ushort>(r2, c2);
				ushort col = decoded_cols.at<ushort>(r2, c2);
				if (row >= 0 && row < viewed.size().height &&
					col >= 0 && col < viewed.size().width) {
					viewed.at<uchar>(row, col) = 1;
				}
			}
		}
		blockVisCount[p] += viewed;

		buildBlockVisColor(p);
	}
}

void GrayCodeState::saveImage() {
	cv::imwrite(makeFilename(saveProj, currentSet, currentCode), camImage);
	currentCode = (currentCode + 1);
	if (currentCode >= grayCodes.size()) {
		currentCode = 0;
		saveProj++;
		if (saveProj >= 3) {
			currentSet++;
			isSaving = false;
			updateBlockVis();
		}
	}
}

IAppState::Ptr GrayCodeState::onUserInput(G3D::UserInput * ui) {
	using namespace G3D;

	if (ui->keyPressed(GKey('z'))) {
		// save image
		tt = 0;
		isSaving = true;
		saveProj = 0;
	}
	if (ui->keyPressed(GKey('c'))) {
		// advance projector
		currentProjector = (currentProjector + 1) % 3;
	}
	if (ui->keyPressed(GKey('b'))) {
		// advance adj projector
		if (m_adj == 1) m_adj = 2;
		else if (m_adj == 2) m_adj = 1;
	}

	return stayInState();
}

// Calculate the base 2 logarithm.
double log2(double x) {
	return log(x)/log(2.0);
}

int GrayCodeState::generateGrayCodes(int width, int height, 
									 std::vector<cv::Mat>& gray_codes,
									 int& n_cols, int& n_rows,
									 int& col_shift, int& row_shift,
									 bool sl_scan_cols, bool sl_scan_rows) {

	// Determine number of required codes and row/column offsets
	if (sl_scan_cols) {
		n_cols = (int)ceil(log2(width));
		col_shift = (int)floor((pow(2.0, n_cols) - width) / 2);
	} else {
		n_cols = 0;
		col_shift = 0;
	}
	if (sl_scan_rows) {
		n_rows = (int)ceil(log2(height));
		row_shift = (int)floor((pow(2.0, n_rows) - height) / 2);
	} else {
		n_rows = 0;
		row_shift = 0;
	}

	// Allocate Gray codes
	gray_codes.clear();
	for (int i = 0; i < 2*(n_cols + n_rows)+1; ++i) {
		cv::Mat nm(height, width, CV_8UC1);
		gray_codes.push_back(nm);
	}
	int step = gray_codes[0].step / sizeof(unsigned char);

	// Define first code as white image
	gray_codes[0].setTo(255);

	// Define Gray codes for projector columns.
	for (int c = 0; c < width; ++c) {
		for (int i = 0; i < n_cols; ++i) {
			unsigned char * data = gray_codes[2*i+1].data;
			if (i > 0) {
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1)^(((c+col_shift) >> (n_cols-i)) & 1);
			} else {
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1);
			}
			data[c] *= 255;
			for (int r = 1; r < height; r++) {
				data[r*step+c] = data[c];
			}

			// inverse image
			gray_codes[2*i+1+1] = ~gray_codes[2*i+1];
		}
	}

	for (int r = 0; r < height; r++) {
		for (int i = 0; i < n_rows; i++) {
			unsigned char* data = gray_codes[2*(i + n_cols) + 1].data;
			if (i > 0) {
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1)^(((r+row_shift) >> (n_rows-i)) & 1);
			} else {
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1);
			}
			data[r*step] *= 255;
			for (int c=1; c<width; c++) {
				data[r*step+c] = data[r*step];
			}

			// inverse image
			gray_codes[2*(i+n_cols)+1+1] = ~gray_codes[2*(i+n_cols)+1];
		}
	}

	return 0;
}

cv::Mat GrayCodeState::computeGreenMask(cv::Mat& input) {
	int targetHue = GREEN_HALF_HUE;
	int delta = GREEN_DELTA;
	int blursz = 7;
	Mat blur, hsv, out(input.size(), CV_8UC1);
	out.setTo(0);
	//cv::blur(input, blur, Size(blursz, blursz));
	cv::medianBlur(input, blur, blursz);
	cvtColor(blur, hsv, CV_BGR2HSV);
	for (int r = 0; r < hsv.size().height; ++r) {
		for (int c = 0; c < hsv.size().width; ++c) {
			Vec3b clr = hsv.at<Vec3b>(r, c);
			if (abs(clr[0] - targetHue) < delta) {
				out.at<uchar>(r, c) = 255;
			}
		}
	}

	// return only largest contour
	vector<vector<Point>> contours;
	// http://bytefish.de/blog/extracting_contours_with_opencv/

	return out;
}

// single image gray codes
// TODO: test image + inverse gray codes, would be much easier to decode ...
bool GrayCodeState::decodeGrayCodes(int proj_width, int proj_height,
					 std::vector<cv::Mat>& gray_code_images,
					 cv::Mat& regionMask,
					 cv::Mat& decoded_cols, cv::Mat& decoded_rows,
					 cv::Mat& decoded_mask, int thresh) {
	using namespace cv;
	if (thresh == -1) thresh = DEFAULT_DECODE_THRESH;

	int n_cols = (int)ceil(log2(proj_width));
	int col_shift = (int)floor((pow(2.0, n_cols)-proj_width)/2);
	int n_rows = (int)ceil(log2(proj_height));
	int row_shift = (int)floor((pow(2.0, n_rows)-proj_height)/2);

	if (gray_code_images.size() != 2*(n_cols+n_rows)) return false;

	// Extract width and height of images.
	int cam_width = gray_code_images[0].size().width;
	int cam_height = gray_code_images[0].size().height;

	// Allocate temp variables
	Mat gray_1(Size(cam_width, cam_height), CV_8UC1);
	Mat gray_2(Size(cam_width, cam_height), CV_8UC1);
	Mat bit_plane_1(Size(cam_width, cam_height), CV_8UC1);
	Mat bit_plane_2(Size(cam_width, cam_height), CV_8UC1);
	Mat temp(Size(cam_width, cam_height), CV_8UC1);

	// Allocate output
	decoded_cols = Mat(Size(cam_width, cam_height), CV_16UC1);
	decoded_rows = Mat(Size(cam_width, cam_height), CV_16UC1);
	decoded_mask = Mat(Size(cam_width, cam_height), CV_8UC1);

	// initialize decoded mask to 0 (indicates no gray code)
	decoded_mask.setTo(0);

	// decode projector columns
	decoded_cols.setTo(0);
	for (int i = 0; i < n_cols; ++i) {
		// binarize image w.r.t. regionMask
		cvtColor(gray_code_images[2*i], gray_1, CV_BGR2GRAY);
		cvtColor(gray_code_images[2*i+1], gray_2, CV_BGR2GRAY);
		gray_1 = gray_1 & regionMask;
		gray_2 = gray_2 & regionMask;
		absdiff(gray_1, gray_2, temp);
		compare(temp, Scalar(thresh), temp, CV_CMP_GE);
		decoded_mask = temp | decoded_mask;
		compare(gray_1, gray_2, bit_plane_2, CV_CMP_GE);
		
		//stringstream ss; ss << "bin_col_" << i << ".jpg";
		//imwrite(ss.str(), bit_plane_2);

		// convert gray code to decimal
		if (i > 0)
			bit_plane_1 = bit_plane_1 ^ bit_plane_2;
		else
			bit_plane_1 = bit_plane_2.clone();
		add(decoded_cols, Scalar(pow(2.0, (int)n_cols-i-1)), decoded_cols, bit_plane_1);
	}
	subtract(decoded_cols, Scalar(col_shift), decoded_cols);

	// decode projector rows
	decoded_rows.setTo(0);
	for (int i = 0; i < n_rows; ++i) {
		// binarize
		cvtColor(gray_code_images[2*(i+n_cols)], gray_1, CV_BGR2GRAY);
		cvtColor(gray_code_images[2*(i+n_cols)+1], gray_2, CV_BGR2GRAY);
		gray_1 = gray_1 & regionMask;
		gray_2 = gray_2 & regionMask;
		absdiff(gray_1, gray_2, temp);
		compare(temp, Scalar(thresh), temp, CV_CMP_GE);
		decoded_mask = temp | decoded_mask;
		compare(gray_1, gray_2, bit_plane_2, CV_CMP_GE);

		//stringstream ss; ss << "bin_row_" << i << ".jpg";
		//imwrite(ss.str(), bit_plane_2);

		// convert gray code to decimal
		if (i > 0)
			bit_plane_1 = bit_plane_1 ^ bit_plane_2;
		else
			bit_plane_1 = bit_plane_2.clone();
		add(decoded_rows, Scalar(pow(2.0, n_rows-i-1)), decoded_rows, bit_plane_1);
	}
	subtract(decoded_rows, Scalar(row_shift), decoded_rows);

	// eliminate invalid row/col estimates
	// will remove pixels that have either row or col missing or erroneous.
	compare(decoded_cols, Scalar(proj_width-1), temp, CV_CMP_LE);
	decoded_mask &= temp;
	compare(decoded_cols, Scalar(0), temp, CV_CMP_GE);
	decoded_mask &= temp;
	compare(decoded_rows, Scalar(proj_height-1), temp, CV_CMP_LE);
	decoded_mask &= temp;
	compare(decoded_rows, Scalar(0), temp, CV_CMP_GE);
	decoded_mask &= temp;
	// zero invalid pixels
	temp = ~decoded_mask;
	decoded_rows.setTo(0, temp);
	decoded_cols.setTo(0, temp);

	// return success
	return true;
}

// tries to match the "Winter" color scheme from Matlab
cv::Mat GrayCodeState::colorize(cv::Mat& decoded, int max) {
	Mat out(decoded.size(), CV_8UC3);

	for (int r = 0; r < decoded.size().height; ++r) {
		for (int c = 0; c < decoded.size().width; ++c) {
			Vec3b clr(0, 0, 0);
			ushort val = decoded.at<ushort>(r, c);
			if (val != 0) {
				clr[2] = 0;
				clr[1] = 255 * (decoded.at<ushort>(r, c) / (float)max);
				clr[0] = 255 - clr[1];
			}
			out.at<Vec3b>(r, c) = clr;
		}
	}

	return out;
}
