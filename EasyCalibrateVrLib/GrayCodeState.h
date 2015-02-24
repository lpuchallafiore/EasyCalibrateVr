#pragma once

#include "AppState.h"

class GrayCodeState : public IAppState {
protected:
	cv::Mat camImage;
	std::vector<cv::Mat> grayCodes;
	std::vector<G3D::Texture::Ref> grayCodeTextures;

	int n_rows, n_cols, col_shift, row_shift;

	int currentSet;
	int currentCode;
	int currentProjector;

	G3D::RealTime dt, tt;
	bool isSaving;
	int saveProj;

	std::vector<cv::Mat> blockVisCount, blockVisColor;

	int m_adj;

public:
	GrayCodeState(const std::shared_ptr<ProcessData>& pd);
	virtual ~GrayCodeState() { }

	virtual Ptr onNetwork();
	virtual Ptr onGraphics2D(G3D::RenderDevice* rd,
		                     G3D::Array<std::shared_ptr<G3D::Surface2D> >& posed2D);
	virtual Ptr onUserInput(G3D::UserInput * ui);
	virtual Ptr onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) { 
		dt = rdt; 
		tt += dt; 
		return stayInState(); 
	}


	static cv::Mat computeGreenMask(cv::Mat& input);
	static bool decodeGrayCodes(int proj_width, int proj_height,
					 std::vector<cv::Mat>& gray_code_images,
					 cv::Mat& regionMask,
					 cv::Mat& decoded_cols, cv::Mat& decoded_rows,
					 cv::Mat& decoded_mask, int thresh = -1);
	static cv::Mat colorize(cv::Mat& decoded, int max);


		// based on "build your own 3d scanner" SIGGRAPH tutorial
	static int generateGrayCodes(int width, int height, 
						  std::vector<cv::Mat>& gray_codes,
						  int& n_cols, int& n_rows,
						  int& col_shift, int& row_shift,
						  bool sl_scan_cols, bool sl_scan_rows);

protected:
	void saveImage();

	void buildBlockVisColor(int proj);
	void updateBlockVis();
};