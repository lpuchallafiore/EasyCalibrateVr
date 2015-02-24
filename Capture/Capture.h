#pragma once

#include <AppState.h>

enum {
	LEFT_EYE = 0,
	RIGHT_EYE,
	NUM_EYES,
};

class CaptureApp : public G3D::GApp {
protected:
	int eye;
	std::shared_ptr<IAppState> currState;

protected:
	void findEllipses(cv::Mat& frame);

public:
	CaptureApp(const GApp::Settings& settings = GApp::Settings());

	virtual void onInit() override;
	virtual void onAI() override;
	virtual void onNetwork() override;
	virtual void onSimulation(G3D::RealTime rdt, G3D::SimTime sdt, G3D::SimTime idt) override;
	virtual void onPose(G3D::Array<G3D::Surface::Ref>& posed3D,
		                G3D::Array<G3D::Surface2D::Ref>& posed2D) override;
	
	virtual void onGraphics(G3D::RenderDevice* rd,
		                    G3D::Array<G3D::Surface::Ref>& posed3D,
							G3D::Array<G3D::Surface2D::Ref>& posed2D);
	virtual void onGraphics3D(G3D::RenderDevice* rd, 
		                      G3D::Array<G3D::Surface::Ref>& posed3D) override;
	virtual void onGraphics2D(G3D::RenderDevice* rd,
		                      G3D::Array<G3D::Surface2D::Ref>& posed2D) override;

	virtual bool onEvent(const G3D::GEvent& e) override;
	virtual void onUserInput(G3D::UserInput* ui) override;
	virtual void onCleanup() override;

	virtual void endProgram();
};
