#pragma once

#include <opencv2\core\core.hpp>

class ScreenMesh {
public:
	class Triangle {
	public:
		size_t vertIndices[3]; ///< index into projector vertex list
		bool operator==(const Triangle& other) const;
		Triangle(size_t idx1 = 0, size_t idx2 = 0, size_t idx3 = 0) {
			vertIndices[0] = idx1; vertIndices[1] = idx2; vertIndices[2] = idx3;
		}
	};

	class ProjVert {
	public:
		cv::Point2f screenPt; ///< w.r.t. projector ROI from 0.0 to 1.0 (like texture coords)
		cv::Point3d worldPt;
		cv::Vec3b color;
		bool operator==(const ProjVert& other) const;
		ProjVert(cv::Point2f s = cv::Point2f(), 
				 cv::Point3d w = cv::Point3d(), 
				 cv::Vec3b c = cv::Vec3b()) {
			screenPt = s;
			worldPt = w;
			color = c;
		}
	};

	class Projector {
	public:
		int top, left, width, height;
		std::vector<ProjVert> vertices; 
		std::vector<Triangle> triangles;
		bool operator==(const Projector& other) const;
		Projector(int t = 0, int l = 0, int w = 0, int h = 0) {
			top = t; left = l; width = w; height = h;
		}
	};

	std::vector<Projector> projectors;

	// additional mesh data, not associated with the projector
	// used for arUco marker positions, but could be extended in the future
	// these do not use the screen_pt variable
	std::vector<ProjVert> vertices;
	std::vector<Triangle> triangles;

	bool writePLY(const std::string& filename, bool saveAR = true) const;
	bool readPLY(const std::string& filename);

	bool operator==(const ScreenMesh& other) const;
};