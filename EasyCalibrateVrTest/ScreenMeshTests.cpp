#include "stdafx.h"
#include "catch.hpp"

#include <ScreenMesh.h>

using namespace std;
using namespace cv;

TEST_CASE("ScreenMesh/PLY-I/O-empty", "[ScreenMesh]") {
	ScreenMesh A, B;

	A.projectors.push_back(ScreenMesh::Projector());
	A.projectors.push_back(ScreenMesh::Projector());
	A.projectors.push_back(ScreenMesh::Projector());

	A.writePLY("test.ply");
	B.readPLY("test.ply");

	REQUIRE(A == B);
}

TEST_CASE("ScreenMesh/PLY-I/O-non-empty", "[ScreenMesh]") {
	ScreenMesh A, B;

	for (int i = 0; i < 2; ++i) {
		ScreenMesh::Projector p;
		p.left = i;
		p.top = i + 1;
		p.width = i + 2;
		p.height = i + 3;

		for (int j = 0; j < 3; ++j) {
			ScreenMesh::ProjVert a, b, c;
			a.color = Vec3b(i+j, i+j, i+j); a.screenPt = Point2f(2*j+i, 3*j+i); a.worldPt = Point3d(4*j+i, 5*j+i, 6*j+i);
			b.color = Vec3b(i+j, i+j, i+j); b.screenPt = Point2f(3*j+i, 4*j+i); b.worldPt = Point3d(5*j+i, 6*j+i, 7*j+i);
			c.color = Vec3b(i+j, i+j, i+j); c.screenPt = Point2f(4*j+i, 5*j+i); c.worldPt = Point3d(6*j+i, 7*j+i, 8*j+i);

			p.vertices.push_back(a);
			p.vertices.push_back(b);
			p.vertices.push_back(c);

			ScreenMesh::Triangle tri;
			tri.vertIndices[0] = p.vertices.size() - 1;
			tri.vertIndices[1] = p.vertices.size() - 2;
			tri.vertIndices[2] = p.vertices.size() - 3;

			p.triangles.push_back(tri);
		}
		A.projectors.push_back(p);
	}

	for (int j = 0; j < 3; ++j) {
		ScreenMesh::ProjVert a, b, c;
		a.color = Vec3b(j, j, j); a.screenPt = Point2f(2*j, 3*j); a.worldPt = Point3d(4*j, 5*j, 6*j);
		b.color = Vec3b(j, j, j); b.screenPt = Point2f(3*j, 4*j); b.worldPt = Point3d(5*j, 6*j, 7*j);
		c.color = Vec3b(j, j, j); c.screenPt = Point2f(4*j, 5*j); c.worldPt = Point3d(6*j, 7*j, 8*j);

		A.vertices.push_back(a);
		A.vertices.push_back(b);
		A.vertices.push_back(c);

		ScreenMesh::Triangle tri;
		tri.vertIndices[0] = A.vertices.size() - 1;
		tri.vertIndices[1] = A.vertices.size() - 2;
		tri.vertIndices[2] = A.vertices.size() - 3;

		A.triangles.push_back(tri);
	}

	A.writePLY("test.ply");
	B.readPLY("test.ply");

	REQUIRE(A == B);
}