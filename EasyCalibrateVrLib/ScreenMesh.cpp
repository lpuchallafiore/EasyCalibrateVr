#include "stdafx.h"
#include "ScreenMesh.h"

// include the RPLY library, which for now is included as part of VRWindowLib
#include "rply.h"

bool ScreenMesh::writePLY(const std::string& filename, 
						  bool saveAR /* = true */) const {
	p_ply ply = ply_create(filename.c_str(), PLY_ASCII, NULL, 0, NULL);
	if (!ply) return false;

	// total up vertices and triangles
	size_t num_verts = 0;
	size_t num_tris = 0;
	
	// projectors
	for (size_t i = 0; i < this->projectors.size(); ++i) {
		num_verts += this->projectors[i].vertices.size();
		num_tris += this->projectors[i].triangles.size();
	}

	// arUco
	if (saveAR) {
		num_verts += this->vertices.size();
		num_tris += this->triangles.size();
	}

	// add comments
	ply_add_comment(ply, "author: VRWindow2");
	ply_add_comment(ply, "object: screen mesh");

	// add projector element definition
	ply_add_element(ply, "projector", this->projectors.size());
	ply_add_scalar_property(ply, "left", PLY_INT32);
	ply_add_scalar_property(ply, "top", PLY_INT32);
	ply_add_scalar_property(ply, "width", PLY_INT32);
	ply_add_scalar_property(ply, "height", PLY_INT32);
	ply_add_scalar_property(ply, "nverts", PLY_UINT32);
	ply_add_scalar_property(ply, "ntris", PLY_UINT32);

	// add vertex element defintion
	ply_add_element(ply, "vertex", num_verts);
	ply_add_scalar_property(ply, "proj_id", PLY_INT32);
	ply_add_scalar_property(ply, "x", PLY_FLOAT64);
	ply_add_scalar_property(ply, "y", PLY_FLOAT64);
	ply_add_scalar_property(ply, "z", PLY_FLOAT64);
	ply_add_scalar_property(ply, "proj_x", PLY_FLOAT32);
	ply_add_scalar_property(ply, "proj_y", PLY_FLOAT32);
	ply_add_scalar_property(ply, "red", PLY_UCHAR);
	ply_add_scalar_property(ply, "green", PLY_UCHAR);
	ply_add_scalar_property(ply, "blue", PLY_UCHAR);

	// add triangle ("face" actually) element definition
	ply_add_element(ply, "face", num_tris);
	ply_add_scalar_property(ply, "proj_id", PLY_INT32);
	ply_add_list_property(ply, "vertex_indices", PLY_UCHAR, PLY_UINT32);

	// write header (comments + element defs)
	if (!ply_write_header(ply)) return false;

	// now write all the data
	// ply_write() must be called in the same order that the elements
	// and properties were defined above in the header.

	// write projector data
	for (size_t i = 0; i < this->projectors.size(); ++i) {
		ply_write(ply, this->projectors[i].left);
		ply_write(ply, this->projectors[i].top);
		ply_write(ply, this->projectors[i].width);
		ply_write(ply, this->projectors[i].height);
		ply_write(ply, this->projectors[i].vertices.size());
		ply_write(ply, this->projectors[i].triangles.size());
	}

	// write projector vertices
	for (size_t i = 0; i < this->projectors.size(); ++i) {
		for (auto it = this->projectors[i].vertices.begin(); 
			it != this->projectors[i].vertices.end();
			++it)
		{
				ply_write(ply, i);
				ply_write(ply, it->worldPt.x);
				ply_write(ply, it->worldPt.y);
				ply_write(ply, it->worldPt.z);
				ply_write(ply, it->screenPt.x);
				ply_write(ply, it->screenPt.y);
				ply_write(ply, it->color[0]);
				ply_write(ply, it->color[1]);
				ply_write(ply, it->color[2]);
		}
	}

	// write arUco vertices
	if (saveAR) {
		for (auto it = this->vertices.begin(); 
			it != this->vertices.end(); 
			++it)
		{
				ply_write(ply, -1);
				ply_write(ply, it->worldPt.x);
				ply_write(ply, it->worldPt.y);
				ply_write(ply, it->worldPt.z);
				ply_write(ply, it->screenPt.x);
				ply_write(ply, it->screenPt.y);
				ply_write(ply, it->color[0]);
				ply_write(ply, it->color[1]);
				ply_write(ply, it->color[2]);
		}
	}

	// write projector triangles
	int offset = 0;
	for (size_t i = 0; i < this->projectors.size(); ++i) {
		for (auto it = this->projectors[i].triangles.begin();
			it != this->projectors[i].triangles.end();
			++it) 
		{
				ply_write(ply, i);
				ply_write(ply, 3); // list length
				ply_write(ply, it->vertIndices[0] + offset);
				ply_write(ply, it->vertIndices[1] + offset);
				ply_write(ply, it->vertIndices[2] + offset);
		}
		offset += this->projectors[i].vertices.size();
	}

	// write arUco triangles
	if (saveAR) {
		for (auto it = this->triangles.begin(); 
			it != this->triangles.end(); 
			++it) 
		{
				ply_write(ply, -1);
				ply_write(ply, 3); // list length
				ply_write(ply, it->vertIndices[0] + offset);
				ply_write(ply, it->vertIndices[1] + offset);
				ply_write(ply, it->vertIndices[2] + offset);
		}
	}

	ply_close(ply);

	return true;
}

namespace {
	size_t tri_proj_id = -2;
	size_t vert_proj_id = -2;

	int projector_cb(p_ply_argument argument) {
		long idx;
		ScreenMesh *self;
		ply_get_argument_user_data(argument, (void**)&self, &idx);
		double value = ply_get_argument_value(argument);
		switch (idx) {
		case 0: // left
			self->projectors.push_back(ScreenMesh::Projector());
			self->projectors.back().left = value;
			break;
		case 1: // top
			self->projectors.back().top = value;
			break;
		case 2: // width
			self->projectors.back().width = value;
			break;
		case 3: // height
			self->projectors.back().height = value;
			break;
		case 4: // nverts
			self->projectors.back().vertices.reserve(value);
			break;
		case 5: // ntris
			self->projectors.back().triangles.reserve(value);
			break;
		}

		return 1;
	}

	int vertex_cb(p_ply_argument argument) {
		long idx;
		ScreenMesh *self;
		ply_get_argument_user_data(argument, (void**)&self, &idx);
		double value = ply_get_argument_value(argument);
		size_t proj_id = (size_t)value;
		if (proj_id < 0 || proj_id >= self->projectors.size()) proj_id = -1;

		switch (idx) {
		case 0: // proj_id
			if (proj_id >= 0 && proj_id < self->projectors.size()) {
				self->projectors[proj_id].vertices.push_back(ScreenMesh::ProjVert());
				vert_proj_id = proj_id;
			} else if (proj_id == -1) {
				self->vertices.push_back(ScreenMesh::ProjVert());
				vert_proj_id = proj_id;
			}
			break;
		case 1: // x
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().worldPt.x = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().worldPt.x = value;
			}
			break;
		case 2: // y
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().worldPt.y = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().worldPt.y = value;
			}
			break;
		case 3: // z
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().worldPt.z = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().worldPt.z = value;
			}
			break;
		case 4: // proj_x
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().screenPt.x = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().screenPt.x = value;
			}
			break;
		case 5: // proj_y
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().screenPt.y = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().screenPt.y = value;
			}
			break;
		case 6: // red
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().color[0] = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().color[0] = value;
			}
			break;
		case 7: // green
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().color[1] = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().color[1] = value;
			}
			break;
		case 8: // blue
			if (vert_proj_id >= 0 && vert_proj_id <= self->projectors.size()) {
				self->projectors[vert_proj_id].vertices.back().color[2] = value;
			} else if (vert_proj_id == -1) {
				self->vertices.back().color[2] = value;
			}
			break;
		}

		return 1;
	}

	int face_cb(p_ply_argument argument) {
		long idx;
		ScreenMesh *self;
		ply_get_argument_user_data(argument, (void**)&self, &idx);

		if (idx == 0) {
			double value = ply_get_argument_value(argument);
			size_t proj_id = (size_t)value;
			if (proj_id < 0 || proj_id >= self->projectors.size()) proj_id = -1;

			if (proj_id >= 0 && proj_id < self->projectors.size()) {
				self->projectors[proj_id].triangles.push_back(ScreenMesh::Triangle());
				tri_proj_id = proj_id;
			} else if (proj_id == -1) {
				self->triangles.push_back(ScreenMesh::Triangle());
				tri_proj_id = proj_id;
			}
		} else if (idx == 1) {
			long length, value_index;
			ply_get_argument_property(argument, NULL, &length, &value_index);
			if (value_index >= 0 && value_index < 3) {
				if (tri_proj_id >= 0 && tri_proj_id < self->projectors.size()) {
					int offset = 0;
					for (size_t i = 0; i  < tri_proj_id; ++i) {
						offset += self->projectors[i].vertices.size();
					}
					self->projectors[tri_proj_id].triangles.back().
						vertIndices[value_index] = ply_get_argument_value(argument) - offset;
				} else if (tri_proj_id == -1) {
					int offset = 0;
					for (auto it = self->projectors.begin(); it != self->projectors.end(); ++it) {
						offset += it->vertices.size();
					}
					self->triangles.back().vertIndices[value_index] = ply_get_argument_value(argument) - offset;
				}
			}
		}

		return 1;
	}
}

// TODO: read aruco mesh
bool ScreenMesh::readPLY(const std::string& filename) {
	size_t nprojectors, nvertices, ntriangles;
	tri_proj_id = vert_proj_id = -2;

	// open the file
	p_ply ply = ply_open(filename.c_str(), NULL, 0, this);
	if (!ply) return false;

	// read the header
	if (!ply_read_header(ply)) return false;

	// get header information and setup callbacks
	nprojectors = ply_set_read_cb(ply, "projector", "left", projector_cb, this, 0);
	ply_set_read_cb(ply, "projector", "top", projector_cb, this, 1);
	ply_set_read_cb(ply, "projector", "width", projector_cb, this, 2);
	ply_set_read_cb(ply, "projector", "height", projector_cb, this, 3);
	ply_set_read_cb(ply, "projector", "nverts", projector_cb, this, 4);
	ply_set_read_cb(ply, "projector", "ntris", projector_cb, this, 5);

	nvertices = ply_set_read_cb(ply, "vertex", "proj_id", vertex_cb, this, 0);
	ply_set_read_cb(ply, "vertex", "x", vertex_cb, this, 1);
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, this, 2);
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, this, 3);
	ply_set_read_cb(ply, "vertex", "proj_x", vertex_cb, this, 4);
	ply_set_read_cb(ply, "vertex", "proj_y", vertex_cb, this, 5);
	ply_set_read_cb(ply, "vertex", "red", vertex_cb, this, 6);
	ply_set_read_cb(ply, "vertex", "green", vertex_cb, this, 7);
	ply_set_read_cb(ply, "vertex", "blue", vertex_cb, this, 8);

	ntriangles = ply_set_read_cb(ply, "face", "proj_id", face_cb, this, 0);
	ply_set_read_cb(ply, "face", "vertex_indices", face_cb, this, 1);

	// read file
	if (!ply_read(ply)) return false;

	ply_close(ply);

	return true;
}

bool ScreenMesh::operator==(const ScreenMesh& other) const {
	return this->projectors == other.projectors
		&& this->triangles == other.triangles
		&& this->vertices == other.vertices;
}

bool ScreenMesh::Triangle::operator==(const ScreenMesh::Triangle& other) const {
	return this->vertIndices[0] == other.vertIndices[0]
		&& this->vertIndices[1] == other.vertIndices[1]
		&& this->vertIndices[2] == other.vertIndices[2];
}

bool ScreenMesh::ProjVert::operator==(const ScreenMesh::ProjVert& other) const {
	return this->color == other.color
		&& this->screenPt == other.screenPt
		&& this->worldPt == other.worldPt;
}

bool ScreenMesh::Projector::operator==(const ScreenMesh::Projector& other) const {
	return this->left == other.left
		&& this->top == other.top
		&& this->width == other.width
		&& this->height == other.height
		&& this->vertices == other.vertices
		&& this->triangles == other.triangles;
}