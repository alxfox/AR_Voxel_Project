#include<iostream>
#include<sstream>
#include<fstream>
#include<Eigen/Dense>
#include "Model.h"

using Eigen::Vector3f;

Model::Model(int x, int y, int z, float size) : size_x(x), size_y(y), size_z(z), voxel_size(size), voxels(x* y* z), colors(x* y* z), seen(x* y* z) {
	for (int i = 0; i < x * y * z; i++) {
		voxels[i] = MODEL_COLOR;
		seen[i] = false;
	}
};

void Model::set(int x, int y, int z, const Vector4f& v) {
	voxels[flatten(x, y, z)] = v;
}

std::string Model::to_string() {
	std::ostringstream ss;
	for (int z = 0; z < getZ(); z++) {
		ss << "z = " << z << ":\n";
		for (int y = 0; y < getY(); y++) {
			for (int x = 0; x < getX(); x++) {
				Vector4f v = get(x, y, z);
				ss << '(' << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << ')';
			}
			ss << '\n';
		}
		ss << '\n';
	}
	return ss.str();
}

void Model::handleUnseen() {
	std::cout << "LOG - PP: marking unseen voxels from model." << std::endl;
	for (int x = 0; x < getX(); x++) {
		for (int y = 0; y < getY(); y++) {
			for (int z = 0; z < getZ(); z++) {
				if (!seen[flatten(x, y, z)]) {
					set(x, y, z, UNSEEN_COLOR);
				}
			}
		}
	}
}

bool Model::WriteModel(const std::string& filename) {
	std::cout << "LOG - Debug: generating debug mesh from model..." << std::endl;

	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		std::cerr << "LOG(ERR) - Debug: could not open file " << filename << ". Aborting mesh generation!" << std::endl;
		return false;
	}

	outFile << "OFF" << std::endl;

	// calculate vertices and faces
	std::vector<Vector3f> vertices;
	std::vector<Square> faces;

	for (int x = 0; x < getX(); x++) {
		for (int y = 0; y < getY(); y++) {
			for (int z = 0; z < getZ(); z++) {
				if (get(x, y, z)(3) == 0 || isInner(x, y, z)) {
					continue;
				}
				int vId = vertices.size();
				vertices.push_back(Vector3f(x, y, z));				//id	0
				vertices.push_back(Vector3f(x + 1, y, z));			//id+1	r
				vertices.push_back(Vector3f(x, y + 1, z));			//id+2	u
				vertices.push_back(Vector3f(x, y, z + 1));			//id+3	h
				vertices.push_back(Vector3f(x + 1, y + 1, z));		//id+4	ru
				vertices.push_back(Vector3f(x + 1, y, z + 1));		//id+5	rh
				vertices.push_back(Vector3f(x, y + 1, z + 1));		//id+6	uh
				vertices.push_back(Vector3f(x + 1, y + 1, z + 1));	//id+7	ruh

				faces.push_back(Square(vId, vId + 2, vId + 4, vId + 1, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // front
				faces.push_back(Square(vId + 3, vId + 5, vId + 7, vId + 6, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // back
				faces.push_back(Square(vId, vId + 2, vId + 6, vId + 3, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // left
				faces.push_back(Square(vId + 1, vId + 4, vId + 7, vId + 5, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // right
				faces.push_back(Square(vId + 2, vId + 6, vId + 7, vId + 4, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // top
				faces.push_back(Square(vId, vId + 3, vId + 5, vId + 1, get(x, y, z).x(), get(x, y, z).y(), get(x, y, z).z())); // bottom
			}
		}
	}

	// write vertices and faces to file
	outFile << vertices.size() << " " << faces.size() << " 0" << std::endl;

	for (unsigned int i = 0; i < vertices.size(); i++) {
		outFile << vertices[i].x() << " " << vertices[i].y() << " " << vertices[i].z() << std::endl;
	}

	for (unsigned int i = 0; i < faces.size(); i++) {
		outFile << "4 " << faces[i].idx0 << " " << faces[i].idx1 << " " << faces[i].idx2 << " " << faces[i].idx3 << " "
			<< faces[i].r << " " << faces[i].g << " " << faces[i].b << std::endl;
	}

	outFile.close();

	std::cout << "LOG - Debug: debug mesh written." << std::endl;

	return true;
}