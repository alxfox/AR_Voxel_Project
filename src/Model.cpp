#include<iostream>
#include<sstream>
#include<Eigen/Dense>
#include "Model.h"

Model::Model(int x, int y, int z, float size) : size_x(x), size_y(y), size_z(z), voxel_size(size), voxels(x*y*z), colors(x*y*z), seen(x*y*z) {
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
	std::cout << "LOG - VC: marking unseen voxels from model." << std::endl;
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