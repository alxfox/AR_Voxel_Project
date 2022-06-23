#include<iostream>
#include<sstream>
#include<Eigen/Dense>
#include "Model.h"

using Eigen::Vector4f;

Model::Model(int x, int y, int z) : size_x(x), size_y(y), size_z(z), voxels(x*y*z) {
	for (int i = 0; i < x*y*z; i++)
		voxels[i] = Vector4f(0, 0, 0, 1);
};

void Model::set(int x, int y, int z, Vector4f v) {
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
