#include<iostream>
#include<sstream>
#include<Eigen/Dense>
#include "Model.h"

Model::Model(int x, int y, int z) : size_x(x), size_y(y), size_z(z), voxels(x*y*z), colors(x*y*z), visited(x*y*z) {
	for (int i = 0; i < x * y * z; i++) {
		voxels[i] = Vector4f(50, 168, 141, 1);
		visited[i] = false;
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

void Model::removeUnvisited() {
	std::cout << "LOG - VC: removing unvisited voxels from model." << std::endl;
	for (int x = 0; x < getX(); x++) {
		for (int y = 0; y < getY(); y++) {
			for (int z = 0; z < getZ(); z++) {
				if (!visited[flatten(x, y, z)]) {
					set(x, y, z, Vector4f(0, 0, 0, 0));
				}
			}
		}
	}
}