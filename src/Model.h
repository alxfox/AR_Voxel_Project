#ifndef MODEL_H
#define MODEL_H

#include<Eigen/Dense>
#include <opencv2/core/mat.hpp>

using Eigen::Vector4f;

class Model
{
private:
	const int size_x;
	const int size_y;
	const int size_z;
	const float voxel_size;
	std::vector<Vector4f> voxels;
	std::vector<std::vector<Vector4f>> colors;
	std::vector<bool> seen;

	int flatten(int x, int y, int z) { 
		return x + getX() * (y + getY() * z); 
	};

public:
	Model(int x, int y, int z, float size);
	void set(int x, int y, int z, const Vector4f& v);
	void set(cv::Vec3i voxel, const Vector4f& value) {
		set(voxel(0), voxel(1), voxel(2), value);
	}
	int getX() { return size_x; }
	int getY() { return size_y; }
	int getZ() { return size_z; }
	float getSize() { return voxel_size; }

    Vector4f get(int x, int y, int z) {
		if (x < 0 || x >= size_x || y < 0 || y >= size_y || z < 0 || z >= size_z) {
			return Vector4f(0, 0, 0, 0);
		}
		return voxels[flatten(x, y, z)]; 
	}

	cv::Vec4f toWord(int x, int y, int z) {
		return cv::Vec4f(y * voxel_size, x * voxel_size, -1 * z * voxel_size, 1);
	}

	cv::Vec4f toWord(cv::Vec3i v) {
		return cv::Vec4f(v(1) * voxel_size, v(0) * voxel_size, -1 * v(2) * voxel_size, 1);
	}
  
	void addColor(int x, int y, int z, const Vector4f& c) { colors[flatten(x, y, z)].push_back(c); };
	void see(int x, int y, int z) { seen[flatten(x, y, z)] = true; }
	void handleUnseen();

	void visit(cv::Vec3i v) {
		see(v(0), v(1), v(2));
	}

	bool visited(cv::Vec3i v) {
		return seen[flatten(v(0), v(1), v(2))];
	}
	std::string to_string();
};

#endif