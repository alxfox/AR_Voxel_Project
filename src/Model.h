#include<Eigen/Dense>

using Eigen::Vector4f;

class Model
{
private:
	const int size_x;
	const int size_y;
	const int size_z;
	std::vector<Vector4f> voxels;

	int flatten(int x, int y, int z) { 
		return x + getX() * (y + getY() * z); 
	};

public:
	Model(int x, int y, int z);
	void set(int x, int y, int z, Vector4f v);
	int getX() { return size_x; }
	int getY() { return size_y; }
	int getZ() { return size_z; }
	Vector4f get(int x, int y, int z) {
		if (x < 0 || x >= size_x || y < 0 || y >= size_y || z < 0 || z >= size_z) {
			return Vector4f(0, 0, 0, 0);
		}
		return voxels[flatten(x, y, z)]; 
	}
	std::string to_string();
};

