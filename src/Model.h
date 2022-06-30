#include<Eigen/Dense>

using Eigen::Vector4f;

class Model
{
private:
	const int size_x;
	const int size_y;
	const int size_z;
	std::vector<Vector4f> voxels;
	std::vector<std::vector<Vector4f>> colors;

	int flatten(int x, int y, int z) { return x + getY() * (y + getZ() * z); };

public:
	Model(int x, int y, int z);
	void set(int x, int y, int z, Vector4f v);
	int getX() { return size_x; }
	int getY() { return size_y; }
	int getZ() { return size_z; }
	Vector4f get(int x, int y, int z) {return voxels[flatten(x, y, z)]; }
	void addColor(int x, int y, int z, Vector4f c) { colors[flatten(x, y, z)].push_back(c); };
	std::string to_string();
};

