#ifndef MODEL_H
#define MODEL_H

#include "Utils.h"
#include<Eigen/Dense>
#include <opencv2/core/mat.hpp>

#if (GCC_COMPILER_DETECTED || CLANG_COMPILER_DETECTED)

#define for_each_voxel(x, y, z) \
    if (1) \
    { \
        x = 0; \
        for (; x < model.getX(); x++) \
        { \
            y = 0; \
            for (; y < model.getY(); y++) \
            { \
                z = 0; \
                for (; z < model.getZ(); z++) \
                { \
                    goto body; \
                    loop_continue: ; \
                } \
            } \
        } \
    } \
    else \
        while(1) \
            if(1) \
            { \
                goto loop_continue; \
            } \
            else \
                body:

#else

#define for_each_voxel(x, y, z) \
    if (1) \
    { \
        x = 0; \
        for (; x < model.getX(); x++) \
        { \
            y = 0; \
            for (; y < model.getY(); y++) \
            { \
                z = 0; \
                for (; z < model.getZ(); z++) \
                { \
                    goto label(body, __LINE__); \
                    label(loop_continue, __LINE__): ; \
                } \
            } \
        } \
    } \
    else \
        while(1) \
            if(1) \
            { \
                goto label(loop_continue, __LINE__); \
            } \
            else \
                label(body, __LINE__):

#endif

using Eigen::Vector4f;

struct DCLR {
	Vector4f color;
	float depth;
};

struct Square {
	// vertex ids
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;
	unsigned int idx3;
	// colors
	unsigned int r;
	unsigned int g;
	unsigned int b;
	Square(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2, unsigned int _idx3, unsigned int _r = 0, unsigned int _g = 0, unsigned int _b = 0) :
		idx0(_idx0), idx1(_idx1), idx2(_idx2), idx3(_idx3), r(_r), g(_g), b(_b)
	{}
};

#define MODEL_COLOR Vector4f(50, 168, 141, 1) // Vector4f(255, 255, 255, 1)
#define UNSEEN_COLOR Vector4f(204, 0, 0, 1)

class Model
{
private:
	const int size_x;
	const int size_y;
	const int size_z;
	const float voxel_size;
	std::vector<Vector4f> voxels;
	std::vector<std::vector<DCLR>> colors;
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

	bool isInner(int x, int y, int z) {
		return (
			get(x - 1, y, z)(3) != 0 && get(x + 1, y, z)(3) != 0 &&
			get(x, y - 1, z)(3) != 0 && get(x, y + 1, z)(3) != 0 &&
			get(x, y, z - 1)(3) != 0 && get(x, y, z + 1)(3) != 0
			);
	}

	cv::Vec4f toWord(int x, int y, int z) {
		return cv::Vec4f(y * voxel_size, x * voxel_size, -1 * z * voxel_size, 1);
	}

	cv::Vec4f toWord(cv::Vec3i v) {
		return cv::Vec4f(v(1) * voxel_size, v(0) * voxel_size, -1 * v(2) * voxel_size, 1);
	}

	void addColor(int x, int y, int z, const Vector4f& color, float depth) {
		DCLR c = { color, depth };
		colors[flatten(x, y, z)].push_back(c);
	};

	std::vector<DCLR> getColors(int x, int y, int z) {
		return colors[flatten(x, y, z)];
	}

	void see(int x, int y, int z) { seen[flatten(x, y, z)] = true; }
	void handleUnseen();

	void visit(cv::Vec3i v) {
		see(v(0), v(1), v(2));
	}

	bool visited(cv::Vec3i v) {
		return seen[flatten(v(0), v(1), v(2))];
	}
	std::string to_string();
	bool WriteModel(const std::string& filename = "./out/model_mesh.off");
};

#endif
