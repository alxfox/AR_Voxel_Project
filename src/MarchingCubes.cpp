#pragma once

#include<iostream>

#include "MarchingCubes.h"

bool marchingCubes(Model* model, float threshold, std::string outFileName) {
	std::cout << "LOG - MC: starting to process Voxels." << std::endl;
	SimpleMesh mesh;
	for (int x = -1; x < model->getX(); x++) {
		for (int y = -1; y < model->getY(); y++) {
			for (int z = -1; z < model->getZ(); z++) {
				ProcessVoxel(model, x, y, z, &mesh, threshold);
			}
		}
	}
	std::cout << "LOG - MC: voxel processing completed." << std::endl;

	// write mesh to file
	if (!mesh.WriteMesh(outFileName)) {
		std::cout << "ERR - MC: unable to write output file!" << std::endl;
		return false;
	}

	std::cout << "LOG - MC: Mesh written, marchingCubes completed." << std::endl;

	return true;
}

// Testing
bool testMarchingCubes() {
	std::cout << "Testing marching cubes" << std::endl;
	Model model(4, 4, 4, 1);
	for (int x = 0; x < model.getX(); x++) {
		for (int y = 0; y < model.getY(); y++) {
			for (int z = 0; z < model.getZ(); z++) {
				model.set(x, y, z, Vector4f(0, 255, 100, 1));
			}
		}
	}

	model.set(0, 0, 0, Vector4f(0, 0, 0, 0));
	model.set(3, 0, 0, Vector4f(0, 0, 0, 0));
	model.set(0, 3, 0, Vector4f(0, 0, 0, 0));
	model.set(3, 3, 0, Vector4f(0, 0, 0, 0));
	model.set(0, 0, 3, Vector4f(0, 0, 0, 0));
	model.set(3, 0, 3, Vector4f(0, 0, 0, 0));
	model.set(0, 3, 3, Vector4f(0, 0, 0, 0));
	model.set(3, 3, 3, Vector4f(0, 0, 0, 0));
	model.set(0, 0, 2, Vector4f(0, 0, 0, 0));
	model.set(3, 0, 2, Vector4f(0, 0, 0, 0));
	model.set(0, 3, 2, Vector4f(0, 0, 0, 0));
	model.set(3, 3, 2, Vector4f(0, 0, 0, 0));
	model.set(0, 0, 1, Vector4f(0, 0, 0, 0));
	model.set(3, 0, 1, Vector4f(0, 0, 0, 0));
	model.set(0, 3, 1, Vector4f(0, 0, 0, 0));
	model.set(3, 3, 1, Vector4f(0, 0, 0, 0));

	std::cout << "Model setup completed: " << model.to_string() << std::endl;

	if (!marchingCubes(&model)) {
		std::cout << "Could not perform marching cubes!";
		return false;
	}

	return true;
}