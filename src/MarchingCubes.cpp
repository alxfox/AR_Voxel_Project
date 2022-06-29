#include<iostream>

#include "MarchingCubes.h"

bool marchingCubes(Model model, std::string outFileName = "out/mesh.off", float threshold = 0.5f) {
	std::cout << "LOG - MC: starting to process Voxels." << std::endl;
	SimpleMesh mesh;
	for (int x = 0; x < model.getX() - 1; x++) {
		for (int y = 0; y < model.getY() - 1; y++) {
			for (int z = 0; z < model.getZ() - 1; z++) {
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
	Model model(2, 2, 2);
	model.set(1, 1, 0, Vector4f(255, 255, 0, 0.4));
	model.set(1, 1, 1, Vector4f(255, 255, 0, 0.4));
	if (!marchingCubes(model)) {
		std::cout << "Could not perform marching cubes!";
		return false;
	}

	return true;
}