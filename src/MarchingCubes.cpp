#include<iostream>

#include "MarchingCubes.h"

bool marchingCubes(Model model, float threshold = 0.5f, std::string outFileName = "out/mesh.off") {
	std::cout << "LOG - MC: starting to process Voxels." << std::endl;
	SimpleMesh mesh;
	for (int x = -1; x < model.getX(); x++) {
		for (int y = -1; y < model.getY(); y++) {
			for (int z = -1; z < model.getZ(); z++) {
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
	Model model(2, 2, 4);
	//std::cout << model.getX() << ", " << model.getY() << ", " << model.getZ() << std::endl;
	for (int x = 0; x < model.getX(); x++) {
		for (int y = 0; y < model.getY(); y++) {
			//std::cout << y << ", " << z << std::endl;
			model.set(x, y, 0, Vector4f(133, 133, 133, 0));
			model.set(x, y, 1, Vector4f(133, 133, 133, 0.2));
			model.set(x, y, 2, Vector4f(133, 133, 133, 0.9));
			model.set(x, y, 3, Vector4f(133, 133, 133, 1));
		}
	}
	std::cout << "Model setup completed: " << model.to_string() << std::endl;

	if (!marchingCubes(model)) {
		std::cout << "Could not perform marching cubes!";
		return false;
	}

	return true;
}