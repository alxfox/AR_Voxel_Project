#include "Postprocessing3d.h"
#include "Benchmark.h"
#include <iostream>
int applyClosure(Model* model, int kernelSize) {
	std::cout << "LOG - PP: starting postprocessing." << std::endl;
	Benchmark::GetInstance().LogPostProcessing(true);
	float thresh = 0;
	if (kernelSize % 2 != 1) {
		std::cerr << "Invalid kernel size for post processing, skipping..." << std::endl;
		return -1;
	}
	int size = (kernelSize-1)/2;
	int x_size = model->getX();
	int y_size = model->getY();
	int z_size = model->getZ();
	Model temp(x_size, y_size, z_size,1.f);
	std::cout << "LOG - PP: starting dilution." << std::endl;
	int counter = 0;
	//dilution
	for (int x = 0; x < x_size; x++) {
		for (int y = 0; y < y_size; y++) {
			for (int z = 0; z < z_size; z++) {
				Vector4f origin_v = model->get(x, y, z);
				if (origin_v.w() > thresh) {
					counter++;
					temp.set(x, y, z, origin_v);
					continue;
				}
				int count = 0;
				Vector4f sum(0, 0, 0, 0);
				for (int i = -size; i <= size; i++) {
					int x_n = x + i;
					//std::cout << "hello? " << size << std::endl;
					if (x_n < 0 || x_n >= x_size) continue;
					for (int j = -size; j <= size; j++) {
						int y_n = y + j;
						if (y_n < 0 || y_n >= y_size) continue;
						for (int k = -size; k <= size; k++) {
							int z_n = z + k;
							if (z_n < 0 || z_n >= z_size) continue;
							//std::cout << "miracle" << std::endl;
							Vector4f val = model->get(x_n, y_n, z_n);
							if (val.w() > thresh) {
								count++;
								sum = sum + val;
							}
						}
					}
				}
				if (count > 0) {
					sum /= count;
					//std::cout << "count was > 0" << std::endl;
				}
				//std::cout << "from " << temp.get(x, y, z) << " to " << sum << std::endl;
				temp.set(x, y, z, sum);
			}
		}
	}
	//erosion
	std::cout << "LOG - PP: starting erosion." << std::endl;
	for (int x = 0; x < x_size; x++) {
		for (int y = 0; y < y_size; y++) {
			for (int z = 0; z < z_size; z++) {
				Vector4f zero(0, 0, 0, 0);
				bool failed = false;
				Vector4f temp_v = model->get(x, y, z);
				if (temp_v.w() < thresh) {
					model->set(x, y, z, temp_v);
					continue;
				}
				for (int i = -size; i <= size; i++) {
					int x_n = x + i;
					if (x_n < 0 || x_n >= x_size) continue;
					for (int j = -size; j <= size; j++) {
						int y_n = y + j;
						if (y_n < 0 || y_n >= y_size) continue;
						for (int k = -size; k <= size; k++) {
							int z_n = z + k;
							if (z_n < 0 || z_n >= z_size) continue;
							Vector4f val = temp.get(x_n, y_n, z_n);
							if (val.w() < thresh) {
								failed = true;
								i = j = k = size + 1;
							}
						}
					}
				}
				if (failed) {
					model->set(x, y, z, zero);
				}
				else {
					model->set(x, y, z, temp.get(x, y, z));
				}
			}
		}
	}
	Benchmark::GetInstance().LogPostProcessing(false);
	std::cout << "LOG - PP: postprocessing completed." << std::endl;
	return 0;
}