#pragma once

#include<Eigen/Dense>
#include<chrono>
#include<ctime>

#define PERFORMANCE_LOG(performance, start) \
	if(start) \
		performance.start = std::chrono::system_clock::now(); \
	else \
		performance.end = std::chrono::system_clock::now();

#define TIME(performance) \
	(performance.end - performance.start).count()

struct Performance {
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point end = start;
};

class Benchmark {
private:
	Benchmark() {
		runNames = std::vector<std::string>();
		modelSizes = std::vector<Eigen::Vector4f>();
		carving = std::vector<Performance>();
		coloring = std::vector<Performance>();
		postProcessing = std::vector<Performance>();
		marchingCubes = std::vector<Performance>();
		overall = std::vector<Performance>();
		NextRun("dummy", Eigen::Vector4f());
	}

	//static Benchmark* instance;

	std::vector<std::string> runNames;
	std::vector<Eigen::Vector4f> modelSizes;

	std::vector<Performance> carving;
	std::vector<Performance> coloring;
	std::vector<Performance> postProcessing;
	std::vector<Performance> marchingCubes;

	std::vector<Performance> overall;

public:
	Benchmark(Benchmark const&) = delete;

	void operator=(Benchmark const&) = delete;

	static Benchmark& GetInstance() {
		static Benchmark instance;
		/**
		if (instance == nullptr) {
			instance = new Benchmark();
		}*/
		return instance;
	}

	void NextRun(const std::string& runName, Eigen::Vector4f modelSize) {
		runNames.push_back(runName);
		modelSizes.push_back(modelSize);
		carving.push_back(Performance());
		coloring.push_back(Performance());
		postProcessing.push_back(Performance());
		marchingCubes.push_back(Performance());
		overall.push_back(Performance());
	}

	void LogCarving(bool start) {
		PERFORMANCE_LOG(carving.back(), start);
	}

	void LogColoring(bool start) {
		PERFORMANCE_LOG(coloring.back(), start);
	}

	void LogPostProcessing(bool start) {
		PERFORMANCE_LOG(postProcessing.back(), start);
	}

	void LogMarchingCubes(bool start) {
		PERFORMANCE_LOG(marchingCubes.back(), start);
	}

	void LogOverall(bool start) {
		PERFORMANCE_LOG(overall.back(), start);
	}

	std::string to_string() {
		std::ostringstream ss;
		ss << std::endl << "Benchmark (all times in millisekonds)" << std::endl;
		ss << "Name\t\t\t\t" << "|  Model size (x,y,z, voxel size)\t" << "|  Carving time\t" << "|  Coloring time\t" <<
			"|  Postprocessing time\t" << "|  Marching cubes time\t" << "|  Overall time" << std::endl;
		for (int i = 0; i < 190; i++) {
			ss << "-";
		}
		ss << std::endl;
		for (int i = 1; i < runNames.size(); i++) {
			ss << runNames.at(i) << "\t|  " <<
				modelSizes.at(i).x() << "x" << modelSizes.at(i).y() << "x" << modelSizes.at(i).z() << ", " << modelSizes.at(i)(3) << "\t\t\t|  " <<
				TIME(carving.at(i)) << "\t|  " << TIME(coloring.at(i)) << "\t\t|  " << TIME(postProcessing.at(i)) << "\t\t|  " <<
				TIME(marchingCubes.at(i)) << "\t\t|  " << TIME(overall.at(i)) << std::endl;
		}

		return ss.str();
	}
};