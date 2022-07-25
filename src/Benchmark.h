#pragma once

#include<Eigen/Dense>
#include<chrono>
#include<ctime>

using namespace std::chrono;

#define PERFORMANCE_LOG(performance, start) \
	if(start) \
		performance.start = system_clock::now(); \
	else \
		performance.end = system_clock::now();

#define TIME(performance) \
	((duration<float, std::milli>)(performance.end - performance.start)).count()

struct Performance {
	system_clock::time_point start = system_clock::now();
	system_clock::time_point end = start;
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
		// initialize dummy run to avoid exceptions when benchmarking is disabled
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

	/**
	* @brief Method to access singleton instance (with lazy initialization)
	*
	* @return Benchmark	the singleton instance
	*/
	static Benchmark& GetInstance() {
		static Benchmark instance;

		return instance;
	}

	/**
	* @brief Method to start a new benchmarking run
	*
	* @runName		name or short description of new run
	* @modelSize	dimensions and voxel size of model used in new run
	*/
	void NextRun(const std::string& runName, Eigen::Vector4f modelSize) {
		runNames.push_back(runName);
		modelSizes.push_back(modelSize);
		carving.push_back(Performance());
		coloring.push_back(Performance());
		postProcessing.push_back(Performance());
		marchingCubes.push_back(Performance());
		overall.push_back(Performance());
	}

	/**
	* @brief Method to log carving time for current run
	*
	* @param start	whether carving time starts or ends
	*/
	void LogCarving(bool start) {
		PERFORMANCE_LOG(carving.back(), start);
	}

	/**
	* @brief Method to log coloring time for current run
	*
	* @param start	whether coloring time starts or ends
	*/
	void LogColoring(bool start) {
		PERFORMANCE_LOG(coloring.back(), start);
	}

	/**
	* @brief Method to log post processing time for current run
	*
	* @param start	whether post processing time starts or ends
	*/
	void LogPostProcessing(bool start) {
		PERFORMANCE_LOG(postProcessing.back(), start);
	}

	/**
	* @brief Method to log marching cubes time for current run
	*
	* @param start	whether marching cubes time starts or ends
	*/
	void LogMarchingCubes(bool start) {
		PERFORMANCE_LOG(marchingCubes.back(), start);
	}

	/**
	* @brief Method to log overall time for current run
	*
	* @param start	whether overall time starts or ends
	*/
	void LogOverall(bool start) {
		PERFORMANCE_LOG(overall.back(), start);
	}

	/**
	* @brief Method to generate std::string representation of benchmarking since program start
	*
	* @return std::string	output string of benchmark
	*/
	std::string to_string() {
		std::ostringstream ss;
		ss << std::endl << "Benchmark (all times in milliseconds)" << std::endl;
		// write table hadder
		ss << "Name\t\t\t\t" << "|  Model size (x,y,z, voxel size)\t" << "|  Carving time\t" << "|  Coloring time\t" <<
			"|  Postprocessing time\t" << "|  Marching cubes time\t" << "|  Overall time" << std::endl;
		for (int i = 0; i < 177; i++) {
			ss << "-";
		}
		ss << std::endl;
		// skip "dummy" run
		for (int i = 1; i < runNames.size(); i++) {
			ss << runNames.at(i) << "|  " <<
				modelSizes.at(i).x() << "x" << modelSizes.at(i).y() << "x" << modelSizes.at(i).z() << ", " << modelSizes.at(i)(3) << "\t\t\t|  " <<
				TIME(carving.at(i)) << "\t|  " << TIME(coloring.at(i)) << "\t\t|  " << TIME(postProcessing.at(i)) << "\t\t|  " <<
				TIME(marchingCubes.at(i)) << "\t\t|  " << TIME(overall.at(i)) << std::endl;
		}

		return ss.str();
	}
};
