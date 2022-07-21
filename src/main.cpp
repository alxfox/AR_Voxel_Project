#include <vector>
#include <iostream>
#include <filesystem>
#include "Calibration.h"
#include "PoseEstimation.h"
#include "Segmentation.h"
#include "VoxelCarving.h"
#include "ColorReconstruction.h"
#include "MarchingCubes.h"
#include "Postprocessing3d.h"
#include "Benchmark.h"
namespace fs = std::filesystem;
namespace {
	const char* about =
		"AR Voxel Carving Project\n";
	const char* keys =
		"{c        		|       | 1 for AruCo board creation, 2 for camera calibration, 3 for pose estimation, 5 for carving, 6 for predefined benchmarking}"
		"{resize        | 1.0   | Resize the image preview during calibration by this factor (for very high/low res cameras)}"
		"{live          | true  | Whether to use live camera calibration, otherwise images will be taken from ../Data/calib_%02d.jpg}"
		"{images        |       | Give the path to the directory containing the images for pose estimation/carving}"
		"{calibration   |       | Give the path to the result of the camera calibration (eg. kinect_v1.yml)}"
		"{video_id      | -1    | Give the id to the video stream for which you want to estimate the pose}"
		"{carve         | 1     | 1 for standard carving, 2 for fast carving}"
		"{masks         |       | Give the path to the directory containing the image masks}"
		"{x             | 100   | Give the number of voxels in x direction.}"
		"{y             | 100   | Give the number of voxels in y direction.}"
		"{z             | 100   | Give the number of voxels in z direction.}"
		"{size          | 0.0028| Give the side length of a voxel.}"
		"{color         | 0     | 0 for no color reconstruction, 1 for nearest camera, 2 for average color.}"
		"{scale         | 1.0   | Give the scale factor for the output model.}"
		"{dx            | 0.0   | Move model in x direction (unscaled).}"
		"{dy            | 0.0   | Move model in y direction (unscaled).}"
		"{dz            | 0.0   | Move model in z direction (unscaled).}"
		"{model_debug   | false | Whether to generate a raw cube-mesh of the model.}"
		"{postprocessing | true  | Whether to apply posprocessing on the model.}"
		"{intermediateMesh | false  | Whether to generate a mesh after each image (only carving method 1).}"
		"{outFile | ./out/mesh.off  | The filepath (including .off file) the generated mesh should be written to.}"
		;
}

int main(int argc, char* argv[])
{
	// generate folders within build directory
	std::filesystem::create_directories("./out");
	std::filesystem::create_directories("./out/tmp");
	std::filesystem::create_directories("./out/intermediate");
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	if (argc < 2) {
		parser.printMessage();
		return 0;
	}
	int choose = parser.get<int>("c");

	switch (choose) { // entry point for all parts of the program
	case 1: { // generate a charuco board and save it as an image file (print on paper to use for camera calibration and pose estimation)
		Calibration cal{};
		cal.createBoard("out/BoardImage.jpg");
		break;
	}
	case 2:	// calibrate the camera, i.e. generate intrinsics for a specific camera needed to remove distortions when processing images during voxel carving
	{
		// whether the calibration should be performed 
		// live: taking pictures using the default connected webcam
		// in post: using pictures from a specified folder
		bool liveCalibration = parser.get<bool>("live");

		Calibration cal{};
		std::string image_dir = "./out/tmp";
		cal.createBoard();
		cal.resizeFactor = parser.get<float>("resize"); // resize factor for shown images during the calibration process
		if (!liveCalibration) {
			image_dir = parser.get<std::string>("images"); // path to images to be used for calibration
			if (image_dir.empty()) {
				std::cerr << "You need to define an images path (--images)" << std::endl;
				break;
			}
		}
		vector<int> excludedImages;	// images that have been excluded from calibration by the user
		bool calibrating = true;
		bool firstCalibration = true;
		do {
			cal.calibrate(excludedImages, liveCalibration, firstCalibration, image_dir);
			firstCalibration = false;
			liveCalibration = false; //images can only be taken live during the first iteration of calibration
			string input;
			std::cout << "Enter ids of images to exclude, then enter y to repeat calibration. Use l to list current exclusions or n to exit" << std::endl;
			while (true) {
				std::cin >> input;
				try
				{
					const int excludeId{ stoi(input) };
					auto found = std::find(excludedImages.begin(), excludedImages.end(), excludeId);
					if (found != excludedImages.end()) // if already excluded, include it (toggle)
					{
						std::cout << "Now including: " << excludeId << std::endl;
						excludedImages.erase(found);
					}
					else {
						std::cout << "Now excluding: " << excludeId << std::endl;
						excludedImages.push_back(excludeId);
					}
				}
				catch (const std::exception&)//exclusion finished, repeat calibration
				{
					if (input.size() > 0 && input.at(0) == 'n') { // exit calibration
						calibrating = false;
						break;
					}
					else if (input.size() > 0 && input.at(0) == 'l') { // list ids, then continue prompting for more
						std::cout << "List of excluded images: ";
						for (int i = 0; i < excludedImages.size(); i++) {
							if (i != 0)std::cout << ", ";
							std::cout << excludedImages.at(i);
						}
						std::cout << std::endl;
					}
					else { // continue calibration loop with exclusions
						break;
					}
				}
			}
		} while (calibrating);
	}
	break;
	case 3: // pose estimation
	{
		cv::Mat camera_matrix, dist_Coeffs;
		// use calibration file generated previously (-c=2)
		loadCalibrationFile(parser.get<std::string>("calibration"), &camera_matrix, &dist_Coeffs);
		std::cout << camera_matrix << dist_Coeffs << std::endl;

		// Determine whether to run with images or a video stream
		// Pose estimation is then performed for each individual image/frame
		if (parser.get<std::string>("video_id").empty()) {
			std::string image_dir = parser.get<std::string>("images");
			if (image_dir.empty()) {
				std::cout << "You need to define a images path (--images) if you do not set a video stream id (--video_id)" << std::endl;
			}
			else {
				std::vector<std::string> filenames;
				cv::glob(parser.get<std::string>("images"), filenames);
				for (int i = 0; i < filenames.size(); i++) {
					cv::Mat image = cv::imread(filenames[i], 1);
					cv::Mat transformation_matrix = estimatePoseFromImage(camera_matrix, dist_Coeffs, image, true);
					std::cout << transformation_matrix << std::endl;
				}
			}
		}
		else {
			cv::VideoCapture inputVideo;
			inputVideo.open(parser.get<int>("video_id"));
			while (inputVideo.grab()) {
				cv::Mat image;
				inputVideo.retrieve(image);
				cv::Mat transformation_matrix = estimatePoseFromImage(camera_matrix, dist_Coeffs, image, true);
			}
		}
	}
	break;
	case 4: // image segmentation
	{
		cv::VideoCapture inputVideo;
		inputVideo.open(parser.get<int>("video_id"));
		while (inputVideo.grab()) {
			cv::Mat image;
			inputVideo.retrieve(image);
			cv::Mat mask = color_segmentation(image); //segmentation based on color range
			cv::Mat segmentated_img;
			image.copyTo(segmentated_img, mask);
			imshow("mask", mask);
			imshow("source", image);
			imshow("segmentation", segmentated_img);
			char key = (char)waitKey(1);
		}
	}
	break;
	case 5: // voxel carving
	{
		int carveArg = parser.get<int>("carve");
		if (carveArg < 1 || 2 < carveArg) {
			std::cerr << "Invalid carve argument.";
			break;
		}
		// we read both images and masks for voxel carving
		// the masks are used for the actual carving, the images allow us to reconstruct color
		std::string image_dir = parser.get<std::string>("images");
		if (image_dir.empty()) {
			std::cerr << "You need to define a images path (--images)" << std::endl;
			break;
		}

		if (parser.get<float>("scale") == 0) {
			std::cerr << "The programm won't compute stuff for fun! Enter a scale != 0.0" << std::endl;
		}

		std::vector<std::string> image_filenames;
		cv::glob(image_dir, image_filenames);
		std::vector<cv::Mat> images;
		for (int i = 0; i < image_filenames.size(); i++) {
			images.push_back(cv::imread(image_filenames[i], 1));
		}

		std::cout << "LOG - VC: images read." << std::endl;

		// masks for the images must have been previously generated using image segmentation (-c=4)
		std::string masks_dir = parser.get<std::string>("masks");
		if (masks_dir.empty()) {
			std::cerr << "You need to define a image masks path (--masks)" << std::endl;
			break;
		}
		
		std::vector<std::string> mask_filenames;
		cv::glob(masks_dir, mask_filenames);
		std::vector<cv::Mat> masks;
		for (int i = 0; i < mask_filenames.size(); i++) {
			masks.push_back(cv::imread(mask_filenames[i], 1));
		}

		std::cout << "LOG - VC: masks read." << std::endl;

		if (images.size() != masks.size()) {
			std::cerr << "Number of images doesn't match number of masks." << std::endl;
			break;
		}

		int x = parser.get<int>("x");
		int y = parser.get<int>("y");
		int z = parser.get<int>("z");
		if (x < 1 || y < 1 || z < 1) {
			std::cerr << "You need to define a valid number of voxels for the model. (--x/--y/--z)";
			break;
		}

		float size = parser.get<float>("size");
		if (size <= 0)
		{
			std::cerr << "You need to define a strictly positive voxel size. (--size)";
			break;
		}
		// 100, 100, 100, 0.0028 ~ Caruco
		Model model = Model(x, y, z, size);

		cv::Mat cameraMatrix, distCoeffs;
		if (parser.get<std::string>("calibration").empty())
		{
			std::cerr << "You need to define the path to the calibration file (--calibration)" << std::endl;
			break;
		}
		loadCalibrationFile(parser.get<std::string>("calibration"), &cameraMatrix, &distCoeffs);
		std::cout << "LOG - VC: read cameraMatrix and distCoefficients." << std::endl;

		// carve
		switch (carveArg)
		{
		case 1: carve(cameraMatrix, distCoeffs, model, images, masks, parser.get<bool>("intermediateMesh"));
			break;
		case 2: fastCarve(cameraMatrix, distCoeffs, model, images, masks);
			break;
		default:
			std::cerr << "Ups, something went wrong!" << std::endl;
		}

		// color reconstruction
		int color = parser.get<int>("color");
		if (color < 0 || 3 < color)
		{
			std::cerr << "You need to select a predefined color reconstruction mode. (--color)";
			break;
		}

		switch (color)
		{
		case 0:
			break;
		case 1: reconstructClosestColor(cameraMatrix, distCoeffs, model, images, masks);
			break;
		case 2: reconstructAvgColor(cameraMatrix, distCoeffs, model, images, masks);
			break;
		default:
			std::cerr << "Ups, something went wrong!" << std::endl;
		}

		//apply postprocessing
		model.handleUnseen();

		if (parser.get<bool>("model_debug")) {
			model.WriteModel();
		}

		if (parser.get<bool>("postprocessing")) {
			applyClosure(&model, 3);
		}

		//generate triangle mesh
		Vector3f modelTranslation = Vector3f(parser.get<float>("dx"), parser.get<float>("dy"), parser.get<float>("dz"));
		marchingCubes(&model, parser.get<float>("scale"), modelTranslation, 0.5f, parser.get<std::string>("outFile"));
	}
	break;
	case 6: // benchmarking, shows runtime of individual steps of the program (segmentation, voxel carving, post-processing)
	{
		std::string image_dir = parser.get<std::string>("images");
		if (image_dir.empty()) {
			std::cerr << "You need to define a images path (--images)" << std::endl;
			break;
		}

		std::vector<std::string> image_filenames;
		cv::glob(image_dir, image_filenames);
		std::vector<cv::Mat> images;
		for (int i = 0; i < image_filenames.size(); i++) {
			images.push_back(cv::imread(image_filenames[i], 1));
		}

		std::cout << "LOG - Benchmark: images read." << std::endl;

		std::string masks_dir = parser.get<std::string>("masks");
		if (masks_dir.empty()) {
			std::cerr << "You need to define a image masks path (--masks)" << std::endl;
			break;
		}

		std::vector<std::string> mask_filenames;
		cv::glob(masks_dir, mask_filenames);
		std::vector<cv::Mat> masks;
		for (int i = 0; i < mask_filenames.size(); i++) {
			masks.push_back(cv::imread(mask_filenames[i], 1));
		}

		std::cout << "LOG - Benchmark: masks read." << std::endl;

		cv::Mat cameraMatrix, distCoeffs;
		if (parser.get<std::string>("calibration").empty())
		{
			std::cerr << "You need to define the path to the calibration file (--calibration)" << std::endl;
			break;
		}
		loadCalibrationFile(parser.get<std::string>("calibration"), &cameraMatrix, &distCoeffs);
		std::cout << "LOG - Benchmark: read cameraMatrix and distCoefficients." << std::endl;

		std::filesystem::create_directories("./out/bench");
		// perform benchmarks here
		// small, v1, avg coloring
		Benchmark::GetInstance().NextRun("Small, V1, avg. coloring\t", Vector4f(10, 10, 5, 0.028f));
		Benchmark::GetInstance().LogOverall(true);
		Model model = Model(10, 10, 5, 0.028f);
		carve(cameraMatrix, distCoeffs, model, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model, images, masks);
		model.handleUnseen();
		applyClosure(&model, 3);
		Vector3f modelTranslation = Vector3f(parser.get<float>("dx"), parser.get<float>("dy"), parser.get<float>("dz"));
		marchingCubes(&model, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_small_1_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		// medium, v1, avg coloring
		Benchmark::GetInstance().NextRun("Medium, V1, avg. coloring\t", Vector4f(50, 50, 25, 0.0056f));
		Benchmark::GetInstance().LogOverall(true);
		Model model2 = Model(50, 50, 25, 0.0056f);
		carve(cameraMatrix, distCoeffs, model2, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model2, images, masks);
		model2.handleUnseen();
		applyClosure(&model2, 3);
		marchingCubes(&model2, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_medium_1_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		// medium, v1, closest coloring
		Benchmark::GetInstance().NextRun("Medium, V1, closest coloring\t", Vector4f(50, 50, 25, 0.0056f));
		Benchmark::GetInstance().LogOverall(true);
		Model model2b = Model(50, 50, 25, 0.0056f);
		carve(cameraMatrix, distCoeffs, model2b, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model2b, images, masks);
		model2b.handleUnseen();
		applyClosure(&model2b, 3);
		marchingCubes(&model2b, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_medium_1_closest.off");
		Benchmark::GetInstance().LogOverall(false);

		// large, v1, avg coloring
		Benchmark::GetInstance().NextRun("Large, V1, avg. coloring\t", Vector4f(100, 100, 50, 0.0028f));
		Benchmark::GetInstance().LogOverall(true);
		Model model3 = Model(100, 100, 50, 0.0028f);
		carve(cameraMatrix, distCoeffs, model3, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model3, images, masks);
		model3.handleUnseen();
		applyClosure(&model3, 3);
		marchingCubes(&model3, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_large_1_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		// small, v2, avg coloring
		Benchmark::GetInstance().NextRun("Small, V2, avg. coloring\t", Vector4f(10, 10, 5, 0.028f));
		Benchmark::GetInstance().LogOverall(true);
		Model model4 = Model(10, 10, 5, 0.028f);
		fastCarve(cameraMatrix, distCoeffs, model4, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model4, images, masks);
		model4.handleUnseen();
		applyClosure(&model4, 3);
		marchingCubes(&model4, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_small_2_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		// medium, v2, avg coloring
		Benchmark::GetInstance().NextRun("Medium, V2, avg. coloring\t", Vector4f(50, 50, 25, 0.0056f));
		Benchmark::GetInstance().LogOverall(true);
		Model model5 = Model(50, 50, 25, 0.0056f);
		fastCarve(cameraMatrix, distCoeffs, model5, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model5, images, masks);
		model5.handleUnseen();
		applyClosure(&model5, 3);
		marchingCubes(&model5, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_medium_2_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		// medium, v2, closest coloring
		Benchmark::GetInstance().NextRun("Medium, V2, closest coloring\t", Vector4f(50, 50, 25, 0.0056f));
		Benchmark::GetInstance().LogOverall(true);
		Model model6b = Model(50, 50, 25, 0.0056f);
		fastCarve(cameraMatrix, distCoeffs, model6b, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model6b, images, masks);
		model6b.handleUnseen();
		applyClosure(&model6b, 3);
		marchingCubes(&model6b, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_medium_2_closest.off");
		Benchmark::GetInstance().LogOverall(false);

		// large, v2, avg coloring
		Benchmark::GetInstance().NextRun("Large, V2, avg. coloring\t", Vector4f(100, 100, 50, 0.0028f));
		Benchmark::GetInstance().LogOverall(true);
		Model model7 = Model(100, 100, 50, 0.0028f);
		fastCarve(cameraMatrix, distCoeffs, model7, images, masks);
		reconstructAvgColor(cameraMatrix, distCoeffs, model7, images, masks);
		model7.handleUnseen();
		applyClosure(&model7, 3);
		marchingCubes(&model7, parser.get<float>("scale"), modelTranslation, 0.5f, "out/bench/mesh_large_2_avg.off");
		Benchmark::GetInstance().LogOverall(false);

		std::cout << Benchmark::GetInstance().to_string() << std::endl;
	}
	break;
	default:
		break;
	}
	return 0;
}