#include <vector>
#include <iostream>
#include <filesystem>
#include "Calibration.h"
#include "PoseEstimation.h"
#include "Segmentation.h"
#include "VoxelCarving.h"
#include "MarchingCubes.h"
#include "Postprocessing3d.h"
namespace fs = std::filesystem;
namespace {
	const char* about =
		"AR Voxel Carving Project\n";
	const char* keys =
		"{c        		|       | 1 for AruCo board creation, 2 for camera calibration, 3 for pose estimation, 5 for carving}"
		"{resize        | 1.0   | Resize the image preview during calibration by this factor}"
		"{live          | true  | Whether to use live camera calibration, otherwise images will be taken from ../Data/calib_%02d.jpg}"
		"{images        |       | Give the path to the directory containing the images for pose estimation/carving}"
		"{calibration   |       | Give the path to the result of the camera calibration (eg. kinect_v1.txt)}"
		"{video_id      | -1    | Give the id to the video stream for which you want to estimate the pose}"
		"{carve         | 1     | 1 for standard carving, 2 for fast carving}"
		"{masks         |       | Give the path to the directory containing the image masks}"
		"{x             | 100   | Give the number of voxels in x direction.}"
		"{y             | 100   | Give the number of voxels in y direction.}"
		"{z             | 100   | Give the number of voxels in z direction.}"
		"{size          | 0.0028| Give the side length of a voxel.}"
		"{scale         | 1.0   | Give the scale factor for the output model.}"
		"{dx            | 0.0   | Move model in x direction (unscaled).}"
		"{dy            | 0.0   | Move model in y direction (unscaled).}"
		"{dz            | 0.0   | Move model in z direction (unscaled).}"
		;
}

int main(int argc, char* argv[])
{
	std::filesystem::create_directories("./out");
	std::filesystem::create_directories("./out/tmp");
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	if (argc < 2) {
		parser.printMessage();
		return 0;
	}
	int choose = parser.get<int>("c");
	//std::cout << "test" << choose << std::endl;
	switch (choose) {
	case 1: {
		Calibration cal{};
		cal.createBoard("out/BoardImage.jpg");
		break;
	}
	case 2:
	{
		try
		{
			String xxx = parser.get<String>("live");
		}
		catch (const std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}

		bool liveCalibration = parser.get<bool>("live");

		Calibration cal{};
		cal.createBoard();
		cal.resizeFactor = 2;
		vector<int> excludedImages;
		bool calibrating = true;
		bool firstCalibration = true;
		if (!liveCalibration) cal.resizeFactor = parser.get<float>("resize");
		do {
			cal.calibrate(excludedImages, liveCalibration, !firstCalibration, "../Data/calib_%02d.jpg");
			liveCalibration = false;
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
					else if (input.size() > 0 && input.at(0) == 'x') { // exit calibration
						liveCalibration = true;
						break;
					}
					else { // continue calibration loop with exclusions
						break;
					}
				}
			}
		} while (calibrating);
	}
	break;
	case 3:
	{
		cv::Mat camera_matrix, dist_Coeffs;
		loadCalibrationFile(parser.get<std::string>("calibration"), &camera_matrix, &dist_Coeffs);
		std::cout << camera_matrix << dist_Coeffs << std::endl;

		// Determine whether to run with images or a video stream
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
				//std::cout << transformation_matrix << std::endl;
			}
		}
	}
	break;
	case 4:
	{
		cv::VideoCapture inputVideo;
		inputVideo.open(parser.get<int>("video_id"));
		while (inputVideo.grab()) {
			cv::Mat image;
			inputVideo.retrieve(image);
			//cv::Mat segmentation_map = kmeans_segmentation(image);
			cv::Mat mask = color_segmentation(image);
			cv::Mat segmentated_img;
			image.copyTo(segmentated_img, mask);
			imshow("mask", mask);
			imshow("source", image);
			imshow("segmentation", segmentated_img);
			char key = (char)waitKey(1);
			//std::cout << transformation_matrix << std::endl;
		}
	}
	break;
	case 5:
	{
		int carveArg = parser.get<int>("carve");
		if (carveArg < 1 || 2 < carveArg) {
			std::cerr << "Invalid carve argument.";
			break;
		}
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
		std::cout << "LOG - VC: read carmeraMatrix and distCoefficients." << std::endl;

		// carve
		if (carveArg == 1)
		{
			carve(cameraMatrix, distCoeffs, model, images, masks);
		}
		else
		{
			fastCarve(cameraMatrix, distCoeffs, model, images, masks);
		}

		//apply postprocessing
		model.handleUnseen();
		applyClosure(&model, 3);

		//generate triangle mesh
		Vector3f modelTranslation = Vector3f(parser.get<float>("dx"), parser.get<float>("dy"), parser.get<float>("dz"));
		marchingCubes(&model, parser.get<float>("scale"), modelTranslation);
	}
	break;
	default:
		break;
	}
	return 0;
}