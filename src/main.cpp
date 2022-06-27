#include <vector>
#include <iostream>
#include "Calibration.h"
#include "PoseEstimation.h"
#include "Segmentation.h"
#include <filesystem>
namespace fs = std::filesystem;
namespace {
	const char* about =
		"AR Voxel Carving Project\n";
	const char* keys =
		"{c        		|       | 1 for AruCo board creation, 2 for camera calibration, 3 for pose estimation}"
		"{resize        | 1.0      | Resize the image preview during calibration by this factor}"
		"{live        | true      | Whether to use live camera calibration, otherwise images will be taken from ../Data/calib_%02d.jpg}"
		"{images        |       | Give the path to the directory containing the images for pose estimation}"
		"{calibration   |       | Give the path to the result of the camera calibration (eg. kinect_v1.txt)}"
		"{video_id      | -1    | Give the id to the video stream for which you want to estimate the pose}";
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
					cv::Mat image = cv::imread(filenames[i], 0);
					cv::Mat transformation_matrix = estimatePoseFromImage(camera_matrix, dist_Coeffs, image, true);
					//std::cout << transformation_matrix << std::endl;
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
	default:
		break;
	}
	return 0;
}