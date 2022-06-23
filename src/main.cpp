#include <vector>
#include <iostream>
#include "Calibration.h"
#include "PoseEstimation.h"
#include <filesystem>
namespace fs = std::filesystem;
namespace {
	const char* about =
		"AR Voxel Carving Project\n";
	const char* keys =
		"{c        		|       | 1 for AruCo board creation, 2 for camera calibration, 3 for pose estimation}"
		"{images        |       | Give the path to the directory containing the images}"
        "{calibration   |       | Give the path to the result of the camera calibration (eg. kinect_v1.txt)}"
        "{video_id      | -1    | Give the id to the video stream for which you want to estimate the pose}";
}

// generate board with default values (fits A4 format)
static inline void createBoard()
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	//! [createBoard]
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
	cv::Mat boardImage;
	board->draw(cv::Size(2490, 3510), boardImage, 10, 1);
	//! [createBoard]
	cv::imwrite("out/BoardImage.jpg", boardImage);
}
int main(int argc, char* argv[])
{
	std::filesystem::create_directories("./out");
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	if (argc < 2) {
		parser.printMessage();
		return 0;
	}
	int choose = parser.get<int>("c");
	//std::cout << "test" << choose << std::endl;
	switch (choose) {
	case 1:
		createBoard();
		std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
		break;
	case 2:
	{
		// TODO: add customization (data source, output etc.)
		/*if (parser.has("v")) {
			string video = parser.get<String>("v");
			//video = "../Data/calib_%02d.jpg";
			video = "../Data/calib.mp4";
		}*/
		bool calibrating = true;
		vector<int> excludedImages;
		do {
			calibrate(excludedImages);
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
	case 3:
	{
		cv::Mat camera_matrix, dist_Coeffs; 
		loadCalibrationFile(parser.get<std::string>("calibration"), &camera_matrix, &dist_Coeffs);
		std::cout << camera_matrix << dist_Coeffs << std::endl;

		// Determine whether to run with images or a video stream
		if (parser.get<std::string>("video_id").empty()){
			std::string image_dir = parser.get<std::string>("images");
			if (image_dir.empty()){
				std::cout << "You need to define a images path (--images) if you do not set a video stream id (--video_id)" << std::endl;
			} else {
				std::vector<std::string> filenames;
				cv::glob(parser.get<std::string>("images"), filenames);
				for (int i=0; i < filenames.size(); i++){
					cv::Mat image = cv::imread(filenames[i], 0);
					cv::Mat transformation_matrix = estimatePoseFromImage(camera_matrix, dist_Coeffs, image, true);
					//std::cout << transformation_matrix << std::endl;
				}
			}
		} else {
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
	default:
		break;
	}
	return 0;
}