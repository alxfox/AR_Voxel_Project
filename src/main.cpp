#include <vector>
#include <iostream>
#include "Calibration.h"
#include <filesystem>
namespace fs = std::filesystem;
namespace {
	const char* about =
		"AR Voxel Carving Project\n";
	const char* keys =
		"{c        |       | Put value of c=1 to create charuco board;\nc=2 for camera calibration using ChArUco boards;\nc=3 not yet}"
		"{w        |       | Number of squares in X direction }"
		"{h        |       | Number of squares in Y direction }"
		"{sl       |       | Square side length (in meters) }"
		"{ml       |       | Marker side length (in meters) }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{cd       |       | Input file with custom dictionary }"
		"{@outfile |<none> | Output file with calibrated camera parameters }"
		"{v        |       | Input from video file, default is '../Data/calib_%02d.jpg' }"
		"{rs       | false | Apply refind strategy }"
		"{sc       | false | Show detected chessboard corners after calibration }";
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
		//detectChArUco();
		break;
	default:
		break;
	}
	return 0;
}