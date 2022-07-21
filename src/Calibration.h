#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include "aruco_samples_utility.hpp"
#include <iomanip>
#include <filesystem>
/*
	The following implementation for camera calibration is a heavily adapted version of the opencv sample: \opencv_contrib\modules\aruco\samples\calibrate_camera_charuco.cpp
*/

/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

						  License Agreement
			   For Open Source Computer Vision Library
					   (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

using namespace std;
//using namespace cv;
/**
	Class for camera calibration using charuco boards.
	A live video stream or a series of images or a video file may be used for calibration
**/
class Calibration {
private:
	string outputFile = "out/cameraration.yml";
    cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
    cv::Ptr<cv::aruco::Board> board;
	bool hasBoard = false;
	int liveImageCount = 0; // track how many images have been taken so far during live calibration
public:
	/*
	Generate a charuco board using the given parameters.
	Providing a target file name stores the board as an image file.
	The board generated is used for following calibrations
	*/
	bool refindStrategy = false; // whether to refine the charuco marker search
	float resizeFactor = 1; //resize image preview (to downsize high-resolution images)
    cv::String tmpFolder = "./out/tmp/";
	int createBoard(cv::String targetFile = "", int squaresX = 5, int squaresY = 7, float squareLength = 0.04, float markerLength = 0.02, int dictionaryId = 10) {
		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

		charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
		board = charucoboard.staticCast<cv::aruco::Board>();
		if (!targetFile.empty()) {
			cv::Mat boardImage;
			charucoboard->draw(cv::Size(2490, 3510), boardImage, 10, 1);
			cv::imwrite(targetFile, boardImage);
			std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
		}
		hasBoard = true;
		return 0;
	}
	Calibration(std::string outFile) {
		outputFile = outFile;
	}
	Calibration() {
	}
	int calibrate(vector<int>& excludedImages, bool captureLive, bool firstCalibration, cv::String imageLocation = "./out/tmp/calib_%03d.jpg", int camId = 0) {//"../Data/calib_%02d.jpg"
		if (!hasBoard) {
			cerr << "no charuco board has been defined to be used for calibration";
			return 0;
		}
		bool skipUserPrompt = false;
		int calibrationFlags = 0;
		float aspectRatio = 1;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

        cv::VideoCapture inputVideo;
		std::vector<cv::Mat> images;
		std::vector<std::string> imagesFilenames;
		int waitTime;
		if (captureLive) {
			inputVideo.open(camId);
			waitTime = 10;
			//clear tmp folder
			for (const auto& entry : std::filesystem::directory_iterator(tmpFolder))
				std::filesystem::remove_all(entry.path());
		}
		else {
			cv::glob(imageLocation, imagesFilenames);
			for (int i = 0; i < imagesFilenames.size(); i++) {
				images.push_back(cv::imread(imagesFilenames[i], 1));
			}
			//inputVideo.open(imageLocation);
			waitTime = 0;
		}

		// collect data from each frame
		vector< vector< vector< cv::Point2f > > > allCorners;
		vector< vector< int > > allIds;
		vector< cv::Mat > allImgs;
        cv::Size imgSize;
		int imageIndex = 0;

		while (true) {
            cv::Mat image, imageCopy;
			// get current image from camera stream/image folder
			if (captureLive) {
				if (!inputVideo.grab()) break;
				inputVideo.retrieve(image);
			}
			else {
				while (std::find(excludedImages.begin(), excludedImages.end(), imageIndex) != excludedImages.end()) imageIndex++;
				if (imageIndex >= images.size()) break;
				image = images[imageIndex];
			}

			bool addImageData = false; // whether to include current image's data in calibration
			vector< int > ids;
			vector< vector< cv::Point2f > > corners, rejected;

			// detect markers
            cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

			// refind strategy to detect more markers
			if (refindStrategy) cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

			// interpolate charuco corners
            cv::Mat currentCharucoCorners, currentCharucoIds;
			if (ids.size() > 0)
                cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
					currentCharucoIds);
			// draw results (highlight markers)
			image.copyTo(imageCopy);
			if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy, corners);

			if (currentCharucoCorners.total() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
			if (skipUserPrompt && !captureLive) { // once skipUserPrompt==true, accept all images
				addImageData = true;
			}
			else if (firstCalibration) // during initial calibration, prompt user for each image
			{
                cv::Mat resizedImage;
				cv::resize(imageCopy, resizedImage, cv::Size(), resizeFactor, resizeFactor);
				putText(resizedImage, "Press 'c' to add current frame, 'a' to add all. 'ESC' to finish and calibrate",
                        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
				imshow("out", resizedImage);
				char key = (char) cv::waitKey(waitTime);
				if (key == 27) break;
				if (key == 'c') {
					if (ids.size() > 0) {
						cout << "Frame captured" << endl;
						addImageData = true;
					}
					else {
						cout << "Frame skipped, no markers detected" << endl;
					}
				}
				if (key == 'a') {
					skipUserPrompt = true;
					addImageData = true;
				}
			}
			else if (std::find(excludedImages.begin(), excludedImages.end(), imageIndex) == excludedImages.end() && ids.size() > 0) // if image is not excluded, add it for calibration
			{
				cout << "Frame captured (" << imageIndex << ")" << endl;
				addImageData = true;
			}

			if (addImageData && ids.size() > 0) {
				//save data from accepted images for usage in calibration
				if (captureLive) {//images taken during live capture are saved to tmp so they can later be excluded by the user
					stringstream path;
					path << tmpFolder << "calib_" << std::setfill('0') << std::setw(3) << liveImageCount << ".jpg";
					imagesFilenames.push_back(path.str());
					imwrite(path.str(), image);
					liveImageCount++;
				}
				allCorners.push_back(corners);
				allIds.push_back(ids);
				allImgs.push_back(image);
				imgSize = image.size();
			}
			if(!addImageData && !captureLive) {//when doing non-live initial calibration, skipped images are added to the exclusion list
				excludedImages.push_back(imageIndex);
			}
			imageIndex++;
		}
		cv::destroyAllWindows();
		if (allIds.size() < 1) {
			cerr << "Not enough captures for calibration" << endl;
			return 0;
		}

        cv::Mat cameraMatrix, distCoeffs;
		vector< cv::Mat > rvecs, tvecs;
		double repError;

		if (calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			cameraMatrix.at< double >(0, 0) = aspectRatio;
		}

		// prepare data for calibration
		vector< vector< cv::Point2f > > allCornersConcatenated;
		vector< int > allIdsConcatenated;
		vector< int > markerCounterPerFrame;
		markerCounterPerFrame.reserve(allCorners.size());
		for (unsigned int i = 0; i < allCorners.size(); i++) {
			markerCounterPerFrame.push_back((int)allCorners[i].size());
			for (unsigned int j = 0; j < allCorners[i].size(); j++) {
				allCornersConcatenated.push_back(allCorners[i][j]);
				allIdsConcatenated.push_back(allIds[i][j]);
			}
		}

		// calibrate camera using aruco markers
		double arucoRepErr;
		arucoRepErr = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
			markerCounterPerFrame, board, imgSize, cameraMatrix,
			distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);

		// prepare data for charuco calibration
		int nFrames = (int)allCorners.size();
		vector< cv::Mat > allCharucoCorners;
		vector< cv::Mat > allCharucoIds;
		vector< cv::Mat > filteredImages;
		allCharucoCorners.reserve(nFrames);
		allCharucoIds.reserve(nFrames);

		for (int i = 0; i < nFrames; i++) {
			// interpolate using camera parameters
            cv::Mat currentCharucoCorners, currentCharucoIds;
            cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
				currentCharucoCorners, currentCharucoIds, cameraMatrix,
				distCoeffs);

			allCharucoCorners.push_back(currentCharucoCorners);
			allCharucoIds.push_back(currentCharucoIds);
			filteredImages.push_back(allImgs[i]);
		}

		if (allCharucoCorners.size() < 4) {
			cerr << "Not enough corners for calibration" << endl;
			return 0;
		}

		// calibrate camera using charuco and calculate reprojection error
        cv::Mat stdDeviationIn, stdDeviationOut, individualRepErrors;
		repError =
            cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
				cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationIn, stdDeviationOut, individualRepErrors, calibrationFlags);

		int offset = 0;
		for (size_t i = 0; i < individualRepErrors.rows; i++)
		{	// output used image files to console along with their individual reprojection error
			while (std::find(excludedImages.begin(), excludedImages.end(), i+offset) != excludedImages.end())offset++; // to ensure the labels for the images are correct, exclude the images that were not used
			std::cout << i + offset << ":\t" << individualRepErrors.row(i) << "\t" << imagesFilenames[i+offset] << std::endl;
		}


		bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
			cameraMatrix, distCoeffs, repError);
		if (!saveOk) {
			cerr << "Cannot save output file" << endl;
			return 0;
		}

		cout << "Rep Error: " << repError << endl;
		cout << "Rep Error Aruco: " << arucoRepErr << endl;
		cout << "Calibration saved to " << outputFile << endl;
		return 0;
	}
};
