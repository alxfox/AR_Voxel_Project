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
using namespace cv;
/**
	Class for camera calibration using charuco boards.
	A live video stream or a series of images or a video file may be used for calibration
**/
class Calibration {
private:
	string outputFile = "out/cameracalibration.yml";
	Ptr<aruco::Dictionary> dictionary;
	Ptr<aruco::CharucoBoard> charucoboard;
	Ptr<aruco::Board> board;
	bool hasBoard = false;
	int liveImageCount = 0; // track how many images have been taken so far during live calibration
public:
	/*
	Generate a charuco board using the given parameters.
	Providing a target file name stores the board as an image file.
	The board generated is used for following calibrations
	*/
	bool refindStrategy = false; // whether to refine the charuco marker search
	bool showChessboardCorners = false;
	float resizeFactor = 1; //resize image preview (to downsize high-resolution images)
	String tmpFolder = "./out/tmp/";
	int createBoard(String targetFile = "", int squaresX = 5, int squaresY = 7, float squareLength = 0.04, float markerLength = 0.02, int dictionaryId = 10) {
		dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

		charucoboard = aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
		board = charucoboard.staticCast<aruco::Board>();
		if (!targetFile.empty()) {
			cv::Mat boardImage;
			charucoboard->draw(cv::Size(2490, 3510), boardImage, 10, 1);
			cv::imwrite(targetFile, boardImage);
			std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
		}
		hasBoard = true;
		return 0;
	}
	Calibration() {
		
	}
	int calibrate(vector<int>& excludedImages, bool captureLive, bool skipUserPrompt = false, String imageLocation = "./out/tmp/calib_%03d.jpg", int camId = 0) {//"../Data/calib_%02d.jpg"
		if (!hasBoard) {
			cerr << "no charuco board has been defined to be used for calibration";
			return 0;
		}
		int calibrationFlags = 0;
		float aspectRatio = 1;
		Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

		VideoCapture inputVideo;
		int waitTime;
		if (captureLive) {
			inputVideo.open(camId);
			waitTime = 10;
			//clear tmp folder
			for (const auto& entry : std::filesystem::directory_iterator(tmpFolder))
				std::filesystem::remove_all(entry.path());
		}
		else {
			inputVideo.open(imageLocation);
			waitTime = 0;
		}

		// collect data from each frame
		vector< vector< vector< Point2f > > > allCorners;
		vector< vector< int > > allIds;
		vector< Mat > allImgs;
		Size imgSize;
		int imageIndex = 0;
		while (inputVideo.grab()) {
			bool addImageData = false; // whether to include current image's data in calibration
			Mat image, imageCopy;
			inputVideo.retrieve(image);
			Mat imag;
			//cv::resize(image, imag, (0, 0), fx = 0.5, fy = 0.5);
			vector< int > ids;
			vector< vector< Point2f > > corners, rejected;

			// detect markers
			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

			// refind strategy to detect more markers
			if (refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

			// interpolate charuco corners
			Mat currentCharucoCorners, currentCharucoIds;
			if (ids.size() > 0)
				aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
					currentCharucoIds);
			// draw results
			image.copyTo(imageCopy);
			if (ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

			if (currentCharucoCorners.total() > 0)
				aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
			if (skipUserPrompt && !captureLive) {
				addImageData = true;
			}
			else if (excludedImages.size() == 0) // if no excludedImages are provided, prompt user
			{
				Mat resizedImage;
				cv::resize(imageCopy, resizedImage, cv::Size(), resizeFactor, resizeFactor);
				putText(resizedImage, "Press 'c' to add current frame, 'a' to add all. 'ESC' to finish and calibrate",
					Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
				imshow("out", resizedImage);
				char key = (char)waitKey(waitTime);
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
			else if (std::find(excludedImages.begin(), excludedImages.end(), imageIndex) == excludedImages.end() && ids.size() > 0) //if image is not excluded, add the data
			{
				cout << "Frame captured (" << imageIndex << ")" << endl;
				addImageData = true;
			}

			if (addImageData && ids.size() > 0) {
				if (captureLive) {
					stringstream path;
					path << tmpFolder << "calib_" << std::setfill('0') << std::setw(3) << liveImageCount << ".jpg";
					imwrite(path.str(), image);
					stringstream pathMarked;
					pathMarked << tmpFolder << "calib_m_" << std::setfill('0') << std::setw(3) << liveImageCount << ".jpg";
					imwrite(pathMarked.str(), imageCopy);
					liveImageCount++;
				}
				allCorners.push_back(corners);
				allIds.push_back(ids);
				allImgs.push_back(image);
				imgSize = image.size();
			}
			imageIndex++;
		}
		cv::destroyAllWindows();
		if (allIds.size() < 1) {
			cerr << "Not enough captures for calibration" << endl;
			return 0;
		}

		Mat cameraMatrix, distCoeffs;
		vector< Mat > rvecs, tvecs;
		double repError;

		if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
			cameraMatrix = Mat::eye(3, 3, CV_64F);
			cameraMatrix.at< double >(0, 0) = aspectRatio;
		}

		// prepare data for calibration
		vector< vector< Point2f > > allCornersConcatenated;
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
		arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
			markerCounterPerFrame, board, imgSize, cameraMatrix,
			distCoeffs, noArray(), noArray(), calibrationFlags);

		// prepare data for charuco calibration
		int nFrames = (int)allCorners.size();
		vector< Mat > allCharucoCorners;
		vector< Mat > allCharucoIds;
		vector< Mat > filteredImages;
		allCharucoCorners.reserve(nFrames);
		allCharucoIds.reserve(nFrames);

		for (int i = 0; i < nFrames; i++) {
			// interpolate using camera parameters
			Mat currentCharucoCorners, currentCharucoIds;
			aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
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

		// calibrate camera using charuco
		Mat stdDeviationIn, stdDeviationOut, individualRepErrors;

		repError =
			aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
				cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationIn, stdDeviationOut, individualRepErrors, calibrationFlags);

		int offset = 0;
		for (size_t i = 0; i < individualRepErrors.rows; i++)
		{
			if (std::find(excludedImages.begin(), excludedImages.end(), i) != excludedImages.end())offset++; // to ensure the labels for the images are correct, exclude the images that were not used
			std::cout << i + offset << ": " << individualRepErrors.row(i) << std::endl;
		}
		//std::cout << individualRepErrors << std::endl;


		bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
			cameraMatrix, distCoeffs, repError);
		if (!saveOk) {
			cerr << "Cannot save output file" << endl;
			return 0;
		}

		cout << "Rep Error: " << repError << endl;
		cout << "Rep Error Aruco: " << arucoRepErr << endl;
		cout << "Calibration saved to " << outputFile << endl;

		// show interpolated charuco corners for debugging
		if (showChessboardCorners) {
			for (unsigned int frame = 0; frame < filteredImages.size(); frame++) {
				Mat imageCopy = filteredImages[frame].clone();
				if (allIds[frame].size() > 0) {

					if (allCharucoCorners[frame].total() > 0) {
						aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],
							allCharucoIds[frame]);
					}
				}

				imshow("out", imageCopy);
				char key = (char)waitKey(0);
				if (key == 27) break;
			}
		}
		return 0;
	}
};