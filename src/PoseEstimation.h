#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <iostream>

/**
 * @brief This function reads the 
 * 
 * @param calibrration_file_path File path to 
 * @return cv::Mat the intrinsic matrix as well as the distortion coefficients
 */
static inline void loadCalibrationFile(std::string calibration_file_path, cv::Mat *cameraMatrix, cv::Mat *distCoeffs){
    bool readOk = readCameraParameters(calibration_file_path, *cameraMatrix, *distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    }
}

static cv::Mat estimatePoseFromImage(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat image, bool visualize)
{
    cv::Mat imageCopy;
    image.copyTo(imageCopy);

    // Create the expected camera board
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    
    // Detect the markers in the image
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

    // Estimate pose if at least 1 marker has been detected
    cv::Mat4f transformation_matrix = cv::Mat::eye(4,4,CV_32F);
    if (markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
        // Check if at least 1 Charuco corner has been detected
        if (charucoIds.size() > 0) {
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);

            // Estimate the pose of the camera
            cv::Vec3d rvec, tvec;
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
            if (valid)
                cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                cv::Mat rotation_matrix = cv::Mat::eye(3,3,CV_32F);
                cv::Rodrigues(rvec, rotation_matrix);
                rotation_matrix = rotation_matrix.t();
                cv::Mat translation = -rotation_matrix * tvec;

                // Build the transformation matrix
                cv::Mat transformation_matrix = cv::Mat::eye(4, 4, rotation_matrix.type()); // T is 4x4
                transformation_matrix( cv::Range(0,3), cv::Range(0,3) ) = rotation_matrix * 1; // copies R into T
                transformation_matrix( cv::Range(0,3), cv::Range(3,4) ) = translation * 1; // copies tvec into T
                std::cout << translation << std::endl;
        }
    }
    if (visualize){
        cv::imshow("out", imageCopy);
        cv::waitKey(1);
    }
    return transformation_matrix;
}