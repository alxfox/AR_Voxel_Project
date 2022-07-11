#pragma once

#include "ColorReconstruction.h"
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "Segmentation.h"

void reconstructColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - CR: starting color reconstruction." << std::endl;

    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);

    // Estimate pose for each image and remove distortion from images/masks
    std::vector<cv::Vec3f> cameras;
    std::vector<cv::Mat> poses;
    std::vector<cv::Mat> undist_imgs;
    std::vector<cv::Mat> undist_masks;
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, images[i], false);
        cv::Mat1f proj = pose * intr.inv()* cv::Vec3f(0, 0, 1);
        cameras.push_back(cv::Vec3f(proj(0) / proj(3), proj(1) / proj(3), proj(2) / proj(3)));
        poses.push_back(pose.inv()(cv::Rect(0, 0, 4, 3)));

        cv::Mat undist_img;
        cv::undistort(images[i], undist_img, cameraMatrix, distCoeffs);
        undist_imgs.push_back(undist_img);
        cv::Mat undist_mask;
        cv::undistort(masks[i], undist_mask, cameraMatrix, distCoeffs);
        undist_masks.push_back(undist_mask);
    }

    cv::Rect image_borders = cv::Rect(0, 0, images[0].cols, images[0].rows);

    for (int x = 0; x < model.getX(); x++)
    {
        for (int y = 0; y < model.getY(); y++)
        {
            for (int z = 0; z < model.getZ(); z++)
            {
                
            }
        }
    }

    std::cout << "LOG - CR: color reconstruction finished." << std::endl;
}