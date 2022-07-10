#pragma once
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "VoxelCarving.h"
#include "Segmentation.h"

static cv::Vec3f worldToCamera(cv::Vec4f world, cv::Mat& pose, cv::Mat& intr) {
    // cv::Mat1f proj = intr * pose * cv::Vec4f(x, y, z, 1);
    cv::Mat1f proj = intr * pose * world;
    return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

static void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, cv::Mat& image, cv::Mat& mask) {
    // Estimate camera extrinsics
    cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, image, false);
    pose = pose.inv();

    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);

    cv::Rect image_borders = cv::Rect(0, 0, image.cols, image.rows);
    cv::Mat undist_img;
    cv::undistort(image, undist_img, cameraMatrix, distCoeffs);
    cv::Mat undist_mask;
    cv::undistort(mask, undist_mask, cameraMatrix, distCoeffs);

    for (int x = 0; x < model.getX(); x++)
    {
        for (int y = 0; y < model.getY(); y++)
        {
            for (int z = 0; z < model.getZ(); z++)
            {
                // for each voxel check if corresponding pixel is valid or background
                cv::Mat subm = pose(cv::Rect(0, 0, 4, 3));
                cv::Vec4f word_coord = model.toWord(x, y, z);
                cv::Vec3f camera_coord = worldToCamera(word_coord, subm, intr);
                cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1]));
                if (!pixel_pos.inside(image_borders))
                {
                    continue;
                }
                cv::Vec3b pixel = undist_mask.at<cv::Vec3b>(pixel_pos);
                if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0) // masked pixel -> set alpha = 0
                {
                    model.set(x, y, z, Eigen::Vector4f(0, 0, 0, 0));
                }
                model.see(x, y, z);
            }
        }
    }

    std::cout << "LOG - VC: completed carving of a single image." << std::endl;
}

void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - VC: starting carving process." << std::endl;
    for (int i = 0; i < images.size(); i++)
        carve(cameraMatrix, distCoeffs, model, images[i], masks[i]);
    std::cout << "LOG - VC: carving complete." << std::endl;
}